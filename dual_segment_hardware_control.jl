"""Two-IMU, four-channel commissioning and control for the two-section PCC arm.

No motor is enabled by including this file. The operator must complete
initialization and explicitly zero the arm before selecting a tracking shape.
"""
module DualSegmentHardwareControl

using LinearAlgebra, LibSerialPort, TOML, Printf
import GLMakie as GM

const PCC = Main.TwoSegmentPCCIMUServoControl
const IMU = PCC.IMU
const Bridge = IMU.Bridge
const CALIBRATION_PATH = joinpath(@__DIR__, "two_segment_hardware_calibration_v3.toml")
const IMU_MOUNT_PATH = joinpath(@__DIR__, "two_segment_imu_mount_v3.toml")

Base.@kwdef struct Config
    first::IMU.Config = IMU.Config(port="COM8", imu_id=1, servo_id=1, axial_servo_id=2,
        bend_motor_sign=-1, imu_bend_sign=-1, axial_rotation_sign=1,
        command_min=500, command_max=2500, max_speed=1000, max_command_rate=2500.0,
        zero_kp=5.0, zero_max_speed=200)
    second::IMU.Config = IMU.Config(port="COM8", imu_id=2, servo_id=3,
        axial_servo_id=4, bend_motor_sign=-1, imu_bend_sign=-1,
        axial_rotation_sign=-1, kp=5.0, command_min=500, command_max=2500,
        max_speed=400, max_command_rate=600.0, zero_kp=5.0, zero_max_speed=200)
end

"""Map each IMU's sensor frame to the PCC frame at the operator-confirmed straight pose.
The recorded absolute attitude becomes the zero reference; world-up alignment is not assumed.
"""
function validate_rotation(C)
    size(C) == (3, 3) && all(isfinite, C) &&
        norm(C' * C - I) < 1e-3 && abs(det(C) - 1) < 1e-3 ||
        error("IMU 安装坐标不是有效旋转矩阵")
    Matrix{Float64}(C)
end

function straight_imu_mounts(R1, R2, F1, F2)
    (validate_rotation(R1' * F1), validate_rotation(R2' * F2))
end

function validate_imu_mounts(mounts)
    length(mounts) == 2 || error("需要两只 IMU 的安装姿态")
    (validate_rotation(mounts[1]), validate_rotation(mounts[2]))
end

function save_imu_mounts(path::AbstractString, mounts)
    M1, M2 = validate_imu_mounts(mounts)
    open(path, "w") do io
        TOML.print(io, Dict("imu_mount" => Dict("version" => 3,
            "first_column_major" => collect(vec(M1)),
            "second_column_major" => collect(vec(M2)))))
    end
    path
end

function load_imu_mounts(path::AbstractString=IMU_MOUNT_PATH)
    isfile(path) || error("缺少双 IMU 安装标定；请在初始化窗口把操作臂摆到直态并点击「外部确认直态：仅记录双段直态」")
    d = TOML.parsefile(path)["imu_mount"]
    d["version"] == 3 || error("双 IMU 安装标定格式较旧；请在初始化窗口重新确认直态并记录")
    values = (Float64.(d["first_column_major"]), Float64.(d["second_column_major"]))
    all(v -> length(v) == 9, values) || error("每只 IMU 的安装姿态应包含 9 个矩阵元素")
    validate_imu_mounts((reshape(values[1], 3, 3), reshape(values[2], 3, 3)))
end

"""Return a new control configuration with only the tracking Kp values changed.
The trajectory plans, saved calibration, homing gains, and all other settings stay intact.
"""
function with_tracking_kp(c::Config; first=c.first.kp, second=c.second.kp)
    function replace_kp(segment::IMU.Config, kp)
        isfinite(kp) && kp > 0 || error("跟踪 Kp 必须为正的有限数")
        fields = Dict{Symbol,Any}(name => getfield(segment, name)
                                  for name in fieldnames(typeof(segment)))
        fields[:kp] = Float64(kp)
        IMU.Config(; fields...)
    end
    Config(first=replace_kp(c.first, first), second=replace_kp(c.second, second))
end

function validate(c::Config, adapter)
    IMU.validate(c.first, adapter)
    IMU.validate(c.second, adapter)
    c.first.port == c.second.port && c.first.baud == c.second.baud ||
        error("两只 IMU 和四路舵机必须通过同一块 ESP32 的 USB 串口通信")
    c.first.imu_id == 1 && c.second.imu_id == 2 ||
        error("接线约定：第一段 IMU1(GPIO8/9)，第二段 IMU2(GPIO10/11)")
    (c.first.servo_id, c.first.axial_servo_id, c.second.servo_id,
     c.second.axial_servo_id) == (1, 2, 3, 4) ||
        error("舵机通道必须依次为 CH1、CH2、CH3、CH4")
    c.first.neutral == c.second.neutral == 1500 || error("两路弯曲舵机必须以 1500 停止")
    c.first.axial_initial_signal == c.second.axial_initial_signal == 1500 ||
        error("两路位置舵机的中位必须为 1500")
    c
end

Base.@kwdef struct Calibration
    imu_signs::NTuple{2,Int}
    motor_signs::NTuple{2,Int}
    axial_signs::NTuple{2,Int}
    bend_sides::NTuple{2,Int} = (-1, -1)
end
Calibration(imu_signs::NTuple{2,Int}, motor_signs::NTuple{2,Int},
            axial_signs::NTuple{2,Int}) =
    Calibration(imu_signs, motor_signs, axial_signs, (-1, -1))

function validate(cal::Calibration)
    all(s -> s in (-1, 1), (cal.imu_signs..., cal.motor_signs...,
                             cal.axial_signs..., cal.bend_sides...)) ||
        error("初始化方向只能是 ±1")
    cal
end

function flipped_bend_pair(cal::Calibration, segment::Integer)
    segment in 1:2 || throw(BoundsError(cal.motor_signs, segment))
    validate(Calibration(
        imu_signs=ntuple(i -> i == segment ? -cal.imu_signs[i] : cal.imu_signs[i], 2),
        motor_signs=ntuple(i -> i == segment ? -cal.motor_signs[i] : cal.motor_signs[i], 2),
        axial_signs=cal.axial_signs, bend_sides=cal.bend_sides))
end

"""Change the allowed physical bending side; keep the IMU coordinate fixed.
Only the motor response polarity and physical geometry change, so they cannot
cancel each other as two coordinate-sign flips would.
"""
function flip_physical_bend_side(cal::Calibration, segment::Integer)
    segment in 1:2 || throw(BoundsError(cal.bend_sides, segment))
    validate(Calibration(imu_signs=cal.imu_signs,
        motor_signs=ntuple(i -> i == segment ? -cal.motor_signs[i] : cal.motor_signs[i], 2),
        axial_signs=cal.axial_signs,
        bend_sides=ntuple(i -> i == segment ? -cal.bend_sides[i] : cal.bend_sides[i], 2)))
end

function save_calibration(path::AbstractString, cal::Calibration)
    validate(cal)
    open(path, "w") do io
        TOML.print(io, Dict("calibration" => Dict(
            "version" => 4, "imu_signs" => collect(cal.imu_signs),
            "motor_signs" => collect(cal.motor_signs),
            "axial_signs" => collect(cal.axial_signs),
            "bend_sides" => collect(cal.bend_sides))))
    end
    path
end

function load_calibration(path::AbstractString=CALIBRATION_PATH)
    d = TOML.parsefile(path)["calibration"]
    d["version"] in (3, 4) || error("旧版保存了非绝对弯曲零位，必须重新初始化两段")
    validate(Calibration(imu_signs=Tuple(Int.(d["imu_signs"])),
        motor_signs=Tuple(Int.(d["motor_signs"])),
        axial_signs=Tuple(Int.(d["axial_signs"])),
        bend_sides=d["version"] == 4 ? Tuple(Int.(d["bend_sides"])) : (-1, -1)))
end

"""CH1/CH3 sliders span 800–2200 around stop=1500; outputs skip the measured dead band."""
function slider_commands(values, c::Config)
    length(values) == 4 || error("需要 CH1～CH4 四条滑轨")
    v = Int.(values)
    all(x -> 800 <= x <= 2200, (v[1], v[3])) ||
        error("CH1/CH3 初始化弯曲指令必须在 800～2200")
    all(x -> 1300 <= x <= 1700, (v[2], v[4])) ||
        error("CH2/CH4 初始化轴向位置必须在 1300～1700")
    function bend(x, s::IMU.Config)
        offset = x - 1500
        offset == 0 && return 1500
        command = offset > 0 ? s.positive_start + 1 + max(offset, s.min_speed_positive) :
                               s.negative_start - 1 - max(-offset, s.min_speed_negative)
        clamp(command, 800, 2200)
    end
    (bend1=bend(v[1], c.first), axial1=v[2],
     bend2=bend(v[3], c.second), axial2=v[4])
end

rotation_x(deg) = [1.0 0.0 0.0;
                   0.0 cosd(deg) -sind(deg);
                   0.0 sind(deg) cosd(deg)]

rotation_y(rad) = [cos(rad) 0.0 sin(rad);
                   0.0 1.0 0.0;
                   -sin(rad) 0.0 cos(rad)]
rotation_z(rad) = [cos(rad) -sin(rad) 0.0;
                   sin(rad) cos(rad) 0.0;
                   0.0 0.0 1.0]

"""Recover Rz(axial)*Ry(bend) from the complete terminal orientations.
The local +Z axis is the PCC tangent. Both bend and axial come from measured
IMU frames, so the fitted terminal tangent follows the sensor's physical X.
"""
function imu_pose(R1, R2, mounts; base_frame=diagm(0 => [1.0, -1.0, -1.0]),
                  bend_sides::NTuple{2,Int}=(-1, -1))
    M1, M2 = validate_imu_mounts(mounts)
    F1, F2 = R1 * M1, R2 * M2
    local1, local2 = base_frame' * F1, F1' * F2
    function section(Q, side)
        # The PCC terminal tangent is local +Z. Recover it from the full
        # rotation's third column, rather than using Q[3,1] as a proxy for
        # bend. The latter only works when Q lies exactly on Rz(a)*Ry(b).
        tangent = Q[:, 3]
        transverse = hypot(tangent[1], tangent[2])
        bend_sign = iszero(Q[3, 1]) ? 1.0 : -sign(Q[3, 1])
        bend_rad = atan(bend_sign * transverse, tangent[3])
        axial_rad = transverse > 1e-8 ?
            atan(bend_sign * tangent[2], bend_sign * tangent[1]) :
            atan(-Q[1, 2], Q[2, 2])
        reconstructed = rotation_z(axial_rad) * rotation_y(bend_rad)
        residual = full_orientation_error_deg(reconstructed, Q)
        (side * rad2deg(bend_rad), rad2deg(axial_rad), residual)
    end
    b1, a1, e1 = section(local1, bend_sides[1])
    b2, a2, e2 = section(local2, bend_sides[2])
    (bend=(b1, b2), axial=(a1, a2), residual=(e1, e2), frames=(F1, F2))
end

# This involution changes only the scalar control coordinate. It must be
# applied to BOTH target and measurement, and inverted for physical geometry.
# A sign change alone never rotates an IMU or changes the world-space path.
signed_bend_angles(bends, signs) = ntuple(i -> -signs[i] * bends[i], 2)
control_bends(pose, signs) = signed_bend_angles(pose.bend, signs)
tangent_alignment_deg(model_frame, imu_rotation) =
    acosd(clamp(dot(model_frame[:, 3], imu_rotation[:, 1]), -1.0, 1.0))
calibrated_tangent_alignment_deg(model_frame, imu_frame) =
    acosd(clamp(dot(model_frame[:, 3], imu_frame[:, 3]), -1.0, 1.0))
full_orientation_error_deg(model_frame, imu_frame) =
    acosd(clamp((tr(model_frame' * imu_frame) - 1) / 2, -1.0, 1.0))
validated_straight_mounts(mounts; kwargs...) = validate_imu_mounts(mounts)
unwrap_step_deg(current, previous) = mod(current - previous + 180.0, 360.0) - 180.0
initial_branch_valid(angles) = all(x -> isfinite(x) && abs(x) < 90.0, angles)
"""Select the physical 0–270° branch from an SO(3) principal bend angle."""
function one_sided_bend_branch(wrapped_deg; max_bend_deg=270.0)
    isfinite(wrapped_deg) || error("IMU 弯曲角无效，停止自动回直")
    -2.0 <= wrapped_deg < 0.0 && return 0.0  # near-zero sensor noise
    0.0 <= wrapped_deg <= 180.0 && return Float64(wrapped_deg)
    -180.0 <= wrapped_deg <= max_bend_deg - 360.0 &&
        return Float64(wrapped_deg + 360.0)
    error("IMU 弯曲角 $(round(wrapped_deg; digits=1))° 不在单侧 0～$(max_bend_deg)° 可判定分支内")
end

homing_capture_ready(physical, stable, required, tolerance) =
    stable >= required && all(i -> isfinite(physical[i]) &&
        abs(physical[i]) <= tolerance[i], 1:2)
zero_motion_worsened(start_abs_deg, current_abs_deg; threshold_deg=3.0) =
    current_abs_deg - start_abs_deg >= threshold_deg

"""Absolute angle between each IMU's X axis and world down, in degrees."""
function absolute_straight_errors(R1, R2)
    down = [0.0, 0.0, -1.0]
    ntuple(seg -> acosd(clamp(dot((seg == 1 ? R1 : R2)[:, 1], down), -1.0, 1.0)), 2)
end

resolved_motor_signs(observed, defaults) =
    ntuple(i -> observed[i] in (-1, 1) ? observed[i] : defaults[i], 2)

motor_sign_after_jog(current, inferred, manually_set) =
    manually_set ? current : inferred

"""Advance zeroing only after the currently active section holds its own zero."""
function zero_phase_step(phase, angles, tolerances, stable, hold_samples)
    if phase == :zero_first
        count = abs(angles[1]) <= tolerances[1] ? stable + 1 : 0
        return count >= hold_samples[1] ?
            (phase=:zero_second, stable=0, stop=true, active=0) :
            (phase=:zero_first, stable=count, stop=false, active=1)
    elseif phase == :zero_second
        abs(angles[1]) <= tolerances[1] ||
            return (phase=:zero_first, stable=0, stop=true, active=0)
        count = abs(angles[2]) <= tolerances[2] ? stable + 1 : 0
        return count >= hold_samples[2] ?
            (phase=:ready, stable=0, stop=true, active=0) :
            (phase=:zero_second, stable=count, stop=false, active=2)
    end
    error("未知的双段归零阶段：$phase")
end

active_motor_segments(phase, active_zero) =
    phase == :tracking ? (1, 2) :
    phase == :zero_first && active_zero == 1 ? (1,) :
    phase == :zero_second && active_zero == 2 ? (2,) : ()

"""Pulse the minimum effective motor speed during homing, with a coast/settle gap.
The continuous-rotation servo has a substantial dead band, so sustained low-gain
commands still cause large overshoot around zero.
"""
function homing_pulse(error_deg, now, pulse_until, next_pulse, direction,
                      cfg::IMU.Config; on_time=0.10, off_time=0.30)
    if abs(error_deg) <= cfg.zero_tolerance_deg
        return (command=cfg.neutral, pulse_until=now,
                next_pulse=now + off_time, direction=0)
    end
    requested_direction = error_deg > 0 ? 1 : -1
    if direction != 0 && direction != requested_direction && now < pulse_until
        return (command=cfg.neutral, pulse_until=now,
                next_pulse=now + off_time, direction=0)
    end
    if now < pulse_until && direction == requested_direction
        command = IMU.servo_speed_command(error_deg, 0.0, 0.0, cfg;
            kp=cfg.zero_kp, ki=0.0, kd=0.0, max_speed=cfg.zero_max_speed)
        return (command=command, pulse_until, next_pulse, direction)
    end
    now < next_pulse && return (command=cfg.neutral, pulse_until, next_pulse, direction)
    command = IMU.servo_speed_command(error_deg, 0.0, 0.0, cfg;
        kp=cfg.zero_kp, ki=0.0, kd=0.0, max_speed=cfg.zero_max_speed)
    (command=command, pulse_until=now + on_time,
     next_pulse=now + on_time + off_time, direction=requested_direction)
end

function send_bend!(serial, c::IMU.Config, value)
    Bridge.write_command(serial, IMU.servo_command(c, value))
    Int(value)
end

function stop_bends!(serial, c::Config)
    # Send both even if the first write fails; a serial disconnect is still
    # not a hardware watchdog and must be handled physically by the operator.
    try send_bend!(serial, c.first, 1500) finally
        send_bend!(serial, c.second, 1500)
    end
end

mutable struct SensorState
    buffer::Vector{UInt8}
    latest::Dict{Int,NamedTuple}
    filtered::Dict{Int,Matrix{Float64}}
    accepted::Dict{Int,Matrix{Float64}}
    last_valid::Dict{Int,Float64}
    last_seq::Dict{Int,UInt32}
end
SensorState() = SensorState(UInt8[], Dict{Int,NamedTuple}(), Dict{Int,Matrix{Float64}}(),
                            Dict{Int,Matrix{Float64}}(), Dict{Int,Float64}(),
                            Dict{Int,UInt32}())

function read_sensors!(serial, s::SensorState, c::Config, now)
    new_ids = Int[]
    n = LibSerialPort.bytesavailable(serial)
    n == 0 && return new_ids, String[]
    records, messages = Bridge.decode!(s.buffer, read(serial, n))
    for (id, record) in records
        id in (c.first.imu_id, c.second.imu_id) || continue
        get(s.last_seq, id, typemax(UInt32)) == record.seq && continue
        s.last_seq[id] = record.seq
        config = id == c.first.imu_id ? c.first : c.second
        raw = Matrix{Float64}(IMU.sensor_rotation(record, config))
        dt = now - get(s.last_valid, id, now)
        if haskey(s.accepted, id) &&
           IMU.rotation_step_deg(s.accepted[id], raw) >
           config.max_imu_rate_deg_s * max(dt, 0.0) + config.imu_outlier_margin_deg
            continue  # Isolated jump: retain previous valid pose, no PID update.
        end
        s.latest[id] = record
        s.accepted[id] = raw
        s.filtered[id] = haskey(s.filtered, id) ?
            IMU.lowpass_rotation(s.filtered[id], raw, max(dt, 1e-4), config.imu_filter_tau) : raw
        s.last_valid[id] = now
        push!(new_ids, id)
    end
    new_ids, messages
end

fresh(s::SensorState, c::Config, now) =
    all((cfg -> haskey(s.last_valid, cfg.imu_id) &&
         now - s.last_valid[cfg.imu_id] < cfg.imu_timeout), (c.first, c.second))

paired(s::SensorState, c::Config) =
    haskey(s.last_valid, c.first.imu_id) && haskey(s.last_valid, c.second.imu_id) &&
    abs(s.last_valid[c.first.imu_id] - s.last_valid[c.second.imu_id]) <= 0.25

function predicted_joints(prepared, physical_bends, axial_commands, cal::Calibration;
                          measured_axial=nothing)
    first(predicted_geometry(prepared, physical_bends, axial_commands, cal;
                             measured_axial))
end

function predicted_geometry(prepared, physical_bends, axial_commands, cal::Calibration;
                            measured_axial=nothing)
    a1 = measured_axial === nothing ?
        cal.axial_signs[1] * (axial_commands[1] - 1500) * 300 / 2000 : measured_axial[1]
    a2 = measured_axial === nothing ?
        cal.axial_signs[2] * (axial_commands[2] - 1500) * 300 / 2000 : measured_axial[2]
    Main.calculate_multi_segment_kinematics(
        [Main.R_DRIVE * deg2rad(physical_bends[1]), Main.R_DRIVE * deg2rad(physical_bends[2])],
        [a1, a2], prepared.data.two_segment_state.base_z_offset;
        bend_sides=cal.bend_sides)
end

function frame_axis_line(origin, rotation, axis_id; length_mm=42.0)
    p0 = Float64.(origin)
    p1 = p0 + length_mm .* Float64.(rotation[:, axis_id])
    GM.Point3f[GM.Point3f(p0...), GM.Point3f(p1...)]
end

function make_initial_view(prepared, c::Config)
    GM.activate!()
    fig = GM.Figure(size=(1450, 1000))
    ax = GM.Axis3(fig[1, 1]; title="双 IMU：两段 PCC 初始化", aspect=:data,
                  xlabel="X / mm", ylabel="Y / mm", zlabel="Z / mm")
    state = prepared.data.two_segment_state
    planned = GM.Observable(copy(state.joints_history[1]))
    predicted = GM.Observable(copy(state.joints_history[1]))
    actual_tangents = [GM.Observable(GM.Point3f[]) for _ in 1:2]
    fitted_tangents = [GM.Observable(GM.Point3f[]) for _ in 1:2]
    split = Main.N_LINKS + 2
    GM.lines!(ax, GM.lift(points -> copy(points[1:split]), planned);
              color=:dodgerblue, linewidth=5, label="直态第一段")
    GM.lines!(ax, GM.lift(points -> copy(points[split:end]), planned);
              color=:orange, linewidth=5, label="直态第二段")
    GM.lines!(ax, GM.lift(points -> copy(points[1:split]), predicted);
              color=:dodgerblue, linewidth=3, linestyle=:dash, label="IMU 拟合第一段")
    GM.lines!(ax, GM.lift(points -> copy(points[split:end]), predicted);
              color=:orange, linewidth=3, linestyle=:dash, label="IMU 拟合第二段")
    for seg in 1:2
        GM.lines!(ax, actual_tangents[seg]; color=seg == 1 ? :red : :purple,
                  linewidth=3, label="IMU 实测切向 $seg")
        GM.lines!(ax, fitted_tangents[seg]; color=:black, linewidth=2,
                  linestyle=:dash, label="PCC 重建切向 $seg")
    end
    GM.scatter!(ax, predicted; color=:black, markersize=5, label="关节点")
    GM.limits!(ax, (-200, 200), (-250, 350), (-800, 200))
    GM.axislegend(ax)
    panel = GM.GridLayout(); fig[1, 2] = panel
    status = GM.Observable("观察 IMU 与实体弯曲方向；已有安装标定时可点击 IMU 自动回直")
    values = GM.Observable("CH1=1500  CH2=1500  CH3=1500  CH4=1500")
    angles = GM.Observable("等待 IMU1（第一段）与 IMU2（第二段）")
    GM.Label(panel[1, 1], status; tellwidth=false)
    GM.Label(panel[2, 1], values; tellwidth=false)
    GM.Label(panel[3, 1], angles; tellwidth=false)
    sliders = Any[]; labels = Any[]
    for (i, name) in enumerate(("CH1 第一段弯曲速度", "CH2 第一段轴向位置",
                                "CH3 第二段弯曲速度", "CH4 第二段轴向位置"))
        label = GM.Observable("$(name)：1500")
        GM.Label(panel[2i+2, 1], label; tellwidth=false)
        range = isodd(i) ? (800:1:2200) : (1300:5:1700)
        push!(sliders, GM.Slider(panel[2i+3, 1]; range, startvalue=1500, tellwidth=false))
        push!(labels, label)
    end
    flip1 = GM.Button(panel[12, 1]; label="反转第一段控制角定义（不改物理侧）", tellwidth=false)
    flip2 = GM.Button(panel[13, 1]; label="反转第二段控制角定义（不改物理侧）", tellwidth=false)
    flip_motor1 = GM.Button(panel[14, 1]; label="切换第一段实际弯曲侧", tellwidth=false)
    flip_motor2 = GM.Button(panel[15, 1]; label="切换第二段实际弯曲侧", tellwidth=false)
    flip_axis1 = GM.Button(panel[16, 1]; label="反转 CH2 轴向定义", tellwidth=false)
    flip_axis2 = GM.Button(panel[17, 1]; label="反转 CH4 轴向定义", tellwidth=false)
    stop = GM.Button(panel[18, 1]; label="停止 CH1/CH3 并使四条滑轨居中", tellwidth=false)
    auto_zero = GM.Button(panel[19, 1]; label="IMU 自动回直并记录双段直态", tellwidth=false)
    capture_straight = GM.Button(panel[20, 1]; label="外部确认直态：仅记录双段直态", tellwidth=false)
    finish = GM.Button(panel[21, 1]; label="完成初始化并进入两段控制", tellwidth=false)
    GM.Label(panel[22, 1], "滑轨中心 1500；CH1/CH3 左右为反向/正向转动，\nCH2/CH4 左右为位置偏移。连续点动最多 2 秒。\n切换实际弯曲侧会改变该段物理模型与电机方向，世界空间轨迹保持不变。\n请由操作者确认两段直态并记录完整 IMU 安装姿态；\n基座倾斜不作为弯曲，也不要求 IMU X 轴对准世界竖直方向。";
             tellwidth=false)
    GM.colsize!(fig.layout, 1, GM.Relative(0.65))
    display(fig)
    (; fig, ax, predicted, actual_tangents, fitted_tangents, status, values, angles, sliders, labels,
       flip1, flip2, flip_motor1, flip_motor2, flip_axis1, flip_axis2,
       stop, auto_zero, capture_straight, finish)
end

function center_sliders!(ui)
    for slider in ui.sliders
        GM.set_close_to!(slider, 1500)
    end
end

"""Commissioning with optional sequential IMU homing; tracking never starts here."""
function run_initialization(prepared; config=Config(), open_port=LibSerialPort.open,
                            save_path=CALIBRATION_PATH,
                            mount_path=IMU_MOUNT_PATH)
    c = validate(config, prepared.data.ref.adapter)
    ui = make_initial_view(prepared, c)
    requests = Dict(k => Ref(false) for k in (:flip1, :flip2, :flip_motor1, :flip_motor2,
                                               :flip_axis1, :flip_axis2,
                                               :stop, :auto_zero, :capture_straight, :finish))
    for key in keys(requests)
        button = getproperty(ui, key)
        GM.on(_ -> (requests[key][] = true), button.clicks)
    end
    s = SensorState()
    saved_calibration = isfile(save_path) ? load_calibration(save_path) : nothing
    saved_mounts = isfile(mount_path) ? load_imu_mounts(mount_path) : nothing
    mounts = Ref(saved_mounts)
    signs = saved_calibration === nothing ?
        [c.first.imu_bend_sign, c.second.imu_bend_sign] :
        collect(saved_calibration.imu_signs)
    axis_signs = saved_calibration === nothing ?
        [c.first.axial_rotation_sign, c.second.axial_rotation_sign] :
        collect(saved_calibration.axial_signs)
    motor_signs = saved_calibration === nothing ?
        [c.first.bend_motor_sign, c.second.bend_motor_sign] :
        collect(saved_calibration.motor_signs)
    bend_sides = saved_calibration === nothing ? [-1, -1] :
        collect(saved_calibration.bend_sides)
    saved_mounts === nothing && (ui.status[] = "请手动摆到直态后点击「外部确认直态：仅记录双段直态」；直态由你确认，不检查世界竖直方向")
    # A deliberate button choice must not be overwritten by subsequent jog inference.
    manual_motor_override = falses(2)
    legacy_path = joinpath(@__DIR__, "two_segment_hardware_calibration.toml")
    if saved_calibration === nothing && isfile(legacy_path)
        legacy = TOML.parsefile(legacy_path)["calibration"]
        if get(legacy, "version", 0) == 1
            signs[1] = Int(legacy["imu_bend_sign"])
            axis_signs[1] = Int(legacy["axial_rotation_sign"])
            ui.status[] = "已载入第一段历史方向；可按需观察并反转方向定义"
        end
    end
    serial = open_port(c.first.port, c.first.baud)
    conflicting = [false, false]
    last_bends = [1500, 1500]; last_axes = [1500, 1500]
    last_bend_send = [0.0, 0.0]
    last_axis_send = -Inf
    jog_start = [0.0, 0.0]; anchor = [0.0, 0.0]; active = [0, 0]
    candidates = [0, 0]; counts = [0, 0]; result = nothing
    auto_phase = Ref(:idle)
    auto_phase_start = Ref(0.0)
    auto_stable = Ref(0)
    auto_previous_command = [1500, 1500]
    auto_pulse_until = [0.0, 0.0]
    auto_next_pulse = [0.0, 0.0]
    auto_pulse_direction = [0, 0]
    auto_monitor_angle = [NaN, NaN]
    auto_monitor_time = [NaN, NaN]
    auto_last_seq = [typemax(UInt32), typemax(UInt32)]
    now_s() = time_ns() / 1e9
    function stop_auto!(message, now)
        stop_bends!(serial, c)
        last_bends .= 1500
        last_bend_send .= now
        auto_previous_command .= 1500
        auto_phase[] = :idle
        auto_stable[] = 0
        ui.status[] = message
    end
    try
        stop_bends!(serial, c)
        Bridge.write_command(serial, "PING\n"); Bridge.write_command(serial, "SCAN\n")
        while GM.isopen(ui.fig.scene)
            now = now_s()
            new_ids, messages = read_sensors!(serial, s, c, now)
            for msg in messages
                (startswith(msg, "ERR") || startswith(msg, "WATCHDOG")) &&
                    (ui.status[] = "ESP32: $msg")
            end
            ok = fresh(s, c, now) && paired(s, c)
            if requests[:stop][]
                requests[:stop][] = false; center_sliders!(ui)
                requests[:auto_zero][] = false
                stop_auto!("CH1/CH3 已停止；四条滑轨回到 1500", now)
            end
            if requests[:auto_zero][]
                requests[:auto_zero][] = false
                if auto_phase[] != :idle
                    ui.status[] = "IMU 自动回直已在运行；需要中止请点击停止"
                elseif mounts[] === nothing
                    ui.status[] = "尚无直态安装标定；先手动摆直并点击「外部确认直态：仅记录双段直态」"
                elseif !ok
                    ui.status[] = "两只 IMU 必须同时在线才能自动回直"
                elseif any(sl -> sl.value[] != 1500, ui.sliders)
                    ui.status[] = "先停止点动并使四条滑轨回到 1500，再启动自动回直"
                elseif any(conflicting)
                    ui.status[] = "点动方向曾与保存定义冲突；请先核对方向，暂不启动自动回直"
                else
                    try
                        pose = imu_pose(s.filtered[c.first.imu_id],
                            s.filtered[c.second.imu_id], mounts[];
                            bend_sides=Tuple(bend_sides))
                        ntuple(i -> one_sided_bend_branch(pose.bend[i]), 2)
                        stop_bends!(serial, c)
                        last_bends .= 1500
                        last_bend_send .= now
                        for cfg in (c.first, c.second)
                            Bridge.write_command(serial,
                                IMU.axial_servo_command(cfg, 1500))
                        end
                        last_axes .= 1500
                        last_axis_send = now
                        auto_previous_command .= 1500
                        auto_pulse_until .= now
                        auto_next_pulse .= now
                        auto_pulse_direction .= 0
                        auto_monitor_angle .= NaN
                        auto_monitor_time .= NaN
                        auto_last_seq .= [s.latest[c.first.imu_id].seq,
                                           s.latest[c.second.imu_id].seq]
                        auto_phase[] = :axis_homing
                        auto_phase_start[] = now
                        auto_stable[] = 0
                        ui.status[] = "IMU 自动回直：CH2/CH4 先回中位，随后仅 CH1 回直第一段"
                    catch err
                        stop_auto!("无法启动自动回直：$(sprint(showerror, err))", now)
                    end
                end
            end
            if requests[:capture_straight][]
                requests[:capture_straight][] = false
                if auto_phase[] != :idle
                    ui.status[] = "自动回直进行中，不能提前记录直态"
                elseif !ok
                    ui.status[] = "两只 IMU 数据须同时在线，才能记录双段直态"
                elseif any(x -> x != 1500, (last_bends..., last_axes...)) ||
                       any(sl -> sl.value[] != 1500, ui.sliders)
                    ui.status[] = "先停止四路舵机并使滑轨居中，再记录双段直态"
                elseif now - last_axis_send < 1.5 || now - maximum(last_bend_send) < 0.5
                    ui.status[] = "等待舵机停止、轴向位置稳定后再记录双段直态"
                else
                    R1 = s.filtered[c.first.imu_id]
                    R2 = s.filtered[c.second.imu_id]
                    _, straight_frames = Main.calculate_multi_segment_kinematics(
                        [0.0, 0.0], [0.0, 0.0],
                        prepared.data.two_segment_state.base_z_offset)
                    measured_mounts = straight_imu_mounts(R1, R2,
                        straight_frames[1][2], straight_frames[2][2])
                    save_imu_mounts(mount_path, measured_mounts)
                    mounts[] = measured_mounts
                    ui.status[] = "已保存你确认的双段直态姿态作为安装基准；世界竖直偏差不参与弯曲计算"
                end
            end
            for (key, seg) in ((:flip1, 1), (:flip2, 2))
                if requests[key][]
                    auto_phase[] != :idle && stop_auto!("方向变更：自动回直已停止", now)
                    requests[key][] = false
                    signs[seg] *= -1
                    motor_signs[seg] *= -1
                    manual_motor_override[seg] = true
                    active[seg] = 0
                    conflicting[seg] = false
                    center_sliders!(ui); stop_bends!(serial, c); last_bends .= 1500
                    ui.status[] = "第 $seg 段仅改变控制角定义；IMU 与电机符号成对反转，物理弯曲侧不变"
                end
            end
            for (key, seg) in ((:flip_motor1, 1), (:flip_motor2, 2))
                if requests[key][]
                    auto_phase[] != :idle && stop_auto!("方向变更：自动回直已停止", now)
                    requests[key][] = false
                    flipped = flip_physical_bend_side(Calibration(
                        imu_signs=Tuple(signs), motor_signs=Tuple(motor_signs),
                        axial_signs=Tuple(axis_signs),
                        bend_sides=Tuple(bend_sides)), seg)
                    motor_signs .= flipped.motor_signs
                    bend_sides .= flipped.bend_sides
                    manual_motor_override[seg] = true
                    active[seg] = 0; conflicting[seg] = false
                    candidates[seg] = 0; counts[seg] = 0
                    center_sliders!(ui); stop_bends!(serial, c); last_bends .= 1500
                    ui.status[] = "第 $seg 段实际弯曲侧=$(bend_sides[seg])，CH$(2seg-1)方向=$(motor_signs[seg])；请先回直并短时点动验证"
                end
            end
            for (key, seg) in ((:flip_axis1, 1), (:flip_axis2, 2))
                if requests[key][]
                    auto_phase[] != :idle && stop_auto!("方向变更：自动回直已停止", now)
                    requests[key][] = false; axis_signs[seg] *= -1
                    ui.status[] = "第 $seg 段轴向方向定义已反转"
                end
            end
            if auto_phase[] != :idle
                if any(sl -> sl.value[] != 1500, ui.sliders) || requests[:finish][]
                    center_sliders!(ui)
                    stop_auto!("手动操作打断 IMU 自动回直，CH1/CH3 已停止", now)
                elseif !ok
                    stop_auto!("任一 IMU 超时或两只数据不同步，自动回直已停止", now)
                elseif now - auto_phase_start[] > 45.0
                    stop_auto!("该段 45 秒内未回直，自动回直已停止；请检查方向或力矩", now)
                elseif auto_phase[] == :axis_homing
                    if now - auto_phase_start[] >= 1.5
                        auto_phase[] = :zero_first
                        auto_phase_start[] = now
                        auto_last_seq .= [s.latest[c.first.imu_id].seq,
                                           s.latest[c.second.imu_id].seq]
                        ui.status[] = "第一段自动回直：只驱动 CH1，CH3 保持 1500"
                    end
                elseif auto_phase[] == :settling
                    if now - auto_phase_start[] >= 0.7
                        try
                            pose = imu_pose(s.filtered[c.first.imu_id],
                                s.filtered[c.second.imu_id], mounts[];
                                bend_sides=Tuple(bend_sides))
                            physical = ntuple(i -> one_sided_bend_branch(pose.bend[i]), 2)
                            if homing_capture_ready(physical, 1, 1,
                                (c.first.zero_tolerance_deg, c.second.zero_tolerance_deg))
                                stop_auto!("两段已由 IMU 依次回直并稳定；保留操作者记录的直态安装基准", now)
                            else
                                stop_auto!("停止后 IMU 直态不稳定；旧标定保留，CH1/CH3 已停止", now)
                            end
                        catch err
                            stop_auto!("直态复核失败，旧标定保留：$(sprint(showerror, err))", now)
                        end
                    end
                else
                    try
                        pose = imu_pose(s.filtered[c.first.imu_id],
                            s.filtered[c.second.imu_id], mounts[];
                            bend_sides=Tuple(bend_sides))
                        physical = ntuple(i -> one_sided_bend_branch(pose.bend[i]), 2)
                        measured = ntuple(i -> -signs[i] * physical[i], 2)
                        tilt = absolute_straight_errors(s.filtered[c.first.imu_id],
                            s.filtered[c.second.imu_id]) # telemetry only; not a straight-pose gate
                        both_advanced = all(i -> s.latest[i == 1 ? c.first.imu_id :
                            c.second.imu_id].seq != auto_last_seq[i], 1:2)
                        first_drift = auto_phase[] == :zero_second &&
                            abs(measured[1]) > c.first.zero_tolerance_deg
                        active_seg = auto_phase[] == :zero_first ? 1 : 2
                        if both_advanced || first_drift
                            transition = zero_phase_step(auto_phase[], measured,
                                (c.first.zero_tolerance_deg, c.second.zero_tolerance_deg),
                                auto_stable[],
                                (c.first.zero_hold_samples, c.second.zero_hold_samples))
                            if both_advanced
                                auto_last_seq .= [s.latest[c.first.imu_id].seq,
                                                   s.latest[c.second.imu_id].seq]
                            end
                            auto_stable[] = transition.stable
                            if transition.stop
                                stop_bends!(serial, c)
                                last_bends .= 1500
                                last_bend_send .= now
                                auto_previous_command .= 1500
                                auto_monitor_angle .= NaN
                                auto_monitor_time .= NaN
                                auto_phase[] = transition.phase
                                auto_phase_start[] = now
                                if transition.phase == :ready
                            if homing_capture_ready(physical,
                                        c.second.zero_hold_samples,
                                        c.second.zero_hold_samples,
                                        (c.first.zero_tolerance_deg,
                                         c.second.zero_tolerance_deg))
                                        auto_phase[] = :settling
                                        ui.status[] = "两段角度已回直；CH1/CH3 停止，等待 0.7 秒再复核 IMU 并记录"
                                    else
                                        stop_auto!("IMU 相对弯曲角未达到直态容差；旧标定保留，CH1/CH3 已停止", now)
                                    end
                                else
                                    ui.status[] = transition.phase == :zero_second ?
                                        "第一段已回直并停止；现在仅 CH3 回直第二段" :
                                        "第一段偏离直态；CH3 停止，重新先归零第一段"
                                end
                                active_seg = 0
                            end
                        end
                        if active_seg != 0 && auto_phase[] in (:zero_first, :zero_second)
                            seg = active_seg
                            cfg = seg == 1 ? c.first : c.second
                            if last_bends[seg] == 1500 || !isfinite(auto_monitor_angle[seg])
                                auto_monitor_angle[seg] = physical[seg]
                                auto_monitor_time[seg] = now
                            elseif now - auto_monitor_time[seg] >= 0.5
                                if zero_motion_worsened(auto_monitor_angle[seg],
                                    physical[seg]; threshold_deg=1.0)
                                    stop_auto!("CH$(2seg-1) 自动回直反而增大弯曲；已停止，请核对电机/IMU 方向", now)
                                    active_seg = 0
                                else
                                    auto_monitor_angle[seg] = physical[seg]
                                    auto_monitor_time[seg] = now
                                end
                            end
                            if active_seg != 0
                                pulse = homing_pulse(-measured[seg], now,
                                    auto_pulse_until[seg], auto_next_pulse[seg],
                                    auto_pulse_direction[seg], cfg)
                                auto_pulse_until[seg] = pulse.pulse_until
                                auto_next_pulse[seg] = pulse.next_pulse
                                auto_pulse_direction[seg] = pulse.direction
                                requested = pulse.command
                                controller = IMU.slew_command(auto_previous_command[seg],
                                    requested, now-last_bend_send[seg], cfg)
                                physical_command = physical_bend_command(controller,
                                    motor_signs[seg], cfg)
                                if physical_command != last_bends[seg] ||
                                   now-last_bend_send[seg] >= 0.1
                                    send_bend!(serial, cfg, physical_command)
                                    last_bend_send[seg] = now
                                end
                                auto_previous_command[seg] = controller
                                last_bends[seg] = physical_command
                            end
                        end
                        if auto_phase[] != :idle
                            ui.angles[] = @sprintf("自动回直：第一段 %.2f°，第二段 %.2f°；IMU X 与世界下方向夹角（仅显示） %.2f° / %.2f°",
                                physical[1], physical[2], tilt[1], tilt[2])
                            ui.predicted[] = predicted_joints(prepared,
                                physical, last_axes,
                                Calibration(imu_signs=Tuple(signs),
                                    motor_signs=Tuple(motor_signs),
                                    axial_signs=Tuple(axis_signs),
                                    bend_sides=Tuple(bend_sides));
                                measured_axial=pose.axial)
                        end
                    catch err
                        stop_auto!("自动回直已停止：$(sprint(showerror, err))", now)
                    end
                end
                ui.values[] = "CH1=$(last_bends[1])  CH2=$(last_axes[1])  CH3=$(last_bends[2])  CH4=$(last_axes[2])"
                for i in 1:4
                    ui.labels[i][] = "CH$i 实际发送：$((last_bends[1],last_axes[1],last_bends[2],last_axes[2])[i])"
                end
                sleep(0.005)
                continue
            end
            values = Int[sl.value[] for sl in ui.sliders]
            # Never jog both bending motors at once during polarity inference.
            if values[1] != 1500 && values[3] != 1500
                center_sliders!(ui); values .= 1500
                ui.status[] = "请一次只点动一段的弯曲电机"
            end
            if (values[1] != 1500 || values[3] != 1500) &&
               (values[2] != 1500 || values[4] != 1500 || now-last_axis_send < 0.5)
                GM.set_close_to!(ui.sliders[1], 1500)
                GM.set_close_to!(ui.sliders[3], 1500)
                values[1] = 1500; values[3] = 1500
                ui.status[] = "轴向舵机移动时不能点动弯曲电机；请等待轴向停止"
            end
            if !ok
                center_sliders!(ui); values .= 1500
                !ok && (ui.status[] = "任一 IMU 超时：CH1/CH3 停止，请检查 SCAN 和接线")
            end
            cmd = slider_commands(values, c)
            bends = [cmd.bend1, cmd.bend2]
            for seg in 1:2
                if bends[seg] != 1500 && (last_bends[seg] == 1500 ||
                    sign(bends[seg]-1500) != sign(last_bends[seg]-1500))
                    jog_start[seg] = now
                    if ok
                        measured = mounts[] === nothing ? (NaN, NaN) :
                            control_bends(imu_pose(s.filtered[c.first.imu_id],
                                s.filtered[c.second.imu_id], mounts[];
                            bend_sides=Tuple(bend_sides)), signs)
                        anchor[seg] = measured[seg]
                    end
                    active[seg] = sign(bends[seg]-1500)
                    candidates[seg] = 0; counts[seg] = 0
                end
                if bends[seg] != 1500 && now-jog_start[seg] >= 2.0
                    GM.set_close_to!(ui.sliders[2seg-1], 1500)
                    bends[seg] = 1500
                    ui.status[] = "CH$(2seg-1) 连续点动到 2 秒，已停止"
                end
                if bends[seg] != last_bends[seg] ||
                   (bends[seg] != 1500 && now-last_bend_send[seg] >= 0.1)
                    send_bend!(serial, seg == 1 ? c.first : c.second, bends[seg])
                    last_bends[seg] = bends[seg]
                    last_bend_send[seg] = now
                end
                if bends[seg] == 1500
                    active[seg] = 0; candidates[seg] = 0; counts[seg] = 0
                end
            end
            axes = [cmd.axial1, cmd.axial2]
            for seg in 1:2
                if ok && axes[seg] != last_axes[seg]
                    cfg = seg == 1 ? c.first : c.second
                    Bridge.write_command(serial, IMU.axial_servo_command(cfg, axes[seg]))
                    last_axes[seg] = axes[seg]
                    last_axis_send = now
                end
            end
            ui.values[] = "CH1=$(last_bends[1])  CH2=$(last_axes[1])  CH3=$(last_bends[2])  CH4=$(last_axes[2])"
            for i in 1:4
                ui.labels[i][] = "CH$i 实际发送：$((last_bends[1],last_axes[1],last_bends[2],last_axes[2])[i])"
            end
            if ok
                cal = Calibration(imu_signs=Tuple(signs), motor_signs=Tuple(motor_signs),
                                  axial_signs=Tuple(axis_signs),
                                  bend_sides=Tuple(bend_sides))
                pose = mounts[] === nothing ? nothing :
                    imu_pose(s.filtered[c.first.imu_id], s.filtered[c.second.imu_id], mounts[];
                             bend_sides=Tuple(bend_sides))
                measured = pose === nothing ? (NaN, NaN) : control_bends(pose, signs)
                preview_bends = pose === nothing ? (0.0, 0.0) : pose.bend
                fitted_joints, fitted_frames = predicted_geometry(prepared, preview_bends,
                    last_axes, cal; measured_axial=pose === nothing ? nothing : pose.axial)
                ui.predicted[] = fitted_joints
                if pose === nothing
                    for seg in 1:2
                        empty!(ui.actual_tangents[seg][])
                        empty!(ui.fitted_tangents[seg][])
                    end
                    ui.angles[] = "IMU在线；尚无有效安装姿态标定，当前仅显示直态参考"
                else
                    straight_error = absolute_straight_errors(s.filtered[c.first.imu_id],
                        s.filtered[c.second.imu_id])
                    tangent_errors = (calibrated_tangent_alignment_deg(fitted_frames[1][2], pose.frames[1]),
                                      calibrated_tangent_alignment_deg(fitted_frames[2][2], pose.frames[2]))
                    for seg in 1:2
                        origin = fitted_frames[seg][1]
                        ui.actual_tangents[seg][] = frame_axis_line(origin, pose.frames[seg], 3; length_mm=70.0)
                        ui.fitted_tangents[seg][] = frame_axis_line(origin, fitted_frames[seg][2], 3; length_mm=70.0)
                    end
                    ui.angles[] = @sprintf("IMU相对弯曲 %.2f° / %.2f°；IMU轴向 %.1f° / %.1f°；切向差 %.1f° / %.1f°；未拟合姿态残差 %.1f° / %.1f°；IMU X 与世界下方向夹角（仅显示） %.1f° / %.1f°",
                        pose.bend[1], pose.bend[2], pose.axial[1], pose.axial[2],
                        tangent_errors[1], tangent_errors[2], pose.residual[1], pose.residual[2],
                        straight_error[1], straight_error[2])
                end
                if !isempty(new_ids) && values[2] == 1500 && values[4] == 1500
                        for seg in 1:2
                            if active[seg] != 0 && (seg == 1 ? c.first.imu_id : c.second.imu_id) in new_ids
                                movement = measured[seg] - anchor[seg]
                                if 2.0 <= abs(movement) <= 30.0
                                    inferred = active[seg] * Int(sign(movement))
                                    counts[seg] = candidates[seg] == inferred ? counts[seg]+1 : 1
                                    candidates[seg] = inferred
                                    if counts[seg] >= 3
                                        if motor_signs[seg] != 0 && motor_signs[seg] != inferred
                                            conflicting[seg] = true
                                            GM.set_close_to!(ui.sliders[2seg-1], 1500)
                                            send_bend!(serial, seg == 1 ? c.first : c.second, 1500)
                                            last_bends[seg] = 1500
                                            ui.status[] = manual_motor_override[seg] ?
                                                "CH$(2seg-1) 观测与手动设置相反，点动已停止；保留手动符号，勿开始自动跟踪直到查明原因" :
                                                "CH$(2seg-1) 响应方向前后矛盾，已停止；请检查 IMU 和安装"
                                        else
                                            motor_signs[seg] = motor_sign_after_jog(
                                                motor_signs[seg], inferred,
                                                manual_motor_override[seg])
                                            ui.status[] = manual_motor_override[seg] ?
                                                "CH$(2seg-1) 观测方向与手动设置一致；手动符号已保留" :
                                                "CH$(2seg-1) 方向已观测为 $(inferred)；请核对实体与青色预测"
                                        end
                                        active[seg] = 0
                                    end
                                end
                            end
                        end
                end
            end
            if requests[:finish][]
                requests[:finish][] = false
                center_sliders!(ui); stop_bends!(serial, c); last_bends .= 1500
                defaults = (c.first.bend_motor_sign, c.second.bend_motor_sign)
                result = validate(Calibration(imu_signs=Tuple(signs),
                    motor_signs=resolved_motor_signs(motor_signs, defaults),
                    axial_signs=Tuple(axis_signs),
                    bend_sides=Tuple(bend_sides)))
                save_calibration(save_path, result)
                @info "双段方向初始化已保存" path=save_path imu_signs=result.imu_signs motor_signs=result.motor_signs axial_signs=result.axial_signs
                ui.status[] = mounts[] === nothing ?
                    "方向已保存；仍需由你确认直态并记录安装基准后才能跟踪" :
                    "物理弯曲侧、方向与双 IMU 安装坐标已保存，初始化完成"
                break
            end
            sleep(0.005)
        end
    finally
        try stop_bends!(serial, c) catch end
        LibSerialPort.close(serial)
    end
    result === nothing && error("初始化未完成；两段自动控制没有启动")
    for screen in copy(ui.fig.scene.current_screens)
        try close(screen) catch end
    end
    result
end

function make_live_view(plans, c::Config; send_servo=true,
                        calibration::Calibration, mounts)
    GM.activate!()
    fig = GM.Figure(size=(1650, 1400))
    ax = GM.Axis3(fig[1, 1]; title="双 IMU：两段 PCC 实时跟踪", aspect=:data,
                  xlabel="X / mm", ylabel="Y / mm", zlabel="Z / mm")
    state = plans[:rectangle].data.two_segment_state
    joints0 = copy(state.joints_history[1])
    split = Main.N_LINKS + 2
    planned1 = GM.Observable(copy(joints0[1:split]))
    planned2 = GM.Observable(copy(joints0[split:end]))
    predicted1 = GM.Observable(copy(joints0[1:split]))
    predicted2 = GM.Observable(copy(joints0[split:end]))
    planned_joints = GM.Observable(copy(joints0))
    predicted_joints_obs = GM.Observable(copy(joints0))
    GM.lines!(ax, planned1; color=:dodgerblue, linewidth=5, label="计划第一段")
    GM.lines!(ax, planned2; color=:orange, linewidth=5, label="计划第二段")
    GM.lines!(ax, predicted1; color=:dodgerblue, linewidth=3, linestyle=:dash,
              label="IMU 实际第一段")
    GM.lines!(ax, predicted2; color=:orange, linewidth=3, linestyle=:dash,
              label="IMU 实际第二段")
    GM.scatter!(ax, planned_joints; color=:black, markersize=6, label="关节点")
    GM.scatter!(ax, predicted_joints_obs; color=:black, markersize=4)
    axis_colors = (:red, :green, :blue) # local X/Y/Z
    reference_axes = [GM.Observable(GM.Point3f[]) for _ in 1:6]
    imu_axes = [GM.Observable(GM.Point3f[]) for _ in 1:6]
    fitted_axes = [GM.Observable(GM.Point3f[]) for _ in 1:6]
    for seg in 1:2, axis_id in 1:3
        idx = (seg - 1) * 3 + axis_id
        GM.lines!(ax, reference_axes[idx]; color=axis_colors[axis_id],
            linestyle=:dash, linewidth=3,
            label=(seg == 1 && axis_id == 1) ? "计划姿态轴（虚线）" : nothing)
        GM.lines!(ax, imu_axes[idx]; color=axis_colors[axis_id],
            linestyle=:solid, linewidth=3,
            label=(seg == 1 && axis_id == 1) ? "安装校正 IMU 姿态轴（实线）" : nothing)
        GM.lines!(ax, fitted_axes[idx]; color=axis_colors[axis_id],
            linestyle=:dot, linewidth=4,
            label=(seg == 1 && axis_id == 1) ? "IMU 反算 PCC 姿态轴（点线）" : nothing)
    end
    for seg in 1:2, axis_id in 1:3
        idx = (seg - 1) * 3 + axis_id
        origin, rotation = state.terminal_frames_history[1][seg]
        reference_axes[idx][] = frame_axis_line(origin, rotation, axis_id)
    end
    expected_path = GM.Observable(vcat(
        state.trajectory[1:state.shape_start_index-1], state.target_trajectory))
    GM.lines!(ax, expected_path; color=:magenta, linewidth=3,
              label="末端期望轨迹")
    actual_path = GM.Observable(GM.Point3f[])
    GM.lines!(ax, actual_path; color=:cyan, linewidth=3, label="实际末端轨迹（IMU 估计）")
    all_points = [p for joints in state.joints_history for p in joints]
    xyz = reduce(hcat, [Float64.(p) for p in all_points])
    lo = minimum(xyz; dims=2)[:]; hi = maximum(xyz; dims=2)[:]
    GM.limits!(ax, (-300, 300), (lo[2]-75, hi[2]+75), (lo[3]-50, hi[3]+50))
    GM.axislegend(ax)
    panel = GM.GridLayout(); fig[1, 2] = panel
    status = GM.Observable(send_servo ? "等待两只 IMU；请先点‘归零回直’" :
                             "观察模式：不会发送电机指令")
    values = GM.Observable("CH1=1500  CH2=1500  CH3=1500  CH4=1500")
    angles = GM.Observable("等待 IMU1 与 IMU2")
    GM.Label(panel[1, 1], status; tellwidth=false)
    zero = GM.Button(panel[2, 1]; label="第一步：两段归零回直", tellwidth=false)
    rectangle = GM.Button(panel[3, 1]; label="跟踪矩形", tellwidth=false)
    circle = GM.Button(panel[4, 1]; label="跟踪圆形", tellwidth=false)
    infinity = GM.Button(panel[5, 1]; label="跟踪 ∞ 形", tellwidth=false)
    s_curve = GM.Button(panel[6, 1]; label="跟踪 S 形", tellwidth=false)
    stop = GM.Button(panel[7, 1]; label="停止 CH1/CH3", tellwidth=false)
    GM.Label(panel[8, 1], values; tellwidth=false)
    GM.Label(panel[9, 1], angles; tellwidth=false)
    GM.Label(panel[10, 1], "第一段跟踪 Kp=$(c.first.kp)，速度余量上限=$(c.first.max_speed)；第二段跟踪 Kp=$(c.second.kp)，速度余量上限=$(c.second.max_speed)。\n先用 15 s 平滑过渡到轨迹起点，再跟踪 25 s；完成后依次归零第一段、第二段。结果只记录形状轨迹的 X–Z 投影。";
             tellwidth=false)
    GM.Label(panel[11, 1], "坐标轴：红=X、绿=Y、蓝=Z；虚线为计划姿态，实线为安装标定后的 IMU 姿态，点线为用 IMU 弯曲/轴向角正运动学反算的 PCC 姿态。第二段使用两只 IMU 的相对旋转消除第一段共同转动。IMU 不测位置，姿态轴原点放在 PCC 估计末端；直态零位来自初始化时操作者确认的安装姿态。";
             tellwidth=false)
    positive_ch3 = physical_bend_command(
        c.second.positive_start + 1 + c.second.min_speed_positive,
        calibration.motor_signs[2], c.second)
    GM.Label(panel[12, 1], "已加载物理弯曲侧=$(calibration.bend_sides)，控制角符号=$(calibration.imu_signs)，弯曲电机=$(calibration.motor_signs)；CH3 正控制误差的起转指令=$positive_ch3";
             tellwidth=false)
    plot_specs = (("第一段弯曲电机 CH1", "舵机指令", (2, 1)),
                  ("第一段轴向电机 CH2", "舵机指令", (2, 2)),
                  ("第一段弯曲角", "角度 / °", (3, 1)),
                  ("第二段弯曲电机 CH3", "舵机指令", (3, 2)),
                  ("第二段轴向电机 CH4", "舵机指令", (4, 1)),
                  ("第二段弯曲角", "角度 / °", (4, 2)))
    traces = [GM.Observable(GM.Point2f[]) for _ in 1:8]
    trace_axes = Any[]
    for (i, (title, ylabel, (row, col))) in enumerate(plot_specs)
        trace_ax = GM.Axis(fig[row, col]; title, xlabel="运行时间 / s", ylabel)
        push!(trace_axes, trace_ax)
        if i == 3 || i == 6
            first_trace = i == 3 ? 3 : 7
            second_trace = first_trace + 1
            GM.lines!(trace_ax, traces[first_trace]; color=:magenta, linewidth=2, label="目标")
            GM.lines!(trace_ax, traces[second_trace]; color=:dodgerblue, linewidth=2, label="IMU 实测")
            GM.axislegend(trace_ax; position=:lt)
        else
            trace_index = i < 3 ? i : i + 1
            color = i == 1 || i == 4 ? :orange : :green
            GM.lines!(trace_ax, traces[trace_index]; color, linewidth=2)
        end
    end
    GM.colsize!(fig.layout, 1, GM.Relative(0.68))
    display(fig)
    (;fig, ax, planned1, planned2, predicted1, predicted2, planned_joints,
      predicted_joints_obs, expected_path, actual_path, status, values, angles, zero,
      reference_axes, imu_axes, fitted_axes,
      rectangle, circle, infinity, s_curve,
      stop, trace_axes, traces)
end

function reference_at(prepared, t)
    state = prepared.data.two_segment_state
    times = prepared.times
    bend = ntuple(seg -> rad2deg(PCC.interpolate(
        view(state.rope_drive_history, seg, :), times, t) / Main.R_DRIVE), 2)
    axial = ntuple(seg -> PCC.interpolate(
        view(state.axial_rotation_history, seg, :), times, t), 2)
    k = clamp(searchsortedlast(times, t), 1, length(times))
    (; bend, axial, k)
end

function target_point_at(prepared, t)
    state = prepared.data.two_segment_state
    times = prepared.times
    targets = vcat(state.trajectory[1:state.shape_start_index-1],
                   state.target_trajectory)
    t <= times[1] && return targets[1]
    t >= times[end] && return targets[end]
    i = searchsortedlast(times, t)
    u = (t - times[i]) / (times[i + 1] - times[i])
    (1-u) * targets[i] + u * targets[i+1]
end

function axis_target(angle, sign, c::IMU.Config)
    clamp(round(Int, c.axial_initial_signal + sign * angle * 2000 / 300),
          c.command_min, c.command_max)
end

function physical_bend_command(controller::Integer, motor_sign::Integer, c::IMU.Config)
    motor_sign in (-1, 1) || error("弯曲电机方向必须为 ±1")
    controller == c.neutral && return c.neutral
    if motor_sign == 1
        return clamp(controller, c.command_min, c.command_max)
    end
    # The measured dead band is asymmetric. Reflect the *effective speed*
    # beyond 1400/1600, not the raw pulse around 1500: 1616 -> 1379.
    if controller > c.positive_start
        speed = clamp(controller-c.positive_start-1,
                      c.min_speed_negative, c.max_speed)
        return c.negative_start-1-speed
    elseif controller < c.negative_start
        speed = clamp(c.negative_start-1-controller,
                      c.min_speed_positive, c.max_speed)
        return c.positive_start+1+speed
    end
    c.neutral
end

function validate_reference(prepared)
    state = prepared.data.two_segment_state
    for seg in 1:2
        angles = rad2deg.(state.rope_drive_history[seg, :] ./ Main.R_DRIVE)
        all(isfinite, angles) && minimum(angles) >= -1e-8 &&
            maximum(angles) <= Main.MAX_BEND_DEG + 1e-8 ||
            error("第 $seg 段离线弯曲参考超出 0～$(Main.MAX_BEND_DEG)° 的单侧范围")
        axial = state.axial_rotation_history[seg, :]
        all(isfinite, axial) && maximum(abs, axial) <= 150.0 ||
            error("第 $seg 段离线轴向参考超出 300° 位置舵机的中位两侧 ±150° 行程")
    end
    prepared
end

shape_label(shape::Symbol) = get(Dict(:rectangle=>"矩形", :circle=>"圆形",
    :infinity=>"∞形", :s_curve=>"S形"), shape, string(shape))

function validate_plan_calibration(plans, cal::Calibration)
    required_shapes = (:rectangle, :circle, :infinity, :s_curve)
    all(shape -> haskey(plans, shape), required_shapes) ||
        error("需要矩形、圆形、∞形和 S 形四种仿真参考")
    all(shape -> plans[shape].data.two_segment_state.bend_sides == cal.bend_sides,
        required_shapes) ||
        error("离线计划的物理弯曲侧与保存标定不一致；必须重新计算四种轨迹")
    plans
end

"""Run selected 80 mm paths. Each completed run's X–Z projection is retained
in session memory, then the arm automatically homes section 1 followed by section 2.
"""
function run_live(plans::AbstractDict; config=Config(), calibration::Calibration,
                  send_servo=true, open_port=LibSerialPort.open,
                  mount_path=IMU_MOUNT_PATH,
                  calibration_path=CALIBRATION_PATH,
                  first_tracking_kp=nothing, second_tracking_kp=nothing)
    required_shapes = (:rectangle, :circle, :infinity, :s_curve)
    all(shape -> haskey(plans, shape), required_shapes) ||
        error("需要矩形、圆形、∞形和 S 形四种仿真参考")
    foreach(validate_reference, (plans[s] for s in required_shapes))
    prepared = plans[:rectangle]
    c = (first_tracking_kp === nothing && second_tracking_kp === nothing) ? config :
        with_tracking_kp(config;
            first=first_tracking_kp === nothing ? config.first.kp : first_tracking_kp,
            second=second_tracking_kp === nothing ? config.second.kp : second_tracking_kp)
    validate(c, prepared.data.ref.adapter)
    cal = validate(calibration)
    saved_cal = load_calibration(calibration_path)
    (cal.imu_signs, cal.motor_signs, cal.axial_signs, cal.bend_sides) ==
        (saved_cal.imu_signs, saved_cal.motor_signs, saved_cal.axial_signs,
         saved_cal.bend_sides) ||
        error("传入的方向参数与已保存初始化不同；重新读取标定后再启动，避免沿用旧 Julia 变量")
    validate_plan_calibration(plans, cal)
    mounts = validated_straight_mounts(load_imu_mounts(mount_path))
    ui = make_live_view(plans, c; send_servo, calibration=cal, mounts)
    @info "双段跟踪实际加载方向" imu_signs=cal.imu_signs motor_signs=cal.motor_signs axial_signs=cal.axial_signs ch3_positive_error_command=physical_bend_command(c.second.positive_start + 1 + c.second.min_speed_positive, cal.motor_signs[2], c.second)
    requests = Dict(k => Ref(false) for k in
        (:zero, :rectangle, :circle, :infinity, :s_curve, :stop))
    for key in keys(requests)
        GM.on(_ -> (requests[key][] = true), getproperty(ui, key).clicks)
    end
    s = SensorState(); serial = open_port(c.first.port, c.first.baud)
    phase = :idle; phase_start = 0.0; track_start = 0.0
    last_bends = [1500, 1500]; last_controller = [1500, 1500]
    pulse_until = [0.0, 0.0]; next_pulse = [0.0, 0.0]
    pulse_direction = [0, 0]
    last_axes = [1500, 1500]; last_send = [-Inf, -Inf]; stable = 0
    last_bend_pair = nothing; previous_wrapped_bends = nothing
    continuous_bends = [0.0, 0.0]
    latest_pose = Ref{Any}(nothing)
    initial_branch_known = Ref(false)
    zero_monitor_start = fill(NaN, 2)
    zero_monitor_time = fill(NaN, 2)
    last_stable_seq = [typemax(UInt32), typemax(UInt32)]
    last_axis_update = [time_ns()/1e9, time_ns()/1e9]
    history = NamedTuple[]; commands = NamedTuple[]; saved_runs = NamedTuple[]
    active_plan = prepared; active_shape = :rectangle
    run_rows = NamedTuple[]
    now_s() = time_ns() / 1e9
    function measured_bends!()
        pose = imu_pose(s.filtered[c.first.imu_id], s.filtered[c.second.imu_id], mounts;
                        bend_sides=cal.bend_sides)
        latest_pose[] = pose
        wrapped = control_bends(pose, cal.imu_signs)
        pair = (s.latest[c.first.imu_id].seq, s.latest[c.second.imu_id].seq)
        if pair != last_bend_pair
            if previous_wrapped_bends === nothing
                continuous_bends .= wrapped
                # A single pose beyond 90 degrees cannot distinguish +200 from -160.
                # Once started near straight, keep the same unwrapped branch across
                # tracking and the automatic return to straight.
                initial_branch_known[] = initial_branch_valid(wrapped)
            else
                for seg in 1:2
                    continuous_bends[seg] += unwrap_step_deg(
                        wrapped[seg], previous_wrapped_bends[seg])
                end
            end
            previous_wrapped_bends = wrapped
            last_bend_pair = pair
        end
        Tuple(continuous_bends)
    end
    session_start = now_s()
    function send_both_stop!(reason, now)
        if send_servo
            stop_bends!(serial, c)
            for (seg, cfg) in enumerate((c.first, c.second))
                push!(commands, (time=now, phase=reason, channel=cfg.servo_id, command=1500))
            end
        end
        last_bends .= 1500; last_controller .= 1500
        last_send .= now
    end
    try
        Bridge.write_command(serial, "PING\n"); Bridge.write_command(serial, "SCAN\n")
        while GM.isopen(ui.fig.scene)
            now = now_s()
            new_ids, messages = read_sensors!(serial, s, c, now)
            for msg in messages
                (startswith(msg, "ERR") || startswith(msg, "WATCHDOG")) &&
                    (ui.status[] = "ESP32: $msg")
            end
            ok = fresh(s, c, now) && paired(s, c)
            if requests[:stop][]
                requests[:stop][] = false; requests[:zero][] = false
                for shape in required_shapes; requests[shape][] = false; end
                phase = :idle; stable = 0
                send_both_stop!(:manual_stop, now)
                ui.status[] = "CH1/CH3 已发送 1500 停止；需重新归零才能跟踪"
            end
            if phase != :idle && !ok
                phase = :idle; stable = 0
                send_both_stop!(:imu_timeout, now)
                ui.status[] = "任一 IMU 有效数据超时：CH1/CH3 均已停止"
            end
            if requests[:zero][]
                requests[:zero][] = false
                for shape in required_shapes; requests[shape][] = false; end
                ok && measured_bends!()
                if ok && initial_branch_known[]
                    for trace in ui.traces
                        trace[] = GM.Point2f[]
                    end
                    ui.actual_path[] = GM.Point3f[]
                    send_both_stop!(:axis_homing, now)
                    pulse_until .= now
                    next_pulse .= now
                    pulse_direction .= 0
                    if send_servo
                        # Move the position servos to center before measuring/zeroing bend.
                        for cfg in (c.first, c.second)
                            Bridge.write_command(serial, "S,$(cfg.axial_servo_id),1500,1200\n")
                            push!(commands, (time=now, phase=:axis_homing,
                                             channel=cfg.axial_servo_id, command=1500))
                        end
                    end
                    last_axes .= 1500
                    last_axis_update .= now
                    phase = :axis_homing; phase_start = now; stable = 0
                    ui.status[] = "CH2/CH4 回中位中，CH1/CH3 保持停止；随后先归零第一段"
                else
                    ui.status[] = ok ?
                        "启动姿态角分支不确定；请在初始化窗口手动回直并记录双段直态" :
                        "两只 IMU 都有新鲜数据后才可开始归零"
                end
            end
            if phase == :axis_homing && now-phase_start >= 1.5
                phase = :zero_first; stable = 0
                pulse_until .= now
                next_pulse .= now
                pulse_direction .= 0
                last_stable_seq .= [s.latest[c.first.imu_id].seq,
                                    s.latest[c.second.imu_id].seq]
                ui.status[] = "第一段归零：仅 CH1 闭环，CH3 保持 1500"
            end
            requested_shape = findfirst(shape -> requests[shape][], required_shapes)
            if requested_shape !== nothing
                shape = required_shapes[requested_shape]
                for candidate in required_shapes; requests[candidate][] = false; end
                if phase == :ready && ok
                    measured = measured_bends!()
                    if all(seg -> abs(measured[seg]) <=
                           (seg == 1 ? c.first : c.second).zero_tolerance_deg, 1:2)
                        active_shape = shape
                        active_plan = plans[shape]
                        state = active_plan.data.two_segment_state
                        ui.expected_path[] = vcat(
                            state.trajectory[1:state.shape_start_index-1],
                            state.target_trajectory)
                        phase = :tracking; track_start = now
                        empty!(run_rows)
                        for trace in ui.traces; trace[] = GM.Point2f[]; end
                        ui.actual_path[] = GM.Point3f[]
                        ui.status[] = "$(shape_label(shape))已启动：先平滑过渡至轨迹起点，再跟踪完整轨迹"
                    else
                        phase = :idle
                        send_both_stop!(:drift, now)
                        ui.status[] = "某段偏离直态容差；已停止，请重新归零"
                    end
                else
                    ui.status[] = "两段都归零且 IMU 新鲜后才能启动跟踪"
                end
            end
            if ok && !isempty(new_ids)
                measured = measured_bends!()
                t = phase == :tracking ? clamp(now-track_start, 0.0, active_plan.times[end]) : 0.0
                reference = reference_at(active_plan, t)
                active_zero = 0
                if phase in (:zero_first, :zero_second)
                    both_advanced = all(seg -> s.latest[seg == 1 ? c.first.imu_id : c.second.imu_id].seq !=
                        last_stable_seq[seg], 1:2)
                    first_drift = phase == :zero_second &&
                        abs(measured[1]) > c.first.zero_tolerance_deg
                    if both_advanced || first_drift
                        transition = zero_phase_step(phase, measured,
                            (c.first.zero_tolerance_deg, c.second.zero_tolerance_deg),
                            stable, (c.first.zero_hold_samples, c.second.zero_hold_samples))
                        if both_advanced
                            last_stable_seq .= [s.latest[c.first.imu_id].seq,
                                                s.latest[c.second.imu_id].seq]
                        end
                        stable = transition.stable
                        if transition.stop
                            send_both_stop!(transition.phase, now)
                            phase = transition.phase
                            ui.status[] = phase == :zero_second ?
                                "第一段已稳定回直，CH1 停止；现在仅 CH3 回直第二段" :
                                phase == :zero_first ?
                                "第一段偏离直态，CH1/CH3 已停止；重新先归零第一段" :
                                "两段已依次回直，CH1/CH3 停止；可点击开始跟踪"
                        else
                            phase = transition.phase
                            active_zero = transition.active
                        end
                    else
                        active_zero = phase == :zero_first ? 1 : 2
                    end
                end
                active_segments = active_motor_segments(phase, active_zero)
                if phase in (:zero_first, :zero_second)
                    wrong_way = false
                    for seg in 1:2
                        if !(seg in active_segments)
                            zero_monitor_start[seg] = NaN
                            zero_monitor_time[seg] = NaN
                            continue
                        end
                        imu_id = seg == 1 ? c.first.imu_id : c.second.imu_id
                        if last_bends[seg] == 1500 || !isfinite(zero_monitor_start[seg])
                            zero_monitor_start[seg] = abs(measured[seg])
                            zero_monitor_time[seg] = now
                        elseif imu_id in new_ids && now-zero_monitor_time[seg] >= 0.5
                            if zero_motion_worsened(zero_monitor_start[seg], abs(measured[seg]))
                                wrong_way = true
                                break
                            end
                            zero_monitor_start[seg] = abs(measured[seg])
                            zero_monitor_time[seg] = now
                        end
                    end
                    if wrong_way
                        send_both_stop!(:zero_wrong_way, now)
                        phase = :idle
                        stable = 0
                        active_segments = ()
                        ui.status[] = "自动回直时弯曲角连续增大；CH1/CH3 已停止。请用初始化窗口手动回直并检查方向/角度分支"
                    end
                else
                    zero_monitor_start .= NaN
                    zero_monitor_time .= NaN
                end
                if !isempty(active_segments)
                    # The offline reference is a physical/model bend. Convert
                    # it to the same controller coordinate as the IMU reading.
                    desired = phase == :tracking ?
                        signed_bend_angles(reference.bend, cal.imu_signs) : (0.0, 0.0)
                    for seg in active_segments
                        cfg = seg == 1 ? c.first : c.second
                        error_deg = desired[seg] - measured[seg]
                        kp = phase == :tracking ? cfg.kp : cfg.zero_kp
                        max_speed = phase == :tracking ? cfg.max_speed : cfg.zero_max_speed
                        requested = if phase == :tracking
                            IMU.servo_speed_command(error_deg, 0.0, 0.0, cfg;
                                kp, ki=0.0, kd=0.0, max_speed)
                        else
                            pulse = homing_pulse(error_deg, now, pulse_until[seg],
                                next_pulse[seg], pulse_direction[seg], cfg)
                            pulse_until[seg] = pulse.pulse_until
                            next_pulse[seg] = pulse.next_pulse
                            pulse_direction[seg] = pulse.direction
                            pulse.command
                        end
                        controller = IMU.slew_command(last_controller[seg], requested,
                            now-last_send[seg], cfg)
                        physical = physical_bend_command(controller,
                            cal.motor_signs[seg], cfg)
                        if send_servo && (physical != last_bends[seg] || now-last_send[seg] >= 0.1)
                            send_bend!(serial, cfg, physical)
                            push!(commands, (time=now, phase, channel=cfg.servo_id,
                                             command=physical))
                            last_send[seg] = now
                        end
                        last_controller[seg] = controller
                        last_bends[seg] = physical
                    end
                    if phase == :tracking
                        for seg in 1:2
                            cfg = seg == 1 ? c.first : c.second
                            target = axis_target(reference.axial[seg], cal.axial_signs[seg], cfg)
                            new_axis = IMU.slew_axial_signal(last_axes[seg], target,
                                now-last_axis_update[seg], cfg)
                            if send_servo && new_axis != last_axes[seg]
                                Bridge.write_command(serial, IMU.axial_servo_command(cfg, new_axis))
                                push!(commands, (time=now, phase, channel=cfg.axial_servo_id,
                                                 command=new_axis))
                            end
                            last_axes[seg] = new_axis
                            last_axis_update[seg] = now
                        end
                    end
                elseif phase == :ready
                    if any(seg -> abs(measured[seg]) >
                           (seg == 1 ? c.first : c.second).zero_tolerance_deg, 1:2)
                        send_both_stop!(:drift, now)
                        phase = :idle
                        ui.status[] = "等待期间有臂段偏离直态；请重新归零"
                    end
                end
                planned_points = copy(active_plan.data.two_segment_state.joints_history[reference.k])
                pose = latest_pose[]
                # Undo the scalar controller convention before reconstructing
                # the world-space arm from its calibrated IMU orientations.
                physical_bends = signed_bend_angles(measured, cal.imu_signs)
                predicted_points, predicted_frames = Main.calculate_multi_segment_kinematics(
                    Main.R_DRIVE .* deg2rad.(collect(physical_bends)),
                    collect(pose.axial),
                    active_plan.data.two_segment_state.base_z_offset;
                    bend_sides=cal.bend_sides)
                split = Main.N_LINKS + 2
                ui.planned1[] = planned_points[1:split]
                ui.planned2[] = planned_points[split:end]
                ui.predicted1[] = predicted_points[1:split]
                ui.predicted2[] = predicted_points[split:end]
                ui.planned_joints[] = planned_points
                ui.predicted_joints_obs[] = predicted_points
                reference_frames = active_plan.data.two_segment_state.terminal_frames_history[reference.k]
                for seg in 1:2, axis_id in 1:3
                    idx = (seg - 1) * 3 + axis_id
                    ref_origin, ref_rotation = reference_frames[seg]
                    ui.reference_axes[idx][] = frame_axis_line(ref_origin, ref_rotation, axis_id)
                    ui.fitted_axes[idx][] = frame_axis_line(predicted_frames[seg][1],
                        predicted_frames[seg][2], axis_id)
                    ui.imu_axes[idx][] = frame_axis_line(predicted_frames[seg][1],
                        s.filtered[seg == 1 ? c.first.imu_id : c.second.imu_id], axis_id)
                end
                if phase == :tracking
                    path_points = copy(ui.actual_path[])
                    push!(path_points, predicted_points[end])
                    ui.actual_path[] = path_points
                    target_point = target_point_at(active_plan, t)
                    if t >= active_plan.data.shape_start_time
                        push!(run_rows, (time_s=t-active_plan.data.shape_start_time,
                            target_x_mm=Float64(target_point[1]),
                            target_z_mm=Float64(target_point[3]),
                            measured_x_mm=Float64(predicted_points[end][1]),
                            measured_z_mm=Float64(predicted_points[end][3])))
                    end
                end
                tangent_errors = ntuple(seg -> tangent_alignment_deg(
                    predicted_frames[seg][2],
                    s.filtered[seg == 1 ? c.first.imu_id : c.second.imu_id]), 2)
                calibrated_errors = ntuple(seg -> calibrated_tangent_alignment_deg(
                    predicted_frames[seg][2], pose.frames[seg]), 2)
                full_pose_errors = ntuple(seg -> full_orientation_error_deg(
                    predicted_frames[seg][2], pose.frames[seg]), 2)
                ui.angles[] = @sprintf("第一段弯曲目标 %.2f° / IMU %.2f°；第二段目标 %.2f° / 相对 IMU %.2f°\n拟合切向与原始 IMU X 偏差 %.1f° / %.1f°；与校正后切向偏差 %.1f° / %.1f°\nIMU反算 PCC 与校正后 IMU 全姿态差 %.2f° / %.2f°",
                    reference.bend[1], physical_bends[1], reference.bend[2], physical_bends[2],
                    tangent_errors..., calibrated_errors..., full_pose_errors...)
                ui.values[] = "CH1=$(last_bends[1])  CH2=$(last_axes[1])  CH3=$(last_bends[2])  CH4=$(last_axes[2])"
                if phase in (:zero_first, :zero_second, :tracking)
                    plot_time = now - session_start
                    plot_values = (last_bends[1], last_axes[1],
                        phase == :tracking ? reference.bend[1] : 0.0, physical_bends[1],
                        last_bends[2], last_axes[2],
                        phase == :tracking ? reference.bend[2] : 0.0, physical_bends[2])
                    for i in eachindex(plot_values)
                        points = copy(ui.traces[i][])
                        push!(points, GM.Point2f(plot_time, plot_values[i]))
                        ui.traces[i][] = points
                    end
                    foreach(GM.autolimits!, ui.trace_axes)
                end
                push!(history, (host_time=now, phase, time_s=t,
                    first_imu_seq=s.latest[c.first.imu_id].seq,
                    second_imu_seq=s.latest[c.second.imu_id].seq,
                    target_bend_deg=reference.bend, measured_bend_deg=physical_bends,
                    bend_error_deg=ntuple(i ->
                        (phase == :tracking ? reference.bend[i] : 0.0) - physical_bends[i], 2),
                    control_target_bend_deg=phase == :tracking ?
                        signed_bend_angles(reference.bend, cal.imu_signs) : (0.0, 0.0),
                    control_measured_bend_deg=measured,
                    controller_bend_commands=Tuple(last_controller),
                    measured_axial_deg=pose.axial,
                    tangent_error_deg=tangent_errors,
                    calibrated_tangent_error_deg=calibrated_errors,
                    target_axial_deg=reference.axial,
                    bend_commands=Tuple(last_bends), axial_commands=Tuple(last_axes)))
                if phase == :tracking && t >= active_plan.times[end]
                    isempty(run_rows) && error("没有可保留的跟踪采样点")
                    push!(saved_runs, (shape=active_shape, rows=copy(run_rows),
                                       samples=length(run_rows)))
                    send_both_stop!(:completed, now)
                    if send_servo
                        for cfg in (c.first, c.second)
                            Bridge.write_command(serial, "S,$(cfg.axial_servo_id),1500,1200\n")
                            push!(commands, (time=now, phase=:auto_axis_homing,
                                channel=cfg.axial_servo_id, command=1500))
                        end
                    end
                    last_axes .= 1500
                    last_axis_update .= now
                    phase = :axis_homing; phase_start = now; stable = 0
                    ui.status[] = "$(shape_label(active_shape))数据已保留在 Julia 会话；轴向回中后依次归零两段"
                end
            end
            sleep(0.005)
        end
    finally
        if send_servo
            try stop_bends!(serial, c) catch end
        end
        LibSerialPort.close(serial)
    end
    (; history, commands, saved_runs, calibration=cal,
       two_segment_state=prepared.data.two_segment_state,
       hardware_reference=(bend_deg=ntuple(seg -> rad2deg.(
           prepared.data.two_segment_state.rope_drive_history[seg, :] ./ Main.R_DRIVE), 2),
           axial_deg=prepared.data.two_segment_state.axial_rotation_history),
       second_segment_hardware_output=(bend_channel=3, axial_channel=4,
           bend_target_deg=rad2deg.(prepared.data.two_segment_state.rope_drive_history[2, :] ./ Main.R_DRIVE),
           axial_target_deg=copy(prepared.data.two_segment_state.axial_rotation_history[2, :]),
           commands=[entry for entry in commands if entry.channel in (3, 4)]))
end

end # module DualSegmentHardwareControl

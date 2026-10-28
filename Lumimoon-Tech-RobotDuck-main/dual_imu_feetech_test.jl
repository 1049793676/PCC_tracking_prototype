# ESP32-S3 双 IMU + FEETECH HD-1910 + LSC-16 轴向舵机的函数文件。
# 先烧录子目录 esp32_dual_imu_feetech_bridge 中的同名 .ino。
# 逐行运行示例见同目录 dual_imu_feetech_examples.jl；本文件不自动打开串口或发送指令。
# 依赖：LibSerialPort.jl（沿用 portable_two_segment_pcc 项目环境）。

using LibSerialPort
using LinearAlgebra: cross, dot, norm, I
import GLMakie as GM
import TOML
include(joinpath(@__DIR__, "pcc_reference_profiles.jl"))
include(joinpath(@__DIR__, "pcc_fixed_axial_fit.jl"))

# Windows 上列出 COM 设备；当前按用户确认的第二个 COM 连接 ESP32 的 USB CDC。
function list_com_ports()
    Sys.iswindows() || error("当前 COM 口枚举函数仅支持 Windows")
    ports = String[]
    for n in 1:256
        name = "COM$(n)"
        buffer = zeros(UInt16, 4096)
        count = ccall((:QueryDosDeviceW, "kernel32"), UInt32,
            (Cwstring, Ptr{UInt16}, UInt32), name, buffer, length(buffer))
        if count > 0
            push!(ports, name)
            terminator = findfirst(iszero, buffer)
            target = transcode(String, buffer[1:terminator-1])
            println(name, " => ", target)
        end
    end
    isempty(ports) && println("未检测到 COM 口")
    ports
end

const ESP32_USB_BAUD = 115200
const ESP32_PORT_OVERRIDE = nothing # 若第二个 COM 不是 ESP32，改成实际端口，例如 "COM8"
const SERVO_ID = 1                 # 新舵机当前按 ID 1 测试；按需改这里
const TICKS_PER_TURN = 4096        # HD-1910 编码器刻度/圈
const MAX_TEST_DELTA_TICKS = 200   # 相对点动安全上限；确认无干涉后再调大
const RX_BUFFER = UInt8[]
const PENDING_LINES = String[]
const ACTIVE_PORT = Ref("")
const USB_WRITE_LOCK = ReentrantLock()

function open_controller()
    ports = list_com_ports()
    port = if ESP32_PORT_OVERRIDE === nothing
        isempty(ports) && error("没有检测到 COM 口；未打开 ESP32")
        ports[2]
    else
        String(ESP32_PORT_OVERRIDE)
    end
    ACTIVE_PORT[] = port
    println("打开 ESP32 候选串口：$(port) @ $(ESP32_USB_BAUD) baud")
    sp = LibSerialPort.open(port, ESP32_USB_BAUD)
    LibSerialPort.set_flow_control(sp; dtr=LibSerialPort.SP_DTR_ON)
    LibSerialPort.set_write_timeout(sp, 1.0)
    # Opening an ESP32 USB CDC port can reset the board; let setup and its
    # first IMU frames finish before clearing startup text and sending commands.
    sleep(1.0)
    drain_serial!(sp)
    sp
end

function drain_serial!(sp)
    LibSerialPort.nonblocking_read(sp)
    empty!(RX_BUFFER)
    empty!(PENDING_LINES)
    nothing
end

function poll_lines!(sp)
    incoming = LibSerialPort.nonblocking_read(sp)
    isempty(incoming) || append!(RX_BUFFER, incoming)
    while true
        newline = findfirst(byte -> byte == UInt8(0x0a), RX_BUFFER)
        newline === nothing && break
        line = strip(String(copy(RX_BUFFER[1:newline-1])))
        deleteat!(RX_BUFFER, 1:newline)
        isempty(line) || push!(PENDING_LINES, line)
    end
    nothing
end

function wait_line!(sp, predicate; timeout=1.0)
    deadline = time() + timeout
    while time() < deadline
        poll_lines!(sp)
        for i in eachindex(PENDING_LINES)
            if predicate(PENDING_LINES[i])
                return splice!(PENDING_LINES, i)
            end
        end
        sleep(0.005)
    end
    recent = isempty(PENDING_LINES) ? "（没有收到任何完整行）" :
        join(PENDING_LINES[max(1, length(PENDING_LINES)-4):end], " | ")
    error("等待 ESP32 回复超时；当前串口=$(ACTIVE_PORT[])；收到的最近数据：$(recent)。确认 Julia 选中的端口就是 ESP32 的 USB CDC，并关闭 Arduino 串口监视器后重试。")
end

function send_line!(sp, command::AbstractString)
    lock(USB_WRITE_LOCK) do
        write(sp, codeunits(string(command, "\n")))
        # sp_drain waits until outgoing bytes have actually been transmitted.
        LibSerialPort.sp_drain(sp)
    end
    nothing
end

function ping!(sp)
    send_line!(sp, "PING")
    line = wait_line!(sp, x -> x == "PONG"; timeout=1.0)
    println(line)
    true
end

function read_imu_pair!(sp; seconds=1.0)
    """收集指定时长内两只 IMU 的最新四元数，返回 Dict(1/2 => 记录)。"""
    latest = Dict{Int,NamedTuple}()
    deadline = time() + seconds
    while time() < deadline
        poll_lines!(sp)
        while !isempty(PENDING_LINES)
            line = popfirst!(PENDING_LINES)
            fields = split(line, ',')
            if length(fields) == 9 && fields[1] == "IMU"
                try
                    id = parse(Int, fields[2])
                    id in (1, 2) || continue
                    q = parse.(Float64, fields[5:8])
                    all(isfinite, q) || continue
                    latest[id] = (sequence=parse(UInt32, fields[3]),
                        timestamp_us=parse(UInt32, fields[4]), quaternion=q,
                        norm=parse(Float64, fields[9]))
                catch
                    # 丢弃损坏或不完整的输出行，继续收集下一帧。
                end
            elseif !isempty(line)
                println(line)
            end
        end
        sleep(0.005)
    end
    for id in 1:2
        if haskey(latest, id)
            println("IMU$(id) 最新四元数 (w,x,y,z) = ", latest[id].quaternion,
                    "；序号 = ", latest[id].sequence)
        else
            println("IMU$(id) 在 $(seconds) 秒内没有有效数据")
        end
    end
    latest
end

function servo_position!(sp, id::Integer=SERVO_ID; timeout=1.0, verbose::Bool=true)
    0 <= id <= 253 || error("舵机 ID 应为 0–253")
    send_line!(sp, "Q,$(id)")
    line = wait_line!(sp, x -> startswith(x, "POS,$(id),") || startswith(x, "ERR,"); timeout)
    startswith(line, "ERR,") && error("舵机反馈错误：$(line)")
    fields = split(line, ',')
    position = parse(Int, fields[3])
    verbose && println("舵机 ID $(id) 当前位置：$(position) ticks（约 $(round(position/TICKS_PER_TURN; digits=3)) 圈）")
    position
end

# 需将本目录最新版 .ino 重新烧录，旧固件没有 M 命令。
function servo_mode!(sp, id::Integer=SERVO_ID)
    send_line!(sp, "M,$(id)")
    line = wait_line!(sp, x -> startswith(x, "MODE,$(id),") || startswith(x, "ERR,"); timeout=1.0)
    startswith(line, "ERR,") && error("读取舵机模式失败：$(line)")
    mode = parse(Int, split(line, ',')[3])
    println("舵机 ID $(id) 模式：$(mode)")
    mode
end

# 将舵机角度限位设为多圈 (0,0)，过程由 ESP32 负责保持当前位置。
# 此操作会修改舵机配置；需先烧录支持 U 命令的最新版桥接固件。
function enable_servo_multiturn!(sp, id::Integer=SERVO_ID)
    send_line!(sp, "U,$(id)")
    line = wait_line!(sp, x -> startswith(x, "ACK,U,$(id),") || startswith(x, "ERR,"); timeout=3.0)
    startswith(line, "ERR,") && error("启用舵机多圈位置控制失败：$(line)；确认 ESP32 已烧录支持 U 命令的固件")
    println("舵机 ID $(id) 已启用多圈位置控制；保持位置 $(split(line, ',')[4]) ticks")
    parse(Int, split(line, ',')[4])
end

# 每次只取指定 IMU 的新一帧；丢弃打开串口后堆积的旧 IMU 行。
function fresh_imu!(sp, id::Integer; timeout::Real=0.5)
    id in (1, 2) || error("IMU ID 只能为 1 或 2")
    poll_lines!(sp)
    filter!(line -> !startswith(line, "IMU,"), PENDING_LINES)
    deadline = time() + timeout
    while time() < deadline
        poll_lines!(sp)
        for i in eachindex(PENDING_LINES)
            fields = split(PENDING_LINES[i], ',')
            if length(fields) == 9 && fields[1] == "IMU" && fields[2] == string(id)
                splice!(PENDING_LINES, i)
                try
                    q = parse.(Float64, fields[5:8])
                    n = sqrt(sum(abs2, q))
                    all(isfinite, q) && 0.9 < n < 1.1 || break
                    return q ./ n
                catch
                    break
                end
            end
        end
        sleep(0.005)
    end
    error("IMU$(id) 在 $(timeout) 秒内没有新数据；闭环停止")
end

# 同一控制周期内取两只 IMU 的新帧，避免用相隔很久的姿态相减。
function fresh_imu_pair!(sp; timeout::Real=0.5, max_skew_us::Integer=2_000)
    poll_lines!(sp)
    filter!(line -> !startswith(line, "IMU,"), PENDING_LINES)
    latest = Dict{Int,NamedTuple}()
    deadline = time() + timeout
    while time() < deadline
        poll_lines!(sp)
        i = 1
        while i <= length(PENDING_LINES)
            line = PENDING_LINES[i]
            fields = split(line, ',')
            if length(fields) == 9 && fields[1] == "IMU" && fields[2] in ("1", "2")
                splice!(PENDING_LINES, i)
                try
                    q = parse.(Float64, fields[5:8])
                    n = sqrt(sum(abs2, q))
                    all(isfinite, q) && 0.9 < n < 1.1 || continue
                    id = parse(Int, fields[2])
                    latest[id] = (q=q ./ n, raw_q=q,
                                  reported_norm=parse(Float64, fields[9]),
                                  sequence=parse(UInt32, fields[3]),
                                  timestamp_us=parse(UInt32, fields[4]))
                catch
                    # 损坏的单帧不会影响另一只 IMU。
                end
            else
                i += 1
            end
        end
        if haskey(latest, 1) && haskey(latest, 2)
            delta = Int64(reinterpret(Int32,
                latest[1].timestamp_us - latest[2].timestamp_us))
            skew = abs(delta)
            if skew <= max_skew_us
                return (imu1=latest[1].q, imu2=latest[2].q,
                        imu1_raw_q=latest[1].raw_q, imu2_raw_q=latest[2].raw_q,
                        imu1_reported_norm=latest[1].reported_norm,
                        imu2_reported_norm=latest[2].reported_norm,
                        imu1_sequence=latest[1].sequence,
                        imu2_sequence=latest[2].sequence,
                        imu1_timestamp_us=latest[1].timestamp_us,
                        imu2_timestamp_us=latest[2].timestamp_us,
                        skew_us=skew)
            end
            delete!(latest, delta < 0 ? 1 : 2)
        end
        sleep(0.005)
    end
    error("双 IMU 在 $(timeout) 秒内没有时间接近的新帧")
end

# 四元数顺序为 (w,x,y,z)。从绝对姿态反算传感器坐标系中的重力方向；
# 当传感器 +X 轴垂直向下时角度为 0°，绕传感器 Z 轴正转为正。
quat_conj(q) = [q[1], -q[2], -q[3], -q[4]]
function quat_mul(a, b)
    w, x, y, z = a
    v, p, r, s = b
    [w*v-x*p-y*r-z*s, w*p+x*v+y*s-z*r,
     w*r-x*s+y*v+z*p, w*s+x*r-y*p+z*v]
end
function imu_gravity_body(q; convention::Symbol=:sensor_to_world,
                           world_down_sign::Integer=-1)
    convention in (:sensor_to_world, :world_to_sensor) ||
        error("四元数坐标约定须为 :sensor_to_world 或 :world_to_sensor")
    world_down_sign in (-1, 1) || error("世界坐标 Z 向下的符号须为 -1 或 1")
    all(isfinite, q) && length(q) == 4 || error("IMU 四元数无效")
    n = sqrt(sum(abs2, q))
    n > 0 || error("IMU 四元数长度为零")
    qn = q ./ n
    gravity_world = [0.0, 0.0, 0.0, Float64(world_down_sign)]
    gravity_body = convention == :sensor_to_world ?
        quat_mul(quat_mul(quat_conj(qn), gravity_world), qn) :
        quat_mul(quat_mul(qn, gravity_world), quat_conj(qn))
    gravity_body[2:4]
end

function imu_z_bend_deg(q; convention::Symbol=:sensor_to_world,
                         world_down_sign::Integer=-1)
    gravity_body = imu_gravity_body(q; convention, world_down_sign)
    gx, gy = gravity_body[1], gravity_body[2]
    hypot(gx, gy) >= 0.2 || error("IMU X/Y 轴接近水平，无法可靠计算绕 Z 轴弯曲角")
    rad2deg(atan(-gy, gx))
end

# 仅适用于两段轴向关节固定、IMU 安装方向已对齐的平面弯曲。
# 第二段相对角 = IMU2 末端投影角 - IMU1 第一段投影角。
function imu_planar_difference_deg(first_q, second_q;
                                   convention::Symbol=:sensor_to_world,
                                   world_down_sign::Integer=-1)
    first = imu_z_bend_deg(first_q; convention, world_down_sign)
    second = imu_z_bend_deg(second_q; convention, world_down_sign)
    mod(second - first + 180, 360) - 180
end

function imu_axis_world(q, axis::AbstractVector;
                        convention::Symbol=:sensor_to_world)
    length(axis) == 3 || error("坐标轴必须是三维向量")
    qn = q ./ sqrt(sum(abs2, q))
    v = [0.0; Float64.(axis)]
    rotated = convention == :sensor_to_world ?
        quat_mul(quat_mul(qn, v), quat_conj(qn)) :
        quat_mul(quat_mul(quat_conj(qn), v), qn)
    rotated[2:4]
end

function rotate_about_axis(v, axis, degrees)
    n = axis ./ sqrt(sum(abs2, axis))
    a = deg2rad(degrees)
    v .* cos(a) .+ cross(n, v) .* sin(a) .+ n .* dot(n, v) .* (1 - cos(a))
end

function signed_tangent_bend_deg(base_tangent, tip_tangent, plane_normal)
    n = plane_normal ./ sqrt(sum(abs2, plane_normal))
    a = base_tangent .- dot(base_tangent, n) .* n
    b = tip_tangent .- dot(tip_tangent, n) .* n
    norm(a) > 0.2 && norm(b) > 0.2 || error("末端方向接近弯曲平面法向，无法计算有效弯曲角")
    a ./= norm(a); b ./= norm(b)
    rad2deg(atan(dot(n, cross(a, b)), dot(a, b)))
end

# 在三维单位方向上直接求指定弯曲平面的有符号分量，不对投影向量归一化。
# 因而结果的绝对值不会超过两条末端方向本身的三维夹角。
function signed_spatial_bend_deg(base_tangent, tip_tangent, plane_normal)
    a = base_tangent ./ norm(base_tangent)
    b = tip_tangent ./ norm(tip_tangent)
    n = plane_normal ./ norm(plane_normal)
    cosine = clamp(dot(a, b), -1.0, 1.0)
    gap = rad2deg(acos(cosine))
    projected = rad2deg(atan(dot(n, cross(a, b)), cosine))
    clamp(projected, -gap, gap)
end

# 完整的直态相对四元数保留两只 IMU 安装时绕 X 轴的方位差。
# 参数按物理段顺序传入；第二段未弯曲时，IMU2 随 IMU1 刚体转动。
# 用预测姿态的 +X/+Z 定义第二段零方向及弯曲平面。
function two_segment_mounted_bends(first_q, second_q, first_zero_q, second_zero_q,
                                   axial_deg::NTuple{2,<:Real};
                                   convention::Symbol=:sensor_to_world)
    convention in (:sensor_to_world, :world_to_sensor) || error("四元数坐标约定无效")
    world_q(q) = convention == :sensor_to_world ? q ./ norm(q) : quat_conj(q ./ norm(q))
    first_world = world_q(first_q)
    first_zero = world_q(first_zero_q)
    second_zero = world_q(second_zero_q)
    expected_second = quat_mul(quat_mul(first_world, quat_conj(first_zero)), second_zero)
    x = [1.0, 0.0, 0.0]; z = [0.0, 0.0, 1.0]
    first_base = imu_axis_world(first_zero, x)
    first_tip = imu_axis_world(first_world, x)
    first_plane = rotate_about_axis(imu_axis_world(first_zero, z),
                                    first_base, axial_deg[1])
    second_base = imu_axis_world(expected_second, x)
    second_tip = imu_axis_world(world_q(second_q), x)
    second_plane = rotate_about_axis(imu_axis_world(expected_second, z),
                                     second_base, axial_deg[2])
    (signed_tangent_bend_deg(first_base, first_tip, first_plane),
     signed_spatial_bend_deg(second_base, second_tip, second_plane))
end

function imu_orientation_distance_deg(q, reference)
    qn = q ./ norm(q)
    rn = reference ./ norm(reference)
    rad2deg(2 * acos(clamp(abs(dot(qn, rn)), 0.0, 1.0)))
end

function check_circle_straight_pose(pair;
                                    convention::Symbol=:sensor_to_world,
                                    world_down_sign::Integer=-1,
                                    max_tilt_deg::Real=20.0)
    down = [0.0, 0.0, Float64(world_down_sign)]
    tilts = ntuple(i -> begin
        q = i == 1 ? pair.imu1 : pair.imu2
        direction = imu_axis_world(q, [1.0, 0.0, 0.0]; convention)
        rad2deg(acos(clamp(dot(direction, down), -1.0, 1.0)))
    end, 2)
    all(t -> t <= max_tilt_deg, tilts) ||
        error("记录圆轨迹直态失败：IMU1/IMU2 的 +X 离竖直向下分别为 $(round.(tilts; digits=1))°；请先将两段回到直态")
    tilts
end

# 第一段只由 IMU1 确定，避免第二段的模型误差把第一段轴向角拉偏；
# 第二段再由 IMU2 确定。两只 IMU 的完整绝对姿态仍用于三维拟合。
function rotation_distance_deg(a::AbstractMatrix, b::AbstractMatrix)
    relative = transpose(a) * b
    cosine = clamp((relative[1, 1] + relative[2, 2] + relative[3, 3] - 1) / 2,
                   -1.0, 1.0)
    rad2deg(acos(cosine))
end

function fit_pcc_to_imu_orientations_sequential(imu_frames::NTuple{2,<:AbstractMatrix},
                                                base_z_offset::Real, initial::AbstractVector;
                                                max_iterations::Integer=12)
    length(initial) == 4 || error("PCC 姿态拟合需要两个弯曲角和两个轴向角")
    x = Float64.(initial)
    function model_frames(parameters)
        bends = parameters[1:2]
        rope = [R_DRIVE * deg2rad(abs(bends[i])) for i in 1:2]
        sides = ntuple(i -> bends[i] >= 0 ? 1 : -1, 2)
        calculate_multi_segment_kinematics(rope, parameters[3:4], base_z_offset;
                                           bend_sides=sides)[2]
    end
    for section in 1:2
        indices = (section, section + 2)
        residual(parameters) = vec(model_frames(parameters)[section][2] .-
                                   imu_frames[section])
        for _ in 1:max_iterations
            r = residual(x)
            norm(r) < 1e-5 && break
            jacobian = Matrix{Float64}(undef, length(r), 2)
            for (column, k) in enumerate(indices)
                perturbed = copy(x)
                perturbed[k] += 0.1
                jacobian[:, column] = (residual(perturbed) .- r) ./ 0.1
            end
            step = -(transpose(jacobian) * jacobian +
                     1e-6 * Matrix{Float64}(I, 2, 2)) \ (transpose(jacobian) * r)
            all(isfinite, step) || error("IMU 姿态拟合产生无效步长")
            x[section] = clamp(x[section] + clamp(step[1], -20.0, 20.0), -270.0, 270.0)
            x[section + 2] = clamp(x[section + 2] + clamp(step[2], -20.0, 20.0), -150.0, 150.0)
            norm(step) < 0.01 && break
        end
    end
    frames = model_frames(x)
    mismatch_deg = ntuple(i -> rotation_distance_deg(frames[i][2], imu_frames[i]), 2)
    (; bend=(x[1], x[2]), axial=(x[3], x[4]), mismatch_deg)
end

# 保留原实时入口使用的四参数联合拟合，便于和离线入口作同条件对照。
function fit_pcc_to_imu_orientations(imu_frames::NTuple{2,<:AbstractMatrix},
                                     base_z_offset::Real, initial::AbstractVector;
                                     max_iterations::Integer=12)
    length(initial) == 4 || error("PCC 姿态拟合需要两个弯曲角和两个轴向角")
    x = Float64.(initial)
    function model_frames(parameters)
        bends = parameters[1:2]
        rope = [R_DRIVE * deg2rad(abs(bends[i])) for i in 1:2]
        sides = ntuple(i -> bends[i] >= 0 ? 1 : -1, 2)
        calculate_multi_segment_kinematics(rope, parameters[3:4], base_z_offset;
                                           bend_sides=sides)[2]
    end
    function residual(parameters)
        frames = model_frames(parameters)
        vcat((vec(frames[i][2] .- imu_frames[i]) for i in 1:2)...)
    end
    for _ in 1:max_iterations
        r = residual(x)
        norm(r) < 1e-5 && break
        jacobian = Matrix{Float64}(undef, length(r), 4)
        for k in 1:4
            perturbed = copy(x)
            perturbed[k] += 0.1
            jacobian[:, k] = (residual(perturbed) .- r) ./ 0.1
        end
        step = -(transpose(jacobian) * jacobian + 1e-6 * Matrix{Float64}(I, 4, 4)) \
                (transpose(jacobian) * r)
        all(isfinite, step) || error("IMU 姿态拟合产生无效步长")
        x .+= clamp.(step, -20.0, 20.0)
        x[1:2] .= clamp.(x[1:2], -270.0, 270.0)
        x[3:4] .= clamp.(x[3:4], -150.0, 150.0)
        norm(step) < 0.01 && break
    end
    frames = model_frames(x)
    mismatch_deg = ntuple(i -> rotation_distance_deg(frames[i][2], imu_frames[i]), 2)
    (; bend=(x[1], x[2]), axial=(x[3], x[4]), mismatch_deg)
end

# 拟合角的单帧突变先隔离；只有相近的下一帧再次出现才承认真实快速运动。
function accept_fit_sample!(pending::Base.RefValue, previous::AbstractVector,
                            candidate::AbstractVector;
                            jump_deg::Real=12.0, confirm_deg::Real=5.0)
    all(isfinite, candidate) || error("IMU 姿态拟合包含无效角度")
    if maximum(abs.(candidate .- previous)) <= jump_deg
        pending[] = nothing
        return true
    end
    confirmed = pending[] !== nothing &&
        maximum(abs.(candidate .- pending[])) <= confirm_deg
    pending[] = confirmed ? nothing : copy(candidate)
    confirmed
end

# 第二段局部姿态：先用两只 IMU 的四元数消除第一段整体运动，
# 再取相对四元数绕第一段局部 Z 轴的有符号 twist。
function imu_relative_z_deg(first_q, second_q;
                             convention::Symbol=:sensor_to_world)
    convention in (:sensor_to_world, :world_to_sensor) ||
        error("四元数坐标约定无效")
    qrel = convention == :sensor_to_world ?
        quat_mul(quat_conj(first_q), second_q) :
        quat_mul(first_q, quat_conj(second_q))
    angle = rad2deg(2 * atan(qrel[4], qrel[1]))
    mod(angle + 180, 360) - 180
end

# 四元数滤波：先拒绝与物理角速度不符的孤立尖峰，再在球面上做短时低通。
# q 与 -q 表示同一姿态，比较和平均前必须统一符号。
mutable struct IMUQuaternionFilter
    filtered::Union{Nothing,Vector{Float64}}
    last_good::Union{Nothing,Vector{Float64}}
    last_good_time::Float64
    last_filter_time::Float64
    rejected::Int
end
IMUQuaternionFilter() = IMUQuaternionFilter(nothing, nothing, -Inf, -Inf, 0)

function filter_imu_quaternion!(filter::IMUQuaternionFilter, q, now::Real;
                                tau_s::Real=0.03, spike_margin_deg::Real=15.0,
                                max_rate_deg_s::Real=1000.0,
                                stale_timeout_s::Real=0.5)
    qn = Float64.(q)
    qn ./= sqrt(sum(abs2, qn))
    if filter.last_good === nothing
        filter.filtered = copy(qn)
        filter.last_good = copy(qn)
        filter.last_good_time = Float64(now)
        filter.last_filter_time = Float64(now)
        return filter.filtered, true
    end
    elapsed_good = max(Float64(now) - filter.last_good_time, 0.0)
    separation_deg = rad2deg(2 * acos(clamp(abs(sum(qn .* filter.last_good)), 0.0, 1.0)))
    # 单帧阈值至多按 50 ms 运动放宽；否则持续坏帧会因等待时间变长而被误收。
    if separation_deg > spike_margin_deg + max_rate_deg_s * min(elapsed_good, 0.05)
        filter.rejected += 1
        elapsed_good > stale_timeout_s &&
            error("IMU 连续 $(round(elapsed_good; digits=2)) 秒出现姿态突变；停止闭环")
        return filter.filtered, false
    end
    sum(qn .* filter.filtered) < 0 && (qn .*= -1)
    dt = max(Float64(now) - filter.last_filter_time, 0.0)
    alpha = tau_s == 0 ? 1.0 : 1 - exp(-dt / tau_s)
    smoothed = (1 - alpha) .* filter.filtered .+ alpha .* qn
    smoothed ./= sqrt(sum(abs2, smoothed))
    filter.filtered = smoothed
    filter.last_good = qn
    filter.last_good_time = Float64(now)
    filter.last_filter_time = Float64(now)
    filter.filtered, true
end

# 两只 IMU 的相对四元数抵消第一段整体运动；Z 分量仅在轴向关节固定且坐标轴对齐时可当作第二段弯曲角。
# 单次读取只给出 [-180°,180°) 的角；超过 180° 需连续采样并逐段展开。
function read_two_segment_angles!(sp; first_imu_id::Integer=1,
                                   second_imu_id::Integer=2,
                                   quaternion_convention::Symbol=:sensor_to_world,
                                   world_down_sign::Integer=-1)
    first_imu_id in (1, 2) && second_imu_id in (1, 2) &&
        first_imu_id != second_imu_id || error("两段须分别指定 IMU1 和 IMU2")
    pair = fresh_imu_pair!(sp)
    q1 = first_imu_id == 1 ? pair.imu1 : pair.imu2
    q2 = second_imu_id == 1 ? pair.imu1 : pair.imu2
    first = imu_z_bend_deg(q1; convention=quaternion_convention,
                           world_down_sign=world_down_sign)
    second_absolute = imu_z_bend_deg(q2; convention=quaternion_convention,
                                     world_down_sign=world_down_sign)
    second_relative = imu_relative_z_deg(q1, q2;
                                         convention=quaternion_convention)
    planar_difference = imu_planar_difference_deg(q1, q2;
        convention=quaternion_convention, world_down_sign=world_down_sign)
    println("第一段 IMU$(first_imu_id) 投影角=$(round(first; digits=2))°，第二段 IMU$(second_imu_id) 末端投影角=$(round(second_absolute; digits=2))°，平面角差=$(round(planar_difference; digits=2))°，相对四元数 Z twist=$(round(second_relative; digits=2))°；双 IMU 时间差=$(pair.skew_us) μs。轴向关节运动时，这些角度不能直接当作实际弯曲角")
    (first_deg=first, second_absolute_deg=second_absolute,
     second_relative_deg=second_relative,
     planar_difference_deg=planar_difference, skew_us=pair.skew_us)
end

# 当前接线：舵机1控制第一段 IMU1；舵机2控制第二段 IMU2。
# 标定使用绝对四元数推算的有符号 Z 轴弯曲角，不自动切换到另一只 IMU。
# 500 ticks 约为舵机输出轴 44°；这是机械行程，不等于操作臂弯曲角。
function calibrate_imu_servo!(sp; servo_id::Integer=1, imu_id::Integer=1,
                              jog_ticks::Integer=500, speed_rpm::Real=8.0,
                              return_tolerance_ticks::Integer=20,
                              relative_to_imu_id::Union{Nothing,Integer}=nothing,
                              quaternion_convention::Symbol=:sensor_to_world,
                              world_down_sign::Integer=-1)
    200 <= jog_ticks <= 800 || error("标定点动须在 200～800 ticks")
    return_tolerance_ticks > 0 || error("回位容差必须大于 0 ticks")
    imu_id in (1, 2) || error("IMU ID 只能为 1 或 2")
    relative_to_imu_id === nothing ||
        (relative_to_imu_id in (1, 2) && relative_to_imu_id != imu_id) ||
        error("相对角的基准 IMU 必须是另一只 IMU")
    isfinite(speed_rpm) && 0 < speed_rpm <= 10 || error("标定速度须在 0～10 RPM")
    servo_mode!(sp, servo_id) in (0, 4) || error("舵机不是位置模式；未点动")
    enable_servo_multiturn!(sp, servo_id)
    origin = servo_position!(sp, servo_id)
    -32767 <= origin + jog_ticks <= 32767 || error("标定目标超出舵机位置范围")
    function one_angle()
        if relative_to_imu_id === nothing
            return imu_z_bend_deg(fresh_imu!(sp, imu_id);
                convention=quaternion_convention, world_down_sign=world_down_sign)
        end
        pair = fresh_imu_pair!(sp)
        q = imu_id == 1 ? pair.imu1 : pair.imu2
        qbase = relative_to_imu_id == 1 ? pair.imu1 : pair.imu2
        imu_planar_difference_deg(qbase, q;
            convention=quaternion_convention, world_down_sign=world_down_sign)
    end
    function measured_angle()
        samples = [one_angle() for _ in 1:5]
        anchor = samples[1]
        unwrapped = [anchor + mod(angle - anchor + 180, 360) - 180
                     for angle in samples]
        sort!(unwrapped)
        mod(unwrapped[3] + 180, 360) - 180
    end
    initial_angle = measured_angle()
    moved = false
    calibration = nothing
    returned_position = origin
    try
        move_servo_relative!(sp, jog_ticks; id=servo_id, speed_rpm=speed_rpm,
                             max_delta_ticks=800)
        moved = true
        deadline = time() + 5.0
        while time() < deadline
            abs(servo_position!(sp, servo_id) - (origin + jog_ticks)) <= 10 && break
            sleep(0.1)
        end
        actual = servo_position!(sp, servo_id)
        abs(actual - (origin + jog_ticks)) <= 10 || error("舵机未到标定目标位置")
        sleep(0.25)
        angle = mod(measured_angle() - initial_angle + 180, 360) - 180
        println("Z 轴标定响应：IMU$(imu_id)=$(round(angle; digits=3))°")
        0.1 <= abs(angle) < 170 || error("IMU$(imu_id) 的 Z 轴响应 $(round(angle; digits=3))° 不足以可靠标定；检查 IMU 的 Z 轴是否与弯曲轴一致")
        gain = angle / (actual - origin)
        calibration = (servo_id=Int(servo_id), imu_id=Int(imu_id),
                       origin=origin,
                       measurement=relative_to_imu_id === nothing ? :gravity_z : :planar_difference,
                       relative_to_imu_id=relative_to_imu_id,
                       quaternion_convention=quaternion_convention,
                       world_down_sign=world_down_sign,
                       deg_per_tick=gain)
        println("Z 轴标定：IMU$(imu_id)，舵机 $(actual-origin) ticks 对应 $(round(angle; digits=2))°；正在回到初始位置")
    finally
        if moved
            send_line!(sp, "P,$(servo_id),$(origin),$(speed_rpm)")
            ack = wait_line!(sp, x -> startswith(x, "ACK,P,$(servo_id),") || startswith(x, "ERR,"); timeout=1.0)
            startswith(ack, "ERR,") && error("标定回程失败：$(ack)")
            deadline = time() + 5.0
            while time() < deadline
                returned_position = servo_position!(sp, servo_id)
                abs(returned_position - origin) <= return_tolerance_ticks && break
                sleep(0.1)
            end
            returned_position = servo_position!(sp, servo_id)
            abs(returned_position - origin) <= return_tolerance_ticks ||
                error("标定回程未到允许范围：目标 $(origin) ticks，实际 $(returned_position) ticks，相差 $(abs(returned_position-origin)) ticks，容差 $(return_tolerance_ticks) ticks")
        end
    end
    returned_position != origin &&
        println("标定回程停在 $(returned_position) ticks，距离原位置 $(returned_position-origin) ticks；后续以实际位置为起点")
    merge(calibration, (origin=returned_position,))
end

function make_imu_feedback_plot(; show_command::Bool=false, show_window::Bool=true,
                                initial_target_deg::Real=10.0,
                                manual_range_ticks::Integer=800)
    figure = GM.Figure(size=(1050, 780))
    angle_axis = GM.Axis(figure[1, 1], xlabel="时间 (s)", ylabel="角度 (°)",
                         title="IMU 局部 Z 轴弯曲角跟踪")
    error_axis = GM.Axis(figure[2, 1], xlabel="时间 (s)", ylabel="误差 (°)",
                         title="跟踪误差")
    target_points = GM.Observable(GM.Point2f[])
    command_points = GM.Observable(GM.Point2f[])
    measured_points = GM.Observable(GM.Point2f[])
    error_points = GM.Observable(GM.Point2f[])
    GM.lines!(angle_axis, target_points; color=:dodgerblue, linewidth=2,
              label="期望角度")
    show_command && GM.lines!(angle_axis, command_points; color=:seagreen,
                              linewidth=1.5, linestyle=:dash, label="提前控制目标")
    GM.lines!(angle_axis, measured_points; color=:orange, linewidth=2,
              label="IMU 实测")
    GM.lines!(error_axis, error_points; color=:crimson, linewidth=2)
    GM.hlines!(error_axis, [0.0]; color=:gray, linestyle=:dash)
    GM.axislegend(angle_axis; position=:rt)
    controls = GM.GridLayout()
    figure[3, 1] = controls
    GM.Label(controls[1, 1], show_command ? "正弦中心角 (deg)" : "期望角度 (deg)")
    target_box = GM.Textbox(controls[1, 2]; stored_string=string(initial_target_deg),
                            validator=Float64, width=120)
    read_angle_button = GM.Button(controls[1, 3], label="读取当前角度")
    start_button = GM.Button(controls[2, 1], label="开始控制")
    stop_button = GM.Button(controls[2, 2], label="结束控制（保持位置）")
    emergency_button = GM.Button(controls[2, 3], label="紧急停止（关闭扭矩）")
    manual_button = GM.Button(controls[3, 1], label="紧急停止后启用手动回位")
    slider_values = sort(unique(vcat(collect(-manual_range_ticks:32:manual_range_ticks),
                                     [-manual_range_ticks, 0, manual_range_ticks])))
    GM.Label(controls[4, 1], "弯曲电机手动偏移 (ticks；中间为 0)")
    manual_slider = GM.Slider(controls[4, 2:3]; range=slider_values, startvalue=0)
    status = GM.Observable("待机：输入目标角度；IMU X 轴垂直向下定义为 0°")
    GM.Label(figure[4, 1], status)
    angle_readout = GM.Observable("当前角度：尚未读取")
    GM.Label(figure[5, 1], angle_readout)
    angle_diagnostic = GM.Observable("四元数与重力方向：尚未读取")
    GM.Label(figure[6, 1], angle_diagnostic)
    show_window && display(figure)
    times = Float32[]
    targets = Float32[]
    commands = Float32[]
    measurements = Float32[]
    errors = Float32[]
    function update_plot(t, desired, control_desired, measured, error_deg)
        push!(times, Float32(t))
        push!(targets, Float32(desired))
        push!(commands, Float32(control_desired))
        push!(measurements, Float32(measured))
        push!(errors, Float32(error_deg))
        target_points[] = GM.Point2f.(times, targets)
        show_command && (command_points[] = GM.Point2f.(times, commands))
        measured_points[] = GM.Point2f.(times, measurements)
        error_points[] = GM.Point2f.(times, errors)
        right = max(2.0, t + 0.5)
        GM.xlims!(angle_axis, 0.0, right)
        GM.xlims!(error_axis, 0.0, right)
        angle_low = min(minimum(targets), minimum(measurements))
        angle_high = max(maximum(targets), maximum(measurements))
        if show_command
            angle_low = min(angle_low, minimum(commands))
            angle_high = max(angle_high, maximum(commands))
        end
        angle_pad = max(1.0, 0.1 * (angle_high - angle_low))
        GM.ylims!(angle_axis, angle_low - angle_pad, angle_high + angle_pad)
        error_low = min(0.0f0, minimum(errors))
        error_high = max(0.0f0, maximum(errors))
        error_pad = max(1.0, 0.1 * (error_high - error_low))
        GM.ylims!(error_axis, error_low - error_pad, error_high + error_pad)
    end
    (figure=figure, update=update_plot, target_box=target_box,
     read_angle_button=read_angle_button, angle_readout=angle_readout,
     angle_diagnostic=angle_diagnostic,
     manual_button=manual_button, manual_slider=manual_slider,
     start_button=start_button,
     stop_button=stop_button, emergency_button=emergency_button,
     status=status, time_s=times,
     target_deg=targets, command_deg=commands,
     measured_deg=measurements, error_deg=errors)
end

function make_two_segment_feedback_plot(; first_target_deg::Real=0.0,
                                         second_target_deg::Real=0.0,
                                         bend_manual_range_ticks::Integer=7*TICKS_PER_TURN,
                                         axial_manual_range_pulse::Integer=500,
                                         circle_mode::Bool=false,
                                         circle_state=nothing,
                                         sequential_fit::Bool=false,
                                         bend_deg_per_tick=(1.0, 1.0),
                                         axial_signs=(1, 1),
                                         quaternion_convention::Symbol=:sensor_to_world,
                                         show_window::Bool=true)
    figure = GM.Figure(size=circle_mode ? (1650, 1220) : (1120, 920))
    axes = [GM.Axis(figure[row, col], xlabel="时间 (s)", ylabel="角度 / 误差 (deg)",
                    title= row == 1 ? (col == 1 ? (circle_mode ? "第一段：IMU 姿态拟合弯曲角" : "第一段：IMU1 投影角") :
                                                   (circle_mode ? "第二段：IMU 姿态拟合弯曲角" : "第二段：IMU2 - IMU1 平面角差")) :
                                      (col == 1 ? "第一段误差" : "第二段误差"))
            for row in 1:2, col in 1:2]
    desired = [GM.Observable(GM.Point2f[]) for _ in 1:2]
    measured = [GM.Observable(GM.Point2f[]) for _ in 1:2]
    errors = [GM.Observable(GM.Point2f[]) for _ in 1:2]
    for i in 1:2
        GM.lines!(axes[1, i], desired[i]; color=:dodgerblue, label="目标")
        GM.lines!(axes[1, i], measured[i]; color=:orange, label="实测")
        GM.axislegend(axes[1, i]; position=:rt)
        GM.lines!(axes[2, i], errors[i]; color=:crimson)
        GM.hlines!(axes[2, i], [0.0]; color=:gray, linestyle=:dash)
    end
    target_tip_trace = GM.Observable(GM.Point3f[])
    fitted_tip_trace = GM.Observable(GM.Point3f[])
    imu_world_alignment = Ref{Any}(nothing)
    active_circle_state = Ref{Any}(circle_state)
    fitted_parameters = Ref([0.0, 0.0, 0.0, 0.0])
    pending_fit = Ref{Any}(nothing)
    rejected_fit_samples = Ref(0)
    target_frame_axes = nothing
    imu_frame_axes = nothing
    drive_diagnostic = GM.Observable("四电机姿态诊断：等待记录直态及 IMU 数据")
    if circle_state !== nothing
        path_axis = GM.Axis3(figure[1:2, 3], xlabel="X (mm)", ylabel="Y (mm)",
                             zlabel="Z (mm)", aspect=:data,
                             title="三维 PCC：蓝色目标 / 橙色重力拟合 + 指令轴向角")
        diagnostic_panel = GM.GridLayout()
        figure[3, 3] = diagnostic_panel
        attitude_title = GM.Observable("姿态误差：等待 IMU 数据")
        GM.Label(diagnostic_panel[1, 1], attitude_title;
                 justification=:left, halign=:left, fontsize=15)
        GM.Label(diagnostic_panel[2, 1], drive_diagnostic;
                 justification=:left, halign=:left, fontsize=15)
        target_arm = GM.Observable(copy(circle_state.joints_history[1]))
        fitted_arm = GM.Observable(copy(circle_state.joints_history[1]))
        target_path = GM.Observable(copy(circle_state.target_trajectory))
        GM.lines!(path_axis, target_path;
                  color=:dodgerblue, linewidth=2, linestyle=:dash)
        GM.lines!(path_axis, target_tip_trace; color=:dodgerblue, linewidth=2)
        GM.lines!(path_axis, fitted_tip_trace; color=:orange, linewidth=2)
        GM.lines!(path_axis, target_arm; color=:dodgerblue, linewidth=8)
        GM.lines!(path_axis, fitted_arm; color=:orange, linewidth=5)
        GM.scatter!(path_axis, GM.lift(arm -> [arm[8], arm[end]], target_arm);
                    color=:dodgerblue, markersize=12)
        GM.scatter!(path_axis, GM.lift(arm -> [arm[8], arm[end]], fitted_arm);
                    color=:orange, markersize=10)
        target_frame_axes = [[GM.Observable(GM.Point3f[]) for _ in 1:3] for _ in 1:2]
        imu_frame_axes = [[GM.Observable(GM.Point3f[]) for _ in 1:3] for _ in 1:2]
        axis_colors = (:red, :green, :blue)
        for section in 1:2, axis in 1:3
            GM.lines!(path_axis, target_frame_axes[section][axis];
                      color=axis_colors[axis], linewidth=4)
            GM.lines!(path_axis, imu_frame_axes[section][axis];
                      color=(axis_colors[axis], 0.45), linewidth=6)
        end
        GM.autolimits!(path_axis)
    end
    function model_pose(angles, rotations)
        drive = [R_DRIVE * deg2rad(abs(angles[i])) for i in 1:2]
        sides = ntuple(i -> angles[i] >= 0 ? 1 : -1, 2)
        calculate_multi_segment_kinematics(drive, collect(rotations),
            active_circle_state[].base_z_offset; bend_sides=sides)
    end
    function imu_rotation(q)
        basis_vectors = ([1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0])
        hcat((imu_axis_world(q, axis; convention=quaternion_convention)
              for axis in basis_vectors)...)
    end
    function sensor_to_model_axes(section)
        side = active_circle_state[].bend_sides[section]
        # IMU +X 沿臂段轴向；+Y 指向正弯曲侧；+Z 由右手系确定。
        [0.0 side 0.0; 0.0 0.0 side; 1.0 0.0 0.0]
    end
    function draw_frame_axes!(observables, section, origin, rotation;
                              axis_length_mm=35.0)
        for axis in 1:3
            # 姿态既可能是 3×3 矩阵，也可能是三个轴向量组成的元组。
            direction = rotation isa Tuple ? rotation[axis] : rotation[:, axis]
            length(direction) == 3 || error("末端姿态轴须为三维向量")
            endpoint = [Float64(origin[i]) + axis_length_mm * direction[i]
                        for i in 1:3]
            observables[section][axis][] = GM.Point3f[
                GM.Point3f(origin[1], origin[2], origin[3]),
                GM.Point3f(endpoint[1], endpoint[2], endpoint[3])]
        end
    end
    function set_imu_baseline!(pair)
        active_circle_state[] === nothing && return
        _, neutral_frames = model_pose((0.0, 0.0), (0.0, 0.0))
        quaternions = (pair.imu1, pair.imu2)
        alignments = ntuple(section -> begin
            neutral_frames[section][2] * sensor_to_model_axes(section) *
                transpose(imu_rotation(quaternions[section]))
        end, 2)
        imu_world_alignment[] = alignments
        fitted_parameters[] = [0.0, 0.0, 0.0, 0.0]
        pending_fit[] = nothing
        rejected_fit_samples[] = 0
        for section in 1:2
            position, frame = neutral_frames[section]
            draw_frame_axes!(target_frame_axes, section, position,
                             frame * sensor_to_model_axes(section))
            draw_frame_axes!(imu_frame_axes, section, position,
                             alignments[section] * imu_rotation(quaternions[section]))
        end
        attitude_title[] = "姿态误差：第一段 0° / 第二段 0°；重力方向残差 0°"
        drive_diagnostic[] = "四电机姿态诊断：直态已记录，等待 IMU 数据"
    end
    function update_tip(bends, axial, reference_bends, reference_axial,
                        imu_quaternions=nothing; render::Bool=true,
                        fit_override=nothing)
        active_circle_state[] === nothing && return
        imu_sensor_frames = imu_world_alignment[] === nothing ||
                            imu_quaternions === nothing ? nothing :
            ntuple(section -> imu_world_alignment[][section] *
                              imu_rotation(imu_quaternions[section]), 2)
        fit = fit_override !== nothing ? fit_override :
              imu_sensor_frames === nothing ? nothing :
            fit_pcc_fixed_axial(
                ntuple(section -> imu_sensor_frames[section] *
                                  transpose(sensor_to_model_axes(section)), 2),
                ntuple(section -> imu_world_alignment[][section] * [0., 0., 1.], 2),
                active_circle_state[].base_z_offset, fitted_parameters[], axial)
        if fit !== nothing
            candidate = [fit.bend..., fit.axial...]
            if fit_override === nothing &&
               !accept_fit_sample!(pending_fit, fitted_parameters[], candidate)
                rejected_fit_samples[] += 1
                return nothing
            end
            fitted_parameters[] = candidate
            if render
                target_parameters = [reference_bends..., reference_axial...]
                measured_parameters = fitted_parameters[]
                parameter_errors = target_parameters .- measured_parameters
                motor_signs = (sign(bend_deg_per_tick[1]), sign(bend_deg_per_tick[2]),
                               axial_signs[1], axial_signs[2])
                direction(i) = abs(parameter_errors[i]) < 0.5 ? "保持" :
                    parameter_errors[i] * motor_signs[i] > 0 ? "增大" : "减小"
                fmt(values) = join((string(round(v; digits=1), "°") for v in values), "  /  ")
                reliable = maximum(fit.mismatch_deg) <= 30.0
                commanded_axial_pulses = ntuple(i ->
                    round(Int, 1500 + axial_signs[i] * axial[i] * 2000 / 300), 2)
                drive_diagnostic[] =
                    "顺序：弯曲电机1  /  弯曲电机2  /  轴向通道1  /  轴向通道2\n" *
                    "PCC目标：" * fmt(target_parameters) * "\n" *
                    "重力拟合弯曲 / 指令轴向：" * fmt(measured_parameters) * "\n" *
                    "目标减估计/指令：  " * fmt(parameter_errors) * "\n" *
                    "轴向已发指令：通道1 $(commanded_axial_pulses[1]) / 通道2 $(commanded_axial_pulses[2]) μs\n" *
                    "轴向指令角：" * fmt(axial) * "（由脉宽换算，非舵机实测）\n" *
                    "弯曲目标 ticks：电机1 " * direction(1) * "  /  电机2 " * direction(2) * "\n" *
                    (reliable ? "弯曲由重力拟合闭环；轴向固定为已发指令角" :
                                "重力方向残差较大；请检查倾斜测量与机械零位")
            end
        end
        render || return fit
        target_joints, reference_frames = model_pose(reference_bends, reference_axial)
        fitted_bends = fit === nothing ? bends : fit.bend
        fitted_axial = fit === nothing ? axial : fit.axial
        measured_joints, measured_frames = model_pose(fitted_bends, fitted_axial)
        target_arm[] = target_joints
        fitted_arm[] = measured_joints
        target_points = copy(target_tip_trace[])
        fitted_points = copy(fitted_tip_trace[])
        push!(target_points, target_joints[end])
        push!(fitted_points, measured_joints[end])
        target_tip_trace[] = target_points
        fitted_tip_trace[] = fitted_points
        if imu_world_alignment[] !== nothing
            pose_error = Float64[]
            for section in 1:2
                reference_position, reference_rotation = reference_frames[section]
                target_sensor_rotation = reference_rotation * sensor_to_model_axes(section)
                draw_frame_axes!(target_frame_axes, section, reference_position,
                                 target_sensor_rotation)
                if imu_sensor_frames !== nothing
                    # 没有末端位置传感器；IMU 姿态轴放在拟合 PCC 的对应末端。
                    measured_position = measured_frames[section][1]
                    draw_frame_axes!(imu_frame_axes, section, measured_position,
                                     imu_sensor_frames[section])
                    push!(pose_error, rotation_distance_deg(target_sensor_rotation,
                                                            imu_sensor_frames[section]))
                end
            end
            if length(pose_error) == 2
                attitude_title[] = "姿态误差：一段 $(round(pose_error[1]; digits=1))° / 二段 $(round(pose_error[2]; digits=1))°；重力方向残差 $(round.(fit.mismatch_deg; digits=1))°"
            end
        end
        GM.autolimits!(path_axis)
        fit
    end
    controls = GM.GridLayout()
    axial_axes = circle_mode ? [
        GM.Axis(figure[3, i], xlabel="时间 (s)", ylabel="已发送脉宽 (μs)",
                title="第 $(i) 段轴向通道 $(i)：控制板已确认的指令") for i in 1:2] : nothing
    axial_traces = circle_mode ? [GM.Observable(GM.Point2f[]) for _ in 1:2] : nothing
    axial_times = Float32[]
    axial_pulse_history = [Float32[] for _ in 1:2]
    if circle_mode
        for i in 1:2
            GM.lines!(axial_axes[i], axial_traces[i]; color=i == 1 ? :purple : :teal)
            GM.hlines!(axial_axes[i], [1500.0]; color=:gray, linestyle=:dash)
        end
    end
    function update_axial(t, pulses)
        circle_mode || return
        push!(axial_times, Float32(t))
        for i in 1:2
            push!(axial_pulse_history[i], Float32(pulses[i]))
            axial_traces[i][] = GM.Point2f.(axial_times, axial_pulse_history[i])
            GM.xlims!(axial_axes[i], 0.0, max(2.0, t + 0.5))
            lo = min(1500.0f0, minimum(axial_pulse_history[i]))
            hi = max(1500.0f0, maximum(axial_pulse_history[i]))
            pad = max(50.0, 0.08 * (hi - lo))
            GM.ylims!(axial_axes[i], max(450.0, lo - pad), min(2550.0, hi + pad))
        end
    end
    figure[circle_mode ? 4 : 3, 1:(circle_mode ? 3 : 2)] = controls
    GM.Label(controls[1, 1], "第一段角度目标 (deg)")
    target1_box = GM.Textbox(controls[1, 2]; stored_string=string(first_target_deg),
                             validator=Float64, width=110)
    GM.Label(controls[1, 3], "第二段角度目标 (deg)")
    target2_box = GM.Textbox(controls[1, 4]; stored_string=string(second_target_deg),
                             validator=Float64, width=110)
    read_button = GM.Button(controls[2, 1], label="读取两段角度")
    start_button = GM.Button(controls[2, 2], label=circle_mode ? "跟踪圆形" : "开始控制")
    rectangle_button = circle_mode ? GM.Button(controls[2, 3], label="跟踪矩形") : nothing
    angle_button = circle_mode ? GM.Button(controls[2, 4], label="开始角度控制") : nothing
    stop_button = GM.Button(circle_mode ? controls[3, 1] : controls[2, 3],
                            label="结束控制（保持位置）")
    emergency_button = GM.Button(circle_mode ? controls[3, 2] : controls[2, 4],
                                 label="紧急停止（关闭弯曲电机扭矩）")
    home_button = GM.Button(circle_mode ? controls[3, 3] : controls[3, 1:2],
        label=circle_mode ? "回中回直（弯曲 0°，轴向 1500）" : "回直（两段目标 0°）")
    manual_button = GM.Button(circle_mode ? controls[3, 4] : controls[3, 3:4],
                              label="紧急停止后启用四电机手动回位")
    bend_values = sort(unique(vcat(collect(-bend_manual_range_ticks:32:bend_manual_range_ticks),
                                    [-bend_manual_range_ticks, 0, bend_manual_range_ticks])))
    axial_values = sort(unique(vcat(collect(-axial_manual_range_pulse:10:axial_manual_range_pulse),
                                     [-axial_manual_range_pulse, 0, axial_manual_range_pulse])))
    slider_labels = ("第一段弯曲电机 1 (ticks)", "第二段弯曲电机 2 (ticks)",
                     "第一段轴向通道 1 (脉宽)", "第二段轴向通道 2 (脉宽)")
    manual_sliders = [begin
        row = i + (circle_mode ? 4 : 3)
        GM.Label(controls[row, 1], slider_labels[i])
        GM.Slider(controls[row, 2:4]; range=i <= 2 ? bend_values : axial_values,
                  startvalue=0)
    end for i in 1:4]
    capture_button = GM.Button(controls[circle_mode ? 9 : 8, 1:4],
                               label="轴向回中 1500 并记录两段直态四元数")
    status = GM.Observable(circle_mode ?
        "角度控制/回中回直可直接启动；圆形、矩形轨迹须先在直态记录四元数" :
        "待机：输入两段目标角度，先用读取按钮核对两段姿态")
    GM.Label(figure[circle_mode ? 5 : 4, 1:(circle_mode ? 3 : 2)], status)
    readout = GM.Observable("两段角度：尚未读取")
    GM.Label(figure[circle_mode ? 6 : 5, 1:(circle_mode ? 3 : 2)], readout)
    times = Float32[]
    target_hist = [Float32[] for _ in 1:2]
    measured_hist = [Float32[] for _ in 1:2]
    error_hist = [Float32[] for _ in 1:2]
    function update_plot(t, targets, actuals)
        push!(times, Float32(t))
        for i in 1:2
            push!(target_hist[i], Float32(targets[i]))
            push!(measured_hist[i], Float32(actuals[i]))
            push!(error_hist[i], Float32(targets[i] - actuals[i]))
            desired[i][] = GM.Point2f.(times, target_hist[i])
            measured[i][] = GM.Point2f.(times, measured_hist[i])
            errors[i][] = GM.Point2f.(times, error_hist[i])
            for row in 1:2
                GM.xlims!(axes[row, i], 0.0, max(2.0, t + 0.5))
            end
            lo = min(minimum(target_hist[i]), minimum(measured_hist[i]))
            hi = max(maximum(target_hist[i]), maximum(measured_hist[i]))
            pad = max(1.0, 0.1 * (hi - lo))
            GM.ylims!(axes[1, i], lo - pad, hi + pad)
            elo = min(0.0f0, minimum(error_hist[i]))
            ehi = max(0.0f0, maximum(error_hist[i]))
            epad = max(1.0, 0.1 * (ehi - elo))
            GM.ylims!(axes[2, i], elo - epad, ehi + epad)
        end
    end
    function clear_history!()
        empty!(times)
        empty!(axial_times)
        for i in 1:2
            empty!(target_hist[i])
            empty!(measured_hist[i])
            empty!(error_hist[i])
            empty!(axial_pulse_history[i])
            desired[i][] = GM.Point2f[]
            measured[i][] = GM.Point2f[]
            errors[i][] = GM.Point2f[]
            circle_mode && (axial_traces[i][] = GM.Point2f[])
        end
        target_tip_trace[] = GM.Point3f[]
        fitted_tip_trace[] = GM.Point3f[]
    end
    function set_reference!(state)
        circle_mode || return
        active_circle_state[] = state
        target_path[] = copy(state.target_trajectory)
        target_arm[] = copy(state.joints_history[1])
        fitted_arm[] = copy(state.joints_history[1])
        GM.autolimits!(path_axis)
    end
    function set_angle_mode!(enabled::Bool)
        circle_mode || return
        axes[1, 1].title[] = enabled ? "第一段：IMU1 投影角" : "第一段：IMU 姿态拟合弯曲角"
        axes[1, 2].title[] = enabled ? "第二段：IMU2 - IMU1 平面角差" : "第二段：IMU 姿态拟合弯曲角"
    end
    show_window && display(figure)
    (; figure, update=update_plot, update_tip, update_axial, clear_history!,
     set_reference!, set_angle_mode!,
     set_imu_baseline!,
     rejected_fit_samples,
     target1_box, target2_box, read_button,
     start_button, rectangle_button, angle_button,
     stop_button, emergency_button, home_button, manual_button, manual_sliders,
     capture_button,
     status, readout,
     time_s=times, target_deg=target_hist, measured_deg=measured_hist,
     error_deg=error_hist)
end

# 外环 PID：P 看当前角度误差，I 累积持续误差，D 用滤波后的 IMU 角速度阻尼。
# 积分在目标或单步命令被裁剪且误差继续推向边界时冻结，避免 windup。
function pid_position_step(error_deg, measured_rate, dt, integral, gain,
                           position, lower_tick, upper_tick, max_step_ticks,
                           kp, ki, kd)
    proposed_integral = integral + error_deg * dt
    function command(candidate_integral)
        effort = kp * error_deg + ki * candidate_integral - kd * measured_rate
        raw_step = effort / gain
        step = round(Int, clamp(raw_step, -max_step_ticks, max_step_ticks))
        requested = position + step
        target = clamp(requested, lower_tick, upper_tick)
        (target=target, requested=requested,
         saturated=abs(raw_step) > max_step_ticks || requested != target,
         raw_step=raw_step, effort=effort)
    end
    candidate = command(proposed_integral)
    # 用角度控制量的符号判断积分方向；标定增益可能为负。
    if candidate.saturated && sign(error_deg) == sign(candidate.effort)
        return merge(command(integral), (integral=integral,))
    end
    merge(candidate, (integral=proposed_integral,))
end

# target_deg 是以 IMU +X 轴竖直向下为 0° 的有符号 Z 轴弯曲角；行程参数约束总位置。
# speed_rpm 是舵机位置指令的速度上限，不是恒定转速；
# max_step_ticks 限制每轮目标变化；默认启用 ±7 圈多圈位置控制和外环 PID。
function imu_feedback!(sp, cal; target_deg::Real=10.0, seconds=nothing,
                       max_excursion_ticks::Integer=7*TICKS_PER_TURN,
                       max_step_ticks::Integer=100,
                       speed_rpm::Real=20.0, kp::Real=0.6,
                       ki::Real=0.03, kd::Real=0.08,
                       imu_filter_tau_s::Real=0.03,
                       imu_spike_margin_deg::Real=15.0,
                       imu_max_rate_deg_s::Real=1000.0,
                       imu_stale_timeout_s::Real=0.5,
                       derivative_filter_s::Real=0.1,
                       loop_pause::Real=0.05, target_function=nothing,
                       control_target_function=nothing,
                       max_error_growth_deg::Real=15.0,
                       manual_range_ticks::Integer=max_excursion_ticks,
                       manual_speed_rpm::Real=8.0,
                       show_command::Bool=false, show_window::Bool=true)
    isfinite(target_deg) || error("闭环目标角度必须是有限数")
    max_excursion_ticks > 0 || error("总行程参数必须大于零")
    max_step_ticks > 0 && isfinite(kp) && kp > 0 || error("闭环步长和增益必须大于零")
    isfinite(ki) && ki >= 0 && isfinite(kd) && kd >= 0 || error("PID 的 ki、kd 必须为非负有限数")
    isfinite(derivative_filter_s) && derivative_filter_s > 0 || error("角速度滤波时间常数必须大于零")
    isfinite(imu_filter_tau_s) && imu_filter_tau_s >= 0 || error("IMU 滤波时间常数不能为负")
    isfinite(imu_spike_margin_deg) && imu_spike_margin_deg > 0 || error("IMU 尖峰余量必须大于零")
    isfinite(imu_max_rate_deg_s) && imu_max_rate_deg_s > 0 || error("IMU 最大角速度必须大于零")
    isfinite(imu_stale_timeout_s) && imu_stale_timeout_s > 0 || error("IMU 连续异常超时必须大于零")
    isfinite(speed_rpm) && 0 < speed_rpm <= 113 || error("速度必须在舵机协议支持的 0～113 RPM 内")
    isfinite(loop_pause) && loop_pause >= 0 || error("闭环等待间隔不能为负")
    isfinite(max_error_growth_deg) && max_error_growth_deg > 0 ||
        error("误差增长停机阈值必须大于零")
    32 <= manual_range_ticks <= 32767 || error("手动滑轨范围须在 32～32767 ticks")
    isfinite(manual_speed_rpm) && 0 < manual_speed_rpm <= 113 ||
        error("手动回位速度须在 0～113 RPM 内")
    hasproperty(cal, :measurement) && cal.measurement == :gravity_z ||
        error("当前标定不适用于绝对姿态角；请重新运行 calibrate_imu_servo!")
    hasproperty(cal, :quaternion_convention) || error("标定缺少四元数坐标约定")
    hasproperty(cal, :world_down_sign) || error("标定缺少世界坐标竖直轴约定")
    isfinite(cal.deg_per_tick) && cal.deg_per_tick != 0 || error("IMU 标定增益无效")
    seconds === nothing || println("seconds 参数已忽略；本次控制仅由窗口按钮结束。")

    const_idle, const_running, const_stopping, const_emergency, const_manual, const_arming = 0, 1, 2, 3, 4, 5
    state = Threads.Atomic{Int}(const_idle)
    worker = Ref{Union{Nothing,Task}}(nothing)
    emergency_task = Ref{Union{Nothing,Task}}(nothing)
    emergency_confirmed = Ref(false)
    manual_base = Ref(0)
    manual_desired = Ref(0)
    slider_sync = Ref(false)
    active_target = Ref(Float64(target_deg))
    plot = make_imu_feedback_plot(; show_command, show_window=false,
                                  initial_target_deg=target_deg,
                                  manual_range_ticks=manual_range_ticks)

    function emergency_stop!(reason="用户按下紧急停止")
        was_emergency = state[] == const_emergency
        state[] = const_emergency
        emergency_confirmed[] = false
        plot.status[] = "紧急停止：正在关闭舵机扭矩"
        if !was_emergency || emergency_task[] === nothing || istaskdone(emergency_task[])
            emergency_task[] = @async begin
                try
                    send_line!(sp, "T,$(cal.servo_id),0")
                    ack = wait_line!(sp, x -> startswith(x, "ACK,T,$(cal.servo_id),0") ||
                                              startswith(x, "ERR,"); timeout=0.5)
                    startswith(ack, "ACK,T,$(cal.servo_id),0") || error("舵机未确认关闭扭矩：$(ack)")
                    if state[] == const_emergency
                        emergency_confirmed[] = true
                        plot.status[] = "紧急停止：扭矩已关闭；可点击手动回位；$(reason)"
                    end
                catch err
                    plot.status[] = "紧急停止请求未确认；请切断舵机电源"
                    @warn "紧急停止命令未确认" exception=err
                end
            end
        end
        nothing
    end

    function run_control!()
        try
            plot.status[] = "准备控制：检查舵机与多圈模式"
            servo_mode!(sp, cal.servo_id) in (0, 4) || error("舵机不是位置模式")
            state[] == const_running || return
            enable_servo_multiturn!(sp, cal.servo_id)
            state[] == const_running || return
            position = servo_position!(sp, cal.servo_id)
            abs(position - cal.origin) <= max_excursion_ticks ||
                error("舵机已超出标定起点允许范围")
            lower_tick = max(cal.origin - max_excursion_ticks, -32767)
            upper_tick = min(cal.origin + max_excursion_ticks, 32767)
            started = time()
            last_limit_notice = -Inf
            integral = 0.0
            previous_measured = nothing
            previous_elapsed = nothing
            filtered_rate = 0.0
            previous_wrapped_z = nothing
            unwrapped_z = 0.0
            imu_filter = IMUQuaternionFilter()
            initial_abs_error = nothing
            plot.status[] = "正在控制；结束按钮保持位置，紧急停止关闭扭矩"
            while state[] == const_running
                q_raw = fresh_imu!(sp, cal.imu_id)
                state[] == const_running || break
                q, accepted = filter_imu_quaternion!(imu_filter, q_raw, time();
                    tau_s=imu_filter_tau_s, spike_margin_deg=imu_spike_margin_deg,
                    max_rate_deg_s=imu_max_rate_deg_s,
                    stale_timeout_s=imu_stale_timeout_s)
                if !accepted
                    plot.status[] = "已剔除 IMU 姿态尖峰 $(imu_filter.rejected) 次；本轮未发送舵机目标"
                    sleep(loop_pause)
                    continue
                end
                wrapped_z = imu_z_bend_deg(q; convention=cal.quaternion_convention,
                                            world_down_sign=cal.world_down_sign)
                unwrapped_z = previous_wrapped_z === nothing ? wrapped_z :
                    unwrapped_z + mod(wrapped_z - previous_wrapped_z + 180, 360) - 180
                previous_wrapped_z = wrapped_z
                measured = unwrapped_z
                elapsed = time() - started
                desired = target_function === nothing ? active_target[] :
                          target_function(elapsed) + active_target[] - target_deg
                isfinite(desired) || error("目标角度不是有限数")
                error_deg = desired - measured
                initial_abs_error === nothing && (initial_abs_error = abs(error_deg))
                if abs(error_deg) > initial_abs_error + max_error_growth_deg
                    emergency_stop!("误差相对启动时扩大超过 $(max_error_growth_deg)°")
                    break
                end
                control_desired = control_target_function === nothing ? desired :
                                  control_target_function(elapsed) + active_target[] - target_deg
                isfinite(control_desired) || error("控制角度不是有限数")
                dt = previous_elapsed === nothing ? 0.0 : max(elapsed - previous_elapsed, 0.0)
                if previous_measured !== nothing && dt > 0
                    raw_rate = (measured - previous_measured) / dt
                    alpha = exp(-dt / derivative_filter_s)
                    filtered_rate = alpha * filtered_rate + (1 - alpha) * raw_rate
                end
                previous_measured = measured
                previous_elapsed = elapsed
                position = servo_position!(sp, cal.servo_id)
                state[] == const_running || break
                result = pid_position_step(control_desired - measured, filtered_rate,
                    dt, integral, cal.deg_per_tick, position, lower_tick, upper_tick,
                    max_step_ticks, kp, ki, kd)
                integral = result.integral
                target = result.target
                if target != result.requested && elapsed - last_limit_notice >= 1.0
                    println("达到 Julia 软件行程边界 [$(lower_tick), $(upper_tick)] ticks；原目标=$(result.requested)，已裁剪为 $(target)。")
                    last_limit_notice = elapsed
                end
                plot.update(elapsed, desired, control_desired, measured, error_deg)
                println("Z轴期望=$(round(desired; digits=2))°，IMU Z轴=$(round(measured; digits=2))°，误差=$(round(error_deg; digits=2))°，舵机=$(position)，目标=$(target)")
                if state[] == const_running && target != position &&
                   (abs(control_desired - measured) > 0.25 || ki > 0)
                    sent = lock(USB_WRITE_LOCK) do
                        state[] == const_running || return false
                        send_line!(sp, "P,$(cal.servo_id),$(target),$(speed_rpm)")
                        true
                    end
                    if sent
                        ack = wait_line!(sp, x -> startswith(x, "ACK,P,$(cal.servo_id),") ||
                                                  startswith(x, "ERR,"); timeout=0.3)
                        startswith(ack, "ERR,") && error("闭环指令失败：$(ack)")
                    end
                end
                sleep(loop_pause)
            end
        catch err
            emergency_stop!("控制出错")
            @warn "反馈控制出错，已请求关闭舵机扭矩" exception=err
        finally
            if state[] == const_stopping
                try
                    position = servo_position!(sp, cal.servo_id; timeout=0.3)
                    if state[] == const_stopping
                        sent = lock(USB_WRITE_LOCK) do
                            state[] == const_stopping || return false
                            send_line!(sp, "P,$(cal.servo_id),$(position),$(speed_rpm)")
                            true
                        end
                        if sent
                            ack = wait_line!(sp, x -> startswith(x, "ACK,P,$(cal.servo_id),") ||
                                                      startswith(x, "ERR,"); timeout=0.3)
                            startswith(ack, "ERR,") && error("保持位置失败：$(ack)")
                            if state[] == const_stopping
                                state[] = const_idle
                                plot.status[] = "已结束控制；舵机保持当前位置"
                            end
                        end
                    end
                catch err
                    emergency_stop!("无法确认保持当前位置")
                    @warn "普通结束失败，已请求紧急停止" exception=err
                end
            end
        end
        nothing
    end

    GM.on(plot.read_angle_button.clicks) do _
        if state[] in (const_running, const_stopping, const_arming)
            plot.status[] = "控制或切换状态中；请先结束控制再单独读取角度"
            return
        end
        @async begin
            try
                q = fresh_imu!(sp, cal.imu_id)
                angle = imu_z_bend_deg(q; convention=cal.quaternion_convention,
                                        world_down_sign=cal.world_down_sign)
                gravity = imu_gravity_body(q; convention=cal.quaternion_convention,
                                            world_down_sign=cal.world_down_sign)
                x_tilt = acosd(clamp(gravity[1], -1.0, 1.0))
                other_convention = cal.quaternion_convention == :sensor_to_world ?
                                   :world_to_sensor : :sensor_to_world
                alternate = try
                    imu_z_bend_deg(q; convention=other_convention,
                                   world_down_sign=cal.world_down_sign)
                catch
                    NaN
                end
                plot.angle_readout[] = "IMU$(cal.imu_id) 绕 Z 轴：$(round(angle; digits=2))°；X 轴偏离竖直向下：$(round(x_tilt; digits=2))°；反向四元数约定：$(round(alternate; digits=2))°"
                plot.angle_diagnostic[] = "q(w,x,y,z)=$(round.(q; digits=4))；重力在 IMU 坐标=$(round.(gravity; digits=4))"
                println(plot.angle_readout[])
                println(plot.angle_diagnostic[])
            catch err
                plot.angle_readout[] = "读取当前角度失败：$(sprint(showerror, err))"
                @warn "读取当前角度失败" exception=err
            end
        end
    end

    GM.on(plot.manual_button.clicks) do _
        if state[] != const_emergency || !emergency_confirmed[]
            plot.status[] = "先点击紧急停止，确认扭矩已关闭后才能启用手动回位"
            return
        end
        state[] = const_arming
        plot.status[] = "正在准备手动回位：读取舵机位置并设定保持目标"
        @async begin
            try
                position = servo_position!(sp, cal.servo_id; timeout=0.5)
                state[] == const_arming || return
                send_line!(sp, "P,$(cal.servo_id),$(position),$(manual_speed_rpm)")
                ack = wait_line!(sp, x -> startswith(x, "ACK,P,$(cal.servo_id),") ||
                                          startswith(x, "ERR,"); timeout=0.5)
                startswith(ack, "ACK,P,") || error("手动回位设定保持位置失败：$(ack)")
                sent = lock(USB_WRITE_LOCK) do
                    state[] == const_arming || return false
                    send_line!(sp, "T,$(cal.servo_id),1")
                    true
                end
                sent || return
                ack = wait_line!(sp, x -> startswith(x, "ACK,T,$(cal.servo_id),1") ||
                                          startswith(x, "ERR,"); timeout=0.5)
                startswith(ack, "ACK,T,$(cal.servo_id),1") || error("手动回位开启扭矩失败：$(ack)")
                state[] == const_arming || return
                manual_base[] = position
                manual_desired[] = position
                slider_sync[] = true
                try
                    GM.set_close_to!(plot.manual_slider, 0)
                finally
                    slider_sync[] = false
                end
                state[] = const_manual
                plot.status[] = "手动回位已启用：滑轨中间为当前编码器位置，两侧为正反方向；速度 $(manual_speed_rpm) RPM"
                @async begin
                    last_sent = position
                    try
                        while state[] == const_manual
                            target = manual_desired[]
                            if target != last_sent
                                sent = lock(USB_WRITE_LOCK) do
                                    state[] == const_manual || return false
                                    send_line!(sp, "P,$(cal.servo_id),$(target),$(manual_speed_rpm)")
                                    true
                                end
                                sent || break
                                ack = wait_line!(sp, x -> startswith(x, "ACK,P,$(cal.servo_id),") ||
                                                          startswith(x, "ERR,"); timeout=0.5)
                                startswith(ack, "ACK,P,") || error("手动回位指令失败：$(ack)")
                                last_sent = target
                                plot.status[] = "手动回位：目标 $(target) ticks；按紧急停止可关闭扭矩"
                            end
                            sleep(0.05)
                        end
                    catch err
                        emergency_stop!("手动回位出错")
                        @warn "手动回位出错" exception=err
                    end
                end
            catch err
                emergency_stop!("启用手动回位失败")
                @warn "启用手动回位失败" exception=err
            end
        end
    end

    function set_manual_offset!(value)
        slider_sync[] && return
        if state[] != const_manual
            value != 0 && (plot.status[] = "滑轨仅在紧急停止并启用手动回位后生效")
            return
        end
        lower = max(-32767, cal.origin - max_excursion_ticks)
        upper = min(32767, cal.origin + max_excursion_ticks)
        manual_desired[] = clamp(manual_base[] + Int(value), lower, upper)
    end
    GM.on(plot.manual_slider.value) do value
        set_manual_offset!(value)
    end

    GM.on(plot.start_button.clicks) do _
        if state[] == const_idle
            entered = tryparse(Float64, strip(plot.target_box.displayed_string[]))
            if entered === nothing || !isfinite(entered)
                plot.status[] = "请输入有效的期望角度，单位 deg"
                return
            end
            active_target[] = entered
            state[] = const_running
            worker[] = @async run_control!()
        end
    end
    GM.on(plot.stop_button.clicks) do _
        if state[] == const_running
            state[] = const_stopping
            plot.status[] = "正在结束：保持当前位置"
        end
    end
    GM.on(plot.emergency_button.clicks) do _
        emergency_stop!()
    end
    GM.on(GM.events(plot.figure.scene).window_open) do open
        open || emergency_stop!("控制窗口已关闭")
    end
    show_window && display(plot.figure)
    (figure=plot.figure, state=state, worker=worker,
     target_box=plot.target_box, read_angle_button=plot.read_angle_button,
     angle_readout=plot.angle_readout, angle_diagnostic=plot.angle_diagnostic,
     manual_button=plot.manual_button,
     manual_slider=plot.manual_slider,
     start_button=plot.start_button, stop_button=plot.stop_button,
     emergency_button=plot.emergency_button, emergency_stop=emergency_stop!,
     status=plot.status, time_s=plot.time_s, target_deg=plot.target_deg,
     command_deg=plot.command_deg, measured_deg=plot.measured_deg,
     error_deg=plot.error_deg)
end

# 正弦目标 + 同一套开始、结束、紧急停止按钮。
function imu_sine_feedback!(sp, cal; offset_deg::Real=0.0,
                            amplitude_deg::Real=10.0, period_s::Real=8.0,
                            phase_lead_s::Real=0.0,
                            kwargs...)
    isfinite(offset_deg) && isfinite(amplitude_deg) && amplitude_deg >= 0 ||
        error("正弦中心角和幅值必须有效，幅值不得为负")
    isfinite(period_s) && period_s > 0 || error("正弦周期必须大于零")
    isfinite(phase_lead_s) && 0 <= phase_lead_s <= period_s / 4 ||
        error("提前控制时间须在 0～四分之一周期内")
    profile(t) = offset_deg + amplitude_deg * sinpi(2t / period_s)
    imu_feedback!(sp, cal; target_deg=offset_deg,
                  target_function=profile,
                  control_target_function=t -> profile(t + phase_lead_s),
                  show_command=true, kwargs...)
end

function make_pcc_reference(; shape::Symbol=:circle,
                             approach_seconds::Real=15.0,
                             transition_pause_s::Real=2.0,
                             trajectory_seconds::Real=60.0,
                             start_slow_s::Real=2.0,
                             bend_sides=nothing)
    if !isdefined(Main, :compute_trajectory)
        include(joinpath(@__DIR__, "..", "two_segment_pcc_path_tracking.jl"))
    end
    if bend_sides === nothing
        config_path = joinpath(@__DIR__, "..", "two_segment_hardware_calibration_v3.toml")
        saved = TOML.parsefile(config_path)
        bend_sides = Tuple(Int.(saved["calibration"]["bend_sides"]))
    end
    bend_sides isa NTuple{2,Int} && all(x -> x in (-1, 1), bend_sides) ||
        error("弯曲侧必须是两个 ±1 符号")
    shape in (:circle, :rectangle) || error("轨迹形状须为 :circle 或 :rectangle")
    all(isfinite, (approach_seconds, trajectory_seconds, transition_pause_s, start_slow_s)) &&
        approach_seconds > 0 && trajectory_seconds > 0 && transition_pause_s > 0 &&
        0 < start_slow_s < trajectory_seconds || error("轨迹时长参数无效")
    state = Base.invokelatest(compute_trajectory; shape, bend_sides)
    n = length(state.trajectory)
    approach_steps = state.shape_start_index - 1
    shape_samples = n - approach_steps
    # 按平滑时间律反求采样点时间；矩形每边单独启停。
    edges = shape == :rectangle ? 4 : 1
    per_edge = shape_samples ÷ edges
    edge_duration = Float64(trajectory_seconds)/edges
    function phase_time(phase)
        phase <= 0 && return 0.0
        phase >= 1 && return edge_duration
        lo, hi = 0.0, edge_duration
        for _ in 1:50
            mid = (lo+hi)/2
            if pcc_phase_profile(mid, edge_duration, start_slow_s).phase < phase
                lo = mid
            else
                hi = mid
            end
        end
        (lo+hi)/2
    end
    shape_times = [(edge-1)*edge_duration + phase_time((i-1)/(per_edge-1))
                   for edge in 1:edges for i in 1:per_edge]
    times = vcat(collect(range(0.0, Float64(approach_seconds); length=approach_steps+1)),
                 [Float64(approach_seconds + transition_pause_s)],
                 Float64(approach_seconds + transition_pause_s) .+
                 shape_times[2:end])
    bend = [bend_sides[i] .* rad2deg.(state.rope_drive_history[i, :] ./ R_DRIVE)
            for i in 1:2]
    axial = [copy(vec(state.axial_rotation_history[i, :])) for i in 1:2]
    if shape == :rectangle
        # 相邻边共享角点；复用同一关节解，消除重复 IK 求解的微小位置跳变。
        for edge in 2:4, values in (bend..., axial...)
            k = state.shape_start_index + (edge-1)*per_edge
            values[k] = values[k-1]
        end
    end
    at = pcc_reference_evaluator(bend, axial, state.shape_start_index, shape;
        approach_seconds, transition_pause_s, trajectory_seconds, start_slow_s)
    for values in (bend..., axial...)
        insert!(values, state.shape_start_index + 1, values[state.shape_start_index])
    end
    all(isfinite, reduce(vcat, bend)) && all(isfinite, reduce(vcat, axial)) ||
        error("轨迹参考包含无效角度")
    maximum(abs, reduce(vcat, axial)) <= 150 || error("轨迹轴向目标超过 ±150°")
    (; state, shape, times, bend, axial, at, duration=times[end], bend_sides,
       approach_end_s=Float64(approach_seconds),
       track_start_s=Float64(approach_seconds + transition_pause_s),
       start_slow_s=Float64(start_slow_s))
end

make_circle_reference(; kwargs...) = make_pcc_reference(; shape=:circle, kwargs...)
make_rectangle_reference(; kwargs...) = make_pcc_reference(; shape=:rectangle, kwargs...)

# 两段串联：舵机1控制第一段 IMU1；舵机2控制第二段 IMU2。
# 当前反馈角仅适用于两段轴向关节机械固定、IMU 轴对齐的平面弯曲。
# 两个标定须分别来自 servo_id=1, imu_id=1 和
# servo_id=2, imu_id=2, relative_to_imu_id=1。
function dual_imu_feedback!(sp, cal1, cal2;
                            first_target_deg::Real=0.0,
                            second_target_deg::Real=0.0,
                            max_excursion_ticks::Integer=7*TICKS_PER_TURN,
                            max_step_ticks=(800, 800),
                            speed_rpm=(20.0, 20.0),
                            kp=(0.8, 0.8), ki=(0.03, 0.03), kd=(0.08, 0.08),
                            imu_filter_tau_s::Real=0.03,
                            imu_spike_margin_deg::Real=15.0,
                            imu_max_rate_deg_s::Real=1000.0,
                            imu_stale_timeout_s::Real=0.5,
                            derivative_filter_s::Real=0.1,
                            max_error_growth_deg::Real=15.0,
                            circle_max_error_deg::Real=90.0,
                            loop_pause::Real=0.02,
                            plot_interval_s::Real=0.2,
                            manual_speed_rpm::Real=8.0,
                            axial_manual_range_pulse::Integer=500,
                            axial_manual_move_ms::Integer=250,
                            circle_reference=nothing,
                            axial_signs::NTuple{2,Int}=(1, 1),
                            deferred_plot::Bool=false,
                            auto_stop_at_reference_end::Bool=false,
                            show_window::Bool=true,
                            assume_axial_fixed::Bool=false)
    (circle_reference !== nothing || assume_axial_fixed) ||
        error("固定目标的双段弯曲角模型要求两段轴向关节固定；请显式传入 assume_axial_fixed=true")
    deferred_plot && circle_reference === nothing &&
        error("延后绘图仅用于 PCC 轨迹")
    all(x -> x in (-1, 1), axial_signs) || error("轴向指令方向须为 ±1")
    cal1.servo_id == 1 && cal1.imu_id == 1 && cal1.measurement == :gravity_z ||
        error("第一段标定须对应舵机1 / IMU1 的投影角；换线前的标定不能沿用")
    cal2.servo_id == 2 && cal2.imu_id == 2 && cal2.measurement == :planar_difference &&
        cal2.relative_to_imu_id == 1 ||
        error("第二段须重新标定为舵机2 / (IMU2 - IMU1) 的平面角差；换线前的标定不能沿用")
    cal1.quaternion_convention == cal2.quaternion_convention &&
        cal1.world_down_sign == cal2.world_down_sign ||
        error("两只 IMU 的四元数坐标约定必须一致")
    all(isfinite, (first_target_deg, second_target_deg, cal1.deg_per_tick,
                   cal2.deg_per_tick)) && cal1.deg_per_tick != 0 && cal2.deg_per_tick != 0 ||
        error("双段目标或标定增益无效")
    max_excursion_ticks > 0 && length(max_step_ticks) == 2 && all(x -> x > 0, max_step_ticks) ||
        error("双段行程和单步限制必须大于零")
    for (name, values) in (("速度", speed_rpm), ("kp", kp), ("ki", ki), ("kd", kd))
        length(values) == 2 && all(isfinite, values) || error("$(name) 须给两个有效数值")
    end
    all(x -> 0 < x <= 113, speed_rpm) || error("双段速度须在 0～113 RPM 内")
    all(x -> x > 0, kp) && all(x -> x >= 0, ki) && all(x -> x >= 0, kd) ||
        error("双段 PID 增益无效")
    isfinite(manual_speed_rpm) && 0 < manual_speed_rpm <= 113 ||
        error("手动回位速度须在 0～113 RPM 内")
    1 <= axial_manual_range_pulse <= 1000 && 20 <= axial_manual_move_ms <= 5000 ||
        error("轴向手动范围或移动时间无效")
    isfinite(imu_filter_tau_s) && imu_filter_tau_s >= 0 &&
        isfinite(imu_spike_margin_deg) && imu_spike_margin_deg > 0 &&
        isfinite(imu_max_rate_deg_s) && imu_max_rate_deg_s > 0 &&
        isfinite(imu_stale_timeout_s) && imu_stale_timeout_s > 0 &&
        isfinite(derivative_filter_s) && derivative_filter_s > 0 &&
        isfinite(max_error_growth_deg) && max_error_growth_deg > 0 &&
        isfinite(circle_max_error_deg) && circle_max_error_deg > 0 &&
        isfinite(loop_pause) && loop_pause >= 0 &&
        isfinite(plot_interval_s) && plot_interval_s > 0 ||
        error("双段滤波、采样或绘图间隔参数无效")

    trajectory_ui = circle_reference !== nothing
    circle_template = circle_reference
    rectangle_cache = Ref{Any}(nothing)
    cals = (cal1, cal2)
    idle, running, stopping, emergency, manual, arming = 0, 1, 2, 3, 4, 5
    state = Threads.Atomic{Int}(idle)
    targets = Ref([Float64(first_target_deg), Float64(second_target_deg)])
    plot = make_two_segment_feedback_plot(; first_target_deg, second_target_deg,
                                           bend_manual_range_ticks=max_excursion_ticks,
                                           axial_manual_range_pulse,
                                           circle_mode=circle_reference !== nothing,
                                           circle_state=circle_reference === nothing ? nothing : circle_reference.state,
                                           sequential_fit=deferred_plot,
                                           bend_deg_per_tick=(cal1.deg_per_tick, cal2.deg_per_tick),
                                           axial_signs,
                                           quaternion_convention=cal1.quaternion_convention,
                                           show_window=false)
    manual_bases = zeros(Int, 2)
    manual_offsets = zeros(Int, 4)
    manual_sync = Ref(false)
    emergency_confirmed = Ref(false)
    zero_pair = Ref{Any}(nothing)
    axial_pulses = [1500, 1500]
    home_mode = Ref(false)
    recorded_samples = NamedTuple[]
    last_record_time = Ref(-Inf)

    function emergency_stop!(reason="用户按下紧急停止")
        state[] = emergency
        emergency_confirmed[] = false
        plot.status[] = "紧急停止：正在关闭两台弯曲电机扭矩；轴向舵机停止接收新目标"
        @async begin
            failed = Int[]
            for id in (1, 2)
                try
                    send_line!(sp, "T,$(id),0")
                catch err
                    push!(failed, id)
                    @warn "双段紧急停止命令发送失败" servo_id=id exception=err
                end
            end
            for id in (1, 2)
                id in failed && continue
                try
                    ack = wait_line!(sp, x -> startswith(x, "ACK,T,$(id),0") ||
                                              startswith(x, "ERR,"); timeout=0.5)
                    startswith(ack, "ACK,T,$(id),0") || error(ack)
                catch err
                    push!(failed, id)
                    @warn "双段紧急停止未获确认" servo_id=id exception=err
                end
            end
            emergency_confirmed[] = isempty(failed)
            plot.status[] = isempty(failed) ? "紧急停止：两台弯曲电机扭矩已关闭；轴向舵机不再接收新目标；$(reason)" :
                "紧急停止未确认：舵机 $(failed)；请切断舵机电源"
        end
        nothing
    end

    function render_recording!()
        isempty(recorded_samples) && return
        plot.status[] = "控制已结束，正在绘制记录的跟踪结果"
        plot.set_imu_baseline!(zero_pair[])
        for sample in recorded_samples
            plot.update(sample.t, sample.target, sample.measured)
            plot.update_tip(sample.measured, sample.axial_command,
                            sample.reference_bend, sample.reference_axial,
                            sample.quaternions; fit_override=sample.fit)
            plot.update_axial(sample.t, sample.pulses)
        end
        plot.status[] = "跟踪结果已绘制；共 $(length(recorded_samples)) 个记录点"
    end

    function run_control!()
        try
            circle_reference === nothing || zero_pair[] !== nothing ||
                error("先将两段置于直态、两台轴向舵机置于 1500，再点击记录直态四元数")
            if deferred_plot
                empty!(recorded_samples)
                last_record_time[] = -Inf
                plot.clear_history!()
            end
            plot.status[] = "准备双段控制：检查两台舵机和多圈模式"
            for cal in cals
                servo_mode!(sp, cal.servo_id) in (0, 4) || error("舵机 $(cal.servo_id) 不是位置模式")
                state[] == running || return
                enable_servo_multiturn!(sp, cal.servo_id)
                state[] == running || return
            end
            lower = [max(cal.origin - max_excursion_ticks, -32767) for cal in cals]
            upper = [min(cal.origin + max_excursion_ticks, 32767) for cal in cals]
            for i in 1:2
                position = servo_position!(sp, cals[i].servo_id)
                lower[i] <= position <= upper[i] || error("舵机 $(i) 已超出标定行程")
            end
            filters = [IMUQuaternionFilter(), IMUQuaternionFilter()]
            previous_wrapped = fill(NaN, 2)
            unwrapped = zeros(2)
            previous_measured = fill(NaN, 2)
            filtered_rate = zeros(2)
            integrals = zeros(2)
            initial_abs_error = fill(NaN, 2)
            previous_elapsed = NaN
            opposite_second_since = NaN
            last_home_mode = home_mode[]
            started = time()
            last_plot_time = -Inf
            last_timing_report = started
            last_phase = :none
            cycle_count = 0
            cycle_time_sum = 0.0
            stage_time_sum = zeros(6)
            plot.status[] = home_mode[] ?
                "回中回直中：两段弯曲目标 0°，两台轴向舵机目标 1500" :
                (circle_reference === nothing ?
                 "双段 PID 正在控制；第二段反馈为 IMU2 - IMU1 平面角差" :
                 "PCC $(circle_reference.shape == :circle ? "圆形" : "矩形")轨迹跟踪中：两段弯曲按双 IMU 姿态闭环，轴向直接执行 PCC 目标角")
            (circle_reference === nothing || deferred_plot) ||
                plot.update_axial(0.0, axial_pulses)
            while state[] == running
                cycle_started = time()
                pair = fresh_imu_pair!(sp)
                state[] == running || break
                stamp = time()
                after_imu = stamp
                qs = (pair.imu1, pair.imu2)
                accepted = Bool[]
                filtered_qs = Vector{Float64}[]
                for i in 1:2
                    q, ok = filter_imu_quaternion!(filters[i], qs[i], stamp;
                        tau_s=imu_filter_tau_s, spike_margin_deg=imu_spike_margin_deg,
                        max_rate_deg_s=imu_max_rate_deg_s,
                        stale_timeout_s=imu_stale_timeout_s)
                    push!(filtered_qs, q)
                    push!(accepted, ok)
                end
                if !all(accepted)
                    plot.status[] = "已剔除双 IMU 姿态尖峰；本轮未发送电机目标"
                    sleep(loop_pause)
                    continue
                end
                if circle_reference !== nothing
                    reference_elapsed = time() - started
                    phase = reference_elapsed < circle_reference.approach_end_s ? :approach :
                            reference_elapsed < circle_reference.track_start_s ? :pause :
                            reference_elapsed < circle_reference.track_start_s +
                                                circle_reference.start_slow_s ? :slow_start : :tracking
                    if !home_mode[] && phase != last_phase
                        plot.status[] = phase == :approach ? "正在从直态平滑接近轨迹起点" :
                            phase == :pause ? "已到轨迹起点，停留 2 秒等待弯曲和轴向跟上" :
                            phase == :slow_start ? "轨迹起步阶段：目标速度逐渐提高" :
                            "正在跟踪 $(circle_reference.shape == :circle ? "圆形" : "矩形")轨迹"
                        last_phase = phase
                    end
                    sample = home_mode[] ? (bend=(0.0, 0.0), axial=(0.0, 0.0)) :
                             circle_reference.at(reference_elapsed)
                    targets[] = Float64[sample.bend...]
                end
                fit = if circle_reference === nothing
                    nothing
                else
                    render_now = !deferred_plot &&
                        time() - last_plot_time >= plot_interval_s
                    axial_actual = ntuple(i -> axial_signs[i] *
                        (axial_pulses[i]-1500) * 300 / 2000, 2)
                    result = plot.update_tip((0.0, 0.0), axial_actual,
                        sample.bend, sample.axial, (filtered_qs[1], filtered_qs[2]);
                        render=render_now)
                    if result === nothing
                        plot.status[] = "已剔除 PCC 姿态拟合突变 $(plot.rejected_fit_samples[]) 帧；本轮不绘图、不发送电机目标"
                        sleep(loop_pause)
                        continue
                    end
                    if maximum(result.mismatch_deg) > 30.0
                        plot.readout[] = "IMU 与 PCC 姿态重力方向残差 $(round.(result.mismatch_deg; digits=1))° 超过 30°；轨迹尚未完成，已保护停止并保持弯曲电机当前位置"
                        plot.status[] = "PCC 姿态拟合不可靠，正在正常结束并保持位置"
                        state[] = stopping
                        break
                    end
                    result
                end
                wrapped_angles = circle_reference === nothing ? (
                    imu_z_bend_deg(filtered_qs[1];
                        convention=cal1.quaternion_convention,
                        world_down_sign=cal1.world_down_sign),
                    imu_planar_difference_deg(filtered_qs[1], filtered_qs[2];
                        convention=cal1.quaternion_convention,
                        world_down_sign=cal1.world_down_sign)) :
                    fit.bend
                for i in 1:2
                    wrapped = wrapped_angles[i]
                    unwrapped[i] = isnan(previous_wrapped[i]) ? wrapped :
                        unwrapped[i] + mod(wrapped - previous_wrapped[i] + 180, 360) - 180
                    previous_wrapped[i] = wrapped
                end
                measured = copy(unwrapped)
                elapsed = time() - started
                if home_mode[] != last_home_mode
                    fill!(integrals, 0.0)
                    fill!(filtered_rate, 0.0)
                    fill!(previous_measured, NaN)
                    previous_elapsed = NaN
                    opposite_second_since = NaN
                    last_home_mode = home_mode[]
                    plot.status[] = "回中回直中：两段弯曲目标 0°，两台轴向舵机目标 1500"
                end
                errors = targets[] .- measured
                for i in 1:2
                    isnan(initial_abs_error[i]) && (initial_abs_error[i] = abs(errors[i]))
                    if circle_reference === nothing
                        abs(errors[i]) <= initial_abs_error[i] + max_error_growth_deg ||
                            error("第 $(i) 段误差相对启动时扩大超过 $(max_error_growth_deg)°")
                    else
                        abs(errors[i]) <= circle_max_error_deg ||
                            error("PCC 轨迹第 $(i) 段跟踪误差超过 $(circle_max_error_deg)°")
                    end
                end
                if circle_reference !== nothing && !home_mode[]
                    opposite_second = abs(targets[][2]) > 5.0 &&
                        abs(measured[2]) > 5.0 && abs(errors[2]) > 15.0 &&
                        targets[][2] * measured[2] < 0
                    if opposite_second
                        isnan(opposite_second_since) && (opposite_second_since = elapsed)
                        elapsed - opposite_second_since >= 0.5 &&
                            error("第二段目标与实测角持续处于相反方向，停止 PCC 轨迹控制")
                    else
                        opposite_second_since = NaN
                    end
                end
                dt = isnan(previous_elapsed) ? 0.0 : max(elapsed - previous_elapsed, 0.0)
                for i in 1:2
                    if !isnan(previous_measured[i]) && dt > 0
                        alpha = exp(-dt / derivative_filter_s)
                        filtered_rate[i] = alpha * filtered_rate[i] +
                            (1 - alpha) * (measured[i] - previous_measured[i]) / dt
                    end
                    previous_measured[i] = measured[i]
                end
                previous_elapsed = elapsed
                before_position_read = time()
                positions = [servo_position!(sp, cals[i].servo_id; verbose=false) for i in 1:2]
                state[] == running || break
                after_position_read = time()
                results = [pid_position_step(errors[i], filtered_rate[i], dt,
                    integrals[i], cals[i].deg_per_tick, positions[i], lower[i], upper[i],
                    max_step_ticks[i], kp[i], ki[i], kd[i]) for i in 1:2]
                for i in 1:2
                    integrals[i] = results[i].integral
                end
                (circle_reference === nothing || render_now) &&
                    plot.update(elapsed, targets[], measured)
                circle_reference === nothing &&
                    println("第一段=$(round(measured[1]; digits=2))° / $(targets[][1])°；第二段相对=$(round(measured[2]; digits=2))° / $(targets[][2])°")
                before_bend_send = time()
                for i in 1:2
                    target = results[i].target
                    if target != positions[i] && state[] == running
                        sent = lock(USB_WRITE_LOCK) do
                            state[] == running || return false
                            send_line!(sp, "P,$(cals[i].servo_id),$(target),$(speed_rpm[i])")
                            true
                        end
                        sent || break
                        ack = wait_line!(sp, x -> startswith(x, "ACK,P,$(cals[i].servo_id),") ||
                                                  startswith(x, "ERR,"); timeout=0.3)
                        startswith(ack, "ACK,P,") || error("第 $(i) 段位置指令失败：$(ack)")
                    end
                end
                after_bend_send = time()
                if circle_reference !== nothing && state[] == running
                    sample = home_mode[] ? (bend=(0.0, 0.0), axial=(0.0, 0.0)) :
                             circle_reference.at(elapsed)
                    for i in 1:2
                        pulse = round(Int, 1500 + axial_signs[i] * sample.axial[i] * 2000 / 300)
                        500 <= pulse <= 2500 || error("第 $(i) 段轴向目标超出 500～2500")
                        if pulse != axial_pulses[i] && state[] == running
                            axial_servo_position!(sp, i, pulse;
                                                  move_ms=home_mode[] ? max(1500, axial_manual_move_ms) : axial_manual_move_ms,
                                                  timeout=0.5, verbose=false)
                            axial_pulses[i] = pulse
                        end
                    end
                    if render_now
                        plot.update_axial(elapsed, axial_pulses)
                        last_plot_time = time()
                    end
                    if deferred_plot && (elapsed - last_record_time[] >= plot_interval_s ||
                                         (auto_stop_at_reference_end && !home_mode[] &&
                                          elapsed >= circle_reference.duration))
                        axial_command = ntuple(i -> axial_signs[i] *
                            (axial_pulses[i] - 1500) * 300 / 2000, 2)
                        push!(recorded_samples,
                              (t=elapsed, target=Tuple(targets[]), measured=Tuple(measured),
                               reference_bend=sample.bend, reference_axial=sample.axial,
                               axial_command, pulses=Tuple(axial_pulses),
                               imu1_sequence=pair.imu1_sequence,
                               imu2_sequence=pair.imu2_sequence,
                               imu1_timestamp_us=pair.imu1_timestamp_us,
                               imu2_timestamp_us=pair.imu2_timestamp_us,
                               skew_us=pair.skew_us,
                               raw_quaternions=(copy(pair.imu1), copy(pair.imu2)),
                               quaternions=(copy(filtered_qs[1]), copy(filtered_qs[2])),
                               fit))
                        last_record_time[] = elapsed
                    end
                end
                if auto_stop_at_reference_end && circle_reference !== nothing &&
                   !home_mode[] && elapsed >= circle_reference.duration
                    state[] = stopping
                    plot.status[] = "$(circle_reference.shape == :circle ? "圆形" : "矩形")轨迹已完成，正在保持位置并生成结果图"
                end
                after_axial_send = time()
                stage_time_sum .+= (after_imu - cycle_started,
                                    before_position_read - after_imu,
                                    after_position_read - before_position_read,
                                    before_bend_send - after_position_read,
                                    after_bend_send - before_bend_send,
                                    after_axial_send - after_bend_send)
                cycle_count += 1
                cycle_time_sum += time() - cycle_started
                if time() - last_timing_report >= 2.0
                    parts_ms = round.(1000 .* stage_time_sum ./ cycle_count; digits=1)
                    println("控制循环平均耗时=$(round(1000*cycle_time_sum/cycle_count; digits=1)) ms；IMU=$(parts_ms[1])，拟合/三维图=$(parts_ms[2])，位置查询=$(parts_ms[3])，PID/二维图=$(parts_ms[4])，弯曲指令=$(parts_ms[5])，轴向指令=$(parts_ms[6]) ms；样本数=$(cycle_count)；绘图间隔=$(plot_interval_s) s")
                    cycle_count = 0
                    cycle_time_sum = 0.0
                    fill!(stage_time_sum, 0.0)
                    last_timing_report = time()
                end
                sleep(loop_pause)
            end
        catch err
            trace = catch_backtrace()
            detail = sprint(showerror, err)
            plot.readout[] = "双段控制故障：$(detail)"
            emergency_stop!("双段控制出错：$(detail)")
            println(stderr, "双段控制触发紧急停止：$(detail)")
            @warn "双段反馈控制出错，已请求关闭两台舵机扭矩" exception=(err, trace)
        finally
            if state[] == stopping
                try
                    for (i, cal) in enumerate(cals)
                        position = servo_position!(sp, cal.servo_id; timeout=0.3)
                        state[] == stopping || break
                        sent = lock(USB_WRITE_LOCK) do
                            state[] == stopping || return false
                            send_line!(sp, "P,$(cal.servo_id),$(position),$(speed_rpm[i])")
                            true
                        end
                        sent || break
                        ack = wait_line!(sp, x -> startswith(x, "ACK,P,$(cal.servo_id),") ||
                                                  startswith(x, "ERR,"); timeout=0.3)
                        startswith(ack, "ACK,P,") || error("保持舵机 $(cal.servo_id) 位置失败")
                    end
                    if state[] == stopping
                        state[] = idle
                        plot.status[] = "双段控制已结束；两台舵机保持当前位置"
                    end
                catch err
                    detail = sprint(showerror, err)
                    plot.readout[] = "双段结束故障：$(detail)"
                    emergency_stop!("双段保持位置失败：$(detail)")
                    println(stderr, "双段结束触发紧急停止：$(detail)")
                    @warn "双段结束控制失败" exception=err
                end
            end
            if deferred_plot && state[] != emergency && !isempty(recorded_samples)
                try
                    render_recording!()
                catch err
                    @warn "离线绘图失败，控制已经结束" exception=err
                    plot.status[] = "控制已结束；离线绘图失败：$(sprint(showerror, err))"
                end
            end
        end
        nothing
    end

    GM.on(plot.read_button.clicks) do _
        if state[] == idle || state[] == emergency
            @async begin
                try
                    if circle_reference === nothing || zero_pair[] === nothing
                        angles = read_two_segment_angles!(sp;
                            quaternion_convention=cal1.quaternion_convention,
                            world_down_sign=cal1.world_down_sign)
                        plot.readout[] = "第一段 $(round(angles.first_deg; digits=2))°；第二段反馈平面角差 $(round(angles.planar_difference_deg; digits=2))°；四元数 Z twist 仅供诊断 $(round(angles.second_relative_deg; digits=2))°；IMU 时间差 $(angles.skew_us) μs"
                    else
                        pair = fresh_imu_pair!(sp)
                        axial = ntuple(i -> axial_signs[i] *
                            (axial_pulses[i]-1500) * 300 / 2000, 2)
                        bends = two_segment_mounted_bends(pair.imu1, pair.imu2,
                            zero_pair[].imu1, zero_pair[].imu2, axial;
                            convention=cal1.quaternion_convention)
                        x1 = imu_axis_world(pair.imu1, [1.0, 0.0, 0.0];
                            convention=cal1.quaternion_convention)
                        x2 = imu_axis_world(pair.imu2, [1.0, 0.0, 0.0];
                            convention=cal1.quaternion_convention)
                        gap = rad2deg(acos(clamp(dot(x1, x2), -1.0, 1.0)))
                        plot.readout[] = "有效弯曲角：第一段 $(round(bends[1]; digits=2))°；第二段 $(round(bends[2]; digits=2))°；两 IMU 末端方向夹角 $(round(gap; digits=2))°；轴向指令角 $(round.(axial; digits=2))°"
                    end
                catch err
                    plot.readout[] = "双 IMU 读取失败：$(sprint(showerror, err))"
                end
            end
        end
    end
    GM.on(plot.capture_button.clicks) do _
        !trajectory_ui && begin
            plot.status[] = "固定目标控制无需记录 PCC 直态"
            return
        end
        (state[] == idle || (state[] == emergency && emergency_confirmed[])) || begin
            plot.status[] = "请先结束或紧急停止当前控制，再记录直态"
            return
        end
        @async begin
            try
                zero_pair[] = nothing
                check_circle_straight_pose(fresh_imu_pair!(sp);
                    convention=cal1.quaternion_convention,
                    world_down_sign=cal1.world_down_sign)
                plot.status[] = "两段接近直态；正在将两台轴向舵机送到 1500 并记录四元数"
                for i in 1:2
                    axial_servo_position!(sp, i, 1500; move_ms=1000, timeout=0.5)
                end
                sleep(1.1)
                pair = fresh_imu_pair!(sp)
                check_circle_straight_pose(pair;
                    convention=cal1.quaternion_convention,
                    world_down_sign=cal1.world_down_sign)
                plot.set_imu_baseline!(pair)
                zero_pair[] = pair
                axial_pulses .= 1500
                state[] = idle
                plot.status[] = "两台轴向舵机已发送 1500，直态四元数已记录；可跟踪圆形或矩形轨迹"
            catch err
                plot.status[] = "记录直态失败：$(sprint(showerror, err))"
            end
        end
    end
    GM.on(plot.manual_button.clicks) do _
        (state[] == emergency && emergency_confirmed[]) || begin
            plot.status[] = "先按紧急停止，确认两台弯曲电机扭矩已关闭，再启用手动回位"
            return
        end
        state[] = arming
        plot.status[] = "正在读取两台弯曲电机的位置并启用手动回位"
        @async begin
            try
                for i in 1:2
                    cal = cals[i]
                    position = servo_position!(sp, cal.servo_id; timeout=0.5)
                    state[] == arming || return
                    manual_bases[i] = position
                    send_line!(sp, "P,$(cal.servo_id),$(position),$(manual_speed_rpm)")
                    ack = wait_line!(sp, x -> startswith(x, "ACK,P,$(cal.servo_id),") ||
                                              startswith(x, "ERR,"); timeout=0.5)
                    startswith(ack, "ACK,P,") || error("手动回位保持位置失败：$(ack)")
                    state[] == arming || return
                    send_line!(sp, "T,$(cal.servo_id),1")
                    ack = wait_line!(sp, x -> startswith(x, "ACK,T,$(cal.servo_id),1") ||
                                              startswith(x, "ERR,"); timeout=0.5)
                    startswith(ack, "ACK,T,") || error("手动回位开启扭矩失败：$(ack)")
                end
                manual_sync[] = true
                try
                    for slider in plot.manual_sliders
                        GM.set_close_to!(slider, 0)
                    end
                    fill!(manual_offsets, 0)
                finally
                    manual_sync[] = false
                end
                state[] == arming || return
                state[] = manual
                plot.status[] = "四电机手动回位：滑轨中间为弯曲电机当前位置 / 轴向脉宽 1500；轴向无位置回读"
                @async begin
                    last_offsets = zeros(Int, 4)
                    try
                        while state[] == manual
                            for i in 1:4
                                state[] == manual || break
                                offset = manual_offsets[i]
                                offset == last_offsets[i] && continue
                                if i <= 2
                                    cal = cals[i]
                                    lower = max(-32767, cal.origin - max_excursion_ticks)
                                    upper = min(32767, cal.origin + max_excursion_ticks)
                                    goal = clamp(manual_bases[i] + offset, lower, upper)
                                    send_line!(sp, "P,$(cal.servo_id),$(goal),$(manual_speed_rpm)")
                                    ack = wait_line!(sp, x -> startswith(x, "ACK,P,$(cal.servo_id),") ||
                                                              startswith(x, "ERR,"); timeout=0.5)
                                    startswith(ack, "ACK,P,") || error("弯曲电机手动指令失败：$(ack)")
                                else
                                    section = i - 2
                                    axial_servo_position!(sp, section, 1500 + offset;
                                                          move_ms=axial_manual_move_ms,
                                                          timeout=0.5)
                                end
                                last_offsets[i] = offset
                                plot.status[] = "手动回位：第 $(i) 条滑轨偏移 $(offset)；紧急停止可关闭弯曲电机扭矩"
                            end
                            sleep(0.05)
                        end
                    catch err
                        emergency_stop!("四电机手动回位出错")
                        @warn "四电机手动回位出错" exception=err
                    end
                end
            catch err
                emergency_stop!("启用四电机手动回位失败")
                @warn "启用四电机手动回位失败" exception=err
            end
        end
    end
    for i in 1:4
        GM.on(plot.manual_sliders[i].value) do value
            manual_sync[] && return
            if state[] == manual
                manual_offsets[i] = Int(value)
            elseif value != 0
                plot.status[] = "滑轨仅在紧急停止并启用手动回位后生效"
            end
        end
    end
    function start_fixed_angles!(a::Real, b::Real)
        state[] == idle || return
        isfinite(a) && isfinite(b) || begin
            plot.status[] = "两段目标都须为有效角度，单位 deg"
            return
        end
        state[] = arming
        @async begin
            try
                if trajectory_ui
                    plot.status[] = "轴向回中 1500；随后按双段 IMU 角度反馈控制"
                    for i in 1:2
                        state[] == arming || return
                        axial_servo_position!(sp, i, 1500; move_ms=1000, timeout=0.5)
                    end
                    sleep(1.1)
                    axial_pulses .= 1500
                    plot.set_angle_mode!(true)
                    plot.clear_history!()
                end
                state[] == arming || return
                circle_reference = nothing
                home_mode[] = false
                targets[] = [Float64(a), Float64(b)]
                state[] = running
                @async run_control!()
            catch err
                state[] == arming && (state[] = idle)
                plot.status[] = "角度控制未启动：$(sprint(showerror, err))"
            end
        end
    end
    function start_trajectory!(shape::Symbol)
        state[] == idle || return
        zero_pair[] !== nothing || begin
            plot.status[] = "轨迹跟踪前须在两段直态、轴向 1500 时记录直态四元数"
            return
        end
        state[] = arming
        @async begin
            try
                reference = if shape == :circle
                    circle_template
                else
                    if rectangle_cache[] === nothing
                        plot.status[] = "正在生成矩形 PCC 参考轨迹"
                        rectangle_cache[] = make_rectangle_reference(;
                            bend_sides=circle_template.bend_sides,
                            approach_seconds=circle_template.approach_end_s,
                            transition_pause_s=circle_template.track_start_s -
                                               circle_template.approach_end_s,
                            trajectory_seconds=circle_template.duration -
                                               circle_template.track_start_s,
                            start_slow_s=circle_template.start_slow_s)
                    end
                    rectangle_cache[]
                end
                pair = fresh_imu_pair!(sp)
                check_circle_straight_pose(pair;
                    convention=cal1.quaternion_convention,
                    world_down_sign=cal1.world_down_sign)
                separations = (imu_orientation_distance_deg(pair.imu1, zero_pair[].imu1),
                               imu_orientation_distance_deg(pair.imu2, zero_pair[].imu2))
                maximum(separations) <= 10.0 ||
                    error("当前姿态已偏离记录的直态 $(round.(separations; digits=1))°；请重新回中并记录直态")
                state[] == arming || return
                circle_reference = reference
                plot.set_reference!(reference.state)
                plot.set_angle_mode!(false)
                plot.clear_history!()
                plot.set_imu_baseline!(zero_pair[])
                home_mode[] = false
                state[] = running
                @async run_control!()
            catch err
                state[] == arming && (state[] = idle)
                plot.status[] = "$(shape == :circle ? "圆形" : "矩形")轨迹未启动：$(sprint(showerror, err))"
            end
        end
    end
    GM.on(plot.start_button.clicks) do _
        if trajectory_ui
            start_trajectory!(:circle)
        else
            a = tryparse(Float64, strip(plot.target1_box.displayed_string[]))
            b = tryparse(Float64, strip(plot.target2_box.displayed_string[]))
            (a === nothing || b === nothing) ?
                (plot.status[] = "两段目标都须为有效角度，单位 deg") :
                start_fixed_angles!(a, b)
        end
    end
    if trajectory_ui
        GM.on(plot.rectangle_button.clicks) do _
            start_trajectory!(:rectangle)
        end
        GM.on(plot.angle_button.clicks) do _
            a = tryparse(Float64, strip(plot.target1_box.displayed_string[]))
            b = tryparse(Float64, strip(plot.target2_box.displayed_string[]))
            (a === nothing || b === nothing) ?
                (plot.status[] = "两段目标都须为有效角度，单位 deg") :
                start_fixed_angles!(a, b)
        end
    end
    GM.on(plot.home_button.clicks) do _
        if state[] == emergency || state[] == manual || state[] == arming
            plot.status[] = "当前处于紧急停止或手动回位状态；请先完成手动复位并重新记录直态"
            return
        end
        if state[] == stopping
            plot.status[] = "正在结束控制；请等待结束后再点击回中回直"
            return
        end
        if state[] == idle && trajectory_ui
            start_fixed_angles!(0.0, 0.0)
            return
        end
        home_mode[] = true
        targets[] = [0.0, 0.0]
        plot.status[] = circle_reference === nothing ?
            "回直中：两段弯曲目标已设为 0°" :
            "回中回直中：两段弯曲目标 0°，两台轴向舵机目标 1500"
        if state[] == idle
            start_fixed_angles!(0.0, 0.0)
        end
    end
    GM.on(plot.stop_button.clicks) do _
        if state[] == running
            state[] = stopping
            plot.status[] = "正在结束双段控制：保持两台舵机当前位置"
        end
    end
    GM.on(plot.emergency_button.clicks) do _
        emergency_stop!()
    end
    GM.on(GM.events(plot.figure.scene).window_open) do open
        open || emergency_stop!("双段窗口已关闭")
    end
    show_window && display(plot.figure)
    (; figure=plot.figure, state, emergency_stop=emergency_stop!,
     target1_box=plot.target1_box, target2_box=plot.target2_box,
     read_button=plot.read_button, start_button=plot.start_button,
     rectangle_button=plot.rectangle_button, angle_button=plot.angle_button,
     home_button=plot.home_button,
     stop_button=plot.stop_button, emergency_button=plot.emergency_button,
     manual_button=plot.manual_button, manual_sliders=plot.manual_sliders,
     capture_button=plot.capture_button,
     status=plot.status, readout=plot.readout, time_s=plot.time_s,
     target_deg=plot.target_deg, measured_deg=plot.measured_deg,
     error_deg=plot.error_deg,
     diagnostic_samples=recorded_samples, imu_baseline=zero_pair)
end

function export_pcc_imu_diagnostics(result;
                                     path=joinpath(pwd(), "pcc_imu_diagnostics.csv"))
    samples = result.diagnostic_samples
    isempty(samples) && error("没有离线 PCC 轨迹诊断记录可导出")
    baseline = result.imu_baseline[]
    baseline === nothing && error("诊断记录缺少启动基线四元数")
    quaternion_columns(prefix) = ["$(prefix)_q$(axis)" for axis in ("w", "x", "y", "z")]
    header = ["time_s", "imu1_sequence", "imu2_sequence",
              "imu1_timestamp_us", "imu2_timestamp_us", "pair_skew_us",
              "pulse_ch1_us", "pulse_ch2_us",
              "target_bend1_deg", "target_bend2_deg",
              "fit_bend1_deg", "fit_bend2_deg",
              "target_axial1_deg", "target_axial2_deg",
              "pulse_axial1_deg", "pulse_axial2_deg",
              "fit_axial1_deg", "fit_axial2_deg",
              "fit_residual1_deg", "fit_residual2_deg"]
    for prefix in ("baseline_imu1", "baseline_imu2", "raw_imu1", "raw_imu2",
                   "filtered_imu1", "filtered_imu2")
        append!(header, quaternion_columns(prefix))
    end
    open(path, "w") do io
        println(io, join(header, ','))
        for sample in samples
            row = Any[sample.t, sample.imu1_sequence, sample.imu2_sequence,
                      sample.imu1_timestamp_us, sample.imu2_timestamp_us,
                      sample.skew_us, sample.pulses...,
                      sample.reference_bend..., sample.fit.bend...,
                      sample.reference_axial..., sample.axial_command...,
                      sample.fit.axial..., sample.fit.mismatch_deg...]
            for q in (baseline.imu1, baseline.imu2,
                      sample.raw_quaternions..., sample.quaternions...)
                append!(row, q)
            end
            println(io, join(string.(row), ','))
        end
    end
    path
end

function pcc_circle_feedback!(sp, first_calibration, second_calibration;
                              circle_reference=make_circle_reference(),
                              axial_signs::NTuple{2,Int}=(1, 1), kwargs...)
    dual_imu_feedback!(sp, first_calibration, second_calibration;
                       circle_reference, axial_signs, kwargs...)
end

# 圆轨迹控制期间只记录采样，不更新 Makie 曲线；到轨迹终点或点击结束后统一绘图。
# 保留原 pcc_circle_feedback! 的实时显示行为。
function pcc_circle_feedback_offline!(sp, first_calibration, second_calibration;
                                      circle_reference=make_circle_reference(),
                                      axial_signs::NTuple{2,Int}=(1, 1), kwargs...)
    dual_imu_feedback!(sp, first_calibration, second_calibration;
                       circle_reference, axial_signs,
                       deferred_plot=true, auto_stop_at_reference_end=true, kwargs...)
end

function move_servo_relative!(sp, delta_ticks::Integer; id::Integer=SERVO_ID,
                               speed_rpm::Real=5.0,
                               max_delta_ticks::Integer=MAX_TEST_DELTA_TICKS)
    """相对当前位置小幅移动；默认最多 200 ticks，约 17.6°。"""
    abs(delta_ticks) <= max_delta_ticks ||
        error("本次增量超过安全上限 $(max_delta_ticks) ticks；未发送运动指令")
    current = servo_position!(sp, id)
    target = current + delta_ticks
    -32767 <= target <= 32767 || error("目标位置超出舵机有符号多圈范围")
    isfinite(speed_rpm) && 0 < speed_rpm <= 113 ||
        error("速度需在 0–113 RPM 范围内")
    send_line!(sp, "P,$(id),$(target),$(speed_rpm)")
    ack = wait_line!(sp, x -> startswith(x, "ACK,P,$(id),") || startswith(x, "ERR,"); timeout=1.0)
    startswith(ack, "ERR,") && error("舵机指令失败：$(ack)")
    println("已发送：ID=$(id)，目标=$(target) ticks，速度上限=$(speed_rpm) RPM")
    ack
end
# LSC-16 轴向位置舵机：第一段在通道 1，第二段在通道 2。
# 500～2500 是控制板协议的脉宽指令范围，不保证安装后整段范围都可机械到达。
# LSC-16 的 ACK 仅表示 ESP32 已发送指令；该协议不回传轴向舵机实际位置。
function axial_servo_position!(sp, section::Integer, pulse::Integer;
                               move_ms::Integer=500, timeout::Real=1.0,
                               verbose::Bool=true)
    section in (1, 2) || error("轴向舵机段号只能是 1 或 2")
    500 <= pulse <= 2500 || error("轴向位置指令须在 500～2500 之间")
    20 <= move_ms <= 5000 || error("轴向移动时间须在 20～5000 ms 之间")
    send_line!(sp, "A,$(section),$(pulse),$(move_ms)")
    ack = wait_line!(sp, x -> startswith(x, "ACK,A,$(section),") ||
                              startswith(x, "ERR,"); timeout)
    startswith(ack, "ERR,") && error("轴向舵机指令失败：$(ack)")
    verbose && println("第 $(section) 段轴向舵机：通道 $(section)，目标 $(pulse)，移动时间 $(move_ms) ms")
    ack
end

# 手动四电机 + 双绝对 IMU 姿态拟合。启动时只读取舵机位置和 IMU；
# 初始姿态自动作为拟合零位，不向任何电机发送运动目标。拖动滑轨才驱动电机。
# 橙色臂形是与 IMU 末端姿态最匹配的 PCC 形状，不是位置传感器测得的真实臂形。
function pcc_imu_manual_fit_test!(sp;
                                  bend_sides=nothing,
                                  max_excursion_ticks::Integer=7*TICKS_PER_TURN,
                                  speed_rpm::NTuple{2,<:Real}=(20.0, 20.0),
                                  axial_move_ms::Integer=350,
                                  refresh_s::Real=0.1,
                                  quaternion_convention::Symbol=:sensor_to_world,
                                  world_down_sign::Integer=-1,
                                  show_window::Bool=true)
    if !isdefined(Main, :calculate_multi_segment_kinematics)
        include(joinpath(@__DIR__, "..", "two_segment_pcc_path_tracking.jl"))
    end
    if bend_sides === nothing
        config_path = joinpath(@__DIR__, "..", "two_segment_hardware_calibration_v3.toml")
        saved = TOML.parsefile(config_path)
        bend_sides = Tuple(Int.(saved["calibration"]["bend_sides"]))
    end
    bend_sides isa NTuple{2,Int} && all(s -> s in (-1, 1), bend_sides) ||
        error("弯曲侧须为两个 ±1 符号")
    1 <= max_excursion_ticks <= 7*TICKS_PER_TURN ||
        error("手动弯曲行程须在 1～7 圈内")
    all(s -> isfinite(s) && 0 < s <= 113, speed_rpm) ||
        error("手动速度须在 0～113 RPM 内")
    20 <= axial_move_ms <= 5000 || error("轴向移动时间须在 20～5000 ms 内")
    isfinite(refresh_s) && refresh_s > 0 || error("刷新间隔须大于零")
    quaternion_convention in (:sensor_to_world, :world_to_sensor) ||
        error("四元数坐标约定无效")

    base_z_offset = -150.0
    axes_map(section) = begin
        side = bend_sides[section]
        [0.0 side 0.0; 0.0 0.0 side; 1.0 0.0 0.0]
    end
    imu_rotation(q) = hcat((imu_axis_world(q, axis;
        convention=quaternion_convention) for axis in
        ([1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]))...)
    neutral_joints, neutral_frames = calculate_multi_segment_kinematics(
        zeros(2), zeros(2), base_z_offset; bend_sides)

    figure = GM.Figure(size=(1900, 1000))
    plot3d = GM.Axis3(figure[1:7, 1:3], xlabel="X (mm)", ylabel="Y (mm)",
                      zlabel="Z (mm)", aspect=:data,
                      title="透明轴：IMU 原始姿态；橙色：姿态拟合 PCC；IMU 位置由模型估算")
    arm_points = GM.Observable(copy(neutral_joints))
    arm_color = GM.Observable(:gray)
    GM.lines!(plot3d, arm_points; color=arm_color, linewidth=7)
    GM.scatter!(plot3d, GM.lift(points -> [points[8], points[end]], arm_points);
                color=arm_color, markersize=14)
    imu_axes = [[GM.Observable(GM.Point3f[]) for _ in 1:3] for _ in 1:2]
    model_axes = [[GM.Observable(GM.Point3f[]) for _ in 1:3] for _ in 1:2]
    for section in 1:2
        for axis in 1:3
            GM.lines!(plot3d, imu_axes[section][axis];
                      color=((:red, :green, :blue)[axis], 0.45), linewidth=6)
            GM.lines!(plot3d, model_axes[section][axis];
                      color=(:red, :green, :blue)[axis], linewidth=3)
        end
    end
    GM.xlims!(plot3d, -400, 400)
    GM.ylims!(plot3d, -400, 400)
    GM.zlims!(plot3d, -750, -100)
    status = GM.Observable("正在读取弯曲电机位置；尚未发送运动目标")
    diagnostics = GM.Observable("正在读取初始 IMU 姿态；灰线只是等待数据时的示意")
    right_panel = GM.GridLayout()
    figure[1:7, 4] = right_panel
    GM.Label(right_panel[1, 1], diagnostics; width=460, height=490,
             justification=:left, halign=:left, valign=:top)
    GM.Label(right_panel[2, 1], status; width=460, height=140,
             justification=:left, halign=:left, valign=:top)
    capture_button = GM.Button(right_panel[3, 1], label="以当前姿态重设拟合零位（不驱动电机）")
    stop_button = GM.Button(right_panel[4, 1], label="结束测试（保持位置）")
    emergency_button = GM.Button(right_panel[5, 1],
        label="紧急停止（关闭两台弯曲电机扭矩）")
    slider_grid = GM.GridLayout()
    figure[8, 1:4] = slider_grid
    bend_range = sort(unique(vcat(collect(-max_excursion_ticks:32:max_excursion_ticks),
                                  [-max_excursion_ticks, 0, max_excursion_ticks])))
    labels = ("第一段弯曲电机 1", "第二段弯曲电机 2",
              "第一段轴向通道 1", "第二段轴向通道 2")
    slider_readouts = [GM.Observable("等待位置反馈") for _ in 1:4]
    sliders = [begin
        GM.Label(slider_grid[i, 1], labels[i])
        slider = GM.Slider(slider_grid[i, 2:8];
                           range=i <= 2 ? bend_range : 500:10:2500,
                           startvalue=i <= 2 ? 0 : 1500)
        GM.Label(slider_grid[i, 9], slider_readouts[i]; width=260,
                 justification=:left, halign=:left)
        slider
    end for i in 1:4]

    idle, active, emergency = 0, 1, 2
    state = Threads.Atomic{Int}(active)
    worker = Ref{Union{Nothing,Task}}(nothing)
    base_ticks = zeros(Int, 2)
    actual_ticks = zeros(Int, 2)
    bend_desired = zeros(Int, 2)
    bend_sent = zeros(Int, 2)
    axial_desired = [1500, 1500]
    axial_sent = [1500, 1500]
    axial_confirmed = [false, false]
    capture_requested = Ref(false)
    baseline = Ref{Any}(nothing)
    alignment = Ref{Any}(nothing)
    fit_previous = zeros(4)
    slider_sync = Ref(false)
    function update_slider_readouts!()
        for section in 1:2
            slider_readouts[section][] =
                "实测 $(actual_ticks[section]) ticks"
            slider_readouts[section+2][] =
                axial_confirmed[section] ?
                "已发送 $(axial_sent[section]) μs" :
                "尚未发送"
        end
    end
    for i in 1:4
        GM.on(sliders[i].value) do value
            slider_sync[] && return
            state[] == active || return
            if i <= 2
                bend_desired[i] = Int(value)
            else
                axial_desired[i-2] = Int(value)
            end
        end
    end
    GM.on(capture_button.clicks) do _
        state[] == active && (capture_requested[] = true)
    end
    GM.on(stop_button.clicks) do _
        state[] = idle
        status[] = "测试已结束；弯曲电机保持最后目标位置"
    end
    function emergency_stop!()
        state[] = emergency
        status[] = "紧急停止：正在关闭两台弯曲电机扭矩"
        @async begin
            if worker[] !== nothing && worker[] !== current_task()
                wait(worker[])
            end
            failed = Int[]
            for id in (1, 2)
                try
                    send_line!(sp, "T,$(id),0")
                    ack = wait_line!(sp, x -> startswith(x, "ACK,T,$(id),0") ||
                                              startswith(x, "ERR,"); timeout=0.5)
                    startswith(ack, "ACK,T,$(id),0") || error(ack)
                catch err
                    push!(failed, id)
                    @warn "手动拟合测试紧急停止未获确认" servo_id=id exception=err
                end
            end
            status[] = isempty(failed) ? "紧急停止：两台弯曲电机扭矩已关闭" :
                "紧急停止未获舵机 $(failed) 确认；请切断舵机电源"
        end
    end
    GM.on(emergency_button.clicks) do _
        emergency_stop!()
    end
    GM.on(GM.events(figure.scene).window_open) do open
        open || (state[] == active && emergency_stop!())
    end

    function draw_axes!(observables, section, origin, rotation; length_mm=70.0)
        for axis in 1:3
            direction = rotation[:, axis]
            observables[section][axis][] = GM.Point3f[
                GM.Point3f(origin[1], origin[2], origin[3]),
                GM.Point3f(origin[1] + length_mm*direction[1],
                           origin[2] + length_mm*direction[2],
                           origin[3] + length_mm*direction[3])]
        end
    end
    # 用 IMU1 的直态对齐只变换 PCC 的显示坐标；IMU 三轴始终直接用原始四元数。
    function display_point(point, rotation)
        base = neutral_joints[1]
        offset = rotation * [point[i] - base[i] for i in 1:3]
        GM.Point3f((base[i] + offset[i] for i in 1:3)...)
    end
    axis_gaps(frames) = ntuple(axis ->
        rad2deg(acos(clamp(dot(frames[1][:, axis],
                                frames[2][:, axis]), -1.0, 1.0))), 3)
    function run_test!()
        try
            for (section, id) in enumerate((1, 2))
                servo_mode!(sp, id) in (0, 4) || error("舵机 $(id) 不是位置模式")
                base_ticks[section] = servo_position!(sp, id; verbose=false)
                actual_ticks[section] = base_ticks[section]
            end
            update_slider_readouts!()
            filters = [IMUQuaternionFilter(), IMUQuaternionFilter()]
            last_render = -Inf
            last_diagnostics = -Inf
            last_axes_render = -Inf
            last_position_read = -Inf
            function record_baseline!(pair)
                quats = (pair.imu1, pair.imu2)
                alignment[] = ntuple(section ->
                    neutral_frames[section][2] * axes_map(section) *
                    transpose(imu_rotation(quats[section])), 2)
                display_rotation = transpose(alignment[][1])
                arm_points[] = [display_point(point, display_rotation)
                                for point in neutral_joints]
                for section in 1:2
                    origin, rotation = neutral_frames[section]
                    draw_axes!(model_axes, section,
                               display_point(origin, display_rotation),
                               display_rotation * rotation * axes_map(section))
                end
                baseline[] = pair
                fill!(fit_previous, 0.0)
                arm_color[] = :orange
                for filter in filters
                    filter.filtered = nothing
                    filter.last_good = nothing
                end
            end
            status[] = "正在读取 IMU 并自动建立拟合零位；尚未发送运动目标"
            record_baseline!(fresh_imu_pair!(sp))
            status[] = "IMU 拟合已启动；初始姿态视为零位，拖动四个滑轨可观察形状变化"
            while state[] == active
                if capture_requested[]
                    capture_requested[] = false
                    try
                        record_baseline!(fresh_imu_pair!(sp))
                        status[] = "拟合零位已重设；未发送任何电机运动指令"
                    catch err
                        status[] = "重设拟合零位失败：$(sprint(showerror, err))"
                    end
                end
                for section in 1:2
                    state[] == active || break
                    if bend_desired[section] != bend_sent[section]
                        id = (1, 2)[section]
                        goal = base_ticks[section] + bend_desired[section]
                        -32767 <= goal <= 32767 ||
                            error("弯曲电机 $(id) 的目标超出多圈位置范围")
                        send_line!(sp, "P,$(id),$(goal),$(speed_rpm[section])")
                        ack = wait_line!(sp, x -> startswith(x, "ACK,P,$(id),") ||
                                                  startswith(x, "ERR,"); timeout=0.5)
                        startswith(ack, "ACK,P,") || error("弯曲电机 $(id) 指令失败：$(ack)")
                        bend_sent[section] = bend_desired[section]
                        update_slider_readouts!()
                    end
                end
                for section in 1:2
                    state[] == active || break
                    if axial_desired[section] != axial_sent[section]
                        axial_servo_position!(sp, section, axial_desired[section];
                                              move_ms=axial_move_ms, timeout=0.5, verbose=false)
                        axial_sent[section] = axial_desired[section]
                        axial_confirmed[section] = true
                        update_slider_readouts!()
                    end
                end
                state[] == active || break
                if time() - last_position_read >= 0.5
                    for (section, id) in enumerate((1, 2))
                        state[] == active || break
                        actual_ticks[section] = servo_position!(sp, id; verbose=false)
                    end
                    update_slider_readouts!()
                    last_position_read = time()
                end
                state[] == active || break
                pair = fresh_imu_pair!(sp)
                now = time()
                qs = (pair.imu1, pair.imu2)
                raw_frames = ntuple(section -> imu_rotation(qs[section]), 2)
                if now - last_axes_render >= refresh_s
                    for section in 1:2
                        origin = alignment[] === nothing ? neutral_frames[section][1] :
                            arm_points[][(8, length(arm_points[]))[section]]
                        draw_axes!(imu_axes, section, origin, raw_frames[section])
                    end
                    last_axes_render = now
                end
                filtered = Vector{Float64}[]
                accepted = Bool[]
                for section in 1:2
                    q, ok = filter_imu_quaternion!(filters[section], qs[section], now)
                    push!(filtered, q)
                    push!(accepted, ok)
                end
                all(accepted) || continue
                filtered_frames = ntuple(section -> imu_rotation(filtered[section]), 2)
                if alignment[] === nothing
                    continue
                end
                sensor_frames = ntuple(section ->
                    alignment[][section] * filtered_frames[section], 2)
                model_frames = ntuple(section ->
                    sensor_frames[section] * transpose(axes_map(section)), 2)
                fit = fit_pcc_fixed_axial(model_frames,
                    ntuple(section -> alignment[][section] * [0., 0., 1.], 2),
                    base_z_offset, fit_previous,
                    ntuple(section -> (axial_sent[section]-1500)*300/2000, 2))
                fit_previous .= [fit.bend..., fit.axial...]
                now - last_render >= refresh_s || continue
                rope = [R_DRIVE*deg2rad(abs(fit.bend[i])) for i in 1:2]
                sides = ntuple(i -> fit.bend[i] >= 0 ? 1 : -1, 2)
                joints, frames = calculate_multi_segment_kinematics(
                    rope, collect(fit.axial), base_z_offset; bend_sides=sides)
                display_rotation = transpose(alignment[][1])
                arm_points[] = [display_point(point, display_rotation) for point in joints]
                tangent_gaps = ntuple(section -> begin
                    origin, model_rotation = frames[section]
                    shown_origin = display_point(origin, display_rotation)
                    draw_axes!(imu_axes, section, shown_origin, raw_frames[section])
                    draw_axes!(model_axes, section, shown_origin,
                               display_rotation * model_rotation * axes_map(section))
                    tangent = display_rotation * model_rotation[:, 3]
                    sensor_x = raw_frames[section][:, 1]
                    rad2deg(acos(clamp(dot(tangent, sensor_x), -1.0, 1.0)))
                end, 2)
                # 坐标范围固定，避免 Makie 自动缩放改变三维视角和比例。
                raw_gaps = round.(axis_gaps(raw_frames); digits=1)
                aligned_gaps = round.(axis_gaps(sensor_frames); digits=1)
                raw_x_axes = (round.(raw_frames[1][:, 1]; digits=3),
                              round.(raw_frames[2][:, 1]; digits=3))
                baseline_pair = baseline[]
                baseline_turns = (
                    imu_orientation_distance_deg(qs[1], baseline_pair.imu1),
                    imu_orientation_distance_deg(qs[2], baseline_pair.imu2))
                if now - last_diagnostics >= 0.5
                    diagnostics[] =
                        "单独直态对齐后的重力方向残差：第一段 $(round(fit.mismatch_deg[1];digits=1))°，第二段 $(round(fit.mismatch_deg[2];digits=1))°\n" *
                        "末端轴向夹角（半透明 IMU +X / 不透明 PCC +X）：$(round.(tangent_gaps;digits=1))°\n" *
                        "原始姿态相对启动基线的总转角 IMU1/IMU2：$(round.(baseline_turns;digits=1))°\n" *
                        "两只 IMU 同名轴夹角 X/Y/Z：原始绝对 $(raw_gaps)°；直态对齐后 $(aligned_gaps)°\n" *
                        "原始 IMU +X 方向向量：IMU1 $(raw_x_axes[1])；IMU2 $(raw_x_axes[2])\n" *
                        "反求弯曲角 $(round.(fit.bend;digits=1))°；指令约束轴向角 $(round.(fit.axial;digits=1))°\n" *
                        "轴向已发脉宽 $(Tuple(axial_sent)) μs；双 IMU 时间差 $(pair.skew_us) μs"
                    last_diagnostics = now
                end
                last_render = now
            end
        catch err
            detail = sprint(showerror, err)
            status[] = "手动拟合测试出错：$(detail)"
            @warn "手动拟合测试出错" exception=err
            state[] == active && emergency_stop!()
        end
    end
    show_window && display(figure)
    worker[] = @async run_test!()
    (; figure, state, worker, sliders, slider_readouts, capture_button, stop_button,
       emergency_button, status, diagnostics, emergency_stop=emergency_stop!)
end

# Direct display of the two reported IMU orientations. This path deliberately
# avoids filtering, zero alignment, sensor-axis remapping, and PCC fitting.
function pcc_imu_raw_orientation_test!(sp;
                                       refresh_s::Real=0.05,
                                       quaternion_convention::Symbol=:sensor_to_world,
                                       show_window::Bool=true)
    isfinite(refresh_s) && refresh_s > 0 || error("刷新间隔须大于零")
    quaternion_convention in (:sensor_to_world, :world_to_sensor) ||
        error("四元数坐标约定无效")

    figure = GM.Figure(size=(1650, 900))
    axes = [
        GM.Axis3(figure[1:7, 1:2]; xlabel="World X", ylabel="World Y",
                 zlabel="World Z", aspect=:data,
                 title="IMU 1 直接四元数姿态"),
        GM.Axis3(figure[1:7, 3:4]; xlabel="World X", ylabel="World Y",
                 zlabel="World Z", aspect=:data,
                 title="IMU 2 直接四元数姿态")]
    triads = [[GM.Observable(GM.Point3f[]) for _ in 1:3] for _ in 1:2]
    colors = (:red, :green, :blue)
    for sensor in 1:2
        GM.xlims!(axes[sensor], -1.1, 1.1)
        GM.ylims!(axes[sensor], -1.1, 1.1)
        GM.zlims!(axes[sensor], -1.1, 1.1)
        for component in 1:3
            GM.lines!(axes[sensor], triads[sensor][component];
                      color=colors[component], linewidth=8)
        end
    end

    raw_status = [GM.Observable("等待 IMU$(id) 数据") for id in 1:2]
    status = GM.Observable("正在等待一对时间同步的 IMU 帧")
    panel = GM.GridLayout()
    figure[1:7, 5:6] = panel
    GM.Label(panel[1, 1], "串口原始四元数 (w, x, y, z)";
             fontsize=20, halign=:left)
    for sensor in 1:2
        GM.Label(panel[sensor+1, 1], raw_status[sensor];
                 justification=:left, halign=:left, valign=:top,
                 fontsize=16)
    end
    GM.Label(panel[4, 1], status; justification=:left, halign=:left,
             valign=:top, fontsize=16)
    stop_button = GM.Button(panel[5, 1], label="停止读取")
    GM.Label(panel[6, 1],
        "红 X / 绿 Y / 蓝 Z；每个图以自身固定原点显示。\n" *
        "视图只将串口四元数作用于单位 XYZ 轴，不做零位对齐、滤波、轴映射或 PCC 拟合。";
        justification=:left, halign=:left, valign=:top, fontsize=15)

    state = Threads.Atomic{Int}(1)
    worker = Ref{Union{Nothing,Task}}(nothing)
    last_frame = Ref(time())
    GM.on(stop_button.clicks) do _
        state[] = 0
        status[] = "已停止读取；本测试没有发送电机指令"
    end
    GM.on(GM.events(figure.scene).window_open) do open
        open || (state[] = 0)
    end

    function update_sensor!(sensor, q, sequence, timestamp_us, reported_norm)
        rotation = hcat((imu_axis_world(q, basis;
            convention=quaternion_convention) for basis in
            ([1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]))...)
        for component in 1:3
            direction = rotation[:, component]
            triads[sensor][component][] = GM.Point3f[
                GM.Point3f(0, 0, 0),
                GM.Point3f(0.85 * direction[1], 0.85 * direction[2],
                           0.85 * direction[3])]
        end
        raw_status[sensor][] = "IMU$(sensor)  seq=$(sequence)  t=$(timestamp_us) μs\n" *
            "norm=$(round(reported_norm; digits=7))\n" *
            "q=$(round.(q; digits=7))；串口帧 age=$(round(time()-last_frame[]; digits=3)) s"
    end

    function read_loop!()
        last_render = -Inf
        try
            while state[] == 1
                pair = fresh_imu_pair!(sp)
                now = time()
                if now - last_render >= refresh_s
                    update_sensor!(1, pair.imu1_raw_q, pair.imu1_sequence,
                                   pair.imu1_timestamp_us, pair.imu1_reported_norm)
                    update_sensor!(2, pair.imu2_raw_q, pair.imu2_sequence,
                                   pair.imu2_timestamp_us, pair.imu2_reported_norm)
                    last_frame[] = time()
                    status[] = "只读 IMU；同步时间差 $(pair.skew_us) μs；未执行姿态拟合"
                    last_render = now
                end
            end
        catch err
            status[] = "IMU 读取停止：$(sprint(showerror, err))"
            @warn "原始 IMU 姿态测试出错" exception=err
        finally
            state[] == 1 && (state[] = 0)
        end
    end

    show_window && display(figure)
    worker[] = @async read_loop!()
    (; figure, state, worker, axes, triads, raw_status, status, stop_button)
end

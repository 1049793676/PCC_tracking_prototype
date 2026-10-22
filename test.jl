"""Live 3-D attitude viewer for the two ESP32 IMU quaternion streams.

Run with an optional serial port argument, for example:
    julia --project=. test.jl COM8
"""

using GLMakie
using LibSerialPort
using LinearAlgebra
using Printf

if !isdefined(Main, :SingleIMUThreeModelControl)
    include(joinpath(@__DIR__, "dual_segment_imu_support.jl"))
end
IMU = SingleIMUThreeModelControl
Bridge = IMU.Bridge
GM = GLMakie

PORT = isempty(ARGS) ? "COM8" : ARGS[1]
BAUD = 115200
IMU_IDS = (1, 2)
FIRST_SEGMENT_IMU_ID = 2
SECOND_SEGMENT_IMU_ID = 1
AXIS_NAMES = ("X", "Y", "Z")
AXIS_COLORS = (:red, :green, :blue)
AXIS_LENGTH = 1.0f0
FILTER_TAU = 0.05
MAX_ANGULAR_RATE_DEG_S = 360.0
OUTLIER_MARGIN_DEG = 2.0
OUTLIER_CONFIRM_DEG = 5.0
OUTLIER_CONFIRM_SAMPLES = 3
MAX_CONSECUTIVE_REJECTS = 10
ORIGINS = Dict(
    :first_absolute => GM.Point3f(-2.4, 0.0, 0.0),
    :second_absolute => GM.Point3f(0.0, 0.0, 0.0),
    :second_relative => GM.Point3f(0.0, 0.0, 0.0),
)
# Keep only the first IMU's starting orientation as the reference for
# removing shared motion. The displayed axes remain the IMUs' own axes.
initial_first_rotation = nothing
initial_second_rotation = nothing
initial_first_received = nothing
initial_second_received = nothing

serial = LibSerialPort.open(PORT, BAUD)
buffer = UInt8[]
latest_rotation = Dict(
    id => Matrix{Float64}(I, 3, 3) for id in IMU_IDS)
latest_received_rotation = Dict(
    id => Matrix{Float64}(I, 3, 3) for id in IMU_IDS)
last_seen_time = Dict{Int,Union{Nothing,Float64}}(
    id => nothing for id in IMU_IDS)
have_rotation = Dict(id => false for id in IMU_IDS)
rejected_samples = Dict(id => 0 for id in IMU_IDS)
consecutive_rejects = Dict(id => 0 for id in IMU_IDS)
pending_rotation = Dict{Int,Union{Nothing,Matrix{Float64}}}(
    id => nothing for id in IMU_IDS)
pending_samples = Dict(id => 0 for id in IMU_IDS)
resync_samples = Dict(id => 0 for id in IMU_IDS)

fig = Figure(size=(1000, 760))
ax = Axis3(fig[1, 1];
    title="ESP32 双 IMU 实时姿态",
    xlabel="X",
    ylabel="Y",
    zlabel="Z",
    aspect=:data,
    limits=(-3.6, 3.6, -1.7, 1.7, -1.7, 1.7),
)

axis_observables = Dict(
    (view, axis) => Observable(GM.Point3f[])
    for view in (:first_absolute, :second_absolute, :second_relative), axis in 1:3)

for (view, label, linestyle) in (
        (:first_absolute, "第一段 IMU2 原始绝对姿态", :solid),
        (:second_absolute, "第二段 IMU1 原始绝对姿态", :solid),
        (:second_relative, "第二段 IMU1 去除共同旋转", :dash))
    for axis in 1:3
        lines!(ax, axis_observables[(view, axis)];
            color=AXIS_COLORS[axis],
            linewidth=4,
            linestyle=linestyle,
            label="$(label) $(AXIS_NAMES[axis])")
    end
    scatter!(ax, [ORIGINS[view]]; color=:white, strokecolor=:black,
        strokewidth=1, markersize=14)
end
axislegend(ax; position=:lt, nbanks=2)
status = Label(fig[2, 1], "正在等待 IMU 数据…"; tellwidth=false)
angles_status = Label(fig[3, 1], "等待两只 IMU 的首帧…"; tellwidth=false)

function axis_line(origin::GM.Point3f, rotation::AbstractMatrix, axis::Int)
    direction = GM.Point3f(Float32.(rotation[:, axis]))
    [origin, origin + AXIS_LENGTH * direction]
end

function update_attitude!(records)
    isempty(records) && return

    now = time()
    for id in IMU_IDS
        haskey(records, id) || continue
        raw_rotation = Matrix{Float64}(records[id].R)
        latest_received_rotation[id] = raw_rotation
        previous_time = last_seen_time[id]
        dt = previous_time === nothing ? 0.0 : max(now - previous_time, 1e-4)
        last_seen_time[id] = now

        if !have_rotation[id]
            latest_rotation[id] = raw_rotation
            have_rotation[id] = true
            continue
        end

        jump_deg = IMU.rotation_step_deg(latest_rotation[id], raw_rotation)
        allowed_jump_deg = MAX_ANGULAR_RATE_DEG_S * dt + OUTLIER_MARGIN_DEG
        if jump_deg > allowed_jump_deg
            rejected_samples[id] += 1
            consecutive_rejects[id] += 1

            candidate = pending_rotation[id]
            if candidate === nothing ||
               IMU.rotation_step_deg(candidate, raw_rotation) > OUTLIER_CONFIRM_DEG
                pending_rotation[id] = raw_rotation
                pending_samples[id] = 1
            else
                pending_samples[id] += 1
            end

            # Confirm repeated, mutually consistent samples instead of
            # allowing one bad frame to lock the filter permanently.
            if pending_samples[id] >= OUTLIER_CONFIRM_SAMPLES ||
               consecutive_rejects[id] >= MAX_CONSECUTIVE_REJECTS
                latest_rotation[id] = raw_rotation
                pending_rotation[id] = nothing
                pending_samples[id] = 0
                consecutive_rejects[id] = 0
                resync_samples[id] += 1
            end
            continue
        end

        pending_rotation[id] = nothing
        pending_samples[id] = 0
        consecutive_rejects[id] = 0
        latest_rotation[id] = IMU.lowpass_rotation(
            latest_rotation[id], raw_rotation, dt, FILTER_TAU)
    end

    if have_rotation[FIRST_SEGMENT_IMU_ID] && have_rotation[SECOND_SEGMENT_IMU_ID]
        first_raw = latest_rotation[FIRST_SEGMENT_IMU_ID]
        second_raw = latest_rotation[SECOND_SEGMENT_IMU_ID]
        if initial_first_rotation === nothing
            global initial_first_rotation = copy(first_raw)
            global initial_second_rotation = copy(second_raw)
            global initial_first_received = copy(latest_received_rotation[FIRST_SEGMENT_IMU_ID])
            global initial_second_received = copy(latest_received_rotation[SECOND_SEGMENT_IMU_ID])
        end
        # R1_zero * R1_current' removes first-segment motion while leaving
        # the second IMU in its original start-of-run world orientation.
        second_compensated = initial_first_rotation * first_raw' * second_raw
        first_received_change = IMU.rotation_step_deg(
            initial_first_received, latest_received_rotation[FIRST_SEGMENT_IMU_ID])
        first_displayed_change = IMU.rotation_step_deg(initial_first_rotation, first_raw)
        second_received_change = IMU.rotation_step_deg(
            initial_second_received, latest_received_rotation[SECOND_SEGMENT_IMU_ID])
        second_displayed_change = IMU.rotation_step_deg(initial_second_rotation, second_raw)
        compensation_difference = IMU.rotation_step_deg(second_raw, second_compensated)

        angles_status.text[] = @sprintf(
            "相对启动：IMU2 原始 %.1f° / 显示 %.1f°；IMU1 原始 %.1f° / 显示 %.1f°；第二段实线与虚线差 %.1f°",
            first_received_change, first_displayed_change,
            second_received_change, second_displayed_change,
            compensation_difference)

        for axis in 1:3
            axis_observables[(:first_absolute, axis)][] =
                axis_line(ORIGINS[:first_absolute], first_raw, axis)
            axis_observables[(:second_absolute, axis)][] =
                axis_line(ORIGINS[:second_absolute], second_raw, axis)
            axis_observables[(:second_relative, axis)][] =
                axis_line(ORIGINS[:second_relative], second_compensated, axis)
        end
    end

    status.text[] = @sprintf(
        "IMU2: %s (丢弃 %d，重同步 %d)    IMU1: %s (丢弃 %d，重同步 %d)    红=X，绿=Y，蓝=Z；实线=原始，虚线=去共同旋转",
        haskey(records, FIRST_SEGMENT_IMU_ID) ? "收到新数据" : "保持上一姿态",
        rejected_samples[FIRST_SEGMENT_IMU_ID],
        resync_samples[FIRST_SEGMENT_IMU_ID],
        haskey(records, SECOND_SEGMENT_IMU_ID) ? "收到新数据" : "保持上一姿态",
        rejected_samples[SECOND_SEGMENT_IMU_ID],
        resync_samples[SECOND_SEGMENT_IMU_ID])
end

try
    screen = display(fig)
    while isopen(screen)
        n = LibSerialPort.bytesavailable(serial)
        if n > 0
            records, messages = Bridge.decode!(buffer, read(serial, n))
            update_attitude!(records)
            isempty(messages) || @debug "ESP32 message" messages
        end
        sleep(0.01)
    end
finally
    LibSerialPort.close(serial)
end

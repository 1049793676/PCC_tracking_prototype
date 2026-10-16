"""Small IMU, serial-protocol, and actuator support for the portable PCC project."""
module SingleIMUThreeModelControl

using LinearAlgebra, Printf, LibSerialPort

module Bridge
using LibSerialPort, LinearAlgebra

function decode!(buffer::Vector{UInt8}, bytes)
    append!(buffer, bytes)
    records = Dict{Int,NamedTuple}()
    messages = String[]
    first = 1
    for i in eachindex(buffer)
        buffer[i] == 0x0a || continue
        line = strip(String(copy(buffer[first:i-1])))
        fields = split(line, ',')
        if length(fields) == 9 && fields[1] == "IMU"
            try
                id = parse(Int, fields[2])
                seq = parse(UInt32, fields[3])
                stamp = parse(UInt32, fields[4])
                q = parse.(Float64, fields[5:8])
                reported_norm = parse(Float64, fields[9])
                if id in (1, 2) && all(isfinite, q) && isfinite(reported_norm) &&
                   0.5 <= norm(q) <= 1.5
                    records[id] = (; id, seq, stamp, R=quat_rotation(q),
                                     q=q ./ norm(q), reported_norm)
                else
                    push!(messages, line)
                end
            catch
                push!(messages, line)
            end
        elseif !isempty(line)
            push!(messages, line)
        end
        first = i + 1
    end
    first > 1 && deleteat!(buffer, 1:first-1)
    length(buffer) > 8192 && empty!(buffer)
    records, messages
end

function quat_rotation(q)
    w, x, y, z = q ./ norm(q)
    [1-2(y*y+z*z) 2(x*y-z*w) 2(x*z+y*w);
     2(x*y+z*w) 1-2(x*x+z*z) 2(y*z-x*w);
     2(x*z-y*w) 2(y*z+x*w) 1-2(x*x+y*y)]
end

function write_command(serial, command::AbstractString; timeout=0.5)
    LibSerialPort.set_write_timeout(serial, timeout)
    write(serial, codeunits(command))
end
end

Base.@kwdef struct Config
    port::String = "COM8"
    baud::Int = 115200
    imu_id::Int = 2
    imu_body::Int = 7
    servo_id::Int = 1
    axial_servo_id::Int = 2
    axial_initial_signal::Int = 1500
    axial_rotation_sign::Int = 1
    bend_motor_sign::Int = -1
    imu_bend_sign::Int = -1
    axial_kp::Float64 = 0.8
    axial_max_rate_counts_s::Float64 = 600.0
    axial_move_ms::Int = 120
    mount::Union{Nothing,Matrix{Float64}} = nothing
    conjugate::Bool = false
    neutral::Int = 1500
    negative_start::Int = 1400
    positive_start::Int = 1600
    command_min::Int = 500
    command_max::Int = 2500
    kp::Float64 = 60.0
    ki::Float64 = 5.0
    kd::Float64 = 2.0
    max_speed::Int = 60
    zero_kp::Float64 = 1.0
    zero_max_speed::Int = 30
    min_speed_positive::Int = 15
    min_speed_negative::Int = 20
    send_period::Float64 = 0.01
    max_command_rate::Float64 = 100.0
    direction_hysteresis_deg::Float64 = 1.0
    imu_filter_tau::Float64 = 0.05
    max_imu_rate_deg_s::Float64 = 360.0
    imu_outlier_margin_deg::Float64 = 2.0
    estimate_period::Float64 = 0.0
    imu_timeout::Float64 = 0.7
    zero_tolerance_deg::Float64 = 2.0
    zero_hold_samples::Int = 8
    noise_deg::Float64 = 1.0
    prior_std::Float64 = 0.08
end

function validate(c::Config, adapter=nothing)
    c.imu_id in (1, 2) || error("ESP32 IMU 编号只能是 1 或 2")
    adapter === nothing || c.imu_body <= length(adapter.bodies) ||
        error("IMU 刚体编号不在当前 PCC 结构中")
    1 <= c.servo_id <= 16 && 1 <= c.axial_servo_id <= 16 ||
        error("舵机通道必须在 1–16")
    c.servo_id != c.axial_servo_id || error("弯曲与轴向舵机必须使用不同通道")
    c.neutral == 1500 && c.negative_start < c.neutral < c.positive_start ||
        error("连续舵机中位或静止带设置无效")
    c.command_min < c.negative_start && c.command_max > c.positive_start ||
        error("连续舵机范围无效")
    c.bend_motor_sign in (-1, 1) && c.imu_bend_sign in (-1, 1) ||
        error("弯曲方向符号只能为 ±1")
    c.axial_rotation_sign in (-1, 1) || error("轴向方向符号只能为 ±1")
    c.max_speed > 0 && c.zero_max_speed > 0 || error("舵机速度上限必须为正")
    c.max_command_rate > 0 && c.send_period > 0 || error("指令更新参数无效")
    c.imu_timeout > c.send_period || error("IMU 超时应大于发送周期")
    c
end

sensor_rotation(record, c::Config) = c.conjugate ? record.R' : record.R
rotation_step_deg(a, b) = rad2deg(norm(log_rotation(a' * b)))

function log_rotation(R)
    c = clamp((tr(R) - 1) / 2, -1.0, 1.0)
    theta = acos(c)
    v = [R[3,2]-R[2,3], R[1,3]-R[3,1], R[2,1]-R[1,2]] / 2
    theta < 1e-7 && return v
    theta / sin(theta) .* v
end

function exp_rotation(v)
    theta = norm(v)
    K = [0.0 -v[3] v[2]; v[3] 0.0 -v[1]; -v[2] v[1] 0.0]
    theta < 1e-8 && return Matrix{Float64}(I, 3, 3) + K
    Matrix{Float64}(I, 3, 3) + sin(theta)/theta*K +
        (1-cos(theta))/theta^2*(K*K)
end

function lowpass_rotation(previous, current, dt, tau)
    tau <= 0 && return current
    alpha = clamp(dt / (tau + dt), 0.0, 1.0)
    previous * exp_rotation(alpha .* log_rotation(previous' * current))
end

function servo_speed_command(error_deg, integral, derivative, c::Config;
                             kp=c.kp, ki=c.ki, kd=c.kd, max_speed=c.max_speed)
    abs(error_deg) <= c.direction_hysteresis_deg && return c.neutral
    direction = sign(error_deg)
    correction = kp*error_deg + ki*integral + kd*derivative
    magnitude = clamp(direction*correction, 0.0, max_speed)
    magnitude <= 0 && return c.neutral
    minimum_running = min(direction > 0 ? c.min_speed_positive : c.min_speed_negative,
                          max_speed)
    magnitude = max(magnitude, minimum_running)
    command = direction > 0 ? c.positive_start + 1 + round(Int, magnitude) :
                              c.negative_start - 1 - round(Int, magnitude)
    clamp(command, c.command_min, c.command_max)
end

function slew_command(previous::Integer, target::Integer, dt, c::Config)
    previous_direction = previous > c.positive_start ? 1 :
                         previous < c.negative_start ? -1 : 0
    target_direction = target > c.positive_start ? 1 :
                       target < c.negative_start ? -1 : 0
    (target_direction == 0 ||
     (previous_direction != 0 && previous_direction != target_direction)) && return c.neutral
    if previous_direction == 0
        return target_direction > 0 ?
            min(target, c.positive_start+1+c.min_speed_positive) :
            max(target, c.negative_start-1-c.min_speed_negative)
    end
    (previous_direction > 0 && target < previous ||
     previous_direction < 0 && target > previous) && return clamp(target, c.command_min, c.command_max)
    step = max(1, round(Int, c.max_command_rate * max(dt, 0.0)))
    clamp(clamp(target, previous-step, previous+step), c.command_min, c.command_max)
end

function slew_axial_signal(previous::Integer, target::Integer, dt, c::Config)
    step = max(1, round(Int, c.axial_max_rate_counts_s * max(dt, 0.0)))
    clamp(target, previous-step, previous+step)
end

servo_command(c::Config, value) = "S,$(c.servo_id),$(clamp(value,c.command_min,c.command_max)),120\n"
axial_servo_command(c::Config, signal=c.axial_initial_signal) =
    "S,$(c.axial_servo_id),$(clamp(signal,500,2500)),$(c.axial_move_ms)\n"

end # module SingleIMUThreeModelControl

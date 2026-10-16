"""Build hardware references directly from the two-section PCC trajectory state."""
module TwoSegmentPCCIMUServoControl

const IMU = Main.SingleIMUThreeModelControl

struct DriveTable
    times::Vector{Float64}
    shortening_mm::Vector{Float64}
end

function interpolate(values, times, t)
    t <= times[1] && return values[1]
    t >= times[end] && return values[end]
    i = searchsortedlast(times, t)
    u = (t-times[i])/(times[i+1]-times[i])
    (1-u)*values[i] + u*values[i+1]
end

(drive::DriveTable)(t::Real) = interpolate(drive.shortening_mm, drive.times, Float64(t))

"""Keep the PCC planner's stored actuator geometry as the hardware reference.

The two-section simulation already stores each section's effective rope drive
and axial angle. Since effective rope drive is `R_DRIVE * bend_angle`, no
single-section natural-coordinate model or second rope-length conversion is
needed here.
"""
function make_reference(state; approach_seconds=15.0, trajectory_seconds=25.0,
                        max_bend_deg=Main.MAX_BEND_DEG)
    approach_seconds > 0 && trajectory_seconds > 0 || error("过渡和轨迹时长必须为正")
    0 < max_bend_deg <= Main.MAX_BEND_DEG ||
        error("PCC 单侧弯曲规划范围必须在 0～$(Main.MAX_BEND_DEG)° 内")
    n = length(state.trajectory)
    n >= 2 || error("两段仿真状态至少需要两个轨迹点")
    approach_steps = state.shape_start_index - 1
    shape_samples = n - approach_steps
    shape_samples >= 2 || error("轨迹参考中形状采样点不足")
    times = vcat(collect(range(0.0, approach_seconds; length=approach_steps+1)),
                 approach_seconds .+ collect(range(trajectory_seconds/(shape_samples-1),
                                                       trajectory_seconds; length=shape_samples-1)))
    length(times) == n || error("PCC 轨迹时间与状态采样数不一致")

    bend_magnitude_deg = rad2deg.(state.rope_drive_history ./ Main.R_DRIVE)
    minimum(bend_magnitude_deg) >= -1e-8 || error("单侧弯曲参考不能包含负的绳驱动量")
    maximum(bend_magnitude_deg) <= max_bend_deg + 1e-8 ||
        error("离线弯曲角超出 PCC 规划范围 $(max_bend_deg)°")
    all(isfinite, state.axial_rotation_history) || error("轴向参考包含非有限值")

    # The live controller only needs a body-count adapter for config validation;
    # pose reconstruction and geometry visualization use the standalone PCC model.
    adapter = (; bodies=collect(1:7))
    ref = (; adapter, times, pcc_qs=state.joints_history, qs=state.joints_history)
    drive = DriveTable(times, copy(vec(state.rope_drive_history[1, :])))
    data = (; drive, ref, two_segment_state=state,
            shape=state.shape, shape_start_time=approach_seconds,
            approach_seconds, trajectory_seconds,
            bend_sign=state.bend_sides[1], one_sided_bend=true)
    (; data, drive, times,
       first_segment_bend_deg=state.bend_sides[1] .* bend_magnitude_deg[1, :],
       first_segment_bend_magnitude_deg=bend_magnitude_deg[1, :],
       first_segment_axial_deg=copy(state.axial_rotation_history[1, :]),
       second_segment_bend_deg=bend_magnitude_deg[2, :],
       second_segment_axial_deg=copy(state.axial_rotation_history[2, :]))
end

make_view(data, config; send_servo=false) = Main.visualize_arm(data.two_segment_state)

end # module TwoSegmentPCCIMUServoControl

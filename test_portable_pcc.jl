using Test

include(joinpath(@__DIR__, "two_segment_pcc_path_tracking.jl"))
include(joinpath(@__DIR__, "dual_segment_imu_support.jl"))
include(joinpath(@__DIR__, "two_segment_pcc_imu_servo_control.jl"))

@testset "standalone PCC trajectory and direct drive reference" begin
    for shape in (:rectangle, :circle, :infinity, :s_curve)
        state = compute_trajectory(shape=shape)
        ref = TwoSegmentPCCIMUServoControl.make_reference(state)
        @test length(ref.times) == length(state.trajectory)
        @test ref.data.two_segment_state === state
        @test ref.drive.shortening_mm ≈ vec(state.rope_drive_history[1, :])
        @test ref.first_segment_bend_magnitude_deg ≈
              rad2deg.(state.rope_drive_history[1, :] ./ R_DRIVE)
        @test ref.second_segment_bend_deg ≈
              rad2deg.(state.rope_drive_history[2, :] ./ R_DRIVE)
    end
end

@testset "portable IMU support" begin
    record = (R=Matrix{Float64}(I, 3, 3),)
    cfg = SingleIMUThreeModelControl.Config()
    @test SingleIMUThreeModelControl.sensor_rotation(record, cfg) ≈ I(3)
    @test SingleIMUThreeModelControl.servo_speed_command(8.0, 0.0, 0.0, cfg) >
          cfg.positive_start
end

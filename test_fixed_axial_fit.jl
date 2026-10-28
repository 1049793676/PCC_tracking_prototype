using Test
include(joinpath(@__DIR__, "two_segment_pcc_path_tracking.jl"))
include(joinpath(@__DIR__, "Lumimoon-Tech-RobotDuck-main", "dual_imu_feetech_test.jl"))

@testset "fixed axial fit" begin
    axial = (35.0, -20.0)
    bend = (24.0, 41.0)
    _, frames = calculate_multi_segment_kinematics(
        [R_DRIVE * deg2rad(abs(bend[i])) for i in 1:2], collect(axial), -150.0;
        bend_sides=(1, 1))
    gravity = ([0.0, 0.0, -1.0], [0.0, 0.0, -1.0])
    result = fit_pcc_fixed_axial(
        (frames[1][2], frames[2][2]), gravity, -150.0, (0.0, 0.0), axial;
        max_iterations=60)
    @test all(isapprox.(result.axial, axial))
    @test all(isapprox.(result.bend, bend; atol=0.2))
    @test maximum(result.mismatch_deg) < 0.1
end

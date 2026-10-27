using Test
include(joinpath(@__DIR__, "Lumimoon-Tech-RobotDuck-main", "dual_imu_feetech_test.jl"))

# Offline only: no serial connection, window, or motor command.
@testset "generated PCC reference timing and interpolation" begin
    for shape in (:circle, :rectangle)
        ref = make_pcc_reference(; shape)
        @test ref.duration == 77.0
        @test ref.approach_end_s == 15.0
        @test ref.track_start_s == 17.0
        @test length(ref.times) == length(ref.bend[1])
        for i in eachindex(ref.times)
            sample = ref.at(ref.times[i])
            for j in 1:2
                @test isapprox(sample.bend[j], ref.bend[j][i]; atol=1e-8)
                @test isapprox(sample.axial[j], ref.axial[j][i]; atol=1e-8)
            end
        end
        stops = shape == :rectangle ? (17., 32., 47., 62., 77.) : (17., 77.)
        for t in stops
            sample = ref.at(t)
            @test all(iszero, (sample.bend_rate..., sample.axial_rate...))
            @test all(isapprox.(ref.at(t-1e-6).bend, ref.at(t+1e-6).bend; atol=1e-6))
            @test all(isapprox.(ref.at(t-1e-6).axial, ref.at(t+1e-6).axial; atol=1e-6))
        end
    end
end

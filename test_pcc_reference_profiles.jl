using Test
include(joinpath(@__DIR__, "Lumimoon-Tech-RobotDuck-main", "pcc_reference_profiles.jl"))

@testset "continuous reference velocity without angular overshoot" begin
    knots = [0.0, 12.0, 15.0, 15.0, -8.0, -4.0]
    f = pcc_monotone_profile(knots)
    for i in 1:length(knots)-1, u in range(0, 1; length=31)
        s = (i-1+u)/(length(knots)-1)
        @test min(knots[i], knots[i+1])-1e-10 <= f(s).value <=
              max(knots[i], knots[i+1])+1e-10
    end
    for s in (1:4)./5
        @test isapprox(f(s-1e-8).derivative, f(s+1e-8).derivative; atol=1e-4)
    end
    for s in (0.13, 0.33, 0.72, 0.91)
        numerical = (f(s+1e-6).value-f(s-1e-6).value)/2e-6
        @test isapprox(f(s).derivative, numerical; atol=1e-5)
    end
end

@testset "smooth start and stop timing" begin
    for duration in (1.0, 15.0, 60.0)
        @test pcc_phase_profile(0.0, duration, 2.0) == (phase=0.0, rate=0.0)
        @test pcc_phase_profile(duration, duration, 2.0) == (phase=1.0, rate=0.0)
        for t in (0.17, 0.5, 0.83).*duration
            p = pcc_phase_profile(t, duration, 2.0)
            numerical = (pcc_phase_profile(t+1e-6, duration, 2.0).phase -
                         pcc_phase_profile(t-1e-6, duration, 2.0).phase)/2e-6
            @test isapprox(p.rate, numerical; atol=1e-6)
        end
    end
end

@testset "15 second approach, 2 second hold, 60 second rectangle" begin
    # Three approach knots; four three-knot edges with duplicated corners.
    values = [0., 0.5, 1., 2., 3., 3., 4., 5., 5., 4., 3., 3., 2., 1.]
    at = pcc_reference_evaluator((values, values), (values, values), 3, :rectangle;
        approach_seconds=15., transition_pause_s=2., trajectory_seconds=60., start_slow_s=2.)
    @test at(15).bend == at(16).bend == at(17).bend == (1., 1.)
    for t in (0., 15., 17., 32., 47., 62., 77., 80.)
        @test all(iszero, (at(t).bend_rate..., at(t).axial_rate...))
    end
    for t in (15., 17., 32., 47., 62., 77.)
        @test isapprox(at(t-1e-6).bend[1], at(t+1e-6).bend[1]; atol=1e-6)
        @test isapprox(at(t-1e-6).bend_rate[1], at(t+1e-6).bend_rate[1]; atol=1e-5)
    end
    @test at(77).bend == at(100).bend == (1., 1.)
    circle = pcc_reference_evaluator((values, values), (values, values), 3, :circle;
        approach_seconds=15., transition_pause_s=2., trajectory_seconds=60., start_slow_s=2.)
    @test circle(77).bend == (1., 1.)
    @test circle(77).bend_rate == (0., 0.)
end

# Axial angles are constraints, not IMU measurements. Gravity directions are
# expressed in each aligned model world; never assume both alignment rotations
# leave world Z invariant. IMU quaternion tilt remains a sensor assumption.
function fit_pcc_fixed_axial(imu_frames, gravity_world, base_z_offset, initial, axial;
                             max_iterations=30)
    length(initial) >= 2 && length(axial) == 2 || error("需要两个弯曲初值和两个轴向角")
    all(isfinite, initial) && all(isfinite, axial) || error("角度必须有限")
    maximum(abs, axial) <= 150 || error("轴向角超过 ±150°")
    x = collect(clamp.(Float64.(initial[1:2]), -270., 270.))
    g = ntuple(i -> gravity_world[i] / norm(gravity_world[i]), 2)
    observed = ntuple(i -> transpose(imu_frames[i]) * g[i], 2)
    frames(b) = calculate_multi_segment_kinematics(
        [R_DRIVE * deg2rad(abs(b[i])) for i in 1:2], collect(axial), base_z_offset;
        bend_sides=ntuple(i -> b[i] >= 0 ? 1 : -1, 2))[2]
    residual(b) = vcat((transpose(frames(b)[i][2]) * g[i] - observed[i] for i in 1:2)...)
    for _ in 1:max_iterations
        r = residual(x)
        norm(r) < 1e-7 && break
        J = hcat((begin
            y = copy(x); y[k] += 0.05
            (residual(y) - r) / 0.05
        end for k in 1:2)...)
        step = -(transpose(J)*J + 1e-8*Matrix{Float64}(I, 2, 2)) \ (transpose(J)*r)
        all(isfinite, step) || error("弯曲拟合产生无效步长")
        step = clamp.(step, -15., 15.)
        accepted = false
        for scale in (1., 0.5, 0.25, 0.125, 0.0625)
            candidate = clamp.(x + scale*step, -270., 270.)
            if norm(residual(candidate)) < norm(r)
                x = candidate
                accepted = true
                break
            end
        end
        (!accepted || norm(step) < 1e-5) && break
    end
    fitted = frames(x)
    mismatch_deg = ntuple(i -> rad2deg(acos(clamp(dot(
        transpose(fitted[i][2])*g[i], observed[i]), -1., 1.))), 2)
    orientation_mismatch_deg = ntuple(i -> rotation_distance_deg(fitted[i][2], imu_frames[i]), 2)
    (; bend=Tuple(x), axial=Tuple(Float64.(axial)), mismatch_deg,
       orientation_mismatch_deg, axial_source=:command, residual_kind=:gravity)
end

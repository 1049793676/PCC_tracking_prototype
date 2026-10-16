using GLMakie
using LinearAlgebra
using StaticArrays

# 两段 PCC 路径跟踪示例。
# 每段由一个 23 mm 驱动装置刚体和 6 个可弯曲单元组成。驱动刚体的位置
# 沿前段末端切线延伸；它仅绕自身轴旋转。每个弯曲单元（含首单元）都经
# 转动副与前一刚体连接，六个副均分该段总弯曲角。
# 弯曲单元采用当前样机约 39.5 mm 节距；绳孔半径 21.5 mm 对应有效绳臂
# 21.5*cos(pi/6)=18.620 mm。R_DRIVE 不是卷盘半径。
const N_SEGMENTS = 2
const N_LINKS = 6
const L_ACTUATOR = 23.0
const L_LINK = 39.5
const R_DRIVE = 21.5 * cos(pi / 6)
const BEND_SIGN = -1.0 # 正向收绳固定采用另一侧弯曲；弯曲幅值只允许非负
const MAX_BEND_DEG = 270.0
const MAX_ROPE_DRIVE = R_DRIVE * deg2rad(MAX_BEND_DEG)
const N_STEPS = 120
const N_APPROACH_STEPS = 60
const SEG_COLORS = [:red, :blue]
const FRAME_COLORS = [:red, :green, :blue]

struct ArmState
    joints_history::Vector{Vector{Point3f}}  # 15 points: 2×23 mm rigid modules + 12 bend links
    terminal_frames_history::Vector{Vector{Tuple{Point3f,Matrix{Float64}}}}
    rope_drive_history::Matrix{Float64}       # 2×N，正值表示该段收绳量（mm）
    axial_rotation_history::Matrix{Float64}   # 2×N，每段轴向旋转角（deg）
    trajectory::Vector{Point3f}
    delta_ls_history::Matrix{Float64}         # 兼容 seg3 示例的字段名
    base_angles_history::Matrix{Float64}      # 兼容 seg3 示例的字段名
    base_z_offset::Float64
    shape::Symbol
    shape_start_index::Int
    bend_sides::NTuple{2,Int}             # 每段世界模型中的物理弯曲侧，±1
    target_trajectory::Vector{Point3f}   # 不随标定改变的空间目标点
end

"按当前结果时间序列线性插值得到某一段的绳索驱动量。"
function rope_drive(state::ArmState, t::Real; segment::Integer=1)
    1 <= segment <= N_SEGMENTS || throw(BoundsError(segment))
    u = clamp(Float64(t), 1.0, length(state.trajectory))
    i = clamp(floor(Int, u), 1, length(state.trajectory) - 1)
    w = u - i
    (1 - w) * state.rope_drive_history[segment, i] +
        w * state.rope_drive_history[segment, i + 1]
end

"按当前结果时间序列线性插值得到某一段的轴向旋转角。"
function axial_rotation(state::ArmState, t::Real; segment::Integer=1)
    1 <= segment <= N_SEGMENTS || throw(BoundsError(segment))
    u = clamp(Float64(t), 1.0, length(state.trajectory))
    i = clamp(floor(Int, u), 1, length(state.trajectory) - 1)
    w = u - i
    (1 - w) * state.axial_rotation_history[segment, i] +
        w * state.axial_rotation_history[segment, i + 1]
end

function calculate_multi_segment_kinematics(delta_ls, base_angles, base_z_offset=0.0;
                                            bend_sides::NTuple{2,Int}=(-1, -1))
    length(delta_ls) == N_SEGMENTS || error("需要两个分段绳索驱动量")
    length(base_angles) == N_SEGMENTS || error("需要两个分段轴向旋转角")
    all(s -> s in (-1, 1), bend_sides) || error("每段物理弯曲侧必须为 ±1")

    total_joints = N_SEGMENTS * (N_LINKS + 1) + 1
    joints = Vector{Point3f}(undef, total_joints)
    joints[1] = Point3f(0, 0, base_z_offset)
    terminal_frames = Tuple{Point3f,Matrix{Float64}}[]

    cum_pos = SVector{3,Float64}(0.0, 0.0, Float64(base_z_offset))
    # 基座局部 +Z 对齐世界 -Z，使直态操作臂朝下。
    cum_R = @SMatrix [1.0 0.0 0.0; 0.0 -1.0 0.0; 0.0 0.0 -1.0]
    joint_idx = 1
    v_idx = 0

    for seg in 1:N_SEGMENTS
        dl = Float64(delta_ls[seg])
        gamma = deg2rad(Float64(base_angles[seg]))
        alpha = bend_sides[seg] * dl / (R_DRIVE * N_LINKS)
        cg, sg = cos(gamma), sin(gamma)
        Rz = @SMatrix [cg -sg 0.0; sg cg 0.0; 0.0 0.0 1.0]
        # 驱动刚体沿前段末端轴线放置。gamma 只改变它绕该轴的姿态，
        # 不改变驱动刚体轴线或端点位置；弯曲从其后的转动副开始。
        seg_start_pos = cum_pos
        seg_start_R = cum_R * Rz
        actuator_end = seg_start_pos + cum_R * @SVector [0.0, 0.0, L_ACTUATOR]
        joint_idx += 1
        joints[joint_idx] = Point3f(actuator_end)
        bend_start_pos = SVector{3,Float64}(actuator_end)
        local_x, local_z = 0.0, 0.0

        # 六个弯曲单元分别通过转动副连接，包含驱动刚体与首单元之间的副。
        for i in 1:N_LINKS
            theta = i * alpha
            ct, st = cos(theta), sin(theta)
            local_end_x = local_x + L_LINK * sin(theta)
            local_end_z = local_z + L_LINK * cos(theta)
            end_vec = seg_start_R * @SVector [local_end_x, 0.0, local_end_z]
            cum_end = bend_start_pos + end_vec
            joint_idx += 1
            joints[joint_idx] = Point3f(cum_end)

            local_x, local_z = local_end_x, local_end_z
        end

        total_bend_angle = N_LINKS * alpha
        ct, st = cos(total_bend_angle), sin(total_bend_angle)
        Ry = @SMatrix [ct 0.0 st; 0.0 1.0 0.0; -st 0.0 ct]
        cum_R = seg_start_R * Ry
        cum_pos = joints[joint_idx]
        push!(terminal_frames, (Point3f(cum_pos), Matrix{Float64}(cum_R)))
    end
    joints, terminal_frames
end

function calculate_end_position(delta_ls, base_angles, base_z_offset=0.0;
                                bend_sides::NTuple{2,Int}=(-1, -1))
    calculate_multi_segment_kinematics(delta_ls, base_angles, base_z_offset;
                                       bend_sides)[1][end]
end

function calculate_jacobian(x, base_z_offset; bend_sides::NTuple{2,Int}=(-1, -1))
    J = zeros(3, 2N_SEGMENTS)
    eps = vcat(fill(1e-3, N_SEGMENTS), fill(1e-2, N_SEGMENTS))
    x0 = calculate_end_position(x[1:N_SEGMENTS], x[N_SEGMENTS + 1:end], base_z_offset;
                                bend_sides)
    for i in eachindex(x)
        xt = copy(x)
        xt[i] += eps[i]
        xp = calculate_end_position(xt[1:N_SEGMENTS], xt[N_SEGMENTS + 1:end], base_z_offset;
                                    bend_sides)
        J[:, i] = (Float64.(xp .- x0)) ./ eps[i]
    end
    J
end

function solve_ik(target_pos, prev_controls, base_z_offset=0.0;
                  max_iter=120, bend_sides::NTuple{2,Int}=(-1, -1))
    x = Float64.(copy(prev_controls))
    length(x) == 2N_SEGMENTS || error("两段 PCC 控制向量需要 4 个量")
    for _ in 1:max_iter
        current = calculate_end_position(x[1:N_SEGMENTS], x[N_SEGMENTS + 1:end],
                                         base_z_offset; bend_sides)
        error_vec = Float64.(target_pos - current)
        norm(error_vec) < 1e-2 && break
        J = calculate_jacobian(x, base_z_offset; bend_sides)
        step = J' * ((J * J' + 0.05^2 * I(3)) \ error_vec)
        x += 0.7 .* step
        x[1:N_SEGMENTS] .= clamp.(x[1:N_SEGMENTS], 0.0, MAX_ROPE_DRIVE)
        x[N_SEGMENTS + 1:end] .= clamp.(x[N_SEGMENTS + 1:end], -150.0, 150.0)
    end
    x
end

function compute_trajectory(; n_steps::Integer=N_STEPS, shape::Symbol=:rectangle,
                            bend_sides::NTuple{2,Int}=(-1, -1))
    n_steps >= 4 || error("n_steps 必须至少为 4")
    all(s -> s in (-1, 1), bend_sides) || error("每段物理弯曲侧必须为 ±1")
    shape in (:rectangle, :circle, :infinity, :s_curve) ||
        error("不支持的轨迹形状：$shape")
    shape == :rectangle && n_steps % 4 != 0 &&
        error("矩形轨迹的 n_steps 必须是 4 的倍数")
    # 机械臂直态沿世界 -Z 方向；矩形中心关于 XY 平面镜像至 [0,300,-500] mm。
    # 总臂长约 520 mm，因此基座安装基准取 [0,0,-150] mm，保证路径可达。
    base_z_offset = -150.0
    initial_controls = zeros(2N_SEGMENTS)
    target_y = 300.0
    center_x, center_z = 0.0, -500.0
    half_x, half_z = 40.0, 40.0
    shape_trajectory = if shape == :rectangle
        p1 = Point3f(center_x-half_x, target_y, center_z-half_z)
        p2 = Point3f(center_x+half_x, target_y, center_z-half_z)
        p3 = Point3f(center_x+half_x, target_y, center_z+half_z)
        p4 = Point3f(center_x-half_x, target_y, center_z+half_z)
        n_edge = n_steps ÷ 4
        path = Point3f[]
        append!(path, [Point3f(x, target_y, p1[3]) for x in range(p1[1], p2[1]; length=n_edge)])
        append!(path, [Point3f(p2[1], target_y, z) for z in range(p2[3], p3[3]; length=n_edge)])
        append!(path, [Point3f(x, target_y, p3[3]) for x in range(p3[1], p4[1]; length=n_edge)])
        append!(path, [Point3f(p4[1], target_y, z) for z in range(p4[3], p1[3]; length=n_edge)])
        path
    elseif shape == :circle
        [Point3f(center_x + half_x*cos(t), target_y, center_z + half_z*sin(t))
         for t in range(0.0, 2pi; length=n_steps)]
    elseif shape == :infinity
        [Point3f(center_x + half_x*sin(t), target_y, center_z + half_z*sin(2t))
         for t in range(0.0, 2pi; length=n_steps)]
    else
        [begin
             t = v
             Point3f(center_x + half_x*sin(t), target_y,
                     center_z + half_z*sin(2t))
         end for v in range(-pi/2, pi/2; length=n_steps)]
    end
    # Use the previously reliable Cartesian continuation only to find a good
    # IK solution at the shape start. These points are NOT the executed approach.
    # The executed approach below interpolates actuator controls instead.
    straight_tip = calculate_end_position(zeros(N_SEGMENTS), zeros(N_SEGMENTS),
                                          base_z_offset; bend_sides)
    approach_steps = N_APPROACH_STEPS
    dx = Float64(shape_trajectory[1][1] - straight_tip[1])
    dy = Float64(shape_trajectory[1][2] - straight_tip[2])
    initial_azimuth = clamp(rad2deg(atan(-bend_sides[1]*dy,
                                          bend_sides[1]*dx)), -150.0, 150.0)
    seed_bend = R_DRIVE * deg2rad(5.0)
    controls = [seed_bend, seed_bend, initial_azimuth, 0.0]
    for i in 1:approach_steps
        u = i / approach_steps
        blend = 3u^2 - 2u^3
        target = shape_trajectory[1]
        continuation_point = Point3f((1-blend)*straight_tip[1] + blend*target[1],
                                     (1-blend)*straight_tip[2] + blend*target[2],
                                     (1-blend)*straight_tip[3] + blend*target[3])
        controls = solve_ik(continuation_point, controls, base_z_offset; bend_sides)
    end
    shape_start_controls = copy(controls)

    straight_controls = zeros(2N_SEGMENTS)
    approach_controls = [begin
        u = i / approach_steps
        blend = 3u^2 - 2u^3 # smoothstep in actuator space
        (1 - blend) .* straight_controls .+ blend .* shape_start_controls
    end for i in 0:approach_steps]
    shape_start_index = length(approach_controls)
    n_total = shape_start_index + length(shape_trajectory) - 1

    joints_history = Vector{Vector{Point3f}}(undef, n_total)
    terminal_frames_history = Vector{Vector{Tuple{Point3f,Matrix{Float64}}}}(undef, n_total)
    rope_history = Matrix{Float64}(undef, N_SEGMENTS, n_total)
    rotation_history = Matrix{Float64}(undef, N_SEGMENTS, n_total)
    controls = copy(initial_controls)
    realized_trajectory = Vector{Point3f}(undef, n_total)

    for i in 1:n_total
        if i <= shape_start_index
            controls = approach_controls[i]
        else
            shape_index = i - shape_start_index + 1
            controls = solve_ik(shape_trajectory[shape_index], controls, base_z_offset;
                                bend_sides)
        end
        rope_history[:, i] = controls[1:N_SEGMENTS]
        rotation_history[:, i] = controls[N_SEGMENTS + 1:end]
        joints_history[i], terminal_frames_history[i] = calculate_multi_segment_kinematics(
            rope_history[:, i], rotation_history[:, i], base_z_offset; bend_sides)
        realized_trajectory[i] = joints_history[i][end]
        if i >= shape_start_index
            target = shape_trajectory[i - shape_start_index + 1]
            residual = norm(Float64.(realized_trajectory[i] - target))
            residual <= 5.0 || error("$(shape) 轨迹第 $(i-shape_start_index+1) 点在物理弯曲侧 $(bend_sides) 下不可达；末端误差 $(round(residual; digits=2)) mm，目标轨迹未改变")
        end
    end

    ArmState(joints_history, terminal_frames_history, rope_history, rotation_history,
             realized_trajectory, rope_history, rotation_history, base_z_offset, shape,
             shape_start_index, bend_sides, shape_trajectory)
end

function visualize_arm(state::ArmState)
    GLMakie.activate!()
    fig = Figure(size=(1200, 900), fontsize=14)
    ax = Axis3(fig[1, 1:3], title="Two-Segment PCC Path Tracking",
        xlabel="X (mm)", ylabel="Y (mm)", zlabel="Z (mm)",
        limits=(-200, 200, nothing, nothing, nothing, nothing), aspect=:data)
    joint_obs = Observable(state.joints_history[1])
    trace = Observable(state.joints_history[1][end:end])
    for seg in 1:N_SEGMENTS
        lo = (seg - 1) * (N_LINKS + 1) + 1
        hi = lo + N_LINKS + 1
        lines!(ax, lift(j -> j[lo:hi], joint_obs), color=SEG_COLORS[seg], linewidth=6)
    end
    scatter!(ax, joint_obs, color=:black, markersize=6)
    frame_lines = [Observable(Point3f[]) for _ in 1:(N_SEGMENTS * 3)]
    for seg in 1:N_SEGMENTS, axis_id in 1:3
        idx = (seg - 1) * 3 + axis_id
        lines!(ax, frame_lines[idx], color=FRAME_COLORS[axis_id], linewidth=3)
    end

    function update_terminal_axes!(frame_index)
        for seg in 1:N_SEGMENTS
            origin, R = state.terminal_frames_history[frame_index][seg]
            for axis_id in 1:3
                direction = R[:, axis_id]
                endpoint = Point3f(origin .+ 35.0 .* direction)
                frame_lines[(seg - 1) * 3 + axis_id][] = Point3f[origin, endpoint]
            end
        end
    end
    update_terminal_axes!(1)

    lines!(ax, state.target_trajectory, color=:orange, linewidth=3,
           label="Cartesian Target Path")
    lines!(ax, trace, color=:red, linewidth=3, label="End-effector Path")
    axislegend(ax, position=:lt)
    slider = Slider(fig[2, 1:3], range=1:length(state.trajectory), startvalue=1,
                    snap=true, update_while_dragging=true)
    Label(fig[3, 1:3], lift(t -> "仿真帧：$(Int(t)) / $(length(state.trajectory))", slider.value))
    play = Button(fig[4, 1]; label="播放 / 暂停")
    Label(fig[4, 2:3], "拖动时间滑块查看构型；在三维视图中拖动鼠标可旋转视角。")
    on(slider.value) do t
        idx = Int(t)
        joint_obs[] = state.joints_history[idx]
        update_terminal_axes!(idx)
        trace[] = [j[end] for j in state.joints_history[1:idx]]
    end
    playing = Ref(false)
    on(play.clicks) do _
        playing[] = !playing[]
        if playing[]
            @async while playing[] && isopen(fig.scene)
                current = Int(slider.value[])
                slider.value[] = current >= length(state.trajectory) ? 1 : current + 1
                sleep(0.04)
            end
        end
    end
    display(fig)
    fig
end

function visualize_results(state::ArmState)
    fig = Figure(size=(1100, 700), fontsize=14)
    t = 1:length(state.trajectory)
    for seg in 1:N_SEGMENTS
        ax1 = Axis(fig[seg, 1], title="Segment $(seg) rope drive", xlabel="Time Step", ylabel="ΔL (mm)")
        lines!(ax1, t, state.rope_drive_history[seg, :], color=SEG_COLORS[seg], linewidth=2.5)
        ax2 = Axis(fig[seg, 2], title="Segment $(seg) axial rotation", xlabel="Time Step", ylabel="Angle (°)")
        lines!(ax2, t, state.axial_rotation_history[seg, :], color=SEG_COLORS[seg], linewidth=2.5)
    end
    Label(fig[0, :], "Two-Segment PCC Callable Control Results", fontsize=18, font=:bold)
    display(fig)
    fig
end

"""用 Makie 绘制 run_live 返回并保留在内存中的二维跟踪结果。"""
function visualize_tracking_results(saved_runs)
    shapes = ((:rectangle, "矩形"), (:circle, "圆形"),
              (:infinity, "∞ 形"), (:s_curve, "S 形"))
    fig = Figure(size=(1200, 900), fontsize=15)
    for (panel, (shape, title)) in enumerate(shapes)
        row, col = (panel + 1) ÷ 2, mod1(panel, 2)
        ax = Axis(fig[row, col]; title, xlabel="X / mm", ylabel="Z / mm",
                  aspect=DataAspect())
        shape_runs = filter(run -> run.shape == shape, saved_runs)
        for (run_index, run) in enumerate(shape_runs)
            rows = run.rows
            suffix = length(shape_runs) > 1 ? " #$run_index" : ""
            lines!(ax, getproperty.(rows, :target_x_mm), getproperty.(rows, :target_z_mm);
                   color=:magenta, linestyle=:dash, linewidth=2, label="期望" * suffix)
            lines!(ax, getproperty.(rows, :measured_x_mm), getproperty.(rows, :measured_z_mm);
                   color=:dodgerblue, linewidth=2.5, label="实测" * suffix)
        end
        if isempty(shape_runs)
            text!(ax, 0.5, 0.5; text="尚无该形状的完成记录", space=:relative,
                  align=(:center, :center), color=:gray)
        else
            axislegend(ax; position=:lt)
        end
    end
    Label(fig[0, :], "两段 PCC 轨迹跟踪结果（X–Z 平面投影）", fontsize=19, font=:bold)
    display(fig)
    fig
end

# Interactive example (run manually after including this file):
# arm_state2 = compute_trajectory(shape=:rectangle)
# visualize_arm(arm_state2)
# visualize_results(arm_state2)

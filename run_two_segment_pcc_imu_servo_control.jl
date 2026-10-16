# 按顺序逐行运行。方向配置默认使用已确认并保存的标定；启动双 IMU 闭环后，
# 仍需先点击“两段归零回直”，再开始轨迹跟踪。需要重新判定方向时可单独运行初始化。
include(joinpath(@__DIR__, "two_segment_pcc_path_tracking.jl"))
include(joinpath(@__DIR__, "dual_segment_imu_support.jl"))
include(joinpath(@__DIR__, "two_segment_pcc_imu_servo_control.jl"))
include(joinpath(@__DIR__, "dual_segment_hardware_control.jl"))

# 1) 先读取已保存标定；轨迹驱动计划必须在选定物理弯曲侧之后生成。
#    位于 Y=300 mm 平面、中心 X=0/Z=-500 mm 的四种空间目标不随标定改变。
# 2) 默认复用已确认的方向标定，设为 true 可重新打开初始化窗口。
#    打开初始化窗口后，先手动摆到两段直态，四路滑轨居中并停止，再点击
#    “外部确认直态：仅记录双段直态”。该操作记录完整安装姿态，不要求 IMU
#    X 轴朝世界竖直；基座倾斜会由直态安装基准吸收。标定写入 v3 文件。
#    记录后才可用“IMU 自动回直”；自动回直不会重写这份操作者确认的基准。
#    此后可设回 false；初始化窗口仍保留四路手动点动。
#    若只想手动回直而不重新判定方向，也可打开窗口后不碰方向按钮再完成。
run_direction_initialization = true
dual_hardware_config = DualSegmentHardwareControl.Config()
saved_two_segment_calibration = DualSegmentHardwareControl.load_calibration()
initialization_plan = TwoSegmentPCCIMUServoControl.make_reference(
    compute_trajectory(shape=:rectangle,
        bend_sides=saved_two_segment_calibration.bend_sides))
two_segment_calibration = run_direction_initialization ?
    DualSegmentHardwareControl.run_initialization(
        initialization_plan; config=dual_hardware_config) :
    saved_two_segment_calibration

# 3) 直接从纯 PCC 轨迹状态读取两段弯曲驱动量与轴向角；不调用自然坐标模型。
#    每种形状含 15 秒平滑过渡和 25 秒跟踪。
two_segment_shapes = (:rectangle, :circle, :infinity, :s_curve)
two_segment_hardware_plans = Dict{Symbol,Any}()
for shape in two_segment_shapes
    two_segment_hardware_plans[shape] = TwoSegmentPCCIMUServoControl.make_reference(
        compute_trajectory(shape=shape,
            bend_sides=two_segment_calibration.bend_sides))
end

# 4) 新窗口先按“两段归零回直”；随后可分别启动矩形、圆形、∞形、S形。
#    每段结束后会自动按第一段、第二段顺序回直，之后可继续选择下一轨迹。
#    同一 Julia 会话中只需修改下方 first_tracking_kp 并重跑这一调用，
#    不用重新生成轨迹或重新初始化；必须先关闭旧窗口并确认 CH1/CH3 停止。
two_segment_session = DualSegmentHardwareControl.run_live(two_segment_hardware_plans;
    config=dual_hardware_config,
    calibration=two_segment_calibration, send_servo=true,
    first_tracking_kp=dual_hardware_config.first.kp)

# two_segment_session.history 含两段的目标角、IMU 局部弯曲角与四路指令。

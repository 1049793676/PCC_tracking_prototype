# 逐行运行示例：先加载函数，再按需运行连接、标定和一种控制入口。
# 控制入口会打开窗口；点击“开始控制”后才开始跟踪。不要一次运行整份文件。
include(joinpath(@__DIR__, "dual_imu_feetech_test.jl"))

# ESP32 串口默认取第二个 COM；如不一致，修改函数文件中的 ESP32_PORT_OVERRIDE。
controller = open_controller()
imu_data = read_imu_pair!(controller; seconds=2.0)

# 当前轴向角偏差排查的第一步：只画两只 IMU 上报的绝对姿态。
# 选中并运行下一行；转动 IMU 或机械臂时观察 RGB 三轴是否按实物变化。
# 此入口不发送舵机命令，不做滤波、对齐、轴映射或 PCC 拟合。
raw_orientation_test = pcc_imu_raw_orientation_test!(controller)
# 观察完成后点击窗口中的“停止读取”，再继续运行其他测试。

# 独立手动拟合测试，不需要运行下面的弯曲标定或轨迹控制入口。
# 打开窗口即读取 IMU 并开始拟合，不驱动电机；启动姿态会被当成拟合零位。
# 最好在两段回直、轴向中位时打开；若要重新设零位，点击窗口中的重设按钮。
# 拖动四个滑轨，实时比较橙色 PCC 臂形与样机；结束或紧急停止后再运行其他控制入口。
manual_fit = pcc_imu_manual_fit_test!(controller)

# 第一段：弯曲电机 1 / IMU1；第二段：弯曲电机 2 / IMU2。
# 标定会点动弯曲电机。启动圆轨迹前先运行这两行。
first_calibration = calibrate_imu_servo!(controller; servo_id=1, imu_id=1)
second_calibration = calibrate_imu_servo!(controller; servo_id=2, imu_id=2, relative_to_imu_id=1)

# 双段固定角度反馈，仅控制两台弯曲电机；轴向关节保持固定。
dual_tracking = dual_imu_feedback!(controller, first_calibration, second_calibration; first_target_deg=0.0, second_target_deg=0.0, speed_rpm=(100.0, 20.0), max_step_ticks=(800, 800), max_excursion_ticks=7*TICKS_PER_TURN, kp=(0.1, 0.08), ki=(0.001, 0.001), kd=(0.001, 0.001), loop_pause=0.02, assume_axial_fixed=true)

# PCC 圆轨迹参考不打开串口、不驱动电机；窗口内也能选择矩形轨迹。
# 两种轨迹均用 15 秒从直态到起点，停留 2 秒，再用 60 秒跟踪形状。
# 目标角使用连续速度插值；矩形在角点减速到零再转向，圆形平滑启停。
circle_reference = make_circle_reference()

# 实时绘图版本：窗口中可选圆形、矩形或输入两段固定角度；也可结束、紧急停止和回中回直。
# 固定角度控制与回中回直可直接启动；圆形、矩形跟踪前需在直态记录四元数。
circle_tracking = pcc_circle_feedback!(controller, first_calibration, second_calibration; circle_reference, axial_signs=(1, 1), speed_rpm=(100.0, 20.0), max_step_ticks=(800, 800), max_excursion_ticks=7*TICKS_PER_TURN, kp=(0.16, 0.2), ki=(0.001, 0.001), kd=(0.001, 0.001), loop_pause=0.02)

# 离线绘图版本：轨迹跟踪时只采样与控制，结束后在同一窗口一次性绘制结果。
# 与上面的实时版本二选一，不要同时启动两个窗口。
offline_circle_tracking = pcc_circle_feedback_offline!(controller, first_calibration, second_calibration; circle_reference, axial_signs=(1, 1), speed_rpm=(100.0, 50.0), max_step_ticks=(800, 800), max_excursion_ticks=7*TICKS_PER_TURN, kp=(0.16, 0.2), ki=(0.001, 0.001), kd=(0.001, 0.001), loop_pause=0.02)

# 测试结束后关闭 ESP32 USB 串口。
close(controller)

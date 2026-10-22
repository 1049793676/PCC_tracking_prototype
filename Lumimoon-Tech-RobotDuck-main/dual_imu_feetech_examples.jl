# 逐行运行示例。先运行下一行加载函数，再按需单独运行后面的某一行。
# 不要一次运行整份文件：其中包含多个会驱动舵机的调用。
include(joinpath(@__DIR__, "dual_imu_feetech_test.jl"))

# ── 连接及 IMU ────────────────────────────────────────────────────────
# 只接收 IMU 数据，不发送命令
connected = test_esp32_connection("COM8")
# 确认 ESP32 对应的 COM 口
ports = list_com_ports()
# 默认第二个 COM；不对时修改函数文件中的 ESP32_PORT_OVERRIDE
controller = open_controller()
# 验证 Julia 与 ESP32 双向通信
ping!(controller)
imu_data = read_imu_pair!(controller; seconds=2.0)
scan_imus!(controller)

# ── LSC-16 轴向舵机：第一段通道 1，第二段通道 2 ─────────────────────
# 先烧录更新后的 ESP32 固件。控制板单独供电，ESP32 与控制板共地。
# 脉宽 1500 为中位指令，500～2500 是协议范围，机械安装后的实际可用范围需自行核对。
# 在合适的机械位置下，先运行一条 1500，再单独运行 1600 或 1400 观察转向。
# 第一段通道 1 到中位
axial_servo_position!(controller, 1, 1500; move_ms=1000)
# 第一段通道 1 到指定位置
axial_servo_position!(controller, 1, 1600; move_ms=1000)
# 第一段通道 1 向另一侧
axial_servo_position!(controller, 1, 1400; move_ms=1000)
# 第二段通道 2 到中位
axial_servo_position!(controller, 2, 1500; move_ms=1000)
# 第二段通道 2 到指定位置
axial_servo_position!(controller, 2, 1600; move_ms=1000)
# 返回 ACK,A 只能证明 ESP32 已把指令送到 LSC-16 串口；此控制板没有提供舵机位置回读。

# ── 微雪总线弯曲舵机 ──────────────────────────────────────────────────
# 只读查询弯曲电机 ID 1
current_ticks = servo_position!(controller, 1)
# 只读，预期模式 4 或 0
servo_mode!(controller, 1)
# 只读；(0,0) 表示多圈限位设置
servo_limits!(controller, 1)
# 开启位置保持力矩
set_servo_torque!(controller, true; id=1)
move_servo_relative!(controller, 50; id=1, speed_rpm=5.0)
move_servo_relative!(controller, -50; id=1, speed_rpm=5.0)
# 写入多圈限位并保持当前位置
enable_servo_multiturn!(controller, 1)

# ── 双段姿态读取与标定 ───────────────────────────────────────────────
# 第一段 IMU2 投影角
read_z_bend_angle!(controller; imu_id=2)
# 第一段 IMU2、第二段 IMU1 角度诊断；双段反馈使用平面角差
read_two_segment_angles!(controller)
# 轴向舵机转动时，上述 Z 投影角不是实际弯曲角。
# 第一段弯曲电机 2 / IMU2
first_calibration = calibrate_imu_servo!(controller; servo_id=2, imu_id=2)
# 第二段弯曲电机 1 / (IMU1 - IMU2) 平面角差；测量定义已改变，必须重新运行本行标定
second_calibration = calibrate_imu_servo!(controller; servo_id=1, imu_id=1, relative_to_imu_id=2)
# ── 单段弯曲控制 ─────────────────────────────────────────────────────
# 原有参数保持不变
fixed_tracking = imu_feedback!(controller, first_calibration; target_deg=10.0, speed_rpm=100.0, max_step_ticks=800, max_excursion_ticks=7*TICKS_PER_TURN, kp=0.8, ki=0.03, kd=0.08, loop_pause=0.02)
# 窗口无法操作时请求关闭弯曲舵机扭矩
fixed_tracking.emergency_stop()
imu_feedback!(controller, first_calibration; target_deg=0.0)
tracking = imu_sine_feedback!(controller, first_calibration; offset_deg=0.0, amplitude_deg=10.0, period_s=8.0, phase_lead_s=0.0)
# ── 双段弯曲舵机控制：电机 2 控第一段，电机 1 控第二段 ──────────────
# 需要先运行上面的 first_calibration 和 second_calibration。
# 当前反馈模型只在两个轴向关节固定且 IMU 轴对齐时适用；两个轴向舵机运动时不要启动此闭环。
# 以下调用只控制两台微雪弯曲电机；LSC-16 的两台轴向舵机目前由上方 axial_servo_position! 单独控制。
dual_tracking = dual_imu_feedback!(controller, first_calibration, second_calibration; first_target_deg=0.0, second_target_deg=0.0, speed_rpm=(100.0, 20.0), max_step_ticks=(800, 800), max_excursion_ticks=7*TICKS_PER_TURN, kp=(0.1, 0.08), ki=(0.001, 0.001), kd=(0.001, 0.001), loop_pause=0.02, assume_axial_fixed=true)

# ── PCC 圆轨迹：两台弯曲电机使用 IMU 姿态反馈；轴向舵机直接接收 PCC 目标角 ──
# 只生成离线参考，不打开串口，也不驱动电机；沿用已保存的弯曲侧配置。
circle_reference = make_circle_reference()
# 先运行上面的两条标定；下面这一行只打开窗口，点击“开始控制”才发送运动指令。
# PID 数值沿用当前双段示例，不在圆轨迹控制中另设一套参数。
circle_tracking = pcc_circle_feedback!(controller, first_calibration, second_calibration; circle_reference, axial_signs=(1, 1), speed_rpm=(100.0, 20.0), max_step_ticks=(800, 800), max_excursion_ticks=7*TICKS_PER_TURN, kp=(0.16, 0.2), ki=(0.001, 0.001), kd=(0.001, 0.001), loop_pause=0.02)
# 新版离线绘图入口：单独运行下一行，不与上面的实时窗口同时启动；到圆轨迹终点自动保持位置并一次性绘图。
offline_circle_tracking = pcc_circle_feedback_offline!(controller, first_calibration, second_calibration; circle_reference, axial_signs=(1, 1), speed_rpm=(100.0, 50.0), max_step_ticks=(800, 800), max_excursion_ticks=7*TICKS_PER_TURN, kp=(0.16, 0.2), ki=(0.001, 0.001), kd=(0.001, 0.001), loop_pause=0.02)
# 在窗口操作：先让两段回直态 → 点击“轴向回中 1500 并记录两段直态四元数” → 开始控制。
# 仅当第二段目标角和实测角持续处于相反方向时才触发方向保护；正常圆轨迹转向不会触发该检查。
# 右上三维视图：虚线为目标圆，蓝色为目标 PCC，橙色为同时拟合两只 IMU 绝对姿态的 PCC；可拖动视角查看。
# 两段末端各有三根姿态轴：实线是目标 PCC 姿态，虚线是 IMU 绝对四元数姿态；X 红、Y 绿、Z 蓝。
# 第一段虚线来自 IMU2，第二段虚线直接来自 IMU1 的绝对姿态；两组姿态轴在记录直态时对齐到 PCC 模型。
# 右侧第 1～2 行为三维操作臂和两段末端姿态轴；第 3 行为四电机诊断、姿态误差及拟合残差。
# 新增的两幅轴向曲线分别显示通道 1、2 已发送并获控制板确认的脉宽（μs）；虚线是 1500 中位。
# 圆轨迹绘图默认每 0.2 秒刷新一次，不改变控制采样；终端每约 2 秒输出 IMU、拟合、串口查询和指令的分项耗时。
# 橙色 PCC 臂形由两只 IMU 完整姿态拟合；圆轨迹弯曲 PID 使用拟合角，轴向按 PCC 目标角直接换算脉宽。
# IMU 四元数先滤波；拟合角若只在一帧突变，该帧不绘图、也不发送新的电机目标。
# 姿态拟合残差超过 30° 时正常结束并保持弯曲电机位置；右下角会保留停止原因。
# “回中回直”可在运行圆轨迹时点击：两段弯曲目标改为 0°，两台轴向舵机目标改为 1500；保持闭环直到点击结束控制。
# “读取两段角度”同时显示两只 IMU +X 末端方向的真实三维夹角，可与第二段弯曲角交叉核对。
# 结束控制保持弯曲电机当前位置；紧急停止关闭弯曲电机扭矩并停止轴向新目标。

# ── 圆轨迹第二段角度排查 ─────────────────────────────────────────────
# 不启动双段或圆轨迹闭环。先让两段接近直态，弯曲电机保持位置，再把两个轴向通道设为 1500。
axial_servo_position!(controller, 1, 1500; move_ms=1000)
axial_servo_position!(controller, 2, 1500; move_ms=1000)
sleep(1.2)
# 此时记录两只 IMU 的直态基准；下列诊断函数本身只读 IMU。
diagnostic_baseline = fresh_imu_pair!(controller)
diagnostic_at_center = diagnose_circle_angles!(controller, diagnostic_baseline; axial_pulses=(1500, 1500))
# 只改变第一段轴向通道 1，第二段轴向和两台弯曲电机都不动。
axial_servo_position!(controller, 1, 1600; move_ms=1000)
sleep(1.2)
diagnostic_at_1600 = diagnose_circle_angles!(controller, diagnostic_baseline; axial_pulses=(1600, 1500))
# 测完将第一段轴向通道 1 返回 1500。
axial_servo_position!(controller, 1, 1500; move_ms=1000)

# 测试结束后关闭 ESP32 USB 串口
close(controller)

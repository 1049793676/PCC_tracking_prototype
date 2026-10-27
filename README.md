# Two-segment PCC hardware tracking

This directory is a portable standalone copy of the two-segment PCC path planner,
dual-IMU initialization, and four-channel hardware tracker. It does not include
or load the Rible natural-coordinate model.

## Set up on another computer

Install Julia 1.10 or newer. The matching ESP32 Arduino firmware is included at
`firmware/dual_i2c_imu_bridge.ino`; open that sketch in Arduino IDE and upload it
to the ESP32-S3 before running the Julia controller. The sketch provides the two
IMU I2C buses and the serial link to the servo controller. Keep the `firmware`
directory with the project when moving computers.

From this directory, run:

```julia
using Pkg
Pkg.activate(".")
Pkg.instantiate()
include("run_two_segment_pcc_imu_servo_control.jl")
```

For the standalone geometry tests, run `include("test_portable_pcc.jl")` in the
same activated Julia session.

The run script computes the four PCC references, loads the saved direction and
straight-pose calibration, and opens the live control window. The default serial
port remains `COM8`; change `port` in `dual_segment_hardware_control.jl` if the
ESP32 enumerates differently on the new computer. Begin with the window's
two-section zeroing action before starting a trajectory.

## Contents

- `two_segment_pcc_path_tracking.jl`: self-contained two-section PCC geometry,
  inverse kinematics, four path shapes, and Makie simulation visualization.
- `two_segment_pcc_imu_servo_control.jl`: direct adapter from stored PCC section
  bend/axial histories to hardware references. Effective rope drive is read as
  `R_DRIVE * bend_angle`; no second model-based rope conversion is performed.
- `dual_segment_imu_support.jl`: ESP32 quaternion parsing, SO(3) filtering,
  actuator commands, and configuration validation.
- `dual_segment_hardware_control.jl`: calibration, commissioning, sequential
  IMU zeroing, live tracking, plots, and saved-in-memory trajectory results.
- `firmware/dual_i2c_imu_bridge.ino`: ESP32-S3 Arduino sketch for both IMUs and
  the servo-controller serial bridge. It is byte-for-byte identical to the
  corresponding sketch in the original `rible.jl` project.
- `two_segment_*_v3.toml`: the current saved direction and straight-pose
  calibration. Keep these files with the project when moving computers.

The project manifest is intentionally generated on the target computer by
`Pkg.instantiate()`, so it resolves binary Makie/serial dependencies for that OS.

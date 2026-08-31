# MPC_fixed 反向规格说明

## 1. 范围

本规格基于 `MPC_fixed` 目录源码反向整理，范围包括三系统总控 GUI、MPC 求解、目标轨迹、倾角传感器、调平执行器、方位执行器、举升/俯仰执行器和公共 Modbus/编码器工具。根目录的独立调试脚本作为历史来源和参考，不属于当前主程序运行路径。

## 2. 技术栈

- 语言：Python。
- GUI：Tkinter / ttk。
- 串口：pyserial。
- 数值计算：numpy、scipy、osqp。
- 日志：CSV，`utf-8-sig` 编码。
- 通信协议：Modbus RTU 风格报文、CiA402 速度模式、实验中验证过的电动缸 `0x3308` 速度帧。

## 3. 模块结构

| 文件 | 观察到的职责 |
| --- | --- |
| `main.py` | 创建 Tk 根窗口并启动 `MasterControlApp`。 |
| `master_controller.py` | GUI、串口连接、模式管理、反馈线程、MPC 调度、命令分发、CSV 日志、停止逻辑。 |
| `mpc_controller.py` | 由 MATLAB/Simulink S-function 迁移来的 MPC 二次规划求解器。 |
| `target_trajectory.py` | 生成未来 3 步目标方向向量。 |
| `leveling_executor.py` | 四条调平腿的连接、速度下发、位置反馈、本地调平和倾角速度到四腿 rpm 的分配。 |
| `azimuth_executor.py` | 方位轴 CiA402 速度模式控制、状态字读取、位置读取、软件置零、多圈角度。 |
| `lift_executor.py` | 举升电缸速度控制、俯仰角编码器、多圈角度、角速度估计、前馈+PID 角速度控制。 |
| `tilt_sensor.py` | 倾角传感器串口接收线程、帧同步、角度解析、跳变过滤。 |
| `common.py` | CRC、限幅、保持寄存器读取、编码器累计和单位换算。 |

## 4. 架构和数据流

1. `main.py` 启动 Tkinter 主循环。
2. `MasterControlApp` 初始化传感器、调平、方位、举升和 MPC 子系统。
3. 倾角传感器通过后台线程更新最新 X/Y/Z 角。
4. 反馈轮询线程周期性读取调平、方位和俯仰反馈，并写 CSV。
5. MPC 模式下，总控周期性构造系统状态，生成目标轨迹，调用 `MPCController.compute()`。
6. MPC 输出四个系统级角速度。
7. 总控把系统级角速度分发到调平、方位和举升/俯仰执行器。
8. 执行器把角速度或 rpm 转换为串口报文下发。

## 5. 观察到的需求（EARS）

- The system shall provide a Tkinter GUI entry point through `MPC_fixed/main.py`.
- The system shall support three runtime modes: `IDLE`, `LOCAL_LEVELING`, and `MPC`.
- When starting local leveling, the system shall require the tilt sensor and all four leveling legs to be connected.
- When local leveling is active, the system shall compute four leg rpm commands from current X/Y tilt angles and send them at the configured control period.
- When starting MPC control, the system shall start CSV logging first; if the user cancels the save dialog, the system shall not start MPC.
- When MPC control starts, the system shall reset the MPC internal velocity memory and set the current sensor Z angle as the MPC Z zero.
- When MPC is active, the system shall build a state containing X/Y/Z tilt, azimuth angle, and lift/pitch angle.
- When MPC is active, the system shall generate a 3-step target direction sequence from `TargetTrajectory`.
- The MPC controller shall output `leveling_x_rate_rad_s`, `leveling_y_rate_rad_s`, `azimuth_rate_rad_s`, and `lift_rate_rad_s`.
- When the lift angle is lower than `40 deg`, the system shall force the actual azimuth rate command to zero while allowing MPC calculation to continue.
- When one MPC solve fails, the system shall keep the previous valid command instead of replacing it with the failed command.
- When MPC solve failures reach 3 consecutive cycles, the system shall stop all controlled actuators.
- When the last valid MPC command exceeds the configured timeout, the system shall stop all controlled actuators.
- When pitch angle is at least `69 deg`, the feedback thread shall stop lift motion and stop the active control loop if one is running.
- While not in `IDLE`, manual debug controls shall be rejected to avoid competing actuator commands.
- The leveling executor shall convert system-level X/Y tilt rates into four leg rpm commands using vehicle length, width, screw lead, and reduction ratio.
- The azimuth executor shall interpret positive rpm and positive azimuth angular rate as counter-clockwise.
- The lift executor shall convert target pitch angular velocity to motor rpm using an angle-dependent feedforward gain table and PID correction.
- The tilt sensor reader shall multiply decoded angles by `angle_sign`, currently `-1.0`.
- The CSV logger shall record sensor angles, actuator states, MPC outputs, solver status, target sequence, and four leg states.

## 6. MPC 算法规格

- State dimension: `Nx = 3` for `[Rx,Ry,Rz]`.
- Control dimension: `Nu = 4` for `[gx_rate, gy_rate, azimuth_rate, lift_rate]`.
- Prediction horizon: `Np = 3`.
- Control horizon: `Nc = 2`.
- The controller shall construct the current direction vector from rotation matrices using `S=[0,0,-1]`.
- The controller shall use an augmented state `[Rx,Ry,Rz,U1,U2,U3,U4]`.
- The controller shall solve a QP over control increments, not direct absolute control values.
- The controller shall update internal `U` by adding the first-step optimized increment.
- The controller shall constrain velocity, angle-derived safe velocity, and control increment.
- The controller shall clear output to zero if an exception or non-finite result occurs.
- The controller shall use OSQP with `eps_abs=1e-6`, `eps_rel=1e-6`, `max_iter=4000`, and `polish=True`.

## 7. 非功能观察

- The code is optimized for laboratory operation and manual supervision rather than unattended production control.
- Hardware protocol details are embedded as constants in executor classes.
- Several safety behaviors are implemented in total-control logic rather than in a separate safety layer.
- CSV writes are buffered and flushed periodically to reduce disk blocking.
- Serial reads and writes use locks per port or per actuator channel.
- The GUI and control loop both run in the same Python process; long blocking hardware calls can affect responsiveness.

## 8. 不确定点和待确认事项

- Current comments mention a 100 ms period in some places, but source defaults are `200 ms` for control, feedback, MPC, and `MPCController.T = 0.20`.
- `AzimuthExecutor.motor_gear_ratio` is currently `1.0`; actual transmission ratio needs calibration.
- Four-leg numbering and sign convention are inherited from the experimental code and must be confirmed on the hardware bench.
- The current tilt sensor sign correction assumes the sensor is reverse-mounted.
- Several lift electric-cylinder feedback reads are present but commented out in `lift_executor.py`.
- Root-level standalone scripts contain earlier test functionality that is not fully ported into `MPC_fixed`.

## 9. Acceptance Criteria Inferred From Code

- Running `.venv\Scripts\python.exe MPC_fixed/main.py` opens the total-control GUI (use the project `.venv`; the system `py` lacks `osqp`).
- Running `.venv\Scripts\python.exe MPC_fixed/mpc_controller.py` performs an offline MPC solve without connecting hardware when dependencies are installed.
- Starting MPC without choosing a CSV file does not start actuator control.
- A non-finite MPC output cannot be passed through to actuators.
- Manual debug actions cannot be used while local leveling or MPC mode is active.
- `stop_all()` stops all active actuator groups and closes CSV logging after writing a final row.

## 10. Recommendations

- Move hardware constants and safety thresholds into a versioned configuration file.
- Add an experiment checklist documenting COM port mapping, four-leg numbering, zeroing sequence, and direction verification results.
- Add unit tests for frame construction, encoder accumulation, target trajectory generation, and MPC failure handling.
- Add a simulation mode that replaces serial ports with mock executors before running new MPC parameters on hardware.
- Resolve the 100 ms vs 200 ms documentation mismatch by choosing and documenting one experiment standard.

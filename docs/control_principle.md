# 控制原理说明

## 总体信号流

当前主程序位于 `MPC_fixed`。入口 `main.py` 创建 Tkinter 窗口并实例化 `MasterControlApp`。总控对象负责连接各串口、启动反馈轮询、读取传感器状态、生成 MPC 目标轨迹、调用 MPC 求解器，并把求解结果分发到三个执行系统。

运行 MPC 时，核心链路如下：

```text
倾角传感器 + 方位编码器 + 俯仰编码器
        ↓
master_controller.get_system_state()
        ↓
target_trajectory.compute(t, dt)
        ↓
mpc_controller.MPCController.compute(state)
        ↓
master_controller.apply_mpc_command(cmd)
        ↓
调平四腿 / 方位轴 / 举升俯仰执行器
```

## MPC 输入状态

`MPCController.compute(state)` 读取以下主要字段：

- `x_angle_deg`：调平 X 轴倾角，单位 deg。
- `y_angle_deg`：调平 Y 轴倾角，单位 deg。
- `z_angle_deg`：MPC 使用的相对 Z 角，单位 deg。总控在 MPC 启动时把当前传感器 Z 角记为零点。
- `azimuth_angle_deg`：方位角 `b`，单位 deg。
- `lift_angle_deg` / `pitch_angle_deg`：举升/俯仰角 `a`，单位 deg。
- `target_sequence`：未来 `Np=3` 步目标方向向量，长度为 9，形如 `[Rx1,Ry1,Rz1, Rx2,Ry2,Rz2, Rx3,Ry3,Rz3]`。

如果没有 `target_sequence`，求解器也兼容 `target_vector=[Rx,Ry,Rz]`，并把单个目标向量复制到整个预测时域。

## 目标轨迹

`TargetTrajectory` 默认生成一个先俯仰后方位的轨迹：

- 俯仰目标角 `tar_a = 60 deg`。
- 俯仰阶段总时间 `time_a = 20 s`，之后开始方位运动。
- 俯仰最大角速度 `wa = 3.4 deg/s`。
- 方位最大角速度 `wb = 3.0 deg/s`。
- 俯仰和方位的加速段时间均为 `2 s`。

每个预测步会把目标俯仰角 `a` 和方位角 `b` 转换成目标方向向量 `[Rx,Ry,Rz]`。转换使用阵面指向基向量 `S=[0,0,-1]` 和旋转矩阵 `Mb`、`Ma`：

```text
S_target = Mb.T * Ma.T * S
```

## MPC 模型

`MPCController` 由 MATLAB/Simulink S-function `MPC_fun` 移植而来。内部维度为：

- 状态量 `Nx = 3`：方向向量 `[Rx,Ry,Rz]`。
- 控制量 `Nu = 4`：四个系统级角速度。
- 预测步长 `Np = 3`。
- 控制步长 `Nc = 2`。

控制量顺序固定为：

```text
U[0] = 调平 X 轴倾角速度，rad/s
U[1] = 调平 Y 轴倾角速度，rad/s
U[2] = 方位角速度，rad/s，逆时针为正
U[3] = 举升/俯仰角速度，rad/s
```

当前方向向量由以下旋转链计算：

```text
S1 = Magz.T * Magx.T * Magy.T * Mb.T * Ma.T * S
```

其中 `gx, gy, gz` 来自倾角传感器，`b` 来自方位角，`a` 来自俯仰角。离散模型使用 `A1 = I`，并根据当前姿态构造 `B1 = T * Magz.T * M`。增广状态为：

```text
kesi = [Rx, Ry, Rz, U1, U2, U3, U4]
```

决策变量是未来 `Nc` 步控制增量 `delta_U`，而不是绝对速度。求解成功后，控制器执行：

```text
U = U + delta_U[0:Nu]
```

这样可以让输出速度随周期连续变化。

## 优化目标和约束

目标函数是二次规划，主要由三部分组成：

- 预测方向向量与目标方向向量的误差，权重 `Q_weight = 1300`。
- 控制增量惩罚 `R0`，用于限制控制变化过猛。
- 位置/角度相关惩罚 `P0`，当前主要作用于调平 X/Y 通道。

约束包括：

- 速度限幅 `vel_max_deg_s`，四个通道分别限制。
- 角度限位 `pos_min_deg` / `pos_max_deg` 转换出的安全速度限幅。
- 控制增量限幅 `delta_u_limit_deg_s = 1.5 deg/s`。
- 数值保护：求解异常或出现 NaN/inf 时，MPC 输出清零并标记失败。

QP 求解器是 OSQP。总控对失败结果的处理策略是：单次失败不覆盖上一拍有效命令；连续失败 3 次停止；若上一拍命令超时也停止。

## 执行器映射

### 调平系统

MPC 输出 `leveling_x_rate_rad_s` 和 `leveling_y_rate_rad_s` 后，`LevelingExecutor.tilt_rate_to_leg_rpm()` 按车体长度 `1565 mm`、宽度 `1215 mm` 把倾角速度换算成四条腿的线速度，再按丝杠导程 `5 mm`、减速比 `5` 换算为电机 rpm。

四腿分配关系为：

```text
leg0 =  pitch - roll
leg1 =  pitch + roll
leg2 = -pitch + roll
leg3 = -pitch - roll
```

电动缸速度帧沿用已验证的 `0x3308` 速度帧，并做 `max_motor_rpm` 限幅。

### 方位系统

方位执行器使用 CiA402/Modbus 速度模式。总控优先下发 `azimuth_rate_rad_s`，执行器按：

```text
rpm = azimuth_rate_rad_s * 60 / (2*pi) * motor_gear_ratio
```

转换成电机 rpm。当前 `motor_gear_ratio = 1.0`，后续应按实际机构标定。正值表示逆时针，负值表示顺时针。

### 举升/俯仰系统

举升/俯仰执行器优先接收 `lift_rate_rad_s`。内部先转换为 deg/s，再通过“角度相关前馈表 + 角速度 PID 修正”得到电动缸 rpm：

- 前馈表 `pitch_gain_table` 表示不同俯仰角附近 `1 rpm` 电缸速度对应多少 `deg/s` 俯仰角速度。
- PID 的误差是 `目标俯仰角速度 - 实际俯仰角速度`。
- PID 修正量有限幅，默认 `50 rpm`，前馈承担主要输出，PID 只做修正。

俯仰角由与方位系统相同的 `0x6064/F010` 编码器位置读取逻辑计算，并做软件置零、多圈累计和角速度滤波。

## 总控保护逻辑

总控包含几类保护：

- MPC 启动时强制开启 CSV 记录，取消保存则不启动。
- MPC 启动时重置内部速度记忆 `U`，避免沿用旧实验速度。
- 俯仰角低于 `40 deg` 时，MPC 可继续求解，但实际方位角速度被强制置 0。
- 反馈线程检测到俯仰角达到 `69 deg` 或更高时停止举升；若控制循环正在运行则触发全部停止。
- 举升最低限位（软限位）：以当前硬件原始俯仰角为基准，限位设在其上方 `0.05°`；俯仰角处于限位及以下时，举升只准上升、禁止下降（下降方向角速度被清零）。该限位锚定原始 count，软件"角度置零"不影响。
- 手动调试按钮要求总控处于 `IDLE`，避免和本地调平或 MPC 循环抢占执行器。
- `stop_all()` 会停止调平、方位，并对举升停止命令重复发送 3 次。

## 需要实机确认的假设

- 方位 `motor_gear_ratio` 当前为 `1.0`，实际传动比需要标定。
- 四条调平腿编号和正负方向沿用原调平实验代码，必须以低速动作确认。
- 倾角传感器当前按反向安装处理，`angle_sign = -1.0`。传感器安装方向变化后需要修改并重新验证。
- README 中历史描述曾提到 100 ms 周期，但当前源码默认控制、反馈和 MPC 周期均为 `200 ms`。

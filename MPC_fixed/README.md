# MPC_fixed 主程序说明

本目录是当前可运行的三系统总控程序，负责把调平、方位、举升/俯仰三个执行系统接入同一个 GUI，并周期性调用 MPC 求解器生成控制命令。

更完整的项目说明见上级目录 [README.md](../README.md)，控制原理见 [docs/control_principle.md](../docs/control_principle.md)，使用步骤见 [docs/usage.md](../docs/usage.md)。

## 文件结构

```text
MPC_fixed/
├─ main.py                 # 程序入口
├─ master_controller.py    # 总控界面、统一调度、MPC 循环、日志、停止逻辑
├─ mpc_controller.py       # MPC 二次规划求解器
├─ target_trajectory.py    # 目标轨迹生成器
├─ leveling_executor.py    # 调平四腿电动缸执行器
├─ azimuth_executor.py     # 方位执行器，CiA402 速度模式
├─ lift_executor.py        # 举升/俯仰执行器，前馈 + PID 角速度控制
├─ tilt_sensor.py          # 倾角传感器读取线程和帧解析
├─ common.py               # CRC、限幅、编码器累计、单位换算
└─ requirements.txt        # Python 依赖
```

## 运行

本工作副本已附带 `.venv`（含全部依赖），直接用项目 `.venv` 运行（不要用 `python`/`py`）：

```bat
cd E:\dsharness\新控制代码
.venv\Scripts\python.exe MPC_fixed\main.py
```

## 无硬件闭环仿真

如需在没有实物的电脑上验证 MPC 指向过程，可运行：

```bat
.venv\Scripts\python.exe MPC_fixed\simulation.py
```

该程序只导入 `MPCController`，不连接串口，也不创建真实执行器对象。可输入初始调平 X/Y 倾角、方位/俯仰角和目标方向向量，实时观察调平、方位、俯仰、指向误差和角度变化曲线。具体说明见上级目录 [docs/simulation.md](../docs/simulation.md)。

## 举升缸独立实机测试

`lift_00_usb_connection_check.py` 至 `lift_06_open_loop_angle.py` 是七个互相独立的测试 GUI。它们只复用 `LiftExecutor` 通信协议，不进入 MPC 或总控循环；速度被测试实例限制为 `100 rpm`，单次脉冲限制为 `2 s`。各程序的 USB/COM 接线、启动方式、安全联锁和操作步骤见 [docs/lift_testbench.md](../docs/lift_testbench.md)。

另有 `lift_07_position_drive.py` 提供闭环位置驱动（输入目标俯仰角，自动规划梯形速度曲线并驱动到位），详见 [docs/lift_position_drive.md](../docs/lift_position_drive.md)。

## MPC 接口

推荐 MPC 输出字段：

```python
{
    "leveling_x_rate_rad_s": 0.0,
    "leveling_y_rate_rad_s": 0.0,
    "azimuth_rate_rad_s": 0.0,
    "lift_rate_rad_s": 0.0,
    "ok": True,
}
```

总控仍兼容旧字段 `leveling_leg_rpm`、`azimuth_rpm`、`lift_rpm`。

## 当前默认参数

- 总控默认控制周期：`200 ms`。
- 反馈轮询默认周期：`200 ms`。
- MPC 调用默认周期：`200 ms`。
- MPC 内部采样周期：`T = 0.20 s`。
- MPC 预测步长：`Np = 3`。
- MPC 控制步长：`Nc = 2`。
- 倾角传感器安装方向修正：`angle_sign = -1.0`。
- 方位低俯仰角保护阈值：`40 deg`。
- 俯仰安全停止阈值：`69 deg`。
- 举升最低限位（软限位）：以当前硬件原始俯仰角为基准，限位设在其上方 `0.05°`；俯仰角处于限位及以下时只准上升、禁止下降；软件"角度置零"不影响该限位。

## 注意

- 本目录保留的是总控整合版本；根目录的中文脚本是早期独立调试程序。
- 上硬件前必须先在 `IDLE` 模式下低速验证方向、限幅、停止和置零。
- 如果修改 `MPCController.T`，应同步修改总控界面的 MPC 周期。

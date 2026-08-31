# 雷达阵面驱动机构 MPC 控制代码

## 代码声明

本仓库整理的是雷达阵面实验台搭建者提供的 MPC 控制代码。当前整理工作的目标是把代码纳入版本管理、说明其控制原理、标明运行和使用方式，便于后续接手、复现实验和继续开发。

本次整理没有改动控制算法和硬件下发逻辑；新增内容主要是文档、仓库说明和 Git 忽略规则。上硬件前仍需按实验台实际接线、传动比、传感器安装方向和限位条件做低速空载验证。

## 项目结构

```text
.
├─ MPC_fixed/                  # 当前三系统总控主程序
│  ├─ main.py                  # Tkinter 程序入口
│  ├─ master_controller.py     # GUI、串口连接、反馈轮询、MPC 调度、CSV 日志、急停
│  ├─ mpc_controller.py        # MPC 求解器，输出四个系统级角速度
│  ├─ target_trajectory.py     # MPC 目标轨迹生成
│  ├─ leveling_executor.py     # 调平四腿电动缸执行器
│  ├─ azimuth_executor.py      # 方位轴执行器
│  ├─ lift_executor.py         # 举升/俯仰执行器
│  ├─ tilt_sensor.py           # 倾角传感器读取与解析
│  ├─ common.py                # CRC、限幅、编码器累计、单位换算
│  └─ requirements.txt         # Python 依赖
├─ docs/
│  ├─ control_principle.md     # 控制原理和信号流说明
│  └─ usage.md                 # 使用步骤和实验注意事项
├─ specs/
│  └─ MPC_fixed_reverse_spec.md # 基于源码反向整理的规格说明
├─ 静态调平界面+堆帧处理.py       # 早期/独立调平与传感器调试程序
├─ 方位调试-多圈计数.py          # 早期/独立方位调试程序
├─ 举升系统调试-前馈pid.py       # 早期/独立举升/俯仰调试程序
└─ 电机控制+读取.py              # 早期/独立电动缸调试程序
```

## 当前主程序

主程序在 `MPC_fixed` 目录。它把调平、方位、举升/俯仰三个执行系统接入同一个 Tkinter 总控界面，并通过 `MPCController` 周期性计算以下四个控制量：

- `leveling_x_rate_rad_s`：调平系统 X 轴倾角速度，单位 rad/s。
- `leveling_y_rate_rad_s`：调平系统 Y 轴倾角速度，单位 rad/s。
- `azimuth_rate_rad_s`：方位角速度，单位 rad/s，正值为逆时针。
- `lift_rate_rad_s`：举升/俯仰角速度，单位 rad/s。

总控再把这些系统级角速度转换成四条调平腿 rpm、方位电机 rpm、举升电缸 rpm，并通过串口下发到各执行器。

## 安装与运行

本工作副本已附带配置好的 `.venv`（含 numpy/scipy/osqp/pyserial 全部依赖）。

> ⚠️ 运行环境注意：请**直接用项目 `.venv`** 运行，不要用 `python` 或 `py`。
> - 裸 `python` 是 WindowsApps 别名，已损坏（运行脚本返回退出码 49 且无输出）；
> - `py` 指向系统 Python，**缺 `osqp`**，跑不了主控；
> - `.venv` **无法重建**（`python -m venv` 在本机不可用），只能整体复制。

```bat
cd E:\dsharness\新控制代码
.venv\Scripts\python.exe MPC_fixed\main.py
```

无硬件时可以只运行 MPC 求解器自检：

```bat
.venv\Scripts\python.exe MPC_fixed\mpc_controller.py
```

也可以启动不连接串口的闭环指向仿真。仿真可设置初始调平 X/Y 倾角、方位/俯仰角和目标方向向量，并实时显示调平、方位、俯仰和指向误差：

```bat
.venv\Scripts\python.exe MPC_fixed\simulation.py
```

详见 [docs/simulation.md](docs/simulation.md)。该仿真不会创建串口或向硬件下发命令。

## 举升缸 / 俯仰实机测试

针对举升缸和俯仰编码器，仓库提供 `lift_00` 至 `lift_06` 七个独立测试程序，分别用于 USB/COM 识别、编码器核验、低速点动、可用行程、速度特性、最小进给和开环目标角验证。每个程序只执行本阶段动作并单独记录 CSV。具体 USB 接线、安全规则、启动命令和逐项操作见 [docs/lift_testbench.md](docs/lift_testbench.md)。

在开环验证之后，另有 `lift_07_position_drive.py` 提供**闭环位置驱动**（输入目标俯仰角，自动规划梯形速度曲线并驱动到位），详见 [docs/lift_position_drive.md](docs/lift_position_drive.md)。

## 基本使用流程

1. 打开程序后点击“刷新端口”，选择传感器、四条调平腿、方位、举升电缸、俯仰角编码器对应的 COM 口。
2. 逐个连接设备。倾角传感器默认波特率为 `230400`，其余执行器默认多为 `115200`。
3. 先在 `IDLE` 模式下使用手动调试按钮做低速验证：方向、速度限幅、停止、失能、故障复位、角度置零。
4. 需要本地调平时，连接倾角传感器和 4 条调平腿后点击“启动本地调平阶段”。
5. 需要 MPC 总控时，先完成连接、低速验证和必要置零，再点击“启动MPC总控”。程序会要求选择 CSV 实验记录文件，取消保存则不会启动 MPC。
6. 任何异常或实验结束时，优先点击“全部停止”。“停止控制循环”只停止循环和记录，不保证自动下发所有零速。

更详细的操作说明见 [docs/usage.md](docs/usage.md)。

## 控制原理摘要

MPC 内部把阵面指向向量 `S=[0,0,-1]` 经过调平 X/Y/Z、方位 `b`、俯仰 `a` 的旋转矩阵转换为当前方向向量 `[Rx,Ry,Rz]`。预测模型使用 `Np=3`、控制步长 `Nc=2`，决策变量为四个角速度通道的控制增量 `delta_U`。目标函数惩罚未来方向向量与目标轨迹的误差，同时惩罚控制增量和部分位置/角度相关项；约束包含速度限幅、角度限位推导出的速度限幅、以及控制增量限幅。二次规划由 OSQP 求解。

详细原理和数据流见 [docs/control_principle.md](docs/control_principle.md)。基于代码证据整理的反向规格见 [specs/MPC_fixed_reverse_spec.md](specs/MPC_fixed_reverse_spec.md)。

## 重要安全约束

- `tilt_sensor.py` 中 `angle_sign = -1.0`，表示当前 MPC 使用的倾角传感器按反向安装修正；若硬件改为正向安装，需要改为 `1.0` 并重新标定。
- MPC 启动时会把当前 Z 角作为零点，运行中使用相对 Z 角。
- 俯仰角低于 `40 deg` 时，总控会强制方位速度为 0。
- 反馈线程检测到俯仰角达到 `69 deg` 或更高时，会停止举升；若正在控制循环中，会触发全部停止。
- MPC 连续 3 次求解失败会停止；单次失败会暂时沿用上一拍有效命令，超过指令超时时间也会停止。
- 方位、俯仰、调平的方向和传动比必须以上台低速验证结果为准。

## 后续建议

- 把实际 COM 口、接线编号、四腿空间位置、传动比和置零流程整理成实验台专用配置文档。
- 将 `motor_gear_ratio`、安全角度阈值、周期、MPC 权重、限幅等参数从代码常量迁移到配置文件。
- 增加硬件断连、串口写失败和传感器跳变场景的离线测试或仿真测试。

# CLAUDE.md — 新控制代码 目录调试指南

本目录 `E:\dsharness\新控制代码` 是从 `E:\dsharness\控制代码` 复制出的**调试工作副本**（不含 `.git`），
用于举升缸位置闭环驱动的继续开发与真机联调。打开本目录即可让 AI 直接接手。

---

## 1. 运行环境（最重要，先读这段）

- **只能用项目 `.venv` 运行，不要用 `python` 或 `py`：**
  - 裸 `python` 是 WindowsApps 别名，已损坏（运行脚本返回退出码 49 且无输出）；
  - `py` 指向系统 `pythoncore-3.14-64`，自带 numpy/scipy/pyserial，但**缺 `osqp`**，跑不了主控 `main.py`；
  - 只有项目 `.venv` 带齐 numpy/scipy/osqp/pyserial。
- **`.venv` 无法重建**：`python -m venv` / `python -m ensurepip` 都返回退出码 49（pythoncore 运行时组件损坏），
  所以 `.venv` 只能**整体复制**，不能重新 `pip` 安装。当前已复制好、依赖齐全。
- 在本目录下统一用：
  ```bat
  .venv\Scripts\python.exe MPC_fixed\main.py
  .venv\Scripts\python.exe MPC_fixed\lift_07_position_drive.py
  ```
  离线验证 / 自测同样用 `.venv\Scripts\python.exe`。
- 本目录**不是 git 仓库**（复制时未带 `.git`）；如需版本管理再 `git init`。

---

## 2. 本次已完成的改动（举升缸位置闭环驱动）

详见 `docs/lift_position_drive.md`。核心是新增了一套**级联位置闭环**：

```
目标俯仰角(deg) → 位置环(梯形速度曲线+位置P(I)(D)) → 目标角速度(deg/s)
               → 速度环(前馈+角速度PID，已有) → rpm → 0x3308 速度帧
```

| 文件 | 改动 |
|---|---|
| `MPC_fixed/lift_position_controller.py` | 新增：纯计算位置环，无硬件依赖，可离线自测 |
| `MPC_fixed/lift_executor.py` | 扩展：位置驱动接口 `start/update/stop/configure/is_arrived/is_fault` |
| `MPC_fixed/lift_07_position_drive.py` | 新增：独立位置驱动测试窗口（含方向系数输入） |
| `MPC_fixed/master_controller.py` | 扩展：IDLE 模式新增「位置闭环驱动」手动控制 |
| `docs/lift_position_drive.md` | 新增：架构/用法/参数/安全/离线自测结果/调参建议 |

---

## 3. 关键参数位置（真机调参主要改这里）

| 参数 | 位置 |
|---|---|
| 位置环参数（max_vel/accel/kp/ki/kd/死区/跟随误差） | `lift_position_controller.py` 的 `LiftPositionController.__init__`（默认 max_vel=0.5 deg/s，保守） |
| 方向系数 `pitch_lift_direction` | `lift_executor.py`（默认 1；角度反向就改 -1）。`lift_07` 窗口和主控里都有「方向系数」输入框可直接改 |
| 前馈增益表（deg/s per rpm） | `lift_executor.py` 的 `self.pitch_gain_table` |
| 速度环 PID | `lift_executor.py` 的 `self.pitch_omega_kp/ki/kd` |
| 软限位 | 位置环 `pos_min_deg=-0.02 / pos_max_deg=65.0`；执行器最低限位 `pitch_min_limit_raw_deg` |

---

## 4. 运行与验证命令

```bat
:: 全部编译检查
.venv\Scripts\python.exe -m compileall -f -q MPC_fixed

:: 位置环离线自测（无硬件，验证收敛/保护）
.venv\Scripts\python.exe MPC_fixed\lift_position_controller.py

:: MPC 求解器自检（验证 osqp）
.venv\Scripts\python.exe MPC_fixed\mpc_controller.py

:: 无硬件闭环指向仿真
.venv\Scripts\python.exe MPC_fixed\simulation.py

:: 位置驱动独立窗口（真机）
.venv\Scripts\python.exe MPC_fixed\lift_07_position_drive.py

:: 主控（真机）
.venv\Scripts\python.exe MPC_fixed\main.py
```

离线端到端闭环自测（位置环→速度环→虚拟电动缸）已通过：正向/反向/大行程到位误差 ≤0.002°，
卡死保护约 1.65s 触发，软限位钳制正常。需要时可重跑同一逻辑验证改动。

---

## 5. 当前进度与下一步

**已完成**：位置驱动全链路实现 + 离线验证。

**待真机确认 / 下一步（按顺序）：**
1. **方向预检**（最关键，离线无法验证）：小角度（当前角 ±0.3°）慢速（0.3 deg/s）点动，
   确认 `pitch_lift_direction` 正确；方向反了会触发「跟随误差 3° 自动停机」。
2. 真机调参：到位精度、振荡、速度提升。
3. 可选增强：把位置目标接入 MPC 轨迹、CSV 增加位置目标列、一键自动判断方向按钮。

---

## 6. 安全规则

完整项目级规则见同目录 `AGENTS.md`。要点：
- 影响硬件动作的修改必须保守、先离线验证、再由实验人员低速空载实机验证。
- 不擅自改硬件协议/方向/传动比/限位/停止逻辑/MPC 权重限幅。
- 修改后至少跑 `compileall` 和相关自测，无法验证要明确说明。

---

## 7. 文档索引

- 项目结构/控制原理/使用流程：`README.md`、`docs/control_principle.md`、`docs/usage.md`
- 举升测试台 lift_00~06：`docs/lift_testbench.md`
- 位置驱动：`docs/lift_position_drive.md`
- 无硬件仿真：`docs/simulation.md`
- MPC 反向规格：`specs/MPC_fixed_reverse_spec.md`

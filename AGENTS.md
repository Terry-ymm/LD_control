# AGENTS.md

## 适用范围

本文件适用于本仓库根目录及其所有子目录。后续由 AI 助手或其他协作者修改本项目时，应优先遵守这里的项目级规则。

本项目是雷达阵面驱动机构的控制代码，涉及调平、方位、举升/俯仰执行机构和 MPC 控制逻辑。任何可能影响硬件动作的修改都必须保守处理、明确说明、先做离线验证，再由实验人员进行低速实机验证。

## 语言和文档风格

- 面向使用者、实验记录和项目说明的文档优先使用中文。
- 代码中的已有中文注释应保持可读，不要改成乱码或无意义拼音。
- 修改控制逻辑、接口、默认参数、安全阈值或使用流程时，必须同步检查是否需要更新 `README.md`、`MPC_fixed/README.md`、`docs/` 和 `specs/`。
- 文档应区分“源码中已观察到的事实”和“需要实机确认的推断”。

## Windows / PowerShell 编码规则

本仓库文本文件按 UTF-8 管理。Windows PowerShell 默认输出编码可能导致中文显示乱码，因此读取中文文件时必须显式指定 UTF-8。

读取文本文件时优先使用：

```powershell
Get-Content -Encoding UTF8 -Raw <path>
Get-Content -Encoding UTF8 <path>
```

在执行可能输出中文的命令前，建议先设置控制台编码：

```powershell
chcp 65001 > $null
$OutputEncoding = [Console]::OutputEncoding = [System.Text.UTF8Encoding]::new()
[Console]::InputEncoding = [System.Text.UTF8Encoding]::new()
$env:PYTHONIOENCODING = "utf-8"
```

查看 Git 中文文件名时优先使用：

```powershell
git -c core.quotepath=false status --short
git -c core.quotepath=false ls-files
```

如果看到中文乱码，先按 UTF-8 重新读取或重新运行命令，不要直接判断文件内容已损坏。

## 搜索和读取代码

- 搜索文件或文本时优先使用 `rg` / `rg --files`。
- 阅读 Python、Markdown、TXT 等含中文文件时，在 PowerShell 中使用 `Get-Content -Encoding UTF8`。
- 阅读控制路径时，应从 `MPC_fixed/main.py`、`master_controller.py`、`mpc_controller.py` 顺着调用关系看，不要只凭文件名推断功能。
- 根目录的独立中文脚本是早期调试程序来源和参考；当前主程序以 `MPC_fixed/` 为准。

## 控制代码安全规则

- 未经用户明确要求，不要修改硬件下发协议、执行器方向、传动比、限位阈值、安全停止逻辑、MPC 默认权重和速度限幅。
- 不要在未说明风险的情况下运行会向串口设备写命令的脚本或 GUI 操作。
- 优先做离线验证，例如语法编译、MPC 求解器自检、纯函数测试；实机验证必须由实验人员在低速、空载或安全工况下完成。
- 修改以下位置时必须在回复中明确说明潜在硬件影响：
  - `MPC_fixed/master_controller.py` 的控制模式、停止逻辑、反馈线程、MPC 调度和命令分发。
  - `MPC_fixed/mpc_controller.py` 的模型、权重、约束、限幅、采样周期和求解失败策略。
  - `MPC_fixed/leveling_executor.py` 的四腿分配、速度帧、腿编号和方向。
  - `MPC_fixed/azimuth_executor.py` 的 CiA402 报文、方向约定、传动比和状态解析。
  - `MPC_fixed/lift_executor.py` 的前馈表、PID、俯仰角速度估计、角度限位和停止命令。
  - `MPC_fixed/tilt_sensor.py` 的帧解析、角度符号 `angle_sign`、跳变过滤。

## 单位和方向约定

保持以下约定一致，除非用户明确要求修改并完成文档同步：

- MPC 推荐输出为系统级角速度：
  - `leveling_x_rate_rad_s`
  - `leveling_y_rate_rad_s`
  - `azimuth_rate_rad_s`
  - `lift_rate_rad_s`
- 角速度接口优先使用 `rad/s`，界面调试或日志可显示 `deg/s`。
- 方位正方向：正值表示逆时针，负值表示顺时针。
- 倾角传感器当前默认 `angle_sign = -1.0`，表示当前实验安装方向按反向修正。
- 当前源码默认控制、反馈和 MPC 周期为 `200 ms`，不要把历史说明中的 `100 ms` 当作当前事实。

## 修改和验证要求

修改代码后至少运行：

```powershell
.\.venv\Scripts\python.exe -m compileall -f -q MPC_fixed
```

如果修改了 MPC、目标轨迹或依赖，运行：

```powershell
.\.venv\Scripts\python.exe MPC_fixed\mpc_controller.py
```

注意：本机 `python -m venv` / `python -m ensurepip` 不可用（返回退出码 49），`.venv` **不能重建**。如需在新路径使用，只能整体复制已有 `.venv`（见根目录 `CLAUDE.md`），不要执行 `python -m venv`。

如果修改了根目录独立调试脚本，可分别做语法检查：

```powershell
.\.venv\Scripts\python.exe -m py_compile '静态调平界面+堆帧处理.py'
.\.venv\Scripts\python.exe -m py_compile '方位调试-多圈计数.py'
.\.venv\Scripts\python.exe -m py_compile '举升系统调试-前馈pid.py'
.\.venv\Scripts\python.exe -m py_compile '电机控制+读取.py'
```

无法运行验证时，必须在最终回复中明确说明原因。

## Git 和仓库规则

- 提交前查看：

```powershell
git -c core.quotepath=false status --short --branch
git diff --cached --name-status
```

- 不要提交 `.venv/`、`__pycache__/`、`.vscode/`、CSV 日志、压缩包和临时文件。
- 不要删除或覆盖用户未要求修改的实验脚本和数据。
- 只有在用户明确要求上传、提交或推送时才执行 `git commit` / `git push`。
- 推送前确认远端：

```powershell
git remote -v
```

## 变更说明要求

最终回复应简要说明：

- 修改了哪些文件。
- 是否改变控制代码行为。
- 做了哪些验证。
- 是否已提交或推送到远端。
- 任何未验证的硬件风险或需要实验人员确认的事项。

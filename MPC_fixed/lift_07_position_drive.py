# -*- coding: utf-8 -*-
"""07 - 举升缸位置闭环驱动（点到点）。

在 06 开环验证之后，本程序使用闭环位置环把俯仰角驱动到目标角度并自动停住：

    LiftPositionController（位置环，梯形速度曲线 + 位置 P(I)(D)）
          -> 目标角速度 deg/s
          -> LiftExecutor.set_lift_rate_deg_s（速度环，前馈 + 角速度 PID）
          -> rpm -> 0x3308 速度帧

安全说明：
    - 位置环输出角速度，速度环照旧走前馈 + PID，最终下发 rpm；
    - 默认电机转速上限 400 rpm（远低于 2200），最大角速度 0.5 deg/s，速度很慢；
    - 跟随误差超限、反馈丢失、急停、断开连接都会立即下发 0 停止；
    - 开始运动前必须：解锁安全联锁 + 开始 CSV + 两路反馈有效且新鲜。
"""
from __future__ import annotations

import tkinter as tk
from tkinter import messagebox, ttk

from lift_test_common import LiftTestApp, run_app


class PositionDriveApp(LiftTestApp):
    phase = "07_position"
    title = "07 - 举升缸位置闭环驱动"
    allow_motion = True

    # 位置控制允许的最高电机转速（rpm），独立于脉冲测试的 100 rpm 上限。
    DEFAULT_POSITION_RPM_CAP = 400.0

    def build_stage_controls(self, parent: tk.Misc) -> None:
        # 把本实例的电机转速上限放宽到位置控制档（仅本测试窗口生效）。
        self.bench.executor.max_motor_rpm = self.DEFAULT_POSITION_RPM_CAP

        self.position_active = False

        # ---- 输入变量 ----
        self.target_angle_var = tk.StringVar(value="10.0")
        self.max_vel_var = tk.StringVar(value="0.5")
        self.accel_var = tk.StringVar(value="1.0")
        self.decel_var = tk.StringVar(value="1.0")
        self.kp_var = tk.StringVar(value="5.0")
        self.ki_var = tk.StringVar(value="0.0")
        self.kd_var = tk.StringVar(value="0.0")
        self.deadband_var = tk.StringVar(value="0.05")
        self.rpm_cap_var = tk.StringVar(value=str(self.DEFAULT_POSITION_RPM_CAP))
        self.direction_var = tk.StringVar(value="1")

        # ---- 状态显示变量 ----
        self.position_state_var = tk.StringVar(value="位置环未启动")
        self.position_omega_var = tk.StringVar(value="---")
        self.position_ref_var = tk.StringVar(value="---")

        ttk.Label(parent, text=(
            "闭环位置驱动：输入目标俯仰角，程序自动规划梯形速度曲线并驱动到位。\n"
            "默认速度很慢（最大 0.5 °/s），请先用小角度、低速度验证方向与到位精度，再逐步提高。"
        ), wraplength=860, foreground="#b00020", justify=tk.LEFT).grid(
            row=0, column=0, columnspan=6, sticky=tk.W, pady=4)

        # ---- 参数输入 ----
        inputs = [
            ("目标俯仰角 (°)", self.target_angle_var),
            ("最大角速度 (deg/s)", self.max_vel_var),
            ("加速度 (deg/s²)", self.accel_var),
            ("减速度 (deg/s²)", self.decel_var),
            ("位置环 Kp", self.kp_var),
            ("位置环 Ki", self.ki_var),
            ("位置环 Kd", self.kd_var),
            ("到位死区 (°)", self.deadband_var),
            ("电机转速上限 (rpm)", self.rpm_cap_var),
            ("方向系数 (±1)", self.direction_var),
        ]
        for index, (label, variable) in enumerate(inputs):
            row = 1 + index // 3
            column = (index % 3) * 2
            ttk.Label(parent, text=label).grid(row=row, column=column, sticky=tk.E, padx=4, pady=3)
            ttk.Entry(parent, textvariable=variable, width=10).grid(row=row, column=column + 1, sticky=tk.W, padx=4, pady=3)

        # ---- 操作按钮 ----
        ttk.Button(parent, text="开始位置运动", command=self.start_position).grid(
            row=5, column=0, columnspan=2, sticky="ew", padx=4, pady=8)
        ttk.Button(parent, text="停止", command=lambda: self.stop_motion("manual_stop")).grid(
            row=5, column=2, columnspan=2, sticky="ew", padx=4, pady=8)

        # ---- 位置环状态 ----
        ttk.Label(parent, text="位置环状态:").grid(row=6, column=0, sticky=tk.E, padx=4, pady=2)
        ttk.Label(parent, textvariable=self.position_state_var, font=("Consolas", 10)).grid(
            row=6, column=1, columnspan=5, sticky=tk.W, pady=2)
        ttk.Label(parent, text="目标角速度:").grid(row=7, column=0, sticky=tk.E, padx=4, pady=2)
        ttk.Label(parent, textvariable=self.position_omega_var, font=("Consolas", 10)).grid(
            row=7, column=1, sticky=tk.W, pady=2)
        ttk.Label(parent, text="曲线参考速度:").grid(row=7, column=2, sticky=tk.E, padx=4, pady=2)
        ttk.Label(parent, textvariable=self.position_ref_var, font=("Consolas", 10)).grid(
            row=7, column=3, columnspan=3, sticky=tk.W, pady=2)

        # ---- 最低限位（软限位，持久化，重启自动加载）----
        _loaded_limit = self.bench.executor.pitch_min_limit_raw_deg
        self.min_limit_var = tk.StringVar(
            value=f"{_loaded_limit:.3f}°" if _loaded_limit is not None else "未设置"
        )
        limit_frame = ttk.LabelFrame(parent, text="最低限位（软限位，重启自动加载）", padding="6")
        limit_frame.grid(row=8, column=0, columnspan=6, sticky="ew", pady=6)
        ttk.Label(limit_frame, text="当前限位:").pack(side=tk.LEFT, padx=(0, 4))
        ttk.Label(limit_frame, textvariable=self.min_limit_var, font=("Consolas", 10)).pack(side=tk.LEFT, padx=(0, 8))
        ttk.Button(limit_frame, text="设为最低限位", command=self.set_min_limit).pack(side=tk.LEFT, padx=3)
        ttk.Button(limit_frame, text="清除", command=self.clear_min_limit).pack(side=tk.LEFT, padx=3)

    # =========================================================
    # 位置运动控制
    # =========================================================
    def _read_position_params(self) -> dict | None:
        """读取并校验位置环参数，失败返回 None。"""
        values = [
            self.read_number(self.target_angle_var, "目标俯仰角"),
            self.read_number(self.max_vel_var, "最大角速度"),
            self.read_number(self.accel_var, "加速度"),
            self.read_number(self.decel_var, "减速度"),
            self.read_number(self.kp_var, "位置环 Kp"),
            self.read_number(self.ki_var, "位置环 Ki"),
            self.read_number(self.kd_var, "位置环 Kd"),
            self.read_number(self.deadband_var, "到位死区"),
            self.read_number(self.rpm_cap_var, "电机转速上限"),
            self.read_number(self.direction_var, "方向系数"),
        ]
        if any(v is None for v in values):
            return None
        target, max_vel, accel, decel, kp, ki, kd, deadband, rpm_cap, direction = values
        if rpm_cap <= 0 or rpm_cap > 2200:
            messagebox.showerror("转速上限错误", "电机转速上限必须在 0 到 2200 rpm 之间。")
            return None
        if max_vel <= 0 or accel <= 0 or decel <= 0:
            messagebox.showerror("参数错误", "最大角速度、加速度、减速度必须为正数。")
            return None
        if deadband <= 0:
            messagebox.showerror("参数错误", "到位死区必须为正数。")
            return None
        return {
            "target_deg": target,
            "max_vel_deg_s": max_vel,
            "accel_deg_s2": accel,
            "decel_deg_s2": decel,
            "kp": kp,
            "ki": ki,
            "kd": kd,
            "deadband_deg": deadband,
            "rpm_cap": rpm_cap,
            "direction": 1 if int(direction) >= 0 else -1,
        }

    def start_position(self) -> None:
        if not self.arm_var.get():
            messagebox.showwarning("未解锁", "请先确认机械区域、急停和人员安全。")
            return
        if not self.logger.active:
            messagebox.showwarning("未记录", "请先开始本阶段 CSV 记录。")
            return
        if self.position_active or self.motion_active:
            messagebox.showwarning("运动中", "已有运动进行中，请先停止。")
            return
        if not self.bench.feedback_is_fresh():
            messagebox.showwarning("反馈无效", "需要两路有效且不超时的反馈，禁止运动。")
            return

        params = self._read_position_params()
        if params is None:
            return

        # 本实例转速上限 + 位置环参数
        self.bench.executor.max_motor_rpm = params["rpm_cap"]
        # 方向系数属于速度环，单独配置（±1）
        self.bench.executor.configure_pitch_pid(direction=params["direction"])
        self.bench.executor.configure_lift_position_control(
            max_vel_deg_s=params["max_vel_deg_s"],
            accel_deg_s2=params["accel_deg_s2"],
            decel_deg_s2=params["decel_deg_s2"],
            kp=params["kp"],
            ki=params["ki"],
            kd=params["kd"],
            deadband_deg=params["deadband_deg"],
            # 输出限幅略高于曲线速度，给位置校正留余量
            max_output_deg_s=min(params["max_vel_deg_s"] * 2.0, 4.0),
            # 跟随误差保护放宽到 3°，避免慢速大行程误报
            follow_error_limit_deg=3.0,
        )

        summary = self.bench.executor.start_lift_position_move(params["target_deg"])
        if summary.get("clamped"):
            messagebox.showwarning(
                "目标已钳制",
                f"目标角超出软限位 [{self.bench.executor.position_controller.pos_min_deg}, "
                f"{self.bench.executor.position_controller.pos_max_deg}]，已钳制到 {summary['target_deg']:.3f}°。",
            )

        self.position_active = True
        self.motion_active = True
        self.motion_command_rpm = 0.0
        self.logger.write(self.bench.last_snapshot, event="position_start", note=str(summary))
        self.status_var.set(
            f"位置运动已开始：目标 {summary['target_deg']:.3f}°，"
            f"曲线时长约 {summary['profile_total_s']:.1f} s。"
        )

    def _run_position_loop(self, snapshot) -> None:
        if not snapshot.pitch_valid:
            self.stop_motion("feedback_lost")
            return

        res = self.bench.executor.update_lift_position_control()
        # 记录本拍实际下发的 rpm（供 CSV 追溯）
        self.motion_command_rpm = float(self.bench.executor.pitch_state.get("last_motor_rpm", 0.0))
        self._update_position_display(res)

        if res["state"] == "fault":
            self.stop_motion("following_error")
            self.logger.write(snapshot, event="position_fault",
                              note=f"跟踪误差超限 {res['tracking_error_deg']:+.4f}°")
        elif res["arrived"]:
            self.stop_motion("arrived")
            self.logger.write(snapshot, event="position_arrived",
                              note=f"到位误差 {res['pos_error_deg']:+.4f}°")

    def _update_position_display(self, res: dict) -> None:
        state_cn = {"idle": "空闲", "moving": "运动中", "holding": "到位保持", "fault": "故障"}.get(res["state"], res["state"])
        self.position_state_var.set(
            f"状态={state_cn}  目标={res['target_deg']:.3f}°  当前={res['current_deg']:.3f}°  "
            f"误差={res['pos_error_deg']:+.4f}°  跟踪误差={res['tracking_error_deg']:+.4f}°"
        )
        self.position_omega_var.set(f"{res['target_omega_deg_s']:+.4f} deg/s")
        self.position_ref_var.set(f"{res['ref_vel_deg_s']:+.4f} deg/s")

    # =========================================================
    # 覆盖基类钩子
    # =========================================================
    def on_snapshot(self, snapshot) -> None:
        if not self.position_active:
            return
        try:
            self._run_position_loop(snapshot)
        except Exception as exc:
            self.stop_motion(f"position_exception:{exc}")

    def stop_motion(self, reason: str) -> None:
        """统一停止入口：位置控制激活时先复位位置环，再走基类急停（下发 0）。"""
        if getattr(self, "position_active", False):
            self.bench.executor.stop_lift_position_move()
            self.position_active = False
        super().stop_motion(reason)
        if hasattr(self, "position_state_var"):
            self.position_state_var.set("位置环已停止")

    # =========================================================
    # 最低限位（软限位，持久化）
    # =========================================================
    def set_min_limit(self) -> None:
        if self.motion_active:
            messagebox.showwarning("运动中", "请先停止运动后再设置最低限位。")
            return
        if self.bench.executor.set_pitch_min_limit_from_current():
            limit = self.bench.executor.pitch_min_limit_raw_deg
            self.min_limit_var.set(f"{limit:.3f}°" if limit is not None else "未设置")
            self.status_var.set("已设置最低限位并保存（重启自动加载）：只准上升，禁止下降")
        else:
            messagebox.showwarning("设置失败", "未读到有效俯仰编码器位置。")

    def clear_min_limit(self) -> None:
        self.bench.executor.clear_pitch_min_limit()
        self.min_limit_var.set("未设置")
        self.status_var.set("已清除最低限位并删除配置")


if __name__ == "__main__":
    run_app(PositionDriveApp)

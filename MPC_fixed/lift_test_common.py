# -*- coding: utf-8 -*-
"""举升缸/俯仰测试程序的公共安全层。

本文件不属于主控流程。它只被 lift_00 至 lift_06 的独立测试窗口使用，
复用已经存在的 LiftExecutor 协议实现，但将测试速度限制为保守值。
"""
from __future__ import annotations

import csv
import json
import math
import os
import time
import tkinter as tk
from dataclasses import dataclass
from datetime import datetime
from tkinter import filedialog, messagebox, ttk
from typing import Callable, Optional

import serial.tools.list_ports

from common import count_to_mm, parse_cylinder_single_turn, update_encoder_accum
from lift_executor import LiftExecutor


DEFAULT_BAUDRATE = 115200
DEFAULT_RPM = 20.0
SAFE_RPM_CAP = 100.0
DEFAULT_PULSE_S = 0.5
MAX_PULSE_S = 2.0
FEEDBACK_PERIOD_MS = 200
FEEDBACK_STALE_S = 1.0

LOG_FIELDS = [
    "timestamp", "elapsed_s", "phase", "event", "note", "command_rpm",
    "motor_valid", "pitch_valid",
    "motor_raw_count", "motor_relative_count", "motor_pos_mm", "motor_speed_mm_s",
    "pitch_raw_count", "pitch_relative_count", "pitch_angle_deg", "pitch_rate_deg_s",
]


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


@dataclass
class Snapshot:
    timestamp: float
    motor_valid: bool
    pitch_valid: bool
    motor_raw_count: int | None
    motor_relative_count: int | None
    motor_pos_mm: float
    motor_speed_mm_s: float
    pitch_raw_count: int | None
    pitch_relative_count: int | None
    pitch_angle_deg: float
    pitch_rate_deg_s: float

    @property
    def valid(self) -> bool:
        return self.motor_valid and self.pitch_valid


class LiftBench:
    """举升缸和俯仰编码器的同步读取与受限点动接口。"""

    def __init__(self) -> None:
        self.executor = LiftExecutor()
        # 仅改变本测试实例的限幅，不改变主程序或 LiftExecutor 的源码默认值。
        self.executor.max_motor_rpm = SAFE_RPM_CAP
        self.last_snapshot: Snapshot | None = None
        self.last_feedback_time = 0.0
        self._previous_motor_mm: float | None = None
        self._previous_motor_time: float | None = None

    def connect_lift(self, port: str, baudrate: int) -> None:
        self.executor.connect_lift(port, baudrate)

    def connect_pitch(self, port: str, baudrate: int) -> None:
        self.executor.connect_encoder(port, baudrate)

    def disconnect_all(self) -> None:
        self.safe_stop()
        if self.executor.lift_connected:
            self.executor.disconnect_lift()
        if self.executor.encoder_connected:
            self.executor.disconnect_encoder()

    def safe_stop(self) -> None:
        """多次发送零速；该方法允许在未解锁时调用。"""
        if not self.executor.lift_connected:
            return
        for _ in range(3):
            self.executor.stop()

    def reset_motor_zero(self) -> bool:
        return self.executor.reset_zero()

    def reset_pitch_zero(self) -> bool:
        return self.executor.reset_pitch_zero()

    def send_rpm(self, rpm: float) -> bool:
        safe_rpm = clamp(float(rpm), -SAFE_RPM_CAP, SAFE_RPM_CAP)
        return self.executor.send_lift_cmd(safe_rpm)

    def read_snapshot(self) -> Snapshot:
        now = time.time()
        motor_valid = False
        motor_raw_count: int | None = None
        motor_relative_count: int | None = None
        motor_pos_mm = float(self.executor.lift_state.get("pos_mm", 0.0))
        motor_speed = 0.0

        if self.executor.lift_connected:
            raw = self.executor.read_lift_registers(0x4202, 2)
            single_turn = parse_cylinder_single_turn(raw)
            accum = update_encoder_accum(self.executor.lift_state, single_turn)
            if accum is not None:
                if self.executor.lift_state["zero_encoder_count"] is None:
                    self.executor.lift_state["zero_encoder_count"] = accum
                motor_raw_count = single_turn
                motor_relative_count = accum - self.executor.lift_state["zero_encoder_count"]
                motor_pos_mm = count_to_mm(
                    motor_relative_count,
                    self.executor.lead_mm,
                    self.executor.reduction_ratio,
                )
                self.executor.lift_state["pos_mm"] = motor_pos_mm
                motor_valid = True
                if self._previous_motor_mm is not None and self._previous_motor_time is not None:
                    dt = now - self._previous_motor_time
                    if dt > 0:
                        motor_speed = (motor_pos_mm - self._previous_motor_mm) / dt
                self._previous_motor_mm = motor_pos_mm
                self._previous_motor_time = now

        self.executor.update_pitch_encoder_feedback()
        pitch = self.executor.pitch_state
        pitch_valid = bool(
            self.executor.encoder_connected
            and pitch.get("health") == "正常"
            and pitch.get("raw_count") is not None
        )
        snapshot = Snapshot(
            timestamp=now,
            motor_valid=motor_valid,
            pitch_valid=pitch_valid,
            motor_raw_count=motor_raw_count,
            motor_relative_count=motor_relative_count,
            motor_pos_mm=motor_pos_mm,
            motor_speed_mm_s=motor_speed,
            pitch_raw_count=pitch.get("raw_count"),
            pitch_relative_count=pitch.get("relative_count"),
            pitch_angle_deg=float(pitch.get("angle_deg", 0.0)),
            pitch_rate_deg_s=float(pitch.get("angle_rate_deg_s", 0.0)),
        )
        self.last_snapshot = snapshot
        if snapshot.valid:
            self.last_feedback_time = now
        return snapshot

    def feedback_is_fresh(self) -> bool:
        return (
            self.last_snapshot is not None
            and self.last_snapshot.valid
            and time.time() - self.last_feedback_time <= FEEDBACK_STALE_S
        )


class CsvLogger:
    def __init__(self, phase: str) -> None:
        self.phase = phase
        self.file = None
        self.writer: csv.DictWriter | None = None
        self.start_time = 0.0

    @property
    def active(self) -> bool:
        return self.writer is not None

    def start(self, parent: tk.Misc) -> bool:
        filename = filedialog.asksaveasfilename(
            parent=parent,
            title="保存本阶段测试 CSV",
            defaultextension=".csv",
            filetypes=[("CSV 文件", "*.csv")],
            initialfile=f"{self.phase}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
        )
        if not filename:
            return False
        self.file = open(filename, "w", newline="", encoding="utf-8-sig")
        self.writer = csv.DictWriter(self.file, fieldnames=LOG_FIELDS)
        self.writer.writeheader()
        self.start_time = time.time()
        return True

    def write(self, snapshot: Snapshot | None, event: str = "feedback", note: str = "", command_rpm: float = 0.0) -> None:
        if not self.writer:
            return
        item = snapshot or Snapshot(time.time(), False, False, None, None, 0.0, 0.0, None, None, 0.0, 0.0)
        self.writer.writerow({
            "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
            "elapsed_s": f"{item.timestamp - self.start_time:.3f}" if self.start_time else "0.000",
            "phase": self.phase,
            "event": event,
            "note": note,
            "command_rpm": f"{command_rpm:.4f}",
            "motor_valid": int(item.motor_valid),
            "pitch_valid": int(item.pitch_valid),
            "motor_raw_count": item.motor_raw_count if item.motor_raw_count is not None else "",
            "motor_relative_count": item.motor_relative_count if item.motor_relative_count is not None else "",
            "motor_pos_mm": f"{item.motor_pos_mm:.8f}",
            "motor_speed_mm_s": f"{item.motor_speed_mm_s:.8f}",
            "pitch_raw_count": item.pitch_raw_count if item.pitch_raw_count is not None else "",
            "pitch_relative_count": item.pitch_relative_count if item.pitch_relative_count is not None else "",
            "pitch_angle_deg": f"{item.pitch_angle_deg:.8f}",
            "pitch_rate_deg_s": f"{item.pitch_rate_deg_s:.8f}",
        })
        self.file.flush()

    def close(self) -> None:
        if self.file:
            self.file.close()
        self.file = None
        self.writer = None
        self.start_time = 0.0


class LiftTestApp:
    """七个独立测试窗口的公共 UI 和安全状态机。"""

    phase = "lift_test"
    title = "举升缸测试"
    allow_motion = False

    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title(self.title)
        self.root.geometry("980x760")
        self.root.minsize(820, 620)
        self.bench = LiftBench()
        self.logger = CsvLogger(self.phase)
        self.poll_job: str | None = None
        self.motion_job: str | None = None
        self.motion_active = False
        self.motion_command_rpm = 0.0

        self.lift_port_var = tk.StringVar()
        self.pitch_port_var = tk.StringVar()
        self.lift_baud_var = tk.StringVar(value=str(DEFAULT_BAUDRATE))
        self.pitch_baud_var = tk.StringVar(value=str(DEFAULT_BAUDRATE))
        self.arm_var = tk.BooleanVar(value=False)
        self.status_var = tk.StringVar(value="请先刷新 USB 串口并连接设备")
        self.log_var = tk.StringVar(value="未开始本阶段 CSV 记录")
        self.port_info_var = tk.StringVar(value="未扫描")
        self.snapshot_vars = {
            "motor": tk.StringVar(value="---"),
            "motor_speed": tk.StringVar(value="---"),
            "pitch": tk.StringVar(value="---"),
            "pitch_rate": tk.StringVar(value="---"),
            "health": tk.StringVar(value="未连接"),
        }
        self._build_ui()
        self.refresh_ports()
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self._poll_feedback()

    def _build_ui(self) -> None:
        outer = ttk.Frame(self.root, padding=10)
        outer.pack(fill=tk.BOTH, expand=True)
        self._build_connection_panel(outer)
        self._build_readback_panel(outer)
        if self.allow_motion:
            self._build_safety_panel(outer)
        body = ttk.LabelFrame(outer, text="本阶段操作", padding=8)
        body.pack(fill=tk.BOTH, expand=True, pady=(8, 0))
        self.build_stage_controls(body)
        ttk.Label(outer, textvariable=self.status_var, relief=tk.SUNKEN, anchor=tk.W).pack(fill=tk.X, pady=(8, 0))

    def _build_connection_panel(self, parent: tk.Misc) -> None:
        frame = ttk.LabelFrame(parent, text="USB 转串口连接（Windows 中显示为 COM 口）", padding=8)
        frame.pack(fill=tk.X)
        ttk.Button(frame, text="刷新 COM 口", command=self.refresh_ports).grid(row=0, column=0, padx=4, pady=3)
        ttk.Label(frame, text="举升缸 COM:").grid(row=0, column=1, sticky=tk.E)
        self.lift_combo = ttk.Combobox(frame, textvariable=self.lift_port_var, width=12)
        self.lift_combo.grid(row=0, column=2, padx=4)
        ttk.Entry(frame, textvariable=self.lift_baud_var, width=8).grid(row=0, column=3, padx=4)
        ttk.Button(frame, text="连接/断开举升缸", command=self.toggle_lift).grid(row=0, column=4, padx=4)
        ttk.Label(frame, text="俯仰编码器 COM:").grid(row=1, column=1, sticky=tk.E)
        self.pitch_combo = ttk.Combobox(frame, textvariable=self.pitch_port_var, width=12)
        self.pitch_combo.grid(row=1, column=2, padx=4)
        ttk.Entry(frame, textvariable=self.pitch_baud_var, width=8).grid(row=1, column=3, padx=4)
        ttk.Button(frame, text="连接/断开俯仰编码器", command=self.toggle_pitch).grid(row=1, column=4, padx=4)
        ttk.Button(frame, text="开始本阶段 CSV", command=self.start_log).grid(row=0, column=5, padx=(20, 4))
        ttk.Button(frame, text="停止 CSV", command=self.stop_log).grid(row=1, column=5, padx=(20, 4))
        ttk.Label(frame, textvariable=self.log_var).grid(row=0, column=6, rowspan=2, sticky=tk.W, padx=4)
        ttk.Label(frame, textvariable=self.port_info_var, wraplength=900).grid(row=2, column=0, columnspan=7, sticky=tk.W, pady=(5, 0))

    def _build_readback_panel(self, parent: tk.Misc) -> None:
        frame = ttk.LabelFrame(parent, text="实时反馈（只读轮询）", padding=8)
        frame.pack(fill=tk.X, pady=(8, 0))
        rows = [
            ("电机编码器 / 缸位移", self.snapshot_vars["motor"]),
            ("缸线速度", self.snapshot_vars["motor_speed"]),
            ("俯仰编码器 / 俯仰角", self.snapshot_vars["pitch"]),
            ("俯仰角速度", self.snapshot_vars["pitch_rate"]),
            ("通信状态", self.snapshot_vars["health"]),
        ]
        for index, (label, value) in enumerate(rows):
            ttk.Label(frame, text=f"{label}:", width=18).grid(row=index, column=0, sticky=tk.W, pady=2)
            ttk.Label(frame, textvariable=value, font=("Consolas", 10)).grid(row=index, column=1, sticky=tk.W, pady=2)
        ttk.Button(frame, text="电机位移清零", command=self.reset_motor_zero).grid(row=0, column=2, padx=(25, 4))
        ttk.Button(frame, text="俯仰角清零", command=self.reset_pitch_zero).grid(row=1, column=2, padx=(25, 4))

    def _build_safety_panel(self, parent: tk.Misc) -> None:
        frame = ttk.LabelFrame(parent, text="运动安全联锁", padding=8)
        frame.pack(fill=tk.X, pady=(8, 0))
        ttk.Checkbutton(
            frame,
            text="我已确认机械活动区域、载荷、独立急停和人员站位安全",
            variable=self.arm_var,
        ).pack(side=tk.LEFT, padx=4)
        tk.Button(
            frame,
            text="立即停止 / 急停",
            command=lambda: self.stop_motion("manual_emergency_stop"),
            bg="#d32f2f",
            fg="white",
            activebackground="#b71c1c",
            activeforeground="white",
        ).pack(side=tk.RIGHT, padx=4)
        ttk.Label(frame, text=f"硬限幅：±{SAFE_RPM_CAP:.0f} rpm，单脉冲 ≤ {MAX_PULSE_S:.1f} s").pack(side=tk.RIGHT, padx=12)

    def build_stage_controls(self, parent: tk.Misc) -> None:
        ttk.Label(parent, text="本阶段没有额外操作。", padding=8).pack(anchor=tk.W)

    def refresh_ports(self) -> None:
        ports = list(serial.tools.list_ports.comports())
        names = [port.device for port in ports]
        self.lift_combo["values"] = names
        self.pitch_combo["values"] = names
        if names and self.lift_port_var.get() not in names:
            self.lift_port_var.set(names[0])
        if len(names) > 1 and self.pitch_port_var.get() not in names:
            self.pitch_port_var.set(names[1])
        details = [f"{port.device}: {port.description}" for port in ports]
        self.port_info_var.set("； ".join(details) if details else "未发现 USB 串口，请检查驱动、USB 连接和设备上电。")
        self.status_var.set("COM 口已刷新；请通过拔插 USB 确认两个设备的映射。")

    def toggle_lift(self) -> None:
        if self.bench.executor.lift_connected:
            self.stop_motion("disconnect_lift")
            self.bench.executor.disconnect_lift()
            self.status_var.set("举升缸已断开")
            return
        try:
            self.bench.connect_lift(self.lift_port_var.get(), int(self.lift_baud_var.get()))
            self.status_var.set("举升缸已连接；仍需等待有效电机编码器反馈。")
        except Exception as exc:
            messagebox.showerror("连接失败", f"举升缸连接失败：{exc}")

    def toggle_pitch(self) -> None:
        if self.bench.executor.encoder_connected:
            self.bench.executor.disconnect_encoder()
            self.status_var.set("俯仰编码器已断开")
            return
        if self.pitch_port_var.get() == self.lift_port_var.get() and self.bench.executor.lift_connected:
            messagebox.showerror("端口错误", "举升缸与俯仰编码器必须使用两个不同的 COM 口。")
            return
        try:
            self.bench.connect_pitch(self.pitch_port_var.get(), int(self.pitch_baud_var.get()))
            self.status_var.set("俯仰编码器已连接；仍需等待有效俯仰反馈。")
        except Exception as exc:
            messagebox.showerror("连接失败", f"俯仰编码器连接失败：{exc}")

    def start_log(self) -> None:
        if self.logger.active:
            self.status_var.set("本阶段 CSV 已在记录")
            return
        if self.logger.start(self.root):
            self.log_var.set("本阶段 CSV 记录中")
            self.status_var.set("CSV 已开始；运动测试现在可以解锁。")

    def stop_log(self) -> None:
        self.logger.close()
        self.log_var.set("未开始本阶段 CSV 记录")
        self.status_var.set("CSV 已停止")

    def reset_motor_zero(self) -> None:
        if self.motion_active:
            messagebox.showwarning("运动中", "请先停止运动后再清零。")
            return
        if self.bench.reset_motor_zero():
            self.status_var.set("举升缸电机编码器相对位移已清零")
        else:
            messagebox.showwarning("清零失败", "未读到有效电机编码器位置。")

    def reset_pitch_zero(self) -> None:
        if self.motion_active:
            messagebox.showwarning("运动中", "请先停止运动后再清零。")
            return
        if self.bench.reset_pitch_zero():
            self.status_var.set("俯仰角度已软件置零")
        else:
            messagebox.showwarning("清零失败", "未读到有效俯仰编码器位置。")

    def _poll_feedback(self) -> None:
        try:
            snapshot = self.bench.read_snapshot()
            self.update_readback(snapshot)
            self.logger.write(snapshot, command_rpm=self.motion_command_rpm)
            self.on_snapshot(snapshot)
        except Exception as exc:
            self.status_var.set(f"反馈读取异常：{exc}")
            if self.motion_active:
                self.stop_motion("feedback_exception")
        self.poll_job = self.root.after(FEEDBACK_PERIOD_MS, self._poll_feedback)

    def update_readback(self, snapshot: Snapshot) -> None:
        motor_raw = snapshot.motor_raw_count if snapshot.motor_raw_count is not None else "---"
        pitch_raw = snapshot.pitch_raw_count if snapshot.pitch_raw_count is not None else "---"
        self.snapshot_vars["motor"].set(f"raw={motor_raw}，pos={snapshot.motor_pos_mm:+.5f} mm")
        self.snapshot_vars["motor_speed"].set(f"{snapshot.motor_speed_mm_s:+.6f} mm/s")
        self.snapshot_vars["pitch"].set(f"raw={pitch_raw}，angle={snapshot.pitch_angle_deg:+.5f} °")
        self.snapshot_vars["pitch_rate"].set(f"{snapshot.pitch_rate_deg_s:+.6f} °/s")
        self.snapshot_vars["health"].set(
            f"电机反馈：{'有效' if snapshot.motor_valid else '无效'}；"
            f"俯仰反馈：{'有效' if snapshot.pitch_valid else '无效'}"
        )

    def on_snapshot(self, snapshot: Snapshot) -> None:
        """供每个阶段覆盖。"""

    def motion_ready(self) -> bool:
        if not self.allow_motion:
            messagebox.showerror("禁止运动", "该程序只读，不提供运动命令。")
            return False
        if not self.arm_var.get():
            messagebox.showwarning("未解锁", "请先确认机械区域、急停和人员安全。")
            return False
        if not self.logger.active:
            messagebox.showwarning("未记录", "请先开始本阶段 CSV 记录，确保动作可追溯。")
            return False
        if self.motion_active:
            messagebox.showwarning("运动中", "当前脉冲尚未结束。")
            return False
        if not self.bench.feedback_is_fresh():
            messagebox.showwarning("反馈无效", "需要两路有效且不超时的反馈，禁止运动。")
            return False
        return True

    def pulse(
        self,
        rpm: float,
        duration_s: float,
        event: str,
        on_finished: Optional[Callable[[], None]] = None,
    ) -> bool:
        if not self.motion_ready():
            return False
        try:
            rpm = float(rpm)
            duration_s = float(duration_s)
        except ValueError:
            messagebox.showerror("输入错误", "rpm 和持续时间必须是数字。")
            return False
        if abs(rpm) < 1e-9 or abs(rpm) > SAFE_RPM_CAP:
            messagebox.showerror("速度错误", f"速度必须在 0 到 {SAFE_RPM_CAP:.0f} rpm 之间。")
            return False
        if duration_s <= 0 or duration_s > MAX_PULSE_S:
            messagebox.showerror("时长错误", f"单次脉冲必须大于 0 且不超过 {MAX_PULSE_S:.1f} s。")
            return False
        if not self.bench.send_rpm(rpm):
            messagebox.showerror("发送失败", "举升缸速度命令发送失败。")
            self.bench.safe_stop()
            return False
        self.motion_active = True
        self.motion_command_rpm = rpm
        self.logger.write(self.bench.last_snapshot, event=f"{event}_start", command_rpm=rpm)
        self.status_var.set(f"已发送 {rpm:+.2f} rpm，{duration_s:.2f} s 后自动停止。")

        def finish() -> None:
            self.stop_motion(f"{event}_finish")
            if on_finished:
                on_finished()

        self.motion_job = self.root.after(max(1, int(duration_s * 1000)), finish)
        return True

    def stop_motion(self, reason: str) -> None:
        if self.motion_job is not None:
            try:
                self.root.after_cancel(self.motion_job)
            except tk.TclError:
                pass
        self.motion_job = None
        self.bench.safe_stop()
        self.logger.write(self.bench.last_snapshot, event="stop", note=reason)
        self.motion_active = False
        self.motion_command_rpm = 0.0
        self.status_var.set(f"举升缸已停止：{reason}")

    def close(self) -> None:
        try:
            self.stop_motion("app_close")
        finally:
            if self.poll_job is not None:
                self.root.after_cancel(self.poll_job)
            self.logger.close()
            self.bench.disconnect_all()
            self.root.destroy()

    @staticmethod
    def read_number(variable: tk.StringVar, name: str) -> float | None:
        try:
            return float(variable.get())
        except ValueError:
            messagebox.showerror("输入错误", f"{name} 必须是数字。")
            return None

    @staticmethod
    def save_json(parent: tk.Misc, data: dict, initialfile: str) -> bool:
        filename = filedialog.asksaveasfilename(
            parent=parent,
            title="保存标定数据",
            defaultextension=".json",
            filetypes=[("JSON 文件", "*.json")],
            initialfile=initialfile,
        )
        if not filename:
            return False
        with open(filename, "w", encoding="utf-8") as file:
            json.dump(data, file, ensure_ascii=False, indent=2)
        return True


class ConnectionCheckApp(LiftTestApp):
    phase = "00_connection"
    title = "00 - USB / COM 连接检查（只读）"

    def build_stage_controls(self, parent: tk.Misc) -> None:
        ttk.Label(parent, text="本程序不发送任何电机速度命令。", font=("Arial", 11, "bold")).pack(anchor=tk.W, pady=4)
        ttk.Label(parent, text=(
            "操作：插入两路 USB 转串口设备，点击“刷新 COM 口”，记录端口描述；"
            "通过单独拔插确认举升缸与俯仰编码器各自的 COM 号。"
        ), wraplength=860, justify=tk.LEFT).pack(anchor=tk.W, pady=4)


class FeedbackCheckApp(LiftTestApp):
    phase = "01_feedback"
    title = "01 - 电机与俯仰编码器反馈检查（只读）"

    def build_stage_controls(self, parent: tk.Misc) -> None:
        self.capture_active = False
        self.capture_samples: list[Snapshot] = []
        self.capture_result_var = tk.StringVar(value="尚未开始 30 s 稳定性记录")
        ttk.Label(parent, text="本程序只读取编码器；请勿在本程序中驱动举升缸。", foreground="#b00020").pack(anchor=tk.W, pady=4)
        ttk.Button(parent, text="开始 30 s 稳定性记录", command=self.start_capture).pack(anchor=tk.W, pady=5)
        ttk.Label(parent, textvariable=self.capture_result_var, justify=tk.LEFT).pack(anchor=tk.W, pady=5)

    def start_capture(self) -> None:
        if not self.logger.active:
            messagebox.showwarning("未记录", "请先开始本阶段 CSV 记录。")
            return
        self.capture_active = True
        self.capture_samples = []
        self.capture_started = time.time()
        self.status_var.set("正在记录 30 s 编码器稳定性。")

    def on_snapshot(self, snapshot: Snapshot) -> None:
        if not self.capture_active:
            return
        self.capture_samples.append(snapshot)
        elapsed = time.time() - self.capture_started
        if elapsed < 30.0:
            self.capture_result_var.set(f"稳定性记录中：{elapsed:.1f}/30.0 s，样本 {len(self.capture_samples)}")
            return
        self.capture_active = False
        motor_values = [item.motor_pos_mm for item in self.capture_samples if item.motor_valid]
        pitch_values = [item.pitch_angle_deg for item in self.capture_samples if item.pitch_valid]
        motor_span = max(motor_values) - min(motor_values) if motor_values else float("nan")
        pitch_span = max(pitch_values) - min(pitch_values) if pitch_values else float("nan")
        self.capture_result_var.set(
            f"完成：{len(self.capture_samples)} 样本；电机位移峰峰值 {motor_span:.6f} mm；"
            f"俯仰角峰峰值 {pitch_span:.6f} °。"
        )
        self.logger.write(snapshot, event="stability_capture_finish", note=self.capture_result_var.get())
        self.status_var.set("稳定性记录完成；如静止时变化异常，请先排查接线和干扰。")


class JogApp(LiftTestApp):
    phase = "02_jog"
    title = "02 - 举升缸低速点动与方向确认"
    allow_motion = True

    def build_stage_controls(self, parent: tk.Misc) -> None:
        self.rpm_var = tk.StringVar(value=str(DEFAULT_RPM))
        self.duration_var = tk.StringVar(value=str(DEFAULT_PULSE_S))
        self._build_motion_inputs(parent, "仅在确认方向前使用“正/负命令”，不要标记为上升或下降。")

    def _build_motion_inputs(self, parent: tk.Misc, hint: str) -> None:
        ttk.Label(parent, text=hint, wraplength=820).grid(row=0, column=0, columnspan=5, sticky=tk.W, pady=4)
        ttk.Label(parent, text="速度 (rpm):").grid(row=1, column=0, sticky=tk.E, padx=4, pady=6)
        ttk.Entry(parent, textvariable=self.rpm_var, width=9).grid(row=1, column=1, sticky=tk.W)
        ttk.Label(parent, text="脉冲时长 (s):").grid(row=1, column=2, sticky=tk.E, padx=4)
        ttk.Entry(parent, textvariable=self.duration_var, width=9).grid(row=1, column=3, sticky=tk.W)
        ttk.Button(parent, text="发送正 rpm 脉冲", command=lambda: self.jog(1)).grid(row=2, column=0, columnspan=2, sticky="ew", padx=4, pady=8)
        ttk.Button(parent, text="发送负 rpm 脉冲", command=lambda: self.jog(-1)).grid(row=2, column=2, columnspan=2, sticky="ew", padx=4, pady=8)

    def jog(self, direction: int) -> None:
        rpm = self.read_number(self.rpm_var, "速度")
        duration = self.read_number(self.duration_var, "脉冲时长")
        if rpm is None or duration is None:
            return
        self.pulse(direction * abs(rpm), duration, "jog")


class StrokeApp(JogApp):
    phase = "03_stroke"
    title = "03 - 举升缸安全可用行程测量"

    def build_stage_controls(self, parent: tk.Misc) -> None:
        self.rpm_var = tk.StringVar(value=str(DEFAULT_RPM))
        self.duration_var = tk.StringVar(value=str(DEFAULT_PULSE_S))
        self.endpoint_a: Snapshot | None = None
        self.endpoint_b: Snapshot | None = None
        self.stroke_result_var = tk.StringVar(value="尚未记录端点。")
        self._build_motion_inputs(parent, "低速接近安全端点后人工停止；绝不依靠程序撞击硬机械限位。")
        ttk.Button(parent, text="记录安全端点 A", command=lambda: self.record_endpoint("A")).grid(row=3, column=0, columnspan=2, sticky="ew", padx=4, pady=5)
        ttk.Button(parent, text="记录安全端点 B", command=lambda: self.record_endpoint("B")).grid(row=3, column=2, columnspan=2, sticky="ew", padx=4, pady=5)
        ttk.Button(parent, text="导出端点标定 JSON", command=self.export_calibration).grid(row=4, column=0, columnspan=2, sticky="ew", padx=4, pady=5)
        ttk.Label(parent, textvariable=self.stroke_result_var, wraplength=820).grid(row=5, column=0, columnspan=5, sticky=tk.W, pady=6)

    def record_endpoint(self, name: str) -> None:
        if self.motion_active:
            messagebox.showwarning("运动中", "停止并确认稳定后才能记录端点。")
            return
        snapshot = self.bench.last_snapshot
        if not snapshot or not snapshot.valid:
            messagebox.showwarning("反馈无效", "需要两路有效反馈才能记录端点。")
            return
        if name == "A":
            self.endpoint_a = snapshot
        else:
            self.endpoint_b = snapshot
        self.logger.write(snapshot, event=f"endpoint_{name}")
        self.update_stroke_result()

    def update_stroke_result(self) -> None:
        if not self.endpoint_a or not self.endpoint_b:
            self.stroke_result_var.set("已记录一个端点；请在另一安全端点稳定后记录另一个端点。")
            return
        delta_mm = self.endpoint_b.motor_pos_mm - self.endpoint_a.motor_pos_mm
        delta_deg = self.endpoint_b.pitch_angle_deg - self.endpoint_a.pitch_angle_deg
        self.stroke_result_var.set(
            f"安全可用行程：{abs(delta_mm):.6f} mm；俯仰范围：{abs(delta_deg):.6f} °；"
            f"方向关系：B-A = {delta_mm:+.6f} mm / {delta_deg:+.6f} °。"
        )

    def export_calibration(self) -> None:
        if not self.endpoint_a or not self.endpoint_b:
            messagebox.showwarning("数据不足", "必须先记录端点 A 和 B。")
            return
        data = {
            "type": "lift_stroke_endpoints",
            "created_at": datetime.now().isoformat(timespec="seconds"),
            "endpoint_a": {"motor_pos_mm": self.endpoint_a.motor_pos_mm, "pitch_angle_deg": self.endpoint_a.pitch_angle_deg},
            "endpoint_b": {"motor_pos_mm": self.endpoint_b.motor_pos_mm, "pitch_angle_deg": self.endpoint_b.pitch_angle_deg},
            "note": "仅为安全端点之间的线性开环前馈参考；使用前必须恢复相同电机零点与机械参考位置。",
        }
        if self.save_json(self.root, data, "03_stroke_endpoints.json"):
            self.status_var.set("端点标定 JSON 已导出。")


class SpeedApp(JogApp):
    phase = "04_speed"
    title = "04 - 举升缸不同速度特性测试"

    def build_stage_controls(self, parent: tk.Misc) -> None:
        self.rpm_var = tk.StringVar(value=str(DEFAULT_RPM))
        self.duration_var = tk.StringVar(value=str(DEFAULT_PULSE_S))
        self.start_snapshot: Snapshot | None = None
        self.last_result: dict | None = None
        self.speed_result_var = tk.StringVar(value="每档速度、每个方向建议至少重复 3 次。")
        self._build_motion_inputs(parent, "选择一档安全速度后执行单次试验；结果为端点平均速度，非瞬时峰值。")
        ttk.Button(parent, text="正方向速度试验", command=lambda: self.run_speed(1)).grid(row=3, column=0, columnspan=2, sticky="ew", padx=4, pady=5)
        ttk.Button(parent, text="负方向速度试验", command=lambda: self.run_speed(-1)).grid(row=3, column=2, columnspan=2, sticky="ew", padx=4, pady=5)
        ttk.Button(parent, text="导出最近速度结果 JSON", command=self.export_speed).grid(row=4, column=0, columnspan=2, sticky="ew", padx=4, pady=5)
        ttk.Label(parent, textvariable=self.speed_result_var, wraplength=820).grid(row=5, column=0, columnspan=5, sticky=tk.W, pady=6)

    def run_speed(self, direction: int) -> None:
        rpm = self.read_number(self.rpm_var, "速度")
        duration = self.read_number(self.duration_var, "脉冲时长")
        if rpm is None or duration is None:
            return
        self.start_snapshot = self.bench.last_snapshot

        def complete() -> None:
            end = self.bench.last_snapshot
            if not self.start_snapshot or not end:
                return
            dt = max(duration, 1e-9)
            delta_mm = end.motor_pos_mm - self.start_snapshot.motor_pos_mm
            delta_deg = end.pitch_angle_deg - self.start_snapshot.pitch_angle_deg
            self.last_result = {
                "type": "lift_speed_result",
                "created_at": datetime.now().isoformat(timespec="seconds"),
                "command_rpm": direction * abs(rpm),
                "duration_s": duration,
                "delta_mm": delta_mm,
                "delta_pitch_deg": delta_deg,
                "average_motor_speed_mm_s": delta_mm / dt,
                "average_pitch_speed_deg_s": delta_deg / dt,
            }
            self.speed_result_var.set(
                f"结果：Δ缸位移={delta_mm:+.6f} mm，平均={delta_mm / dt:+.6f} mm/s；"
                f"Δ俯仰={delta_deg:+.6f}°，平均={delta_deg / dt:+.6f}°/s。"
            )
            self.logger.write(end, event="speed_result", note=self.speed_result_var.get())

        self.pulse(direction * abs(rpm), duration, "speed", complete)

    def export_speed(self) -> None:
        if not self.last_result:
            messagebox.showwarning("数据不足", "请先完成一次速度试验。")
            return
        if self.save_json(self.root, self.last_result, "04_speed_result.json"):
            self.status_var.set("最近速度结果 JSON 已导出。")


class IncrementApp(JogApp):
    phase = "05_increment"
    title = "05 - 最小进给量与俯仰角变化测试"

    def build_stage_controls(self, parent: tk.Misc) -> None:
        self.rpm_var = tk.StringVar(value="5")
        self.duration_var = tk.StringVar(value="0.20")
        self.start_snapshot: Snapshot | None = None
        self.increment_result_var = tk.StringVar(value="从 5 rpm / 0.20 s 开始；仅在确认安全后逐步减小。")
        self._build_motion_inputs(parent, "建议每个脉冲后等待机构完全静止，再记录下一次。")
        ttk.Button(parent, text="正方向最小进给", command=lambda: self.run_increment(1)).grid(row=3, column=0, columnspan=2, sticky="ew", padx=4, pady=5)
        ttk.Button(parent, text="负方向最小进给", command=lambda: self.run_increment(-1)).grid(row=3, column=2, columnspan=2, sticky="ew", padx=4, pady=5)
        ttk.Label(parent, textvariable=self.increment_result_var, wraplength=820).grid(row=4, column=0, columnspan=5, sticky=tk.W, pady=6)

    def run_increment(self, direction: int) -> None:
        rpm = self.read_number(self.rpm_var, "速度")
        duration = self.read_number(self.duration_var, "脉冲时长")
        if rpm is None or duration is None:
            return
        self.start_snapshot = self.bench.last_snapshot

        def complete() -> None:
            end = self.bench.last_snapshot
            if not self.start_snapshot or not end:
                return
            delta_mm = end.motor_pos_mm - self.start_snapshot.motor_pos_mm
            delta_deg = end.pitch_angle_deg - self.start_snapshot.pitch_angle_deg
            self.increment_result_var.set(
                f"本次最小进给：Δ缸位移={delta_mm:+.8f} mm；Δ俯仰={delta_deg:+.8f}°。"
            )
            self.logger.write(end, event="increment_result", note=self.increment_result_var.get())

        self.pulse(direction * abs(rpm), duration, "increment", complete)


class OpenLoopAngleApp(LiftTestApp):
    phase = "06_open_loop"
    title = "06 - 开环目标俯仰角验证"
    allow_motion = True

    def build_stage_controls(self, parent: tk.Misc) -> None:
        self.lower_mm_var = tk.StringVar()
        self.lower_angle_var = tk.StringVar()
        self.upper_mm_var = tk.StringVar()
        self.upper_angle_var = tk.StringVar()
        self.target_angle_var = tk.StringVar()
        self.rpm_var = tk.StringVar(value=str(DEFAULT_RPM))
        self.speed_mm_s_var = tk.StringVar()
        self.plan_var = tk.StringVar(value="请加载或输入端点标定和实测速率。")
        self.start_snapshot: Snapshot | None = None
        self.expected_duration = 0.0
        self.expected_rpm = 0.0

        ttk.Label(parent, text=(
            "严格开环：程序只根据端点标定、当前电机位移、固定 rpm 和实测线速度计算一次运行时长；"
            "运行中不以俯仰角反馈修正。使用前必须恢复与标定相同的电机零点和机械参考位置。"
        ), wraplength=860, foreground="#b00020").grid(row=0, column=0, columnspan=6, sticky=tk.W, pady=4)
        ttk.Button(parent, text="加载 03 端点标定 JSON", command=self.load_stroke).grid(row=1, column=0, columnspan=2, sticky="ew", padx=4, pady=4)
        ttk.Button(parent, text="加载 04 速度结果 JSON", command=self.load_speed).grid(row=1, column=2, columnspan=2, sticky="ew", padx=4, pady=4)
        labels = [
            ("端点 A 位移 (mm)", self.lower_mm_var),
            ("端点 A 俯仰 (°)", self.lower_angle_var),
            ("端点 B 位移 (mm)", self.upper_mm_var),
            ("端点 B 俯仰 (°)", self.upper_angle_var),
            ("目标俯仰 (°)", self.target_angle_var),
            ("固定速度 (rpm)", self.rpm_var),
            ("该 rpm 实测线速度 (mm/s)", self.speed_mm_s_var),
        ]
        for index, (label, variable) in enumerate(labels):
            row = 2 + index // 2
            column = (index % 2) * 3
            ttk.Label(parent, text=label).grid(row=row, column=column, sticky=tk.E, padx=4, pady=3)
            ttk.Entry(parent, textvariable=variable, width=13).grid(row=row, column=column + 1, sticky=tk.W, padx=4, pady=3)
        ttk.Button(parent, text="计算单次开环计划", command=self.calculate_plan).grid(row=6, column=0, columnspan=2, sticky="ew", padx=4, pady=6)
        ttk.Button(parent, text="执行一次开环脉冲", command=self.execute_plan).grid(row=6, column=2, columnspan=2, sticky="ew", padx=4, pady=6)
        ttk.Label(parent, textvariable=self.plan_var, wraplength=860).grid(row=7, column=0, columnspan=6, sticky=tk.W, pady=6)

    def load_stroke(self) -> None:
        filename = filedialog.askopenfilename(parent=self.root, title="选择 03 行程端点 JSON", filetypes=[("JSON 文件", "*.json")])
        if not filename:
            return
        try:
            with open(filename, "r", encoding="utf-8") as file:
                data = json.load(file)
            a, b = data["endpoint_a"], data["endpoint_b"]
            self.lower_mm_var.set(str(a["motor_pos_mm"]))
            self.lower_angle_var.set(str(a["pitch_angle_deg"]))
            self.upper_mm_var.set(str(b["motor_pos_mm"]))
            self.upper_angle_var.set(str(b["pitch_angle_deg"]))
            self.status_var.set("已加载 03 端点标定。")
        except Exception as exc:
            messagebox.showerror("加载失败", f"端点标定文件无效：{exc}")

    def load_speed(self) -> None:
        filename = filedialog.askopenfilename(parent=self.root, title="选择 04 速度结果 JSON", filetypes=[("JSON 文件", "*.json")])
        if not filename:
            return
        try:
            with open(filename, "r", encoding="utf-8") as file:
                data = json.load(file)
            self.rpm_var.set(str(abs(float(data["command_rpm"]))))
            self.speed_mm_s_var.set(str(abs(float(data["average_motor_speed_mm_s"]))))
            self.status_var.set("已加载 04 速度结果；请确认方向和载荷工况一致。")
        except Exception as exc:
            messagebox.showerror("加载失败", f"速度结果文件无效：{exc}")

    def calculate_plan(self) -> bool:
        values = [
            self.read_number(self.lower_mm_var, "端点 A 位移"),
            self.read_number(self.lower_angle_var, "端点 A 俯仰"),
            self.read_number(self.upper_mm_var, "端点 B 位移"),
            self.read_number(self.upper_angle_var, "端点 B 俯仰"),
            self.read_number(self.target_angle_var, "目标俯仰"),
            self.read_number(self.rpm_var, "固定速度"),
            self.read_number(self.speed_mm_s_var, "实测线速度"),
        ]
        if any(value is None for value in values):
            return False
        lower_mm, lower_angle, upper_mm, upper_angle, target_angle, rpm, speed = values
        if abs(upper_angle - lower_angle) < 1e-9 or speed <= 0 or abs(rpm) <= 0:
            messagebox.showerror("标定错误", "端点俯仰差、固定 rpm 和实测线速度必须非零。")
            return False
        target_min, target_max = sorted((lower_angle, upper_angle))
        if target_angle < target_min or target_angle > target_max:
            messagebox.showerror("目标越界", "目标俯仰角必须位于两个安全端点之间。")
            return False
        snapshot = self.bench.last_snapshot
        if not snapshot or not snapshot.valid:
            messagebox.showwarning("反馈无效", "需要两路有效反馈才能计算开环计划。")
            return False
        ratio = (target_angle - lower_angle) / (upper_angle - lower_angle)
        target_mm = lower_mm + ratio * (upper_mm - lower_mm)
        delta_mm = target_mm - snapshot.motor_pos_mm
        self.expected_duration = abs(delta_mm) / speed
        self.expected_rpm = abs(rpm) if delta_mm >= 0 else -abs(rpm)
        self.plan_var.set(
            f"计划：目标位移 {target_mm:+.6f} mm，当前位移 {snapshot.motor_pos_mm:+.6f} mm，"
            f"发送 {self.expected_rpm:+.2f} rpm，固定时长 {self.expected_duration:.3f} s。"
        )
        if self.expected_duration > MAX_PULSE_S:
            self.plan_var.set(self.plan_var.get() + f" 超过 {MAX_PULSE_S:.1f} s 安全脉冲上限，禁止执行；请缩小测试区间。")
        return True

    def execute_plan(self) -> None:
        if not self.calculate_plan():
            return
        if self.expected_duration > MAX_PULSE_S:
            messagebox.showerror("禁止执行", "开环计划超过单脉冲安全上限。请先验证较小区间。")
            return
        self.start_snapshot = self.bench.last_snapshot

        def complete() -> None:
            end = self.bench.last_snapshot
            target = float(self.target_angle_var.get())
            if not end:
                return
            error = end.pitch_angle_deg - target
            message = f"开环完成：最终俯仰 {end.pitch_angle_deg:+.6f}°，目标 {target:+.6f}°，误差 {error:+.6f}°。"
            self.plan_var.set(message)
            self.logger.write(end, event="open_loop_result", note=message)

        self.pulse(self.expected_rpm, self.expected_duration, "open_loop", complete)


def run_app(app_type: type[LiftTestApp]) -> None:
    root = tk.Tk()
    app_type(root)
    root.mainloop()

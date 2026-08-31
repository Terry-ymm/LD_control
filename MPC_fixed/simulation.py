# -*- coding: utf-8 -*-
"""无硬件 MPC 指向仿真。

本模块只复用 :class:`mpc_controller.MPCController` 的算法，不创建串口、
传感器或执行器对象，也不会向任何硬件发送命令。

运行方式（在仓库根目录）：
    .venv\\Scripts\\python.exe MPC_fixed\\simulation.py
"""
from __future__ import annotations

import math
import time
import tkinter as tk
from dataclasses import dataclass, field
from tkinter import messagebox, ttk
from typing import Iterable

import numpy as np

from mpc_controller import MPCController


SIMULATION_PERIOD_S = 0.05
ACTUATOR_TIME_CONSTANT_S = 0.35
MAX_HISTORY_S = 30.0


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def direction_vector(
    leveling_x_deg: float,
    leveling_y_deg: float,
    azimuth_deg: float,
    pitch_deg: float,
) -> np.ndarray:
    """使用与 MPC 相同的旋转链，将当前机构角度转换为阵面指向向量。"""
    gx, gy, b, a = map(
        math.radians,
        (leveling_x_deg, leveling_y_deg, azimuth_deg, pitch_deg),
    )
    magx = np.array([
        [1.0, 0.0, 0.0],
        [0.0, math.cos(gx), math.sin(gx)],
        [0.0, -math.sin(gx), math.cos(gx)],
    ])
    magy = np.array([
        [math.cos(gy), 0.0, -math.sin(gy)],
        [0.0, 1.0, 0.0],
        [math.sin(gy), 0.0, math.cos(gy)],
    ])
    mb = np.array([
        [math.cos(b), math.sin(b), 0.0],
        [-math.sin(b), math.cos(b), 0.0],
        [0.0, 0.0, 1.0],
    ])
    ma = np.array([
        [1.0, 0.0, 0.0],
        [0.0, math.cos(a), math.sin(a)],
        [0.0, -math.sin(a), math.cos(a)],
    ])
    return magx.T @ magy.T @ mb.T @ ma.T @ np.array([0.0, 0.0, -1.0])


def angular_error_deg(current: np.ndarray, target: np.ndarray) -> float:
    """两个单位方向向量的夹角，单位为度。"""
    cosine = _clamp(float(np.dot(current, target)), -1.0, 1.0)
    return math.degrees(math.acos(cosine))


def parse_target_vector(value: str) -> np.ndarray:
    """解析并归一化以空格或逗号分隔的 Rx, Ry, Rz。"""
    numbers = [part for part in value.replace(",", " ").split() if part]
    if len(numbers) != 3:
        raise ValueError("目标向量必须包含 3 个数，例如：0, 0.866025, -0.5")
    vector = np.asarray([float(part) for part in numbers], dtype=float)
    length = float(np.linalg.norm(vector))
    if not math.isfinite(length) or length <= 1e-9:
        raise ValueError("目标向量不能为零向量")
    return vector / length


def target_pitch_deg(target: np.ndarray) -> float:
    """以调平角为零时，目标向量对应的俯仰角。"""
    return math.degrees(math.acos(_clamp(-float(target[2]), -1.0, 1.0)))


@dataclass
class VirtualPlant:
    """四通道虚拟机构。

    输入是 MPC 的四个系统级角速度。为便于观察执行器动态，真实角速度
    以一阶惯性跟随命令，再积分为调平、方位和俯仰角。
    """

    leveling_x_deg: float = 0.0
    leveling_y_deg: float = 0.0
    azimuth_deg: float = 0.0
    pitch_deg: float = 0.0
    actual_rates_rad_s: np.ndarray = field(
        default_factory=lambda: np.zeros(4, dtype=float)
    )

    def vector(self) -> np.ndarray:
        return direction_vector(
            self.leveling_x_deg,
            self.leveling_y_deg,
            self.azimuth_deg,
            self.pitch_deg,
        )

    def step(self, desired_rates_rad_s: Iterable[float], dt: float) -> None:
        desired = np.asarray(list(desired_rates_rad_s), dtype=float)
        follow = min(1.0, dt / ACTUATOR_TIME_CONSTANT_S)
        self.actual_rates_rad_s += follow * (desired - self.actual_rates_rad_s)
        delta_deg = np.degrees(self.actual_rates_rad_s) * dt

        self.leveling_x_deg = _clamp(self.leveling_x_deg + delta_deg[0], -2.0, 2.0)
        self.leveling_y_deg = _clamp(self.leveling_y_deg + delta_deg[1], -2.0, 2.0)
        self.azimuth_deg += float(delta_deg[2])
        self.pitch_deg = _clamp(self.pitch_deg + delta_deg[3], -0.02, 65.0)

        # 到达机械限位时，清除仍然指向限位外侧的虚拟速度。
        if self.leveling_x_deg in (-2.0, 2.0):
            self.actual_rates_rad_s[0] = 0.0
        if self.leveling_y_deg in (-2.0, 2.0):
            self.actual_rates_rad_s[1] = 0.0
        if self.pitch_deg in (-0.02, 65.0):
            self.actual_rates_rad_s[3] = 0.0


class MPCSimulationApp:
    """MPC 仿真 GUI。"""

    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("雷达阵面 MPC 无硬件仿真")
        self.root.geometry("1120x820")
        self.root.minsize(920, 680)

        self.mpc = MPCController()
        self.plant = VirtualPlant()
        self.target = np.array([0.0, math.sin(math.radians(60.0)), -0.5])
        self.command_rates = np.zeros(4, dtype=float)
        self.running = False
        self.job: str | None = None
        self.last_tick = 0.0
        self.last_mpc_tick = 0.0
        self.elapsed_s = 0.0
        self.history: list[tuple[float, float, float, float, float, float]] = []

        self.initial_x_var = tk.StringVar(value="1.20")
        self.initial_y_var = tk.StringVar(value="-0.80")
        self.initial_azimuth_var = tk.StringVar(value="0.00")
        self.initial_pitch_var = tk.StringVar(value="5.00")
        self.target_vector_var = tk.StringVar(value="0, 0.866025, -0.5")
        self.status_var = tk.StringVar(value="已就绪：请设置初始姿态和目标向量")
        self.time_var = tk.StringVar(value="0.00 s")
        self.error_var = tk.StringVar(value="--- °")
        self.solver_var = tk.StringVar(value="未运行")
        self.coordinate_vars = {
            "leveling": tk.StringVar(value="X=+0.000°  Y=+0.000°"),
            "azimuth": tk.StringVar(value="+0.000°"),
            "pitch": tk.StringVar(value="+0.000°"),
            "current_vector": tk.StringVar(value="[+0.0000, +0.0000, -1.0000]"),
            "target_vector": tk.StringVar(value="[+0.0000, +0.8660, -0.5000]"),
            "rate": tk.StringVar(value="[0.000, 0.000, 0.000, 0.000] °/s"),
        }

        self._build_ui()
        self.reset_simulation(show_message=False)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

    def _build_ui(self) -> None:
        outer = ttk.Frame(self.root, padding=10)
        outer.pack(fill=tk.BOTH, expand=True)

        config = ttk.LabelFrame(outer, text="仿真初始条件与目标", padding=8)
        config.pack(fill=tk.X)
        entries = [
            ("初始调平 X 倾角 (°)", self.initial_x_var),
            ("初始调平 Y 倾角 (°)", self.initial_y_var),
            ("初始方位角 (°)", self.initial_azimuth_var),
            ("初始俯仰角 (°)", self.initial_pitch_var),
        ]
        for index, (label, variable) in enumerate(entries):
            ttk.Label(config, text=label).grid(row=0, column=index * 2, sticky=tk.W, padx=(0, 4), pady=3)
            ttk.Entry(config, textvariable=variable, width=9).grid(row=0, column=index * 2 + 1, sticky=tk.W, padx=(0, 12), pady=3)

        ttk.Label(config, text="期望指向向量 [Rx, Ry, Rz]").grid(row=1, column=0, columnspan=2, sticky=tk.W, pady=4)
        ttk.Entry(config, textvariable=self.target_vector_var, width=31).grid(row=1, column=2, columnspan=3, sticky=tk.W, pady=4)
        ttk.Button(config, text="重置为设定值", command=self.reset_simulation).grid(row=1, column=5, padx=8, pady=4)
        self.start_button = ttk.Button(config, text="开始仿真", command=self.toggle_running)
        self.start_button.grid(row=1, column=6, padx=4, pady=4)
        ttk.Button(config, text="停止并保持", command=self.stop).grid(row=1, column=7, padx=4, pady=4)
        ttk.Label(
            config,
            text="可达范围：调平 X/Y ±2°，俯仰 0–65°；向量会自动归一化。",
        ).grid(row=2, column=0, columnspan=8, sticky=tk.W, pady=(5, 0))

        summary = ttk.Frame(outer)
        summary.pack(fill=tk.X, pady=(10, 5))
        for column, (title, variable) in enumerate([
            ("仿真时间", self.time_var),
            ("指向误差", self.error_var),
            ("求解器", self.solver_var),
        ]):
            box = ttk.LabelFrame(summary, text=title, padding=6)
            box.grid(row=0, column=column, sticky="nsew", padx=(0, 8) if column < 2 else 0)
            ttk.Label(box, textvariable=variable, font=("Arial", 13, "bold")).pack(anchor=tk.W)
            summary.columnconfigure(column, weight=1)

        state = ttk.LabelFrame(outer, text="实时机构坐标", padding=8)
        state.pack(fill=tk.X, pady=5)
        rows = [
            ("调平", self.coordinate_vars["leveling"]),
            ("方位", self.coordinate_vars["azimuth"]),
            ("俯仰", self.coordinate_vars["pitch"]),
            ("当前指向", self.coordinate_vars["current_vector"]),
            ("目标指向", self.coordinate_vars["target_vector"]),
            ("MPC输出", self.coordinate_vars["rate"]),
        ]
        for index, (title, variable) in enumerate(rows):
            ttk.Label(state, text=f"{title}:", width=11).grid(row=index, column=0, sticky=tk.W, pady=2)
            ttk.Label(state, textvariable=variable, font=("Consolas", 10)).grid(row=index, column=1, sticky=tk.W, pady=2)

        views = ttk.Frame(outer)
        views.pack(fill=tk.BOTH, expand=True, pady=(5, 0))
        vector_frame = ttk.LabelFrame(views, text="指向向量投影（横轴 Rx，纵轴 Ry；标签含 Rz）", padding=5)
        vector_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))
        self.vector_canvas = tk.Canvas(vector_frame, background="#ffffff", highlightthickness=1, highlightbackground="#b0b0b0")
        self.vector_canvas.pack(fill=tk.BOTH, expand=True)
        self.vector_canvas.bind("<Configure>", lambda _event: self.draw_vector_view())

        history_frame = ttk.LabelFrame(views, text="实时角度变化（最近 30 s）", padding=5)
        history_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(5, 0))
        self.history_canvas = tk.Canvas(history_frame, background="#ffffff", highlightthickness=1, highlightbackground="#b0b0b0")
        self.history_canvas.pack(fill=tk.BOTH, expand=True)
        self.history_canvas.bind("<Configure>", lambda _event: self.draw_history_view())

        ttk.Label(outer, textvariable=self.status_var, relief=tk.SUNKEN, anchor=tk.W).pack(fill=tk.X, pady=(8, 0))

    def reset_simulation(self, show_message: bool = True) -> None:
        self.stop()
        try:
            target = parse_target_vector(self.target_vector_var.get())
            pitch = target_pitch_deg(target)
            if pitch > 65.0 + 1e-9:
                raise ValueError(
                    f"该目标向量对应俯仰 {pitch:.2f}°，超过源码仿真的 65°俯仰上限"
                )
            initial_x = _clamp(float(self.initial_x_var.get()), -2.0, 2.0)
            initial_y = _clamp(float(self.initial_y_var.get()), -2.0, 2.0)
            initial_azimuth = float(self.initial_azimuth_var.get())
            initial_pitch = _clamp(float(self.initial_pitch_var.get()), -0.02, 65.0)
        except ValueError as exc:
            messagebox.showerror("输入错误", str(exc))
            return

        self.target = target
        self.plant = VirtualPlant(initial_x, initial_y, initial_azimuth, initial_pitch)
        self.mpc.reset()
        self.command_rates = np.zeros(4, dtype=float)
        self.elapsed_s = 0.0
        self.last_mpc_tick = 0.0
        self.history = []
        self._record_state()
        self._refresh_display()
        self.status_var.set("仿真已重置；点击“开始仿真”执行 MPC 闭环控制")
        if show_message:
            messagebox.showinfo("仿真已重置", "已使用新的初始姿态和目标向量。")

    def toggle_running(self) -> None:
        if self.running:
            self.stop()
            return
        self.running = True
        self.start_button.config(text="暂停仿真")
        self.last_tick = time.perf_counter()
        self.status_var.set("MPC 闭环仿真运行中")
        self._tick()

    def stop(self) -> None:
        self.running = False
        self.command_rates[:] = 0.0
        self.plant.actual_rates_rad_s[:] = 0.0
        if hasattr(self, "start_button"):
            self.start_button.config(text="开始仿真")
        if self.job is not None:
            try:
                self.root.after_cancel(self.job)
            except tk.TclError:
                pass
        self.job = None

    def _tick(self) -> None:
        if not self.running:
            return
        now = time.perf_counter()
        dt = _clamp(now - self.last_tick, 0.001, 0.15)
        self.last_tick = now
        self.elapsed_s += dt

        if self.elapsed_s - self.last_mpc_tick >= self.mpc.T - 1e-9:
            current_vector = self.plant.vector()
            command = self.mpc.compute({
                "x_angle_deg": self.plant.leveling_x_deg,
                "y_angle_deg": self.plant.leveling_y_deg,
                "z_angle_deg": 0.0,
                "azimuth_angle_deg": self.plant.azimuth_deg,
                "lift_angle_deg": self.plant.pitch_deg,
                "target_vector": self.target,
            })
            self.last_mpc_tick = self.elapsed_s
            if command.get("ok", False):
                self.command_rates = np.asarray(command["u_rad_s"], dtype=float)
                self.solver_var.set(command.get("solver_message", "solved"))
            else:
                self.command_rates[:] = 0.0
                self.solver_var.set(f"失败：{command.get('solver_message', '')}")
                self.status_var.set("MPC 求解失败，仿真已保持当前位置")
            del current_vector

        self.plant.step(self.command_rates, dt)
        self._record_state()
        self._refresh_display()
        self.job = self.root.after(int(SIMULATION_PERIOD_S * 1000), self._tick)

    def _record_state(self) -> None:
        self.history.append((
            self.elapsed_s,
            self.plant.leveling_x_deg,
            self.plant.leveling_y_deg,
            self.plant.azimuth_deg,
            self.plant.pitch_deg,
            angular_error_deg(self.plant.vector(), self.target),
        ))
        cutoff = self.elapsed_s - MAX_HISTORY_S
        while self.history and self.history[0][0] < cutoff:
            self.history.pop(0)

    def _refresh_display(self) -> None:
        current = self.plant.vector()
        error = angular_error_deg(current, self.target)
        self.time_var.set(f"{self.elapsed_s:.2f} s")
        self.error_var.set(f"{error:.4f} °")
        self.coordinate_vars["leveling"].set(
            f"X={self.plant.leveling_x_deg:+.4f}°  Y={self.plant.leveling_y_deg:+.4f}°"
        )
        self.coordinate_vars["azimuth"].set(f"{self.plant.azimuth_deg:+.4f}°")
        self.coordinate_vars["pitch"].set(f"{self.plant.pitch_deg:+.4f}°")
        self.coordinate_vars["current_vector"].set(self._format_vector(current))
        self.coordinate_vars["target_vector"].set(self._format_vector(self.target))
        self.coordinate_vars["rate"].set(
            "[" + ", ".join(f"{value:+.4f}" for value in np.degrees(self.command_rates)) + "] °/s"
        )
        if error < 0.05 and self.running:
            self.status_var.set("已收敛：指向误差小于 0.05°，MPC 继续保持目标")
        self.draw_vector_view()
        self.draw_history_view()

    @staticmethod
    def _format_vector(vector: np.ndarray) -> str:
        return "[" + ", ".join(f"{value:+.4f}" for value in vector) + "]"

    def draw_vector_view(self) -> None:
        canvas = self.vector_canvas
        width, height = max(1, canvas.winfo_width()), max(1, canvas.winfo_height())
        canvas.delete("all")
        if width < 40 or height < 40:
            return
        center_x, center_y = width / 2, height / 2
        radius = max(20, min(width, height) * 0.36)
        canvas.create_oval(center_x - radius, center_y - radius, center_x + radius, center_y + radius, outline="#777777")
        canvas.create_line(center_x - radius, center_y, center_x + radius, center_y, fill="#aaaaaa")
        canvas.create_line(center_x, center_y - radius, center_x, center_y + radius, fill="#aaaaaa")
        canvas.create_text(center_x + radius + 18, center_y + 12, text="Rx", anchor=tk.W)
        canvas.create_text(center_x + 10, center_y - radius - 12, text="Ry", anchor=tk.W)

        def draw_arrow(vector: np.ndarray, color: str, name: str) -> None:
            end_x = center_x + radius * float(vector[0])
            end_y = center_y - radius * float(vector[1])
            canvas.create_line(center_x, center_y, end_x, end_y, fill=color, width=3, arrow=tk.LAST)
            canvas.create_text(end_x, end_y - 12, text=f"{name}  Rz={vector[2]:+.3f}", fill=color, anchor=tk.S)

        draw_arrow(self.target, "#d43d3d", "目标")
        draw_arrow(self.plant.vector(), "#1677c8", "当前")
        canvas.create_text(10, 10, anchor=tk.NW, text="圆周表示 Rx/Ry 投影的最大范围；向量本身始终归一化。")

    def draw_history_view(self) -> None:
        canvas = self.history_canvas
        width, height = max(1, canvas.winfo_width()), max(1, canvas.winfo_height())
        canvas.delete("all")
        if width < 100 or height < 100 or not self.history:
            return
        left, right, top, bottom = 48, 12, 22, 30
        plot_w, plot_h = max(1, width - left - right), max(1, height - top - bottom)
        values = np.asarray(self.history, dtype=float)
        start_t, end_t = values[0, 0], max(values[-1, 0], values[0, 0] + 1.0)
        series = [
            ("调平 X", values[:, 1], "#1f77b4"),
            ("调平 Y", values[:, 2], "#9467bd"),
            ("方位", values[:, 3], "#e37b00"),
            ("俯仰", values[:, 4], "#2b8a3e"),
        ]
        all_values = np.concatenate([item[1] for item in series])
        lower, upper = float(np.min(all_values)), float(np.max(all_values))
        padding = max(1.0, (upper - lower) * 0.12)
        lower, upper = lower - padding, upper + padding

        canvas.create_rectangle(left, top, left + plot_w, top + plot_h, outline="#777777")
        for tick in range(5):
            y = top + plot_h * tick / 4
            value = upper - (upper - lower) * tick / 4
            canvas.create_line(left, y, left + plot_w, y, fill="#e5e5e5")
            canvas.create_text(left - 6, y, text=f"{value:.1f}", anchor=tk.E)
        canvas.create_text(left + plot_w / 2, height - 10, text="仿真时间 (s)")
        canvas.create_text(12, top + plot_h / 2, text="角度 (°)", angle=90)

        for index, (name, data, color) in enumerate(series):
            points: list[float] = []
            for timestamp, value in zip(values[:, 0], data):
                x = left + (timestamp - start_t) / (end_t - start_t) * plot_w
                y = top + (upper - value) / (upper - lower) * plot_h
                points.extend((x, y))
            if len(points) >= 4:
                canvas.create_line(*points, fill=color, width=2, smooth=True)
            canvas.create_text(left + 6 + index * 78, 10, text=name, fill=color, anchor=tk.W)

    def close(self) -> None:
        self.stop()
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    MPCSimulationApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()

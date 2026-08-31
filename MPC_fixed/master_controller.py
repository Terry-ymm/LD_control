# -*- coding: utf-8 -*-
"""三系统总控界面：调平 + 方位 + 举升/俯仰 + MPC 接口。"""
from __future__ import annotations

import csv
import os
import math
import threading
import time
from datetime import datetime

import serial.tools.list_ports
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

from azimuth_executor import AzimuthExecutor
from common import clamp
from leveling_executor import LevelingExecutor
from lift_executor import LiftExecutor
from mpc_controller import MPCController
from target_trajectory import TargetTrajectory
from tilt_sensor import TiltSensorReader


class MasterControlApp:
    def __init__(self, root):
        self.root = root
        self.root.title("三系统总控-MPC预留版（拆分文件）")

        screen_w = self.root.winfo_screenwidth()
        screen_h = self.root.winfo_screenheight()
        win_w = min(1280, int(screen_w * 0.92))
        win_h = min(900, int(screen_h * 0.90))
        self.root.geometry(f"{win_w}x{win_h}")
        self.root.minsize(980, 680)

        # 子系统
        self.sensor = TiltSensorReader(on_data=self._on_sensor_data, on_status=self._thread_status)
        self.leveling = LevelingExecutor()
        self.azimuth = AzimuthExecutor()
        self.lift = LiftExecutor()
        self.mpc = MPCController()
        self.target_trajectory = TargetTrajectory()
        self.mpc_start_time = None

        # MPC 启动时的倾角传感器 Z 角零点
        # MPC 内部使用 z_angle_deg = 当前Z角 - 该零点
        self.mpc_z_zero_deg = 0.0

        # MPC 实验时，举升角度低于该值时禁止方位运动
        # 单位：deg
        self.azimuth_enable_lift_angle_deg = 40.0

        # 周期和总控状态
        # 周期统一设置为 100 ms：输出周期、反馈周期、MPC周期保持一致。
        self.control_period_ms = 200
        self.feedback_period_ms = 200
        self.mpc_period_ms = 200
        self.mpc_cmd_timeout_s = 1.0

        self.control_mode = "IDLE"  # IDLE / LOCAL_LEVELING / MPC
        self.control_job = None
        self.mpc_active = False
        self.mpc_command = {"leveling_leg_rpm": [0, 0, 0, 0], "azimuth_rpm": 0.0, "lift_rpm": 0.0}
        self.last_mpc_cmd_time = 0.0
        self.last_mpc_elapsed_s = 0.0
        self.last_target_sequence = [0.0] * 9

        # MPC 求解失败计数
        # 单次求解失败不立即停机，连续失败达到阈值才停止
        self.mpc_fail_count = 0
        self.mpc_fail_limit = 3
        self.last_mpc_solver_ok = True
        self.last_mpc_solver_message = ""

        self.feedback_thread = None
        self.stop_feedback_thread = False
        self.feedback_running = False

        # CSV 记录
        self.is_logging = False
        self.log_file = None
        self.log_writer = None
        self.log_start_time = None
        self.last_log_time = 0.0
        self.log_interval_s = 0.1

        # CSV 不再每行 flush，改为定时 flush，减少磁盘写入阻塞
        self.last_log_flush_time = 0.0
        self.log_flush_interval_s = 3.0

        # Tk 变量：通信配置
        self.sensor_port_var = tk.StringVar()
        self.sensor_baud_var = tk.StringVar(value="230400")

        self.level_port_vars = [tk.StringVar() for _ in range(4)]
        self.level_baud_vars = [tk.StringVar(value="115200") for _ in range(4)]

        self.azimuth_port_var = tk.StringVar()
        self.azimuth_baud_var = tk.StringVar(value="115200")

        self.lift_port_var = tk.StringVar()
        self.lift_baud_var = tk.StringVar(value="115200")

        self.pitch_encoder_port_var = tk.StringVar()
        self.pitch_encoder_baud_var = tk.StringVar(value="115200")

        # Tk 变量：总控
        self.control_period_var = tk.StringVar(value=str(self.control_period_ms))
        self.feedback_period_var = tk.StringVar(value=str(self.feedback_period_ms))
        self.mpc_period_var = tk.StringVar(value=str(self.mpc_period_ms))
        self.mpc_timeout_var = tk.StringVar(value=str(self.mpc_cmd_timeout_s))
        self.mode_var = tk.StringVar(value="IDLE")
        self.status_var = tk.StringVar(value="就绪")

        # Tk 变量：显示
        self.angle_vars = {
            "x_angle": tk.StringVar(value="+0.000°"),
            "y_angle": tk.StringVar(value="+0.000°"),
            "z_angle": tk.StringVar(value="+0.000°"),
        }
        self.sensor_fps_var = tk.StringVar(value="0.0 Hz / 0 帧")

        self.leg_cmd_vars = [tk.StringVar(value="0 r/min") for _ in range(4)]
        self.leg_pos_vars = [tk.StringVar(value="--- mm") for _ in range(4)]
        # self.leg_vel_vars = [tk.StringVar(value="--- r/min (--- mm/s)") for _ in range(4)]

        # Tk 变量：调平手动/参数调试
        self.level_debug_rpm_vars = [tk.StringVar(value="30") for _ in range(4)]
        self.level_all_rpm_var = tk.StringVar(value="30")
        self.level_x_rate_deg_s_var = tk.StringVar(value="0.2")
        self.level_y_rate_deg_s_var = tk.StringVar(value="0.0")
        self.level_kp_var = tk.StringVar(value=str(self.leveling.k_p))
        self.level_deadband_var = tk.StringVar(value=str(self.leveling.deadband))
        self.level_twist_var = tk.StringVar(value=str(self.leveling.k_twist))
        self.level_max_rpm_var = tk.StringVar(value=str(self.leveling.max_motor_rpm))
        self.level_hold_time_var = tk.StringVar(value=str(self.leveling.hold_time_s))

        self.azimuth_vars = {
            "cmd": tk.StringVar(value="0.000 rpm"),
            "fb": tk.StringVar(value="--- rpm"),
            "angle": tk.StringVar(value="--- °"),
            "status": tk.StringVar(value="未连接"),
            "fault": tk.StringVar(value="---"),
        }

        self.lift_vars = {
            "cmd": tk.StringVar(value="0 r/min"),
            "encoder": tk.StringVar(value="待补协议"),
            "pitch_rate": tk.StringVar(value="--- °/s"),
            "pitch_target": tk.StringVar(value="0.000 °/s"),
            "pitch_gain": tk.StringVar(value="---"),
        }

        # 举升最低限位显示
        _loaded_limit = self.lift.pitch_min_limit_raw_deg
        self.lift_min_limit_var = tk.StringVar(
            value=f"{_loaded_limit:.3f}°" if _loaded_limit is not None else "未设置"
        )
        # "vel": tk.StringVar(value="--- r/min (--- mm/s)"),

        # Tk 变量：方位调试
        self.azimuth_debug_rpm_var = tk.StringVar(value="0.1")
        self.azimuth_debug_rate_deg_s_var = tk.StringVar(value="3.0")

        # Tk 变量：举升/俯仰调试
        self.lift_debug_rpm_var = tk.StringVar(value="30")
        self.pitch_debug_omega_var = tk.StringVar(value="0.5")
        self.pitch_pid_kp_var = tk.StringVar(value="50.0")
        self.pitch_pid_ki_var = tk.StringVar(value="0.0")
        self.pitch_pid_kd_var = tk.StringVar(value="0.0")
        self.pitch_pid_corr_limit_var = tk.StringVar(value="50.0")
        self.pitch_target_limit_var = tk.StringVar(value="3.5")
        self.pitch_direction_var = tk.StringVar(value="1")
        self.pitch_debug_active = False
        self.pitch_debug_job = None

        # Tk 变量：举升位置闭环驱动（点到点）
        self.lift_position_target_var = tk.StringVar(value="10.0")
        self.lift_position_max_vel_var = tk.StringVar(value="0.5")
        self.lift_position_kp_var = tk.StringVar(value="5.0")
        self.lift_position_deadband_var = tk.StringVar(value="0.05")
        self.lift_position_status_var = tk.StringVar(value="位置环未启动")
        self.lift_position_active = False
        self.lift_position_job = None

        self.log_interval_var = tk.StringVar(value="180")
        self.log_status_var = tk.StringVar(value="未记录")

        # 控件引用
        self.sensor_port_combo = None
        self.sensor_connect_btn = None
        self.level_port_combos = []
        self.level_connect_btns = []
        self.azimuth_port_combo = None
        self.azimuth_connect_btn = None
        self.lift_port_combo = None
        self.lift_connect_btn = None
        self.pitch_encoder_port_combo = None
        self.pitch_encoder_connect_btn = None

        self.setup_ui()
        self.refresh_ports()
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    # =========================================================
    # UI
    # =========================================================
    def setup_ui(self):
        outer = ttk.Frame(self.root)
        outer.pack(fill=tk.BOTH, expand=True)

        self.canvas = tk.Canvas(outer, highlightthickness=0)
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar = ttk.Scrollbar(outer, orient=tk.VERTICAL, command=self.canvas.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.canvas.configure(yscrollcommand=scrollbar.set)

        main = ttk.Frame(self.canvas, padding="10")
        win = self.canvas.create_window((0, 0), window=main, anchor="nw")
        main.bind("<Configure>", lambda e: self.canvas.configure(scrollregion=self.canvas.bbox("all")))
        self.canvas.bind("<Configure>", lambda e: self.canvas.itemconfig(win, width=e.width))
        self.canvas.bind_all("<MouseWheel>", lambda e: self.canvas.yview_scroll(int(-1 * (e.delta / 120)), "units"))

        self.setup_comm_ui(main)
        self.setup_control_ui(main)
        self.setup_sensor_ui(main)

        # 方位/俯仰放在调平上方
        self.setup_azimuth_lift_ui(main)
        self.setup_leveling_ui(main)

        self.setup_log_ui(main)

        ttk.Label(self.root, textvariable=self.status_var, relief=tk.SUNKEN, anchor=tk.W).pack(side=tk.BOTTOM, fill=tk.X)

    def setup_comm_ui(self, parent):
        frame = ttk.LabelFrame(parent, text="通信配置", padding="8")
        frame.pack(fill=tk.X, pady=5)
        baud_values = ["9600", "19200", "38400", "57600", "115200", "230400", "460800", "921600"]

        ttk.Label(frame, text="倾角传感器:").grid(row=0, column=0, sticky=tk.W, pady=3)
        self.sensor_port_combo = ttk.Combobox(frame, textvariable=self.sensor_port_var, width=10)
        self.sensor_port_combo.grid(row=0, column=1, padx=4)
        ttk.Combobox(frame, textvariable=self.sensor_baud_var, values=baud_values, width=9).grid(row=0, column=2, padx=4)
        self.sensor_connect_btn = ttk.Button(frame, text="连接传感器", command=self.toggle_sensor_connection)
        self.sensor_connect_btn.grid(row=0, column=3, padx=4)

        for i in range(4):
            ttk.Label(frame, text=f"调平{i + 1}号腿:").grid(row=i + 1, column=0, sticky=tk.W, pady=3)
            combo = ttk.Combobox(frame, textvariable=self.level_port_vars[i], width=10)
            combo.grid(row=i + 1, column=1, padx=4)
            ttk.Combobox(frame, textvariable=self.level_baud_vars[i], values=baud_values, width=9).grid(row=i + 1, column=2, padx=4)
            btn = ttk.Button(frame, text=f"连接{i + 1}号腿", command=lambda idx=i: self.toggle_level_connection(idx))
            btn.grid(row=i + 1, column=3, padx=4)
            self.level_port_combos.append(combo)
            self.level_connect_btns.append(btn)

        ttk.Label(frame, text="方位控制器:").grid(row=0, column=4, sticky=tk.W, padx=(20, 4), pady=3)
        self.azimuth_port_combo = ttk.Combobox(frame, textvariable=self.azimuth_port_var, width=10)
        self.azimuth_port_combo.grid(row=0, column=5, padx=4)
        ttk.Combobox(frame, textvariable=self.azimuth_baud_var, values=baud_values, width=9).grid(row=0, column=6, padx=4)
        self.azimuth_connect_btn = ttk.Button(frame, text="连接方位", command=self.toggle_azimuth_connection)
        self.azimuth_connect_btn.grid(row=0, column=7, padx=4)

        ttk.Label(frame, text="举升电动缸:").grid(row=1, column=4, sticky=tk.W, padx=(20, 4), pady=3)
        self.lift_port_combo = ttk.Combobox(frame, textvariable=self.lift_port_var, width=10)
        self.lift_port_combo.grid(row=1, column=5, padx=4)
        ttk.Combobox(frame, textvariable=self.lift_baud_var, values=baud_values, width=9).grid(row=1, column=6, padx=4)
        self.lift_connect_btn = ttk.Button(frame, text="连接举升", command=self.toggle_lift_connection)
        self.lift_connect_btn.grid(row=1, column=7, padx=4)

        ttk.Label(frame, text="俯仰编码器:").grid(row=2, column=4, sticky=tk.W, padx=(20, 4), pady=3)
        self.pitch_encoder_port_combo = ttk.Combobox(frame, textvariable=self.pitch_encoder_port_var, width=10)
        self.pitch_encoder_port_combo.grid(row=2, column=5, padx=4)
        ttk.Combobox(frame, textvariable=self.pitch_encoder_baud_var, values=baud_values, width=9).grid(row=2, column=6, padx=4)
        self.pitch_encoder_connect_btn = ttk.Button(frame, text="连接编码器", command=self.toggle_pitch_encoder_connection)
        self.pitch_encoder_connect_btn.grid(row=2, column=7, padx=4)

        ttk.Button(frame, text="刷新所有端口", command=self.refresh_ports).grid(row=3, column=4, columnspan=4, padx=8, pady=4, sticky="ew")

    def setup_control_ui(self, parent):
        frame = ttk.LabelFrame(parent, text="总控 / MPC", padding="8")
        frame.pack(fill=tk.X, pady=5)

        row1 = ttk.Frame(frame)
        row1.pack(fill=tk.X, pady=3)
        ttk.Label(row1, text="当前模式:").pack(side=tk.LEFT, padx=4)
        ttk.Label(row1, textvariable=self.mode_var, font=("Arial", 12, "bold")).pack(side=tk.LEFT, padx=4)
        ttk.Button(row1, text="启动本地调平阶段", command=self.start_local_leveling).pack(side=tk.LEFT, padx=8)
        ttk.Button(row1, text="启动MPC总控", command=self.start_mpc_control).pack(side=tk.LEFT, padx=8)
        self.make_red_button(
            row1,
            text="停止控制循环",
            command=self.stop_control_loop
        ).pack(side=tk.LEFT, padx=8)

        self.make_red_button(
            row1,
            text="全部停止/急停",
            command=lambda: self.stop_all("manual_stop")
        ).pack(side=tk.LEFT, padx=8)

        row2 = ttk.Frame(frame)
        row2.pack(fill=tk.X, pady=3)
        for label, var in [
            ("输出周期(ms):", self.control_period_var),
            ("反馈周期(ms):", self.feedback_period_var),
            ("MPC周期(ms):", self.mpc_period_var),
            ("MPC超时(s):", self.mpc_timeout_var),
        ]:
            ttk.Label(row2, text=label).pack(side=tk.LEFT, padx=(8, 2))
            ttk.Entry(row2, textvariable=var, width=8).pack(side=tk.LEFT, padx=2)
        ttk.Button(row2, text="更新周期参数", command=self.update_period_params).pack(side=tk.LEFT, padx=10)
        ttk.Button(row2, text="启动反馈轮询", command=self.start_feedback_polling).pack(side=tk.LEFT, padx=8)
        ttk.Button(row2, text="停止反馈轮询", command=self.stop_feedback_polling).pack(side=tk.LEFT, padx=8)

    def setup_sensor_ui(self, parent):
        frame = ttk.LabelFrame(parent, text="倾角传感器", padding="8")
        frame.pack(fill=tk.X, pady=5)
        items = [("X轴/俯仰", "x_angle"), ("Y轴/滚转", "y_angle"), ("Z轴/航向", "z_angle")]
        for i, (label, key) in enumerate(items):
            sub = ttk.LabelFrame(frame, text=label, padding="6")
            sub.grid(row=0, column=i, sticky="nsew", padx=5)
            ttk.Label(sub, textvariable=self.angle_vars[key], font=("Arial", 18, "bold")).pack()
        frame.columnconfigure((0, 1, 2), weight=1)
        ttk.Label(frame, textvariable=self.sensor_fps_var).grid(row=1, column=0, columnspan=3, sticky=tk.W, padx=8, pady=4)

    def setup_leveling_ui(self, parent):
        frame = ttk.LabelFrame(parent, text="调平系统（只保留姿态调平阶段，含手动调试）", padding="8")
        frame.pack(fill=tk.X, pady=5)

        # ===== 反馈显示 =====
        header = ["腿号", "指令", "位置"]
        for c, text in enumerate(header):
            ttk.Label(frame, text=text, font=("Arial", 10, "bold")).grid(row=0, column=c, padx=8, pady=3)
        for i in range(4):
            ttk.Label(frame, text=f"{i + 1}号腿").grid(row=i + 1, column=0, padx=8, pady=3)
            ttk.Label(frame, textvariable=self.leg_cmd_vars[i], width=14).grid(row=i + 1, column=1, padx=8, pady=3)
            ttk.Label(frame, textvariable=self.leg_pos_vars[i], width=14).grid(row=i + 1, column=2, padx=8, pady=3)

        ttk.Button(frame, text="调平当前位置清零", command=self.reset_leveling_zero).grid(row=5, column=1, columnspan=2, pady=6, sticky="ew")
        ttk.Button(frame, text="重置调平起点", command=self.reset_leveling_start_pos).grid(row=5, column=3, columnspan=2, pady=6, sticky="ew")

        # ===== 单腿手动调试 =====
        manual = ttk.LabelFrame(frame, text="调平手动调试：单腿 / 同步", padding="6")
        manual.grid(row=6, column=0, columnspan=5, sticky="ew", padx=4, pady=8)

        manual_header = ["腿号", "速度(r/min)", "上行", "下行", "停止"]
        for c, text in enumerate(manual_header):
            ttk.Label(manual, text=text, font=("Arial", 10, "bold")).grid(row=0, column=c, padx=6, pady=3)

        for i in range(4):
            ttk.Label(manual, text=f"{i + 1}号腿").grid(row=i + 1, column=0, padx=6, pady=3)
            ttk.Entry(manual, textvariable=self.level_debug_rpm_vars[i], width=8).grid(row=i + 1, column=1, padx=6, pady=3)
            ttk.Button(manual, text="上行", command=lambda idx=i: self.debug_level_leg_move(idx, 1)).grid(row=i + 1, column=2, padx=6, pady=3)
            ttk.Button(manual, text="下行", command=lambda idx=i: self.debug_level_leg_move(idx, -1)).grid(row=i + 1, column=3, padx=6, pady=3)
            self.make_red_button(
                manual,
                text="停止",
                command=lambda idx=i: self.debug_level_leg_stop(idx)
            ).grid(row=i + 1, column=4, padx=6, pady=3)

        ttk.Label(manual, text="同步速度:").grid(row=5, column=0, padx=6, pady=5, sticky=tk.E)
        ttk.Entry(manual, textvariable=self.level_all_rpm_var, width=8).grid(row=5, column=1, padx=6, pady=5)
        ttk.Button(manual, text="四腿同时上行", command=lambda: self.debug_level_all_move(1)).grid(row=5, column=2, padx=6, pady=5, sticky="ew")
        ttk.Button(manual, text="四腿同时下行", command=lambda: self.debug_level_all_move(-1)).grid(row=5, column=3, padx=6, pady=5, sticky="ew")
        self.make_red_button(
           manual,
            text="四腿全部停止",
            command=self.debug_level_all_stop
        ).grid(row=5, column=4, padx=6, pady=5, sticky="ew")

        # ===== 系统级倾角速度调试，模拟后续 MPC 的调平输出 =====
        rate_debug = ttk.LabelFrame(frame, text="调平系统级速度调试（模拟 MPC 输出）", padding="6")
        rate_debug.grid(row=7, column=0, columnspan=5, sticky="ew", padx=4, pady=8)
        ttk.Label(rate_debug, text="X轴倾角速度(deg/s):").grid(row=0, column=0, padx=4, pady=3, sticky=tk.W)
        ttk.Entry(rate_debug, textvariable=self.level_x_rate_deg_s_var, width=8).grid(row=0, column=1, padx=4, pady=3)
        ttk.Label(rate_debug, text="Y轴倾角速度(deg/s):").grid(row=0, column=2, padx=4, pady=3, sticky=tk.W)
        ttk.Entry(rate_debug, textvariable=self.level_y_rate_deg_s_var, width=8).grid(row=0, column=3, padx=4, pady=3)
        ttk.Button(rate_debug, text="发送倾角速度", command=self.debug_level_send_tilt_rate).grid(row=0, column=4, padx=6, pady=3, sticky="ew")
        self.make_red_button(
            rate_debug,
            text="停止调平输出",
            command=self.debug_level_all_stop
        ).grid(row=0, column=5, padx=6, pady=3, sticky="ew")

        # ===== 本地调平参数 =====
        param = ttk.LabelFrame(frame, text="本地调平参数", padding="6")
        param.grid(row=8, column=0, columnspan=5, sticky="ew", padx=4, pady=8)
        for c, (label, var) in enumerate([
            ("Kp:", self.level_kp_var),
            ("死区(deg):", self.level_deadband_var),
            ("抗扭K:", self.level_twist_var),
            ("最大rpm:", self.level_max_rpm_var),
            ("维持时间(s):", self.level_hold_time_var),
        ]):
            ttk.Label(param, text=label).grid(row=0, column=2 * c, padx=4, pady=3, sticky=tk.E)
            ttk.Entry(param, textvariable=var, width=8).grid(row=0, column=2 * c + 1, padx=4, pady=3, sticky=tk.W)
        ttk.Button(param, text="更新调平参数", command=self.update_leveling_params).grid(row=1, column=0, columnspan=10, pady=5, sticky="ew")

    def setup_azimuth_lift_ui(self, parent):
        frame = ttk.Frame(parent)
        frame.pack(fill=tk.BOTH, expand=True, pady=5)

        az = ttk.LabelFrame(frame, text="方位系统（逆时针为正）", padding="8")
        az.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))
        for r, (label, var) in enumerate([
            ("指令速度", self.azimuth_vars["cmd"]),
            ("反馈速度", self.azimuth_vars["fb"]),
            ("方位角", self.azimuth_vars["angle"]),
            ("状态", self.azimuth_vars["status"]),
            ("故障", self.azimuth_vars["fault"]),
        ]):
            ttk.Label(az, text=f"{label}:").grid(row=r, column=0, sticky=tk.W, padx=6, pady=5)
            ttk.Label(az, textvariable=var, font=("Arial", 11, "bold")).grid(row=r, column=1, sticky=tk.W, padx=6, pady=5)

        az_read = ttk.Frame(az)
        az_read.grid(row=5, column=0, columnspan=2, pady=4, sticky="ew")

        ttk.Button(
            az_read,
            text="开始方位读数",
            command=self.start_azimuth_reading
        ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=3)

        ttk.Button(
            az_read,
            text="停止读数",
            command=self.stop_feedback_polling
        ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=3)

        ttk.Button(az, text="方位角度置零", command=self.reset_azimuth_zero).grid(
            row=6, column=0, columnspan=2, pady=8, sticky="ew"
        )

        az_debug = ttk.LabelFrame(az, text="方位手动调试", padding="6")
        az_debug.grid(row=7, column=0, columnspan=2, sticky="ew", padx=4, pady=8)

        ttk.Label(az_debug, text="电机速度(r/min):").grid(row=0, column=0, padx=4, pady=3, sticky=tk.W)
        ttk.Entry(az_debug, textvariable=self.azimuth_debug_rpm_var, width=8).grid(row=0, column=1, padx=4, pady=3)
        ttk.Button(az_debug, text="逆时针", command=lambda: self.debug_azimuth_speed_move(1)).grid(row=0, column=2, padx=4, pady=3)
        ttk.Button(az_debug, text="顺时针", command=lambda: self.debug_azimuth_speed_move(-1)).grid(row=0, column=3, padx=4, pady=3)
        self.make_red_button(
            az_debug,
            text="停止",
            command=self.debug_azimuth_stop
        ).grid(row=0, column=4, padx=4, pady=3)

        ttk.Label(az_debug, text="角速度(deg/s):").grid(row=1, column=0, padx=4, pady=3, sticky=tk.W)
        ttk.Entry(az_debug, textvariable=self.azimuth_debug_rate_deg_s_var, width=8).grid(row=1, column=1, padx=4, pady=3)
        ttk.Button(az_debug, text="按角速度逆时针", command=lambda: self.debug_azimuth_rate_move(1)).grid(row=1, column=2, padx=4, pady=3)
        ttk.Button(az_debug, text="按角速度顺时针", command=lambda: self.debug_azimuth_rate_move(-1)).grid(row=1, column=3, padx=4, pady=3)

        ttk.Button(az_debug, text="使能", command=self.debug_azimuth_enable).grid(row=2, column=0, padx=4, pady=3, sticky="ew")
        ttk.Button(az_debug, text="失能", command=self.debug_azimuth_disable).grid(row=2, column=1, padx=4, pady=3, sticky="ew")
        ttk.Button(az_debug, text="故障复位", command=self.debug_azimuth_fault_reset).grid(row=2, column=2, columnspan=3, padx=4, pady=3, sticky="ew")

        lf = ttk.LabelFrame(frame, text="举升/俯仰系统（保留调试功能）", padding="8")
        lf.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(5, 0))
        for r, (label, var) in enumerate([
            ("举升指令", self.lift_vars["cmd"]),
            ("俯仰角度", self.lift_vars["encoder"]),
            ("俯仰角速度", self.lift_vars["pitch_rate"]),
            ("俯仰目标", self.lift_vars["pitch_target"]),
            ("当前前馈增益", self.lift_vars["pitch_gain"]),
        ]):
            ttk.Label(lf, text=f"{label}:").grid(row=r, column=0, sticky=tk.W, padx=6, pady=4)
            ttk.Label(lf, textvariable=var, font=("Arial", 11, "bold")).grid(row=r, column=1, sticky=tk.W, padx=6, pady=4)

        pitch_read = ttk.Frame(lf)
        pitch_read.grid(row=5, column=0, columnspan=2, pady=4, sticky="ew")

        ttk.Button(
            pitch_read,
            text="开始俯仰读数",
            command=self.start_pitch_reading
        ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=3)

        ttk.Button(
            pitch_read,
            text="停止读数",
            command=self.stop_feedback_polling
        ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=3)

        ttk.Button(lf, text="俯仰角度置零", command=self.reset_pitch_zero).grid(
            row=6, column=0, columnspan=2, pady=4, sticky="ew"
        )

        # 最低限位（软限位）设置
        limit_row = ttk.Frame(lf)
        limit_row.grid(row=7, column=0, columnspan=2, pady=4, sticky="ew")
        ttk.Label(limit_row, text="最低限位:").pack(side=tk.LEFT, padx=(0, 4))
        ttk.Label(limit_row, textvariable=self.lift_min_limit_var, font=("Arial", 11, "bold")).pack(side=tk.LEFT, padx=(0, 6))
        ttk.Button(limit_row, text="设为最低限位", command=self.set_lift_min_limit).pack(side=tk.LEFT, padx=3)
        ttk.Button(limit_row, text="清除", command=self.clear_lift_min_limit).pack(side=tk.LEFT, padx=3)

        debug = ttk.LabelFrame(lf, text="举升/俯仰调试", padding="6")
        debug.grid(row=8, column=0, columnspan=2, sticky="ew", padx=4, pady=8)

        ttk.Label(debug, text="手动电缸速度(r/min):").grid(row=0, column=0, sticky=tk.W, padx=4, pady=3)
        ttk.Entry(debug, textvariable=self.lift_debug_rpm_var, width=8).grid(row=0, column=1, sticky=tk.W, padx=4, pady=3)
        ttk.Button(debug, text="上升", command=lambda: self.debug_lift_manual_move(1)).grid(row=0, column=2, padx=4, pady=3)
        ttk.Button(debug, text="下降", command=lambda: self.debug_lift_manual_move(-1)).grid(row=0, column=3, padx=4, pady=3)
        self.make_red_button(
            debug,
            text="停止",
            command=self.debug_lift_stop
        ).grid(row=0, column=4, padx=4, pady=3)

        ttk.Label(debug, text="目标角速度(deg/s):").grid(row=1, column=0, sticky=tk.W, padx=4, pady=3)
        ttk.Entry(debug, textvariable=self.pitch_debug_omega_var, width=8).grid(row=1, column=1, sticky=tk.W, padx=4, pady=3)
        ttk.Button(debug, text="开始角速度调试", command=self.start_pitch_debug_control).grid(row=1, column=2, columnspan=2, padx=4, pady=3, sticky="ew")
        self.make_red_button(
            debug,
            text="停止角速度调试",
            command=self.stop_pitch_debug_control
        ).grid(row=1, column=4, padx=4, pady=3, sticky="ew")

        ttk.Label(debug, text="Kp:").grid(row=2, column=0, sticky=tk.E, padx=4, pady=3)
        ttk.Entry(debug, textvariable=self.pitch_pid_kp_var, width=8).grid(row=2, column=1, sticky=tk.W, padx=4, pady=3)
        ttk.Label(debug, text="Ki:").grid(row=2, column=2, sticky=tk.E, padx=4, pady=3)
        ttk.Entry(debug, textvariable=self.pitch_pid_ki_var, width=8).grid(row=2, column=3, sticky=tk.W, padx=4, pady=3)
        ttk.Label(debug, text="Kd:").grid(row=2, column=4, sticky=tk.E, padx=4, pady=3)
        ttk.Entry(debug, textvariable=self.pitch_pid_kd_var, width=8).grid(row=2, column=5, sticky=tk.W, padx=4, pady=3)

        ttk.Label(debug, text="PID修正限幅(rpm):").grid(row=3, column=0, sticky=tk.W, padx=4, pady=3)
        ttk.Entry(debug, textvariable=self.pitch_pid_corr_limit_var, width=8).grid(row=3, column=1, sticky=tk.W, padx=4, pady=3)
        ttk.Label(debug, text="角速度限幅(deg/s):").grid(row=3, column=2, sticky=tk.W, padx=4, pady=3)
        ttk.Entry(debug, textvariable=self.pitch_target_limit_var, width=8).grid(row=3, column=3, sticky=tk.W, padx=4, pady=3)
        ttk.Label(debug, text="方向系数:").grid(row=3, column=4, sticky=tk.E, padx=4, pady=3)
        ttk.Entry(debug, textvariable=self.pitch_direction_var, width=5).grid(row=3, column=5, sticky=tk.W, padx=4, pady=3)
        ttk.Button(debug, text="更新PID/限幅", command=self.update_pitch_pid_params).grid(row=4, column=0, columnspan=6, pady=5, sticky="ew")

        # ===== 位置闭环驱动（点到点）=====
        pos = ttk.LabelFrame(lf, text="位置闭环驱动（点到点）", padding="6")
        pos.grid(row=9, column=0, columnspan=2, sticky="ew", padx=4, pady=8)

        ttk.Label(pos, text="目标俯仰角(°):").grid(row=0, column=0, sticky=tk.E, padx=4, pady=3)
        ttk.Entry(pos, textvariable=self.lift_position_target_var, width=8).grid(row=0, column=1, sticky=tk.W, padx=4, pady=3)
        ttk.Label(pos, text="最大角速度(deg/s):").grid(row=0, column=2, sticky=tk.E, padx=4, pady=3)
        ttk.Entry(pos, textvariable=self.lift_position_max_vel_var, width=8).grid(row=0, column=3, sticky=tk.W, padx=4, pady=3)
        ttk.Label(pos, text="Kp:").grid(row=0, column=4, sticky=tk.E, padx=4, pady=3)
        ttk.Entry(pos, textvariable=self.lift_position_kp_var, width=8).grid(row=0, column=5, sticky=tk.W, padx=4, pady=3)
        ttk.Label(pos, text="死区(°):").grid(row=0, column=6, sticky=tk.E, padx=4, pady=3)
        ttk.Entry(pos, textvariable=self.lift_position_deadband_var, width=8).grid(row=0, column=7, sticky=tk.W, padx=4, pady=3)

        ttk.Button(pos, text="开始位置驱动", command=self.start_lift_position_debug).grid(
            row=1, column=0, columnspan=4, sticky="ew", padx=4, pady=5)
        self.make_red_button(pos, text="停止位置驱动", command=self.stop_lift_position_debug).grid(
            row=1, column=4, columnspan=4, sticky="ew", padx=4, pady=5)
        ttk.Label(pos, textvariable=self.lift_position_status_var, font=("Consolas", 10)).grid(
            row=2, column=0, columnspan=8, sticky=tk.W, padx=4, pady=3)

    def setup_log_ui(self, parent):
        frame = ttk.LabelFrame(parent, text="实验数据记录", padding="8")
        frame.pack(fill=tk.X, pady=5)

        ttk.Label(frame, text="记录周期(ms):").pack(side=tk.LEFT, padx=5)

        ttk.Entry(
            frame,
            textvariable=self.log_interval_var,
            width=8
        ).pack(side=tk.LEFT, padx=5)

        ttk.Button(
            frame,
            text="开始实验记录",
            command=self.start_logging
        ).pack(side=tk.LEFT, padx=8)

        ttk.Button(
            frame,
            text="停止实验记录",
            command=self.stop_logging
        ).pack(side=tk.LEFT, padx=8)

        ttk.Label(
            frame,
            textvariable=self.log_status_var
        ).pack(side=tk.LEFT, padx=10)

    # =========================================================
    # UI 辅助与通信连接
    # =========================================================
    def _thread_status(self, text: str):
        try:
            self.root.after(0, self.status_var.set, text)
        except Exception:
            pass

    def set_status(self, text: str):
        self.status_var.set(text)

    def make_red_button(self, parent, text, command):
        """创建红色停止/急停按钮。ttk.Button 在 Windows 下背景色不稳定，所以这里用 tk.Button。"""
        return tk.Button(
            parent,
            text=text,
            command=command,
            bg="#d32f2f",
            fg="white",
            activebackground="#b71c1c",
            activeforeground="white",
            relief=tk.RAISED,
            bd=2,
            padx=6,
            pady=2,
        )

    def refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        combos = [self.sensor_port_combo, self.azimuth_port_combo, self.lift_port_combo, self.pitch_encoder_port_combo] + self.level_port_combos
        for combo in combos:
            if combo is not None:
                combo["values"] = ports

        if ports:
            if self.sensor_port_var.get() not in ports:
                self.sensor_port_var.set(ports[0])
            for var in self.level_port_vars:
                if var.get() not in ports:
                    var.set("")
            if self.azimuth_port_var.get() not in ports:
                self.azimuth_port_var.set(ports[0])
            if self.lift_port_var.get() not in ports:
                self.lift_port_var.set(ports[0])
            if self.pitch_encoder_port_var.get() not in ports:
                self.pitch_encoder_port_var.set("")
        else:
            self.sensor_port_var.set("")
            self.azimuth_port_var.set("")
            self.lift_port_var.set("")
            self.pitch_encoder_port_var.set("")
            for var in self.level_port_vars:
                var.set("")
        self.status_var.set("端口已刷新")

    def toggle_sensor_connection(self):
        if self.sensor.connected:
            self.sensor.disconnect()
            self.sensor_connect_btn.config(text="连接传感器")
            self.status_var.set("倾角传感器已断开")
        else:
            try:
                self.sensor.connect(self.sensor_port_var.get(), int(self.sensor_baud_var.get()))
                self.sensor_connect_btn.config(text="断开传感器")
                self.status_var.set("倾角传感器已连接")
            except Exception as exc:
                messagebox.showerror("连接失败", f"倾角传感器连接失败: {exc}")

    def toggle_level_connection(self, leg_id: int):
        if self.leveling.connected[leg_id]:
            self.leveling.disconnect_leg(leg_id)
            self.level_connect_btns[leg_id].config(text=f"连接{leg_id + 1}号腿")
            self.status_var.set(f"{leg_id + 1}号调平腿已断开")
        else:
            try:
                self.leveling.connect_leg(leg_id, self.level_port_vars[leg_id].get(), int(self.level_baud_vars[leg_id].get()))
                self.level_connect_btns[leg_id].config(text=f"断开{leg_id + 1}号腿")
                self.status_var.set(f"{leg_id + 1}号调平腿已连接")
            except Exception as exc:
                messagebox.showerror("连接失败", f"{leg_id + 1}号调平腿连接失败: {exc}")

    def toggle_azimuth_connection(self):
        if self.azimuth.connected:
            self.azimuth.disconnect()
            self.azimuth_connect_btn.config(text="连接方位")
            self.status_var.set("方位控制器已断开")
        else:
            try:
                self.azimuth.connect(self.azimuth_port_var.get(), int(self.azimuth_baud_var.get()))
                self.azimuth_connect_btn.config(text="断开方位")
                self.status_var.set("方位控制器已连接")
            except Exception as exc:
                messagebox.showerror("连接失败", f"方位控制器连接失败: {exc}")

    def toggle_lift_connection(self):
        if self.lift.lift_connected:
            self.lift.disconnect_lift()
            self.lift_connect_btn.config(text="连接举升")
            self.status_var.set("举升电动缸已断开")
        else:
            try:
                self.lift.connect_lift(self.lift_port_var.get(), int(self.lift_baud_var.get()))
                self.lift_connect_btn.config(text="断开举升")
                self.status_var.set("举升电动缸已连接")
            except Exception as exc:
                messagebox.showerror("连接失败", f"举升电动缸连接失败: {exc}")

    def toggle_pitch_encoder_connection(self):
        if self.lift.encoder_connected:
            self.lift.disconnect_encoder()
            self.pitch_encoder_connect_btn.config(text="连接编码器")
            self.status_var.set("俯仰编码器已断开")
        else:
            try:
                self.lift.connect_encoder(self.pitch_encoder_port_var.get(), int(self.pitch_encoder_baud_var.get()))
                self.pitch_encoder_connect_btn.config(text="断开编码器")
                self.status_var.set("俯仰编码器已连接")
            except Exception as exc:
                messagebox.showerror("连接失败", f"俯仰编码器连接失败: {exc}")

    def start_azimuth_reading(self):
        """启动方位反馈读数。实际使用统一反馈轮询线程。"""
        if not self.azimuth.connected:
            messagebox.showwarning("提示", "请先连接方位控制器")
            return

        self.start_feedback_polling()
        self.status_var.set("方位读数已启动")


    def start_pitch_reading(self):
        """启动俯仰/举升反馈读数。实际使用统一反馈轮询线程。"""
        if not self.lift.encoder_connected:
            messagebox.showwarning("提示", "请先连接俯仰角度编码器")
            return

        self.start_feedback_polling()
        self.status_var.set("俯仰读数已启动")

    def _on_sensor_data(self, data: dict):
        try:
            self.root.after(0, self.update_sensor_display, data)
        except Exception:
            pass

    def update_sensor_display(self, data: dict):
        self.angle_vars["x_angle"].set(f"{data['x_angle']:+.3f}°")
        self.angle_vars["y_angle"].set(f"{data['y_angle']:+.3f}°")
        self.angle_vars["z_angle"].set(f"{data['z_angle']:+.3f}°")
        self.sensor_fps_var.set(f"{data.get('fps', 0.0):.1f} Hz / {data.get('count', 0)} 帧")

    # =========================================================
    # 反馈轮询
    # =========================================================
    def start_feedback_polling(self):
        if self.feedback_running:
            return
        self.update_period_params(show_message=False)
        self.stop_feedback_thread = False
        self.feedback_running = True
        self.feedback_thread = threading.Thread(target=self.feedback_loop, daemon=True)
        self.feedback_thread.start()
        self.status_var.set("反馈轮询已启动")

    def stop_feedback_polling(self):
        self.stop_feedback_thread = True
        self.feedback_running = False
        self.status_var.set("反馈轮询已停止")

    def feedback_loop(self):
        while not self.stop_feedback_thread:
            loop_start = time.time()

            try:
                self.leveling.update_feedback()
                self.azimuth.update_feedback()
                self.lift.update_feedback()
            
                pitch_angle = self.lift.pitch_state.get("angle_deg", 0.0)

                if pitch_angle >= 69.0:
                    try:
                        self.lift.stop()
                    except Exception:
                        pass

                    if self.control_mode != "IDLE":
                        self.stop_all("pitch_angle_limit")
                        self._thread_status("俯仰角超过安全上限，已停止")
                        return

                # 最低限位（软限位）：速度驱动下，一次下降指令会持续运动，需在反馈里主动停下。
                # 俯仰角处于限位及以下且正在变小（下降）时停止举升；上升不受影响。
                if self.lift.pitch_min_limit_raw_deg is not None:
                    raw_angle = self.lift._current_raw_angle_deg()
                    if raw_angle is not None and raw_angle <= self.lift.pitch_min_limit_raw_deg:
                        if self.lift.pitch_angular_velocity_deg_s < 0.0:
                            try:
                                self.lift.stop()
                            except Exception:
                                pass

                self.record_log_snapshot()
                self.root.after(0, self.refresh_feedback_display)

            except Exception as exc:
                self._thread_status(f"反馈读取异常: {exc}")

            elapsed = time.time() - loop_start
            target_period = self.feedback_period_ms / 1000.0
            sleep_time = max(0.02, target_period - elapsed)
            time.sleep(sleep_time)

        self.feedback_running = False

    def refresh_feedback_display(self):
        for i, leg in enumerate(self.leveling.legs):
            self.leg_cmd_vars[i].set(f"{leg['cmd_rpm']} r/min")
            self.leg_pos_vars[i].set(f"{leg['pos']:.3f} mm" if self.leveling.connected[i] else "--- mm")
            # self.leg_vel_vars[i].set(
            #     f"{leg['vel_rpm']} r/min ({leg['vel_mms']:.3f} mm/s)" if self.leveling.connected[i] else "--- r/min (--- mm/s)"
            # )

        az = self.azimuth.state
        self.azimuth_vars["cmd"].set(f"{az['cmd_rpm']:.3f} rpm")
        self.azimuth_vars["fb"].set(f"{az['fb_rpm']:.3f} rpm" if self.azimuth.connected else "--- rpm")
        self.azimuth_vars["angle"].set(f"{az['multi_angle_deg']:.3f} °" if self.azimuth.connected else "--- °")
        self.azimuth_vars["status"].set(az["enable_text"] if self.azimuth.connected else "未连接")
        self.azimuth_vars["fault"].set(az["fault_text"] if self.azimuth.connected else "---")

        lf = self.lift.lift_state
        self.lift_vars["cmd"].set(f"{lf['cmd_rpm']} r/min")
        # self.lift_vars["vel"].set(
        #     f"{lf['vel_rpm']} r/min ({lf['vel_mms']:.3f} mm/s)" if self.lift.lift_connected else "--- r/min (--- mm/s)"
        # )
        pitch = self.lift.pitch_state
        if self.lift.encoder_connected:
            self.lift_vars["encoder"].set(
                f"{pitch['angle_deg']:.3f} ° / 单圈 {pitch.get('single_angle_deg', 0.0):.3f} ° / {pitch['health']}"
            )
            self.lift_vars["pitch_rate"].set(f"{pitch.get('angle_rate_deg_s', 0.0):.3f} °/s")
        else:
            self.lift_vars["encoder"].set("未连接")
            self.lift_vars["pitch_rate"].set("--- °/s")
        self.lift_vars["pitch_target"].set(f"{pitch.get('target_omega_deg_s', 0.0):.3f} °/s")
        gain = pitch.get("gain_deg_s_per_rpm", 0.0)
        self.lift_vars["pitch_gain"].set(f"{gain:.6f} deg/s/rpm" if gain else "---")

    # =========================================================
    # 本地调平阶段
    # =========================================================
    def reset_leveling_start_pos(self):
        angles = self.sensor.get_angles()
        ref = self.leveling.reset_start_pos(angles["x_angle"], angles["y_angle"])
        self.status_var.set(f"调平起点已重置，参考腿={ref + 1}")

    def start_local_leveling(self):
        if not self.sensor.connected:
            messagebox.showwarning("提示", "本地调平阶段需要先连接倾角传感器")
            return
        if not self.leveling.all_connected():
            messagebox.showwarning("提示", "本地调平阶段需要4个调平电动缸全部连接")
            return
        self.update_period_params(show_message=False)
        self.start_feedback_polling()
        self.reset_leveling_start_pos()
        self.control_mode = "LOCAL_LEVELING"
        self.mode_var.set("LOCAL_LEVELING")
        if self.control_job is not None:
            self.root.after_cancel(self.control_job)
            self.control_job = None
        self.local_leveling_loop()
        self.status_var.set("本地调平阶段已启动（不含触地/预顶升）")

    def local_leveling_loop(self):
        if self.control_mode != "LOCAL_LEVELING":
            return
        if not self.sensor.connected:
            self.stop_all("sensor_disconnect")
            self.status_var.set("传感器断开，已停止")
            return
        angles = self.sensor.get_angles()
        leg_cmds, status, finished = self.leveling.compute_local_leveling_leg_rpm(angles["x_angle"], angles["y_angle"])
        self.leveling.apply_leg_commands(leg_cmds)
        self.refresh_feedback_display()
        self.status_var.set(status)
        if finished:
            self.stop_all("local_leveling_finish")
            self.status_var.set("本地调平完成，已停止")
            return
        self.control_job = self.root.after(self.control_period_ms, self.local_leveling_loop)

    # =========================================================
    # MPC
    # =========================================================
    def start_mpc_control(self):
        self.update_period_params(show_message=False)

        # 按设定轨迹进行 MPC 测试时必须开启实验记录。
        # 如果当前没有记录，则先弹出保存文件对话框；用户取消则不启动 MPC。
        if not self.is_logging:
            started = self.start_logging(default_prefix="mpc_trajectory_test")
            if not started:
                self.status_var.set("已取消MPC启动：未开启实验记录")
                return

        self.start_feedback_polling()

        # 启动 MPC 前先重置内部速度记忆和目标曲线计时。
        # 注意：必须放在 self.mpc_loop() 之前，否则第一次循环会沿用旧 U / 旧时间。
        self.mpc.reset()
        self.mpc_start_time = time.time()
        self.mpc_fail_count = 0

        # MPC 启动时，将当前倾角传感器 Z 角作为 MPC 的 Z 零点
        angles = self.sensor.get_angles()
        self.mpc_z_zero_deg = float(angles.get("z_angle", 0.0))

        self.control_mode = "MPC"
        self.mode_var.set("MPC")
        self.mpc_active = True
        self.last_mpc_cmd_time = time.time()
        if self.control_job is not None:
            self.root.after_cancel(self.control_job)
            self.control_job = None
        self.mpc_loop()
        self.status_var.set("MPC总控已启动")

    def schedule_next_mpc_loop(self, loop_start_time: float):
        """
        MPC补偿式调度：
        目标是让两次 MPC 循环开始时间间隔接近 self.mpc_period_ms。
        如果本轮计算/下发已经耗时 elapsed，则下一次只等待 target_period - elapsed。
        """
        elapsed = time.time() - loop_start_time
        target_period = self.mpc_period_ms / 1000.0
        delay_s = max(0.001, target_period - elapsed)
        delay_ms = max(1, int(delay_s * 1000))
        self.control_job = self.root.after(delay_ms, self.mpc_loop)

    def mpc_loop(self):
        if self.control_mode != "MPC" or not self.mpc_active:
            return

        loop_start = time.time()

        state = self.get_system_state()
        if self.mpc_start_time is None:
            self.mpc_start_time = time.time()

        mpc_elapsed_s = time.time() - self.mpc_start_time

        target_sequence = self.target_trajectory.compute(
            t=mpc_elapsed_s,
            dt=self.mpc.T
        )

        self.last_mpc_elapsed_s = mpc_elapsed_s
        self.last_target_sequence = [float(v) for v in target_sequence]

        state["target_sequence"] = target_sequence
        cmd = self.mpc.compute(state)

        self.last_mpc_solver_ok = bool(cmd.get("ok", False))
        self.last_mpc_solver_message = cmd.get("solver_message", "")

        # =====================================================
        # 方位低举升角保护
        # 举升角度低于阈值时，不允许方位电机动作。
        # MPC 可以继续计算，但实际发给方位执行器的速度强制为 0。
        # =====================================================
        lift_angle_for_azimuth = state.get("lift_angle_deg", 0.0)

        if lift_angle_for_azimuth < self.azimuth_enable_lift_angle_deg:
            cmd["azimuth_rate_raw_rad_s"] = cmd.get("azimuth_rate_rad_s", 0.0)
            cmd["azimuth_rate_rad_s"] = 0.0
            cmd["azimuth_blocked_by_lift_angle"] = True
        else:
            cmd["azimuth_rate_raw_rad_s"] = cmd.get("azimuth_rate_rad_s", 0.0)
            cmd["azimuth_blocked_by_lift_angle"] = False

        # =====================================================
        # MPC 求解失败处理
        # 单次失败不立即停机：
        #   - 不保存失败 cmd 为执行命令
        #   - 继续使用上一拍有效 self.mpc_command
        #   - 连续失败达到阈值后才停机
        # =====================================================
        if not cmd.get("ok", False):
            self.mpc_fail_count += 1
            self.status_var.set(
                f"MPC求解失败 {self.mpc_fail_count}/{self.mpc_fail_limit}: "
                f"{cmd.get('solver_message', '')}"
            )

            if self.mpc_fail_count >= self.mpc_fail_limit:
                self.stop_all("mpc_solve_failed")
                self.status_var.set(
                    f"MPC连续求解失败，已停止: {cmd.get('solver_message', '')}"
                )
                return

            # 单次失败：保持上一拍有效控制量继续执行
            # 注意：这里不调用 self.set_mpc_command(cmd)，避免把失败 cmd 覆盖掉上一拍有效命令
            if time.time() - self.last_mpc_cmd_time > self.mpc_cmd_timeout_s:
                self.stop_all("mpc_cmd_timeout")
                self.status_var.set("MPC指令超时，已停止")
                return

            self.apply_mpc_command(self.mpc_command)
            self.schedule_next_mpc_loop(loop_start)
            return

        # 求解成功：清零失败计数，并保存本次有效 MPC 命令
        self.mpc_fail_count = 0
        self.set_mpc_command(cmd)

        if time.time() - self.last_mpc_cmd_time > self.mpc_cmd_timeout_s:
            self.stop_all("mpc_cmd_timeout")
            self.status_var.set("MPC指令超时，已停止")
            return

        self.apply_mpc_command(self.mpc_command)
        self.schedule_next_mpc_loop(loop_start)

    def set_mpc_command(self, cmd: dict):
        if not isinstance(cmd, dict):
            return

        # 保留 MPC 的原始字段，兼容两类输出：
        # 1) 旧接口：leveling_leg_rpm / azimuth_rpm / lift_rpm；
        # 2) 新接口：leveling_x_rate_rad_s, leveling_y_rate_rad_s, azimuth_rate_rad_s, lift_rate_rad_s。
        out = dict(cmd)

        leg_cmd = out.get("leveling_leg_rpm", [0, 0, 0, 0])
        if not isinstance(leg_cmd, (list, tuple)) or len(leg_cmd) != 4:
            leg_cmd = [0, 0, 0, 0]
        leg_cmd = [
            int(clamp(int(v), -int(self.leveling.max_motor_rpm), int(self.leveling.max_motor_rpm)))
            for v in leg_cmd
        ]
        out["leveling_leg_rpm"] = leg_cmd

        if "azimuth_rpm" in out:
            out["azimuth_rpm"] = clamp(float(out.get("azimuth_rpm", 0.0)), -self.azimuth.max_rpm, self.azimuth.max_rpm)
        else:
            out["azimuth_rpm"] = 0.0

        if "lift_rpm" in out:
            out["lift_rpm"] = int(clamp(int(out.get("lift_rpm", 0)), -int(self.lift.max_motor_rpm), int(self.lift.max_motor_rpm)))
        else:
            out["lift_rpm"] = 0

        self.mpc_command = out
        self.last_mpc_cmd_time = time.time()

    def apply_mpc_command(self, cmd: dict):
        # 调平：优先使用新版系统级倾角速度；否则兼容旧版四腿 rpm。
        if "leveling_x_rate_rad_s" in cmd or "leveling_y_rate_rad_s" in cmd:
            self.leveling.set_tilt_rate_rad_s(
                float(cmd.get("leveling_x_rate_rad_s", 0.0)),
                float(cmd.get("leveling_y_rate_rad_s", 0.0)),
            )
        else:
            self.leveling.apply_leg_commands(cmd.get("leveling_leg_rpm", [0, 0, 0, 0]))

        if self.azimuth.connected:
            if "azimuth_rate_rad_s" in cmd:
                self.azimuth.set_azimuth_rate(float(cmd.get("azimuth_rate_rad_s", 0.0)))
            else:
                self.azimuth.set_speed(cmd.get("azimuth_rpm", 0.0))

        if self.lift.lift_connected:
            # 新版 MPC 推荐输出 lift_rate_rad_s；也兼容 lift_rate_deg_s 和旧版 lift_rpm。
            if "lift_rate_rad_s" in cmd:
                self.lift.set_lift_rate_rad_s(float(cmd.get("lift_rate_rad_s", 0.0)))
            elif "lift_rate_deg_s" in cmd:
                self.lift.set_lift_rate_deg_s(float(cmd.get("lift_rate_deg_s", 0.0)))
            else:
                self.lift.send_lift_cmd(cmd.get("lift_rpm", 0))

        self.refresh_feedback_display()

    def get_system_state(self) -> dict:
        angles = self.sensor.get_angles()
        az_state = self.azimuth.get_state()
        lift_state_all = self.lift.get_state()
        pitch_state = lift_state_all["pitch"]
        z_angle_raw_deg = float(angles.get("z_angle", 0.0))
        z_angle_mpc_deg = z_angle_raw_deg - self.mpc_z_zero_deg
        return {
            "time": time.time(),
            "mode": self.control_mode,

            # 扁平字段：方便新版 mpc_controller.py 直接读取
            "x_angle_deg": angles["x_angle"],
            "y_angle_deg": angles["y_angle"],
            # MPC 使用相对 Z 角：当前Z角 - MPC启动时Z角
            "z_angle_deg": z_angle_mpc_deg,
            "azimuth_angle_deg": az_state.get("multi_angle_deg", 0.0),
            "lift_angle_deg": pitch_state.get("angle_deg", 0.0),
            "pitch_angle_deg": pitch_state.get("angle_deg", 0.0),

            # 默认目标序列；后续可由你的 MPC/目标管理器替换
            "target_vector": [0.0, 0.0, -1.0],

            # 嵌套字段：用于日志和调试
            "angles": {
                "x_angle_deg": angles["x_angle"],
                "y_angle_deg": angles["y_angle"],
                "z_angle_deg": z_angle_mpc_deg,
            },
            "leveling": self.leveling.get_state(),
            "azimuth": az_state,
            "lift": lift_state_all["lift"],
            "pitch": pitch_state,
        }

    # =========================================================
    # 停止、清零、参数
    # =========================================================
    def update_period_params(self, show_message=True):
        try:
            self.control_period_ms = max(10, int(self.control_period_var.get()))
            self.feedback_period_ms = max(20, int(self.feedback_period_var.get()))
            self.mpc_period_ms = max(20, int(self.mpc_period_var.get()))
            self.mpc_cmd_timeout_s = max(0.05, float(self.mpc_timeout_var.get()))
            if show_message:
                self.status_var.set("周期参数已更新")
        except Exception:
            messagebox.showwarning("输入错误", "周期参数请输入有效数字")

    def stop_control_loop(self):
        if self.control_job is not None:
            try:
                self.root.after_cancel(self.control_job)
            except Exception:
                pass
        self.control_job = None
        self.control_mode = "IDLE"
        self.mpc_active = False
        self.mode_var.set("IDLE")
        if self.is_logging:
            self.record_log_snapshot(force=True, stop_reason="control_loop_stop")
            self.stop_logging()

        self.status_var.set("控制循环已停止，实验记录已停止；未自动下发零速，如需停机请点全部停止")

    def stop_all(self, reason="manual_stop"):
        self.stop_pitch_debug_control(send_stop=False)
        self.stop_lift_position_debug()
        if self.control_job is not None:
            try:
                self.root.after_cancel(self.control_job)
            except Exception:
                pass
        self.control_job = None
        self.control_mode = "IDLE"
        self.mpc_active = False
        self.mode_var.set("IDLE")

        try:
            self.leveling.stop_all()
        except Exception:
            pass
        try:
            self.azimuth.stop()
        except Exception:
            pass
        try:
            # 举升停止命令重复发送几次，避免单次串口写入失败或驱动器未响应
            for _ in range(3):
                self.lift.stop()
                time.sleep(0.03)
        except Exception:
            pass
        # 停止时如果正在记录，先强制记录最后一行，再关闭 CSV
        if self.is_logging:
            self.record_log_snapshot(force=True, stop_reason=reason)
            self.stop_logging()

        self.status_var.set(f"全部停止: {reason}")
        self.refresh_feedback_display()

    # =========================================================
    # 手动调试：调平 / 方位
    # =========================================================
    def _ensure_idle_for_debug(self, name: str) -> bool:
        if self.control_mode != "IDLE":
            messagebox.showwarning("运行中", f"请先停止控制循环后再{name}")
            return False
        return True

    def update_leveling_params(self):
        try:
            max_rpm = float(self.level_max_rpm_var.get())
            if max_rpm <= 0:
                raise ValueError
            self.leveling.k_p = float(self.level_kp_var.get())
            self.leveling.deadband = float(self.level_deadband_var.get())
            self.leveling.k_twist = float(self.level_twist_var.get())
            self.leveling.max_motor_rpm = max_rpm
            self.leveling.max_cylinder_speed = (
                self.leveling.max_motor_rpm
                * (self.leveling.lead_mm / self.leveling.reduction_ratio)
                / 60.0
            )
            self.leveling.hold_time_s = float(self.level_hold_time_var.get())
            self.status_var.set("调平参数已更新")
        except Exception:
            messagebox.showwarning("输入错误", "调平参数请输入有效数字，最大 rpm 必须大于 0")

    def debug_level_leg_move(self, leg_id: int, direction: int):
        if not self._ensure_idle_for_debug("手动调试调平"):
            return
        if not self.leveling.connected[leg_id]:
            messagebox.showwarning("提示", f"请先连接{leg_id + 1}号调平腿")
            return
        try:
            rpm = abs(float(self.level_debug_rpm_vars[leg_id].get()))
        except Exception:
            messagebox.showwarning("输入错误", f"{leg_id + 1}号腿速度请输入有效数字")
            return
        rpm = rpm if direction > 0 else -rpm
        if self.leveling.send_leg_cmd(leg_id, rpm):
            self.refresh_feedback_display()
            self.status_var.set(f"已发送{leg_id + 1}号腿速度: {rpm:.1f} r/min")
        else:
            self.status_var.set(f"{leg_id + 1}号腿速度发送失败")

    def debug_level_leg_stop(self, leg_id: int):
        if not self._ensure_idle_for_debug("手动调试调平"):
            return
        self.leveling.send_leg_cmd(leg_id, 0)
        self.refresh_feedback_display()
        self.status_var.set(f"{leg_id + 1}号腿已停止")

    def debug_level_all_move(self, direction: int):
        if not self._ensure_idle_for_debug("手动调试调平"):
            return
        missing = [i + 1 for i, ok in enumerate(self.leveling.connected) if not ok]
        if missing:
            messagebox.showwarning("提示", f"请先连接这些调平腿: {missing}")
            return
        try:
            rpm = abs(float(self.level_all_rpm_var.get()))
        except Exception:
            messagebox.showwarning("输入错误", "同步速度请输入有效数字")
            return
        rpm = rpm if direction > 0 else -rpm
        self.leveling.apply_leg_commands([rpm, rpm, rpm, rpm])
        self.refresh_feedback_display()
        self.status_var.set(f"四腿同步速度已发送: {rpm:.1f} r/min")

    def debug_level_all_stop(self):
        if not self._ensure_idle_for_debug("手动调试调平"):
            return
        self.leveling.stop_all()
        self.refresh_feedback_display()
        self.status_var.set("调平四腿已停止")

    def debug_level_send_tilt_rate(self):
        if not self._ensure_idle_for_debug("调平倾角速度调试"):
            return
        if not self.leveling.all_connected():
            messagebox.showwarning("提示", "倾角速度调试需要4条调平腿全部连接")
            return
        try:
            x_rate = float(self.level_x_rate_deg_s_var.get())
            y_rate = float(self.level_y_rate_deg_s_var.get())
        except Exception:
            messagebox.showwarning("输入错误", "X/Y 倾角速度请输入有效数字，单位 deg/s")
            return
        cmd = self.leveling.set_tilt_rate_rad_s(math.radians(x_rate), math.radians(y_rate))
        self.refresh_feedback_display()
        self.status_var.set(f"已发送调平倾角速度: X={x_rate:.3f} deg/s, Y={y_rate:.3f} deg/s, cmd={cmd}")

    def debug_azimuth_enable(self):
        if not self._ensure_idle_for_debug("方位调试"):
            return
        if not self.azimuth.connected:
            messagebox.showwarning("提示", "请先连接方位控制器")
            return
        if self.azimuth.enable():
            self.status_var.set("方位电机已使能")
        else:
            self.status_var.set("方位电机使能失败")
        self.refresh_feedback_display()

    def debug_azimuth_disable(self):
        if not self._ensure_idle_for_debug("方位调试"):
            return
        if not self.azimuth.connected:
            messagebox.showwarning("提示", "请先连接方位控制器")
            return
        if self.azimuth.disable():
            self.status_var.set("方位电机已失能")
        else:
            self.status_var.set("方位电机失能失败")
        self.refresh_feedback_display()

    def debug_azimuth_fault_reset(self):
        if not self._ensure_idle_for_debug("方位调试"):
            return
        if not self.azimuth.connected:
            messagebox.showwarning("提示", "请先连接方位控制器")
            return
        if self.azimuth.fault_reset():
            self.status_var.set("方位故障复位命令已发送")
        else:
            self.status_var.set("方位故障复位失败")
        self.refresh_feedback_display()

    def debug_azimuth_speed_move(self, direction: int):
        if not self._ensure_idle_for_debug("方位速度调试"):
            return
        if not self.azimuth.connected:
            messagebox.showwarning("提示", "请先连接方位控制器")
            return
        try:
            rpm = abs(float(self.azimuth_debug_rpm_var.get()))
        except Exception:
            messagebox.showwarning("输入错误", "方位电机速度请输入有效数字，单位 r/min")
            return
        rpm = rpm if direction > 0 else -rpm
        if self.azimuth.set_speed(rpm):
            self.status_var.set(f"已发送方位电机速度: {rpm:.4f} r/min")
        else:
            self.status_var.set("方位电机速度发送失败")
        self.refresh_feedback_display()

    def debug_azimuth_rate_move(self, direction: int):
        if not self._ensure_idle_for_debug("方位角速度调试"):
            return
        if not self.azimuth.connected:
            messagebox.showwarning("提示", "请先连接方位控制器")
            return
        try:
            rate_deg_s = abs(float(self.azimuth_debug_rate_deg_s_var.get()))
        except Exception:
            messagebox.showwarning("输入错误", "方位角速度请输入有效数字，单位 deg/s")
            return
        rate_deg_s = rate_deg_s if direction > 0 else -rate_deg_s
        if self.azimuth.set_azimuth_rate(math.radians(rate_deg_s)):
            self.status_var.set(f"已发送方位角速度: {rate_deg_s:.3f} deg/s")
        else:
            self.status_var.set("方位角速度发送失败")
        self.refresh_feedback_display()

    def debug_azimuth_stop(self):
        if not self._ensure_idle_for_debug("方位调试"):
            return
        if self.azimuth.connected:
            self.azimuth.stop()
        self.refresh_feedback_display()
        self.status_var.set("方位电机已停止")

    def reset_leveling_zero(self):
        if self.control_mode != "IDLE":
            messagebox.showwarning("运行中", "请先停止控制循环后再清零")
            return
        self.leveling.reset_zero_all()
        self.refresh_feedback_display()
        self.status_var.set("调平当前位置已清零")

    def reset_lift_zero(self):
        if self.control_mode != "IDLE":
            messagebox.showwarning("运行中", "请先停止控制循环后再清零")
            return
        if not self.lift.lift_connected:
            messagebox.showwarning("提示", "请先连接举升电动缸")
            return
        if self.lift.reset_zero():
            self.refresh_feedback_display()
            self.status_var.set("举升当前位置已清零")
        else:
            self.status_var.set("举升清零失败：未读到有效反馈")

    def reset_azimuth_zero(self):
        if self.control_mode != "IDLE":
            messagebox.showwarning("运行中", "请先停止控制循环后再置零")
            return
        if not self.azimuth.connected:
            messagebox.showwarning("提示", "请先连接方位控制器")
            return
        if self.azimuth.set_zero():
            self.refresh_feedback_display()
            self.status_var.set("方位角度已置零")
        else:
            self.status_var.set("方位置零失败：还没有读取到有效方位位置")

    def reset_pitch_zero(self):
        if self.control_mode != "IDLE":
            messagebox.showwarning("运行中", "请先停止控制循环后再置零")
            return
        if not self.lift.encoder_connected:
            messagebox.showwarning("提示", "请先连接俯仰角度编码器")
            return
        if self.lift.reset_pitch_zero():
            self.refresh_feedback_display()
            self.status_var.set("俯仰角度已置零")
        else:
            self.status_var.set("俯仰角度置零失败：还没有读取到有效角度")

    def set_lift_min_limit(self):
        if self.control_mode != "IDLE":
            messagebox.showwarning("运行中", "请先停止控制循环后再设置最低限位")
            return
        if not self.lift.encoder_connected:
            messagebox.showwarning("提示", "请先连接俯仰角度编码器")
            return
        if self.lift.set_pitch_min_limit_from_current():
            limit = self.lift.pitch_min_limit_raw_deg
            self.lift_min_limit_var.set(f"{limit:.3f}°" if limit is not None else "未设置")
            self.status_var.set("已设置最低限位（软限位）：只准上升，禁止下降")
        else:
            self.status_var.set("设置失败：未读到有效俯仰角度")

    def clear_lift_min_limit(self):
        self.lift.clear_pitch_min_limit()
        self.lift_min_limit_var.set("未设置")
        self.status_var.set("已清除最低限位")

    def update_pitch_pid_params(self):
        try:
            self.lift.configure_pitch_pid(
                kp=float(self.pitch_pid_kp_var.get()),
                ki=float(self.pitch_pid_ki_var.get()),
                kd=float(self.pitch_pid_kd_var.get()),
                correction_limit_rpm=float(self.pitch_pid_corr_limit_var.get()),
                target_limit_deg_s=float(self.pitch_target_limit_var.get()),
                direction=int(float(self.pitch_direction_var.get())),
            )
            self.status_var.set("俯仰角速度 PID / 限幅参数已更新")
        except Exception:
            messagebox.showwarning("输入错误", "PID、限幅和方向系数请输入有效数字")

    def debug_lift_manual_move(self, direction: int):
        if self.control_mode != "IDLE":
            messagebox.showwarning("运行中", "请先停止控制循环后再手动调试举升")
            return
        if not self.lift.lift_connected:
            messagebox.showwarning("提示", "请先连接举升电动缸")
            return
        if direction < 0 and self.lift.is_below_pitch_min_limit():
            self.status_var.set("已到最低限位，阻止下降")
            return
        self.stop_pitch_debug_control(send_stop=False)
        try:
            rpm = abs(float(self.lift_debug_rpm_var.get()))
        except Exception:
            messagebox.showwarning("输入错误", "手动电缸速度请输入有效数字")
            return
        rpm = rpm if direction > 0 else -rpm
        if self.lift.send_lift_cmd(rpm):
            self.refresh_feedback_display()
            self.status_var.set(f"已发送举升手动速度: {rpm:.1f} r/min")
        else:
            self.status_var.set("举升手动速度发送失败")

    def debug_lift_stop(self):
        self.stop_pitch_debug_control(send_stop=False)
        if self.lift.lift_connected:
            self.lift.stop()
        self.refresh_feedback_display()
        self.status_var.set("举升调试已停止")

    def start_pitch_debug_control(self):
        if self.control_mode != "IDLE":
            messagebox.showwarning("运行中", "请先停止控制循环后再调试俯仰角速度")
            return
        if not self.lift.lift_connected:
            messagebox.showwarning("提示", "请先连接举升电动缸")
            return
        if not self.lift.encoder_connected:
            messagebox.showwarning("提示", "请先连接俯仰角度编码器")
            return
        try:
            float(self.pitch_debug_omega_var.get())
        except Exception:
            messagebox.showwarning("输入错误", "目标角速度请输入有效数字")
            return
        self.stop_lift_position_debug()
        self.update_pitch_pid_params()
        self.start_feedback_polling()
        self.pitch_debug_active = True
        if self.pitch_debug_job is not None:
            try:
                self.root.after_cancel(self.pitch_debug_job)
            except Exception:
                pass
            self.pitch_debug_job = None
        self.pitch_debug_loop()
        self.status_var.set("俯仰角速度调试已启动")

    def pitch_debug_loop(self):
        if not self.pitch_debug_active:
            return
        if not self.lift.lift_connected or not self.lift.encoder_connected:
            self.stop_pitch_debug_control()
            self.status_var.set("举升或编码器断开，俯仰角速度调试已停止")
            return
        try:
            omega = float(self.pitch_debug_omega_var.get())
        except Exception:
            omega = 0.0
        ok = self.lift.set_lift_rate_deg_s(omega)
        self.refresh_feedback_display()
        if not ok:
            self.status_var.set("俯仰角速度命令发送失败")
        self.pitch_debug_job = self.root.after(max(50, self.feedback_period_ms), self.pitch_debug_loop)

    def stop_pitch_debug_control(self, send_stop: bool = True):
        self.pitch_debug_active = False
        if self.pitch_debug_job is not None:
            try:
                self.root.after_cancel(self.pitch_debug_job)
            except Exception:
                pass
        self.pitch_debug_job = None
        if send_stop and self.lift.lift_connected:
            self.lift.stop()
        self.refresh_feedback_display()

    # =========================================================
    # 位置闭环驱动（点到点，IDLE 手动）
    # =========================================================
    def start_lift_position_debug(self):
        if self.control_mode != "IDLE":
            messagebox.showwarning("运行中", "请先停止控制循环后再使用位置驱动")
            return
        if not self.lift.lift_connected:
            messagebox.showwarning("提示", "请先连接举升电动缸")
            return
        if not self.lift.encoder_connected:
            messagebox.showwarning("提示", "请先连接俯仰角度编码器")
            return
        try:
            target = float(self.lift_position_target_var.get())
            max_vel = float(self.lift_position_max_vel_var.get())
            kp = float(self.lift_position_kp_var.get())
            deadband = float(self.lift_position_deadband_var.get())
        except Exception:
            messagebox.showwarning("输入错误", "位置驱动参数请输入有效数字")
            return
        if max_vel <= 0 or deadband <= 0:
            messagebox.showwarning("参数错误", "最大角速度和到位死区必须为正数")
            return

        # 与角速度调试互斥：同一时间只允许一种举升控制方式
        self.stop_pitch_debug_control(send_stop=False)
        self.lift.configure_lift_position_control(
            max_vel_deg_s=max_vel,
            kp=kp,
            deadband_deg=deadband,
            max_output_deg_s=min(max_vel * 2.0, 4.0),
            follow_error_limit_deg=3.0,
        )
        summary = self.lift.start_lift_position_move(target)
        if summary.get("clamped"):
            messagebox.showwarning(
                "目标已钳制",
                f"目标角超出软限位，已钳制到 {summary['target_deg']:.3f}°",
            )

        self.start_feedback_polling()
        self.lift_position_active = True
        self.status_var.set(
            f"位置驱动已启动：目标 {summary['target_deg']:.3f}°，"
            f"曲线时长约 {summary['profile_total_s']:.1f} s"
        )
        self.lift_position_debug_loop()

    def lift_position_debug_loop(self):
        if not self.lift_position_active:
            return
        if not self.lift.lift_connected or not self.lift.encoder_connected:
            self.stop_lift_position_debug(note="已停止：连接断开")
            self.status_var.set("举升或编码器断开，位置驱动已停止")
            return
        try:
            res = self.lift.update_lift_position_control()
        except Exception as exc:
            self.stop_lift_position_debug(note=f"异常：{exc}")
            self.status_var.set(f"位置驱动异常：{exc}")
            return

        self.refresh_feedback_display()

        if res["state"] == "fault":
            self.status_var.set("位置驱动故障：跟随误差超限，已停止")
            self.stop_lift_position_debug(note="故障：跟随误差超限")
            return
        if res["arrived"]:
            note = f"到位：误差 {res['pos_error_deg']:+.4f}°"
            self.status_var.set(f"位置驱动完成，{note}")
            self.stop_lift_position_debug(note=note)
            return

        self._update_lift_position_display(res)
        self.lift_position_job = self.root.after(max(50, self.feedback_period_ms), self.lift_position_debug_loop)

    def stop_lift_position_debug(self, note: str = "位置环已停止"):
        self.lift_position_active = False
        if self.lift_position_job is not None:
            try:
                self.root.after_cancel(self.lift_position_job)
            except Exception:
                pass
        self.lift_position_job = None
        self.lift.stop_lift_position_move()
        self.lift_position_status_var.set(note)
        self.refresh_feedback_display()

    def _update_lift_position_display(self, res):
        state_cn = {"idle": "空闲", "moving": "运动中", "holding": "到位保持", "fault": "故障"}.get(
            res["state"], res["state"])
        self.lift_position_status_var.set(
            f"{state_cn} 目标={res['target_deg']:.3f}° 当前={res['current_deg']:.3f}° "
            f"误差={res['pos_error_deg']:+.4f}° 目标角速度={res['target_omega_deg_s']:+.3f}°/s"
        )

    # =========================================================
    # CSV 记录
    # =========================================================
    def start_logging(self, default_prefix="master_control"):
        try:
            interval_ms = int(self.log_interval_var.get())
            if interval_ms <= 0:
                messagebox.showwarning("输入错误", "记录周期必须大于 0 ms")
                return False
            self.log_interval_s = interval_ms / 1000.0
        except Exception:
            messagebox.showwarning("输入错误", "记录周期请输入整数，单位 ms")
            return False

        filename = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv")],
            title="保存总控记录",
            initialfile=f"{default_prefix}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
        )
        if not filename:
            return False
        try:
            self.log_file = open(filename, "w", newline="", encoding="utf-8-sig")
            self.log_writer = csv.writer(self.log_file)
            header = [
                "timestamp", "elapsed_s", "mode", "stop_reason",
                "x_angle_deg", "y_angle_deg", "z_angle_deg",
                "az_cmd_rpm", "az_fb_rpm", "az_angle_deg", "az_statusword",
                "lift_cmd_rpm",
                "pitch_angle_deg", "pitch_rate_deg_s",

                # MPC / 目标轨迹记录
                "mpc_elapsed_s",
                "mpc_leveling_x_rate_rad_s",
                "mpc_leveling_y_rate_rad_s",
                "mpc_azimuth_rate_rad_s",
                "mpc_lift_rate_rad_s",
                "mpc_ok",
                "mpc_solver_message",
            ]
            for i in range(3):
                header.extend([
                    f"target_Rx_{i + 1}",
                    f"target_Ry_{i + 1}",
                    f"target_Rz_{i + 1}",
                ])
            for i in range(4):
                header.extend([
                    f"leg{i + 1}_connected", f"leg{i + 1}_cmd_rpm", f"leg{i + 1}_pos_mm",
                ])
                # f"leg{i + 1}_vel_rpm", f"leg{i + 1}_vel_mms", 
            self.log_writer.writerow(header)
            self.is_logging = True
            self.log_start_time = time.time()
            self.last_log_time = 0.0
            self.last_log_flush_time = 0.0
            self.log_status_var.set(f"记录中: {os.path.basename(filename)}")
            self.status_var.set(f"CSV记录已开始: {filename}")
            return True
        except Exception as exc:
            messagebox.showerror("记录失败", f"无法创建CSV文件: {exc}")
            return False

    def stop_logging(self):
        self.is_logging = False
        if self.log_file:
            try:
                self.log_file.close()
            except Exception:
                pass
        self.log_file = None
        self.log_writer = None
        self.log_start_time = None
        self.last_log_time = 0.0
        self.log_status_var.set("未记录")
        self.status_var.set("CSV记录已停止")

    def record_log_snapshot(self, force=False, stop_reason=""):
        if not self.is_logging or self.log_writer is None:
            return
        now = time.time()
        if not force and self.last_log_time != 0.0 and now - self.last_log_time < self.log_interval_s:
            return
        self.last_log_time = now
        elapsed = now - self.log_start_time if self.log_start_time else 0.0
        angles = self.sensor.get_angles()
        z_angle_raw_deg = float(angles.get("z_angle", 0.0))
        z_angle_mpc_deg = z_angle_raw_deg - self.mpc_z_zero_deg

        az = self.azimuth.state
        lf = self.lift.lift_state
        pitch = self.lift.pitch_state
        row = [
            datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
            f"{elapsed:.3f}",
            self.control_mode,
            stop_reason,
            f"{angles['x_angle']:.6f}", f"{angles['y_angle']:.6f}", f"{z_angle_mpc_deg:.6f}",
            f"{az['cmd_rpm']:.6f}", f"{az['fb_rpm']:.6f}", f"{az['multi_angle_deg']:.6f}",
            az["statusword"] if az["statusword"] is not None else "",
            lf["cmd_rpm"],
            f"{pitch['angle_deg']:.6f}", f"{pitch['angle_rate_deg_s']:.6f}",

            f"{getattr(self, 'last_mpc_elapsed_s', 0.0):.3f}",
            f"{self.mpc_command.get('leveling_x_rate_rad_s', 0.0):.9f}",
            f"{self.mpc_command.get('leveling_y_rate_rad_s', 0.0):.9f}",
            f"{self.mpc_command.get('azimuth_rate_rad_s', 0.0):.9f}",
            f"{self.mpc_command.get('lift_rate_rad_s', 0.0):.9f}",
            int(bool(self.last_mpc_solver_ok)),
            self.last_mpc_solver_message,
        ]
        target_sequence = getattr(self, 'last_target_sequence', [0.0] * 9)
        for i in range(9):
            value = target_sequence[i] if i < len(target_sequence) else 0.0
            row.append(f"{float(value):.9f}")
        for i, leg in enumerate(self.leveling.legs):
            row.extend([
                int(self.leveling.connected[i]),
                leg["cmd_rpm"],
                f"{leg['pos']:.6f}",
            ])
        try:
            self.log_writer.writerow(row)

            # 不再每行 flush：
            # 1) force=True 时立即 flush，保证停止前最后一行写入文件；
            # 2) 正常记录时每 1 秒 flush 一次，减少磁盘写入阻塞。
            if self.log_file:
                if force or (now - self.last_log_flush_time >= self.log_flush_interval_s):
                    self.log_file.flush()
                    self.last_log_flush_time = now

        except Exception as exc:
            print(f"[LOG_ERR] {exc}")

    # =========================================================
    # 关闭
    # =========================================================
    def on_closing(self):
        try:
            self.stop_all("app_close")
        except Exception:
            pass
        try:
            self.stop_feedback_polling()
        except Exception:
            pass
        try:
            self.stop_logging()
        except Exception:
            pass
        try:
            self.sensor.disconnect()
        except Exception:
            pass
        for i in range(4):
            try:
                if self.leveling.connected[i]:
                    self.leveling.disconnect_leg(i)
            except Exception:
                pass
        try:
            if self.azimuth.connected:
                self.azimuth.disconnect()
        except Exception:
            pass
        try:
            if self.lift.lift_connected:
                self.lift.disconnect_lift()
        except Exception:
            pass
        try:
            if self.lift.encoder_connected:
                self.lift.disconnect_encoder()
        except Exception:
            pass
        self.root.destroy()

import serial
import serial.tools.list_ports
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import threading
import time
import struct
import csv
from datetime import datetime


class LiftSystemDebugMonitor:
    """
    举升系统调试界面

    结构：
    1. 举升电动缸驱动器端口：
       - 使用“静态调平界面+堆帧处理.py”里的电动缸通讯逻辑
       - 可发送速度 r/min
       - 可读取位置、速度、转矩

    2. 举升角度编码器端口：
       - 协议与方位系统角度编码器相同
       - 读取 0x6064 / F010 当前角度位置
       - 支持软件置零、单圈角度、多圈累计角度
    """

    def __init__(self, root):
        self.root = root
        self.root.title("举升系统调试界面（电动缸驱动器 + 角度编码器）")

        # ===== 窗口大小 =====
        screen_w = self.root.winfo_screenwidth()
        screen_h = self.root.winfo_screenheight()
        win_w = min(1000, int(screen_w * 0.85))
        win_h = min(760, int(screen_h * 0.85))
        self.root.geometry(f"{win_w}x{win_h}")
        self.root.minsize(820, 600)

        # =====================================================
        # 串口对象
        # =====================================================
        self.lift_port = None
        self.lift_connected = False
        self.lift_lock = threading.Lock()

        self.encoder_port = None
        self.encoder_connected = False
        self.encoder_lock = threading.Lock()

        # =====================================================
        # 协议状态
        # =====================================================
        # 电动缸协议已知，直接可用
        self.lift_protocol_ready = True

        # 角度编码器协议与方位系统相同，已启用
        self.encoder_protocol_ready = True

        # =====================================================
        # 举升角度编码器参数
        # 与方位系统角度编码器一致
        # =====================================================
        self.lift_angle_position_ppr = 2147463847

        # 软件置零
        self.lift_angle_zero_count = 0
        self.lift_angle_current_count = None

        # 多圈累计
        self.lift_angle_last_single_count = None
        self.lift_angle_zero_single_count = 0
        self.lift_angle_turn_count = 0
        self.lift_angle_relative_count = 0

        # 角度结果，后续总控/MPC 可以直接提取
        self.lift_single_angle_deg = 0.0
        self.lift_multi_angle_deg = 0.0

        # =====================================================
        # 俯仰轴角速度计算
        # =====================================================
        self.pitch_last_angle_deg = None
        self.pitch_last_time = None
        self.pitch_angular_velocity_deg_s = 0.0

        # 角速度滤波系数，0~1，越大响应越快，越小越平滑
        self.pitch_velocity_filter_alpha = 0.5

        # =====================================================
        # 俯仰轴角速度控制
        # =====================================================
        self.pitch_velocity_control_enabled = False
        self.pitch_control_last_send_time = 0.0
        self.pitch_control_send_period = 0.1      # 控制命令发送周期，单位 s

        # 电缸方向系数：
        # 如果“目标角度变大”时实际角度反而变小，把这里改成 -1
        self.pitch_lift_direction = 1

        # =====================================================
        # 俯仰轴角速度 PID
        # 目标角速度 deg/s - 实际角速度 deg/s -> 电动缸 rpm
        # =====================================================
        self.pitch_omega_kp_var = tk.StringVar(value="50.0")
        self.pitch_omega_ki_var = tk.StringVar(value="0.0")
        self.pitch_omega_kd_var = tk.StringVar(value="0.0")

        self.pitch_omega_integral = 0.0
        self.pitch_omega_last_error = 0.0
        self.pitch_omega_last_time = None

        # 角速度 PID 修正量限幅，单位 r/min
        # 前馈负责主要速度，PID 只负责微调
        self.pitch_pid_correction_max_rpm = 50.0

        # =====================================================
        # 俯仰轴角速度前馈表
        # gain 含义：当前角度附近，1 rpm 电缸速度对应多少 deg/s 俯仰角速度
        # 单位：deg/s/rpm
        # 数据来自开环测试，后续可以继续根据实测修正
        # =====================================================
        self.pitch_gain_table = [
            # 0°~20° 变化较大，细分
            (0.0, 0.00498),
            (2.5, 0.00432),
            (5.0, 0.00383),
            (7.5, 0.00346),
            (10.0, 0.00317),
            (12.5, 0.00294),
            (15.0, 0.00275),
            (17.5, 0.00260),
            (20.0, 0.00243),

            # 20°以后变化放缓
            (25.0, 0.00226),
            (30.0, 0.00214),
            (35.0, 0.00206),
            (40.0, 0.00200),
            (45.0, 0.00197),

            # 重点保证 60°附近
            (50.0, 0.001958),
            (55.0, 0.001960),
            (60.0, 0.001972),
            (65.0, 0.001972),
        ]

        # =====================================================
        # 电动缸机械参数
        # =====================================================
        self.lead_mm = 5.0
        self.reduction_ratio = 5.0
        self.MAX_MOTOR_RPM = 500.0

        # =====================================================
        # 电动缸反馈状态
        # =====================================================
        self.lift_state = {
            "cmd_rpm": 0,
            "pos_mm": 0.0,
            "vel_rpm": 0,
            "vel_mms": 0.0,
            "torque_pct": 0.0,
            "encoder_accum_count": 0,
            "last_single_turn": None,
            "zero_encoder_count": None,
        }

        # =====================================================
        # 轮询状态
        # =====================================================
        self.polling_thread = None
        self.stop_polling = False
        self.is_polling = False
        self.poll_period_ms = 100

        # =====================================================
        # Tk 变量：通信配置
        # =====================================================
        self.lift_port_var = tk.StringVar()
        self.lift_baud_var = tk.StringVar(value="115200")

        self.encoder_port_var = tk.StringVar()
        self.encoder_baud_var = tk.StringVar(value="115200")

        self.poll_period_var = tk.StringVar(value=str(self.poll_period_ms))

        # =====================================================
        # Tk 变量：电动缸控制
        # =====================================================
        self.lift_send_value_var = tk.StringVar(value="30")
        self.lift_last_send_var = tk.StringVar(value="---")

        # =====================================================
        # Tk 变量：电动缸反馈显示
        # =====================================================
        self.lift_status_var = tk.StringVar(value="未连接")
        self.lift_cmd_speed_var = tk.StringVar(value="--- r/min")
        self.lift_pos_var = tk.StringVar(value="--- mm")
        self.lift_vel_var = tk.StringVar(value="--- r/min (--- mm/s)")
        self.lift_torque_var = tk.StringVar(value="--- %")

        # =====================================================
        # Tk 变量：角度编码器显示
        # =====================================================
        self.encoder_status_var = tk.StringVar(value="未连接")
        self.encoder_raw_var = tk.StringVar(value="---")
        self.encoder_single_angle_var = tk.StringVar(value="--- °")
        self.encoder_multi_angle_var = tk.StringVar(value="--- °")
        self.encoder_health_var = tk.StringVar(value="---")

        # =====================================================
        # Tk 变量：俯仰轴角速度和角度控制
        # =====================================================
        self.encoder_angular_velocity_var = tk.StringVar(value="--- °/s")

        # 目标俯仰轴角速度，单位 deg/s
        self.pitch_target_omega_var = tk.StringVar(value="0.0")

        # 角速度指令限幅，单位 deg/s
        # 第一次测试建议不要太大
        self.pitch_target_omega_limit_deg_s = 1

        # =====================================================
        # 俯仰角速度测试数据记录
        # =====================================================
        self.pitch_test_omega_var = tk.StringVar(value="0.5")  # 测试目标角速度 deg/s
        self.pitch_test_running = False
        self.pitch_test_start_time = None
        self.pitch_test_data = []

        # =====================================================
        # 电缸匀速开环测试数据记录
        # =====================================================
        self.lift_speed_test_rpm_var = tk.StringVar(value="30")  # 测试电缸速度 rpm
        self.lift_speed_test_running = False
        self.lift_speed_test_start_time = None
        self.lift_speed_test_data = []

        # =====================================================
        # 状态栏
        # =====================================================
        self.status_var = tk.StringVar(value="就绪")

        self.setup_ui()
        self.refresh_ports()

        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    # =========================================================
    # UI
    # =========================================================
    def setup_ui(self):
        # ===== 底部状态栏先固定在窗口底部 =====
        ttk.Label(
            self.root,
            textvariable=self.status_var,
            relief=tk.SUNKEN,
            anchor=tk.W
        ).pack(side=tk.BOTTOM, fill=tk.X)

        # ===== 中间主体区域增加滚动条 =====
        container = ttk.Frame(self.root)
        container.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        canvas = tk.Canvas(container, highlightthickness=0)
        scrollbar = ttk.Scrollbar(container, orient=tk.VERTICAL, command=canvas.yview)

        self.scrollable_frame = ttk.Frame(canvas, padding="10")

        self.scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )

        canvas_window = canvas.create_window(
            (0, 0),
            window=self.scrollable_frame,
            anchor="nw"
        )

        canvas.configure(yscrollcommand=scrollbar.set)

        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        # 让内部 frame 宽度跟随 canvas，避免横向被截断太多
        def on_canvas_configure(event):
            canvas.itemconfig(canvas_window, width=event.width)

        canvas.bind("<Configure>", on_canvas_configure)

        # 鼠标滚轮支持，Windows 下可用
        def on_mousewheel(event):
            canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

        canvas.bind_all("<MouseWheel>", on_mousewheel)

        # ===== 原来的界面内容放进可滚动区域 =====
        self.setup_comm_ui(self.scrollable_frame)
        self.setup_control_ui(self.scrollable_frame)
        self.setup_data_ui(self.scrollable_frame)

    def setup_comm_ui(self, parent):
        config_frame = ttk.LabelFrame(parent, text="通信配置", padding="10")
        config_frame.pack(fill=tk.X, pady=5)

        baud_values = [
            "9600",
            "19200",
            "38400",
            "57600",
            "115200",
            "230400",
            "460800",
            "921600"
        ]

        # ===== 举升电动缸驱动器端口 =====
        ttk.Label(config_frame, text="举升电动缸端口:").grid(
            row=0, column=0, padx=5, pady=5, sticky=tk.W
        )

        self.lift_port_combo = ttk.Combobox(
            config_frame,
            textvariable=self.lift_port_var,
            width=12
        )
        self.lift_port_combo.grid(row=0, column=1, padx=5, pady=5)

        ttk.Combobox(
            config_frame,
            textvariable=self.lift_baud_var,
            values=baud_values,
            width=10
        ).grid(row=0, column=2, padx=5, pady=5)

        self.lift_connect_btn = ttk.Button(
            config_frame,
            text="连接电动缸",
            command=self.toggle_lift_connection
        )
        self.lift_connect_btn.grid(row=0, column=3, padx=5, pady=5)

        # ===== 角度编码器端口 =====
        ttk.Label(config_frame, text="角度编码器端口:").grid(
            row=1, column=0, padx=5, pady=5, sticky=tk.W
        )

        self.encoder_port_combo = ttk.Combobox(
            config_frame,
            textvariable=self.encoder_port_var,
            width=12
        )
        self.encoder_port_combo.grid(row=1, column=1, padx=5, pady=5)

        ttk.Combobox(
            config_frame,
            textvariable=self.encoder_baud_var,
            values=baud_values,
            width=10
        ).grid(row=1, column=2, padx=5, pady=5)

        self.encoder_connect_btn = ttk.Button(
            config_frame,
            text="连接编码器",
            command=self.toggle_encoder_connection
        )
        self.encoder_connect_btn.grid(row=1, column=3, padx=5, pady=5)

        ttk.Button(
            config_frame,
            text="刷新端口",
            command=self.refresh_ports
        ).grid(row=0, column=4, rowspan=2, padx=12, pady=5, sticky="ns")

    def setup_control_ui(self, parent):
        ctrl_frame = ttk.LabelFrame(parent, text="控制", padding="10")
        ctrl_frame.pack(fill=tk.X, pady=5)

        # ===== 第一行：轮询控制 =====
        top_line = ttk.Frame(ctrl_frame)
        top_line.pack(fill=tk.X, pady=4)

        ttk.Label(top_line, text="轮询周期(ms):").pack(side=tk.LEFT, padx=5)
        ttk.Entry(top_line, textvariable=self.poll_period_var, width=8).pack(side=tk.LEFT, padx=5)

        ttk.Button(top_line, text="开始读取", command=self.start_polling).pack(side=tk.LEFT, padx=8)
        ttk.Button(top_line, text="停止读取", command=self.stop_polling_task).pack(side=tk.LEFT, padx=8)
        ttk.Button(top_line, text="清空显示", command=self.clear_display).pack(side=tk.LEFT, padx=8)

        ttk.Button(
            top_line,
            text="角度置零",
            command=self.reset_lift_angle_zero
        ).pack(side=tk.LEFT, padx=8)

        # ===== 第二行：电动缸输入控制 =====
        lift_ctrl_frame = ttk.LabelFrame(ctrl_frame, text="举升电动缸输入信号", padding="8")
        lift_ctrl_frame.pack(fill=tk.X, pady=6)

        ttk.Label(lift_ctrl_frame, text="目标速度(r/min):").grid(
            row=0, column=0, padx=5, pady=5, sticky=tk.W
        )

        ttk.Entry(
            lift_ctrl_frame,
            textvariable=self.lift_send_value_var,
            width=10
        ).grid(row=0, column=1, padx=5, pady=5)

        ttk.Button(
            lift_ctrl_frame,
            text="上升发送",
            command=lambda: self.send_lift_input_signal(direction=1)
        ).grid(row=0, column=2, padx=8, pady=5)

        ttk.Button(
            lift_ctrl_frame,
            text="下降发送",
            command=lambda: self.send_lift_input_signal(direction=-1)
        ).grid(row=0, column=3, padx=8, pady=5)

        ttk.Button(
            lift_ctrl_frame,
            text="停止电动缸",
            command=self.send_lift_stop_signal
        ).grid(row=0, column=4, padx=8, pady=5)

        ttk.Button(
            lift_ctrl_frame,
            text="当前位置清零",
            command=self.reset_lift_zero
        ).grid(row=0, column=5, padx=8, pady=5)

        ttk.Label(lift_ctrl_frame, text="最近发送:").grid(
            row=1, column=0, padx=5, pady=5, sticky=tk.W
        )
        ttk.Label(
            lift_ctrl_frame,
            textvariable=self.lift_last_send_var,
            font=("Arial", 11, "bold")
        ).grid(row=1, column=1, columnspan=5, padx=5, pady=5, sticky=tk.W)

        # ===== 第三行：俯仰角速度控制 =====
        pitch_ctrl_frame = ttk.LabelFrame(ctrl_frame, text="俯仰角速度控制", padding="8")
        pitch_ctrl_frame.pack(fill=tk.X, pady=6)

        ttk.Label(pitch_ctrl_frame, text="目标角速度(deg/s):").grid(
            row=0, column=0, padx=5, pady=5, sticky=tk.W
        )
        ttk.Entry(
            pitch_ctrl_frame,
            textvariable=self.pitch_target_omega_var,
            width=10
        ).grid(row=0, column=1, padx=5, pady=5)

        ttk.Label(pitch_ctrl_frame, text="速度Kp:").grid(
            row=0, column=2, padx=5, pady=5, sticky=tk.W
        )
        ttk.Entry(
            pitch_ctrl_frame,
            textvariable=self.pitch_omega_kp_var,
            width=8
        ).grid(row=0, column=3, padx=5, pady=5)

        ttk.Label(pitch_ctrl_frame, text="速度Ki:").grid(
            row=0, column=4, padx=5, pady=5, sticky=tk.W
        )
        ttk.Entry(
            pitch_ctrl_frame,
            textvariable=self.pitch_omega_ki_var,
            width=8
        ).grid(row=0, column=5, padx=5, pady=5)

        ttk.Label(pitch_ctrl_frame, text="速度Kd:").grid(
            row=0, column=6, padx=5, pady=5, sticky=tk.W
        )
        ttk.Entry(
            pitch_ctrl_frame,
            textvariable=self.pitch_omega_kd_var,
            width=8
        ).grid(row=0, column=7, padx=5, pady=5)

        ttk.Button(
            pitch_ctrl_frame,
            text="开始角速度控制",
            command=self.start_pitch_velocity_control
        ).grid(row=0, column=8, padx=8, pady=5)

        ttk.Button(
            pitch_ctrl_frame,
            text="停止角速度控制",
            command=self.stop_pitch_velocity_control
        ).grid(row=0, column=9, padx=8, pady=5)

        # ===== 第四行：俯仰角速度测试与数据记录 =====
        pitch_test_frame = ttk.LabelFrame(ctrl_frame, text="俯仰角速度测试与数据记录", padding="8")
        pitch_test_frame.pack(fill=tk.X, pady=6)

        ttk.Label(pitch_test_frame, text="测试角速度(deg/s):").grid(
            row=0, column=0, padx=5, pady=5, sticky=tk.W
        )

        ttk.Entry(
            pitch_test_frame,
            textvariable=self.pitch_test_omega_var,
            width=10
        ).grid(row=0, column=1, padx=5, pady=5)

        ttk.Button(
            pitch_test_frame,
            text="开始测试",
            command=self.start_pitch_velocity_test
        ).grid(row=0, column=2, padx=8, pady=5)

        ttk.Button(
            pitch_test_frame,
            text="停止测试并保存",
            command=self.stop_pitch_velocity_test
        ).grid(row=0, column=3, padx=8, pady=5)

        # ===== 第五行：电缸匀速开环测试与数据记录 =====
        lift_speed_test_frame = ttk.LabelFrame(ctrl_frame, text="电缸匀速开环测试与数据记录", padding="8")
        lift_speed_test_frame.pack(fill=tk.X, pady=6)

        ttk.Label(lift_speed_test_frame, text="电缸速度(r/min):").grid(
            row=0, column=0, padx=5, pady=5, sticky=tk.W
        )

        ttk.Entry(
            lift_speed_test_frame,
            textvariable=self.lift_speed_test_rpm_var,
            width=10
        ).grid(row=0, column=1, padx=5, pady=5)

        ttk.Button(
            lift_speed_test_frame,
            text="开始电缸匀速测试",
            command=self.start_lift_constant_speed_test
        ).grid(row=0, column=2, padx=8, pady=5)

        ttk.Button(
            lift_speed_test_frame,
            text="停止测试并保存",
            command=self.stop_lift_constant_speed_test
        ).grid(row=0, column=3, padx=8, pady=5)

    def setup_data_ui(self, parent):
        data_frame = ttk.Frame(parent)
        data_frame.pack(fill=tk.BOTH, expand=True, pady=5)

        # ===== 左侧：电动缸反馈 =====
        lift_frame = ttk.LabelFrame(data_frame, text="举升电动缸反馈", padding="10")
        lift_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))

        lift_items = [
            ("状态", self.lift_status_var),
            ("指令速度", self.lift_cmd_speed_var),
            ("位置", self.lift_pos_var),
            ("反馈速度", self.lift_vel_var),
            ("转矩", self.lift_torque_var),
        ]

        for i, (label, var) in enumerate(lift_items):
            ttk.Label(lift_frame, text=f"{label}:").grid(
                row=i, column=0, padx=6, pady=8, sticky=tk.W
            )
            ttk.Label(lift_frame, textvariable=var, font=("Arial", 12, "bold")).grid(
                row=i, column=1, padx=6, pady=8, sticky=tk.W
            )

        # ===== 右侧：角度编码器反馈 =====
        encoder_frame = ttk.LabelFrame(data_frame, text="角度编码器反馈", padding="10")
        encoder_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(5, 0))

        encoder_items = [
            ("状态", self.encoder_status_var),
            ("原始值", self.encoder_raw_var),
            ("单圈角度", self.encoder_single_angle_var),
            ("多圈角度", self.encoder_multi_angle_var),
            ("角速度", self.encoder_angular_velocity_var),
            ("状态字", self.encoder_health_var),
        ]

        for i, (label, var) in enumerate(encoder_items):
            ttk.Label(encoder_frame, text=f"{label}:").grid(
                row=i, column=0, padx=6, pady=8, sticky=tk.W
            )
            ttk.Label(encoder_frame, textvariable=var, font=("Arial", 12, "bold")).grid(
                row=i, column=1, padx=6, pady=8, sticky=tk.W
            )

    # =========================================================
    # 串口连接
    # =========================================================
    def refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]

        self.lift_port_combo["values"] = ports
        self.encoder_port_combo["values"] = ports

        if ports:
            if not self.lift_port_var.get() or self.lift_port_var.get() not in ports:
                self.lift_port_var.set(ports[0])

            if not self.encoder_port_var.get() or self.encoder_port_var.get() not in ports:
                if len(ports) >= 2:
                    self.encoder_port_var.set(ports[1])
                else:
                    self.encoder_port_var.set(ports[0])
        else:
            self.lift_port_var.set("")
            self.encoder_port_var.set("")

        self.status_var.set("端口已刷新")

    def toggle_lift_connection(self):
        if not self.lift_connected:
            try:
                port_name = self.lift_port_var.get()
                baud = int(self.lift_baud_var.get())

                if not port_name:
                    messagebox.showwarning("提示", "请选择举升电动缸端口")
                    return

                self.lift_port = serial.Serial(port_name, baud, timeout=0.05)
                self.lift_port.reset_input_buffer()
                self.lift_port.reset_output_buffer()

                self.lift_connected = True
                self.lift_connect_btn.config(text="断开电动缸")
                self.lift_status_var.set("已连接")
                self.status_var.set(f"举升电动缸已连接: {port_name}")

                self.reset_lift_feedback_cache()

            except Exception as e:
                try:
                    if self.lift_port:
                        self.lift_port.close()
                except Exception:
                    pass

                self.lift_port = None
                self.lift_connected = False
                self.lift_connect_btn.config(text="连接电动缸")
                self.lift_status_var.set("未连接")
                messagebox.showerror("连接失败", f"举升电动缸连接失败: {e}")

        else:
            self.disconnect_lift()

    def toggle_encoder_connection(self):
        if not self.encoder_connected:
            try:
                port_name = self.encoder_port_var.get()
                baud = int(self.encoder_baud_var.get())

                if not port_name:
                    messagebox.showwarning("提示", "请选择角度编码器端口")
                    return

                self.encoder_port = serial.Serial(port_name, baud, timeout=0.05)
                self.encoder_port.reset_input_buffer()
                self.encoder_port.reset_output_buffer()

                self.encoder_connected = True
                self.encoder_connect_btn.config(text="断开编码器")
                self.encoder_status_var.set("已连接" if self.encoder_protocol_ready else "已连接-协议待补")

                # 重新连接编码器后，重置角度累计状态
                self.lift_angle_zero_count = 0
                self.lift_angle_current_count = None
                self.lift_angle_last_single_count = None
                self.lift_angle_zero_single_count = 0
                self.lift_angle_turn_count = 0
                self.lift_angle_relative_count = 0
                self.lift_single_angle_deg = 0.0
                self.lift_multi_angle_deg = 0.0

                # 重新连接编码器后，重置角速度计算状态
                self.pitch_last_angle_deg = None
                self.pitch_last_time = None
                self.pitch_angular_velocity_deg_s = 0.0
                self.encoder_angular_velocity_var.set("--- °/s")

                self.status_var.set(f"角度编码器已连接: {port_name}")

            except Exception as e:
                try:
                    if self.encoder_port:
                        self.encoder_port.close()
                except Exception:
                    pass

                self.encoder_port = None
                self.encoder_connected = False
                self.encoder_connect_btn.config(text="连接编码器")
                self.encoder_status_var.set("未连接")
                messagebox.showerror("连接失败", f"角度编码器连接失败: {e}")

        else:
            self.disconnect_encoder()

    def disconnect_lift(self):
        self.lift_speed_test_running = False
        self.lift_speed_test_start_time = None

        self.pitch_test_running = False
        self.pitch_test_start_time = None

        self.pitch_velocity_control_enabled = False
        self.pitch_omega_integral = 0.0
        self.pitch_omega_last_error = 0.0
        self.pitch_omega_last_time = None
        # 断开前先停止电动缸输出
        try:
            self.send_lift_cmd(0)
        except Exception:
            pass

        try:
            with self.lift_lock:
                if self.lift_port:
                    self.lift_port.close()
                    self.lift_port = None
        except Exception:
            pass

        self.lift_connected = False
        self.lift_connect_btn.config(text="连接电动缸")
        self.lift_status_var.set("未连接")
        self.lift_cmd_speed_var.set("--- r/min")
        self.lift_pos_var.set("--- mm")
        self.lift_vel_var.set("--- r/min (--- mm/s)")
        self.lift_torque_var.set("--- %")

        self.status_var.set("举升电动缸已断开")

    def disconnect_encoder(self):
        self.lift_speed_test_running = False
        self.lift_speed_test_start_time = None

        self.pitch_test_running = False
        self.pitch_test_start_time = None

        self.pitch_velocity_control_enabled = False
        self.pitch_omega_integral = 0.0
        self.pitch_omega_last_error = 0.0
        self.pitch_omega_last_time = None
        try:
            with self.encoder_lock:
                if self.encoder_port:
                    self.encoder_port.close()
                    self.encoder_port = None
        except Exception:
            pass

        self.encoder_connected = False
        self.encoder_connect_btn.config(text="连接编码器")
        self.encoder_status_var.set("未连接")
        self.encoder_raw_var.set("---")
        self.encoder_single_angle_var.set("--- °")
        self.encoder_multi_angle_var.set("--- °")
        self.encoder_angular_velocity_var.set("--- °/s")
        self.encoder_health_var.set("---")

        self.lift_angle_zero_count = 0
        self.lift_angle_current_count = None
        self.lift_angle_last_single_count = None
        self.lift_angle_zero_single_count = 0
        self.lift_angle_turn_count = 0
        self.lift_angle_relative_count = 0
        self.lift_single_angle_deg = 0.0
        self.lift_multi_angle_deg = 0.0

        self.pitch_last_angle_deg = None
        self.pitch_last_time = None
        self.pitch_angular_velocity_deg_s = 0.0

        self.status_var.set("角度编码器已断开")

    # =========================================================
    # 电动缸 Modbus RTU 通讯
    # =========================================================
    def calculate_crc16(self, data):
        """标准 Modbus CRC16 计算"""
        crc = 0xFFFF
        for pos in data:
            crc ^= pos
            for _ in range(8):
                if (crc & 1) != 0:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return struct.pack("<H", crc)
    
    def bytes_to_int16(self, raw, signed=True):
        """
        16 位数据正常大端解析。
        """
        if raw is None or len(raw) != 2:
            return None
        return int.from_bytes(raw, byteorder="big", signed=signed)


    def bytes_to_int32_normal(self, raw, signed=True):
        """
        32 位数据正常顺序解析：Aa Bb Cc Dd。
        举升角度编码器与方位系统相同，不交换高低位。
        """
        if raw is None or len(raw) != 4:
            return None
        return int.from_bytes(raw, byteorder="big", signed=signed)

    def send_lift_cmd(self, rpm_val):
        """
        下发举升电动缸转速命令。
        rpm_val 的物理含义：目标电机转速 r/min。
        默认从站地址为 0x01。
        """
        if not self.lift_connected or not self.lift_port:
            return False

        try:
            phys_speed = int(round(rpm_val))
            phys_speed = max(-int(self.MAX_MOTOR_RPM), min(int(self.MAX_MOTOR_RPM), phys_speed))

            # 写 0x3308，逻辑沿用静态调平界面里的电动缸发送协议
            payload = bytearray([0x01, 0x10, 0x33, 0x08, 0x00, 0x02, 0x08])

            # 速度按 16 位有符号整数、大端格式写入
            payload.extend(struct.unpack("BB", struct.pack(">h", phys_speed)))

            # 按原驱动协议补后面 6 个字节
            if phys_speed < 0:
                payload.extend(b"\xFF\xFF\xFF\xFF\xFF\xFF")
            else:
                payload.extend(b"\x00\x00\x00\x00\x00\x00")

            full_packet = payload + self.calculate_crc16(payload)

            with self.lift_lock:
                self.lift_port.write(full_packet)

            self.lift_state["cmd_rpm"] = phys_speed
            return True

        except Exception as e:
            print(f"[LIFT_SEND_ERR] {e}")
            return False

    def read_lift_holding_registers(self, start_addr, reg_count):
        """读取举升电动缸驱动器保持寄存器，默认从站地址 0x01"""
        if not self.lift_connected or not self.lift_port:
            return None

        try:
            slave_id = 0x01

            packet = bytearray([
                slave_id,
                0x03,
                (start_addr >> 8) & 0xFF,
                start_addr & 0xFF,
                (reg_count >> 8) & 0xFF,
                reg_count & 0xFF
            ])
            full_packet = packet + self.calculate_crc16(packet)

            with self.lift_lock:
                self.lift_port.reset_input_buffer()
                self.lift_port.write(full_packet)

                expected_len = 3 + reg_count * 2 + 2
                response = self.lift_port.read(expected_len)

            if len(response) != expected_len:
                return None

            if response[0] != slave_id or response[1] != 0x03 or response[2] != reg_count * 2:
                return None

            data_without_crc = response[:-2]
            recv_crc = response[-2:]
            calc_crc = self.calculate_crc16(data_without_crc)

            if recv_crc != calc_crc:
                return None

            return response[3:-2]

        except Exception as e:
            print(f"[LIFT_READ_ERR] {e}")
            return None
        
    def read_encoder_holding_registers(self, start_addr, reg_count):
        """
        读取角度编码器保持寄存器。
        协议与方位系统相同，使用 03 功能码。
        """
        if not self.encoder_connected or not self.encoder_port:
            return None

        try:
            slave_id = 0x01

            packet = bytearray([
                slave_id,
                0x03,
                (start_addr >> 8) & 0xFF,
                start_addr & 0xFF,
                (reg_count >> 8) & 0xFF,
                reg_count & 0xFF
            ])

            full_packet = packet + self.calculate_crc16(packet)

            with self.encoder_lock:
                self.encoder_port.reset_input_buffer()
                self.encoder_port.write(full_packet)
                self.encoder_port.flush()

                expected_len = 3 + reg_count * 2 + 2
                response = self.encoder_port.read(expected_len)

            if len(response) != expected_len:
                print(f"[ENCODER_READ_LEN_ERR] addr=0x{start_addr:04X}, response={response.hex(' ')}")
                return None

            if response[0] != slave_id or response[1] != 0x03:
                print(f"[ENCODER_READ_HEAD_ERR] addr=0x{start_addr:04X}, response={response.hex(' ')}")
                return None

            if response[2] != reg_count * 2:
                print(f"[ENCODER_READ_BYTECOUNT_ERR] addr=0x{start_addr:04X}, response={response.hex(' ')}")
                return None

            data_without_crc = response[:-2]
            recv_crc = response[-2:]
            calc_crc = self.calculate_crc16(data_without_crc)

            if recv_crc != calc_crc:
                print(f"[ENCODER_READ_CRC_ERR] addr=0x{start_addr:04X}, response={response.hex(' ')}")
                return None

            return response[3:-2]

        except Exception as e:
            print(f"[ENCODER_READ_ERR] addr=0x{start_addr:04X}, {e}")
            return None

    def read_lift_encoder_single_turn(self):
        raw = self.read_lift_holding_registers(0x4202, 2)
        if raw is None or len(raw) != 4:
            return None

        # 沿用静态调平界面里的实测取法：L=raw[0], M=raw[1], H=raw[3]
        byte_l = raw[0]
        byte_m = raw[1]
        byte_h = raw[3]

        value = (byte_h << 16) | (byte_m << 8) | byte_l
        return value & 0x7FFFFF

    def update_lift_encoder_accum(self):
        single_turn = self.read_lift_encoder_single_turn()
        if single_turn is None:
            return None

        last_single = self.lift_state["last_single_turn"]

        if last_single is None:
            self.lift_state["last_single_turn"] = single_turn
            self.lift_state["encoder_accum_count"] = 0
            return 0

        delta = single_turn - last_single
        one_turn = 1 << 23
        half_turn = one_turn // 2

        if delta > half_turn:
            delta -= one_turn
        elif delta < -half_turn:
            delta += one_turn

        self.lift_state["encoder_accum_count"] += delta
        self.lift_state["last_single_turn"] = single_turn

        return self.lift_state["encoder_accum_count"]

    def count_to_mm(self, count_value):
        mm_per_motor_rev = self.lead_mm / self.reduction_ratio
        mm_per_count = mm_per_motor_rev / (1 << 23)
        return count_value * mm_per_count

    def rpm_to_mms(self, rpm_value):
        mm_per_motor_rev = self.lead_mm / self.reduction_ratio
        return rpm_value * mm_per_motor_rev / 60.0
    
    def lift_angle_count_to_deg(self, count_value):
        """
        举升角度编码器 count -> 多圈角度 deg。
        """
        return count_value * 360.0 / self.lift_angle_position_ppr


    def lift_angle_count_to_single_deg(self, count_value):
        """
        举升角度编码器 count -> 单圈角度 0~360 deg。
        """
        one_turn_count = self.lift_angle_position_ppr
        single_count = count_value % one_turn_count
        return single_count * 360.0 / one_turn_count

    def read_lift_velocity_rpm(self):
        raw = self.read_lift_holding_registers(0x4025, 1)
        if raw is None or len(raw) != 2:
            return None
        return struct.unpack(">h", raw)[0]

    def read_lift_torque_pct(self):
        raw = self.read_lift_holding_registers(0x6025, 1)
        if raw is None or len(raw) != 2:
            return None
        return struct.unpack(">h", raw)[0] / 10.0

    # =========================================================
    # 电动缸手动控制
    # =========================================================
    def send_lift_input_signal(self, direction):
        if not self.lift_connected or not self.lift_port:
            messagebox.showwarning("提示", "请先连接举升电动缸驱动器")
            return
        
        if self.lift_speed_test_running:
            messagebox.showwarning("提示", "当前正在电缸匀速开环测试，请先点击“停止测试并保存”")
            return
        
        if self.pitch_test_running:
            messagebox.showwarning("提示", "当前正在俯仰角速度测试，请先点击“停止测试并保存”")
            return

        try:
            value = int(float(self.lift_send_value_var.get()))
        except Exception:
            messagebox.showwarning("输入错误", "目标速度请输入有效数值，单位 r/min")
            return

        cmd_rpm = max(0, min(int(self.MAX_MOTOR_RPM), abs(value)))
        cmd_rpm = cmd_rpm if direction > 0 else -cmd_rpm

        # 手动发送速度时，退出俯仰角度控制，并清空速度 PID 状态
        self.pitch_velocity_control_enabled = False
        self.pitch_omega_integral = 0.0
        self.pitch_omega_last_error = 0.0
        self.pitch_omega_last_time = None

        ok = self.send_lift_cmd(cmd_rpm)

        if ok:
            self.lift_last_send_var.set(f"{cmd_rpm} r/min")
            self.lift_cmd_speed_var.set(f"{cmd_rpm} r/min")
            self.status_var.set(f"已发送举升电动缸速度命令: {cmd_rpm} r/min")
        else:
            self.status_var.set("举升电动缸命令发送失败")

    def send_lift_stop_signal(self):
        if not self.lift_connected or not self.lift_port:
            messagebox.showwarning("提示", "请先连接举升电动缸驱动器")
            return

        self.lift_speed_test_running = False
        self.lift_speed_test_start_time = None

        self.pitch_test_running = False
        self.pitch_test_start_time = None

        self.pitch_velocity_control_enabled = False
        self.pitch_omega_integral = 0.0
        self.pitch_omega_last_error = 0.0
        self.pitch_omega_last_time = None

        ok = self.send_lift_cmd(0)

        if ok:
            self.lift_last_send_var.set("停止命令")
            self.lift_cmd_speed_var.set("0 r/min")
            self.status_var.set("已发送举升电动缸停止命令")
        else:
            self.status_var.set("举升电动缸停止命令发送失败")

    def reset_lift_feedback_cache(self):
        self.lift_state["cmd_rpm"] = 0
        self.lift_state["pos_mm"] = 0.0
        self.lift_state["vel_rpm"] = 0
        self.lift_state["vel_mms"] = 0.0
        self.lift_state["torque_pct"] = 0.0
        self.lift_state["encoder_accum_count"] = 0
        self.lift_state["last_single_turn"] = None
        self.lift_state["zero_encoder_count"] = None

    def reset_lift_zero(self):
        if not self.lift_connected or not self.lift_port:
            messagebox.showwarning("提示", "请先连接举升电动缸驱动器")
            return

        accum = self.update_lift_encoder_accum()
        if accum is not None:
            self.lift_state["zero_encoder_count"] = accum
            self.lift_state["pos_mm"] = 0.0
            self.lift_pos_var.set("0.000 mm")
            self.status_var.set("举升电动缸当前位置已设为零点")
        else:
            self.status_var.set("当前位置清零失败：未读到有效编码器反馈")

    # =========================================================
    # 反馈读取
    # =========================================================
    def update_lift_feedback(self):
        if not self.lift_connected or not self.lift_port:
            return

        accum_count = self.update_lift_encoder_accum()
        if accum_count is not None:
            if self.lift_state["zero_encoder_count"] is None:
                self.lift_state["zero_encoder_count"] = accum_count

            rel_count = accum_count - self.lift_state["zero_encoder_count"]
            self.lift_state["pos_mm"] = self.count_to_mm(rel_count)
            self.lift_pos_var.set(f"{self.lift_state['pos_mm']:.3f} mm")

        vel_rpm = self.read_lift_velocity_rpm()
        if vel_rpm is not None:
            self.lift_state["vel_rpm"] = vel_rpm
            self.lift_state["vel_mms"] = self.rpm_to_mms(vel_rpm)
            self.lift_vel_var.set(
                f"{vel_rpm} r/min ({self.lift_state['vel_mms']:.3f} mm/s)"
            )

        torque_pct = self.read_lift_torque_pct()
        if torque_pct is not None:
            self.lift_state["torque_pct"] = torque_pct
            self.lift_torque_var.set(f"{torque_pct:.1f} %")

        self.lift_status_var.set("已连接")
        self.lift_cmd_speed_var.set(f"{self.lift_state['cmd_rpm']} r/min")

    def read_angle_encoder_data(self):
        """
        角度编码器读取。
        协议与方位系统相同：
        1. 读取 0x6064 / F010 当前角度位置；
        2. 正常 int32 解析，不交换高低位；
        3. 软件置零；
        4. 单圈角度与多圈角度累计。
        """
        if not self.encoder_connected or not self.encoder_port:
            return None

        # 方位系统相同地址：0x6064 对应 RTU 地址 F010
        ADDR_ACTUAL_POS = 0xF010

        raw_actual_pos = self.read_encoder_holding_registers(ADDR_ACTUAL_POS, 2)
        actual_pos_count = self.bytes_to_int32_normal(raw_actual_pos, signed=True)

        if actual_pos_count is None:
            return {
                "status": "读取失败",
                "raw": "---",
                "single_angle": "--- °",
                "multi_angle": "--- °",
                "angular_velocity": "--- °/s",
                "health": "读取失败",
            }

        self.lift_angle_current_count = actual_pos_count

        one_turn_count = self.lift_angle_position_ppr
        current_single_count = actual_pos_count % one_turn_count

        # 用 1/4 圈和 3/4 圈判断跨圈
        low_threshold = one_turn_count * 0.25
        high_threshold = one_turn_count * 0.75

        if self.lift_angle_last_single_count is None:
            # 第一次读取只记录单圈位置，不判断跨圈
            self.lift_angle_last_single_count = current_single_count
        else:
            # 正向跨圈：例如 359° -> 0°
            if self.lift_angle_last_single_count > high_threshold and current_single_count < low_threshold:
                self.lift_angle_turn_count += 1

            # 反向跨圈：例如 0° -> 359°
            elif self.lift_angle_last_single_count < low_threshold and current_single_count > high_threshold:
                self.lift_angle_turn_count -= 1

            self.lift_angle_last_single_count = current_single_count

        # 当前单圈相对置零点的位置
        relative_single_count = current_single_count - self.lift_angle_zero_single_count

        # 多圈累计位置 = 圈数累计 + 当前单圈相对零点
        relative_pos_count = self.lift_angle_turn_count * one_turn_count + relative_single_count

        # 保存给后续总控/MPC 运算
        self.lift_angle_relative_count = relative_pos_count
        self.lift_multi_angle_deg = self.lift_angle_count_to_deg(relative_pos_count)
        self.lift_single_angle_deg = self.lift_angle_count_to_single_deg(relative_pos_count)

        # 根据多圈角度差分计算俯仰轴角速度
        self.update_pitch_angular_velocity(self.lift_multi_angle_deg)

        # 俯仰角速度测试数据记录
        self.record_pitch_velocity_test_data(actual_pos_count)

        # 电缸匀速开环测试数据记录
        self.record_lift_constant_speed_test_data(actual_pos_count)

        raw_pos_text = f"{actual_pos_count} P"
        single_angle_text = f"{self.lift_single_angle_deg:.3f} °"
        multi_angle_text = f"{self.lift_multi_angle_deg:.3f} °"
        angular_velocity_text = f"{self.pitch_angular_velocity_deg_s:.3f} °/s"

        return {
            "status": "已连接",
            "raw": raw_pos_text,
            "single_angle": single_angle_text,
            "multi_angle": multi_angle_text,
            "angular_velocity": angular_velocity_text,
            "health": "正常",
        }

    def update_encoder_display(self, data):
        if data is None:
            return

        self.encoder_status_var.set(str(data.get("status", "---")))
        self.encoder_raw_var.set(str(data.get("raw", "---")))
        self.encoder_single_angle_var.set(str(data.get("single_angle", "---")))
        self.encoder_multi_angle_var.set(str(data.get("multi_angle", "---")))
        self.encoder_angular_velocity_var.set(str(data.get("angular_velocity", "--- °/s")))
        self.encoder_health_var.set(str(data.get("health", "---")))

    def reset_lift_angle_zero(self):
        """
        举升角度编码器软件置零。
        只影响界面显示和后续运算，不写入编码器/驱动器参数。
        """
        if not self.encoder_connected or not self.encoder_port:
            messagebox.showwarning("提示", "请先连接角度编码器")
            return

        if self.lift_angle_current_count is None:
            messagebox.showwarning("提示", "当前还没有读取到角度编码器位置，无法置零")
            return

        one_turn_count = self.lift_angle_position_ppr

        # 当前原始位置作为新的软件零点
        self.lift_angle_zero_count = self.lift_angle_current_count
        self.lift_angle_zero_single_count = self.lift_angle_current_count % one_turn_count

        # 置零后，从当前位置重新开始累计圈数
        self.lift_angle_last_single_count = self.lift_angle_zero_single_count
        self.lift_angle_turn_count = 0
        self.lift_angle_relative_count = 0
        self.lift_single_angle_deg = 0.0
        self.lift_multi_angle_deg = 0.0

        # 角度置零后，重置角速度计算缓存，避免角速度瞬间跳变
        self.pitch_last_angle_deg = None
        self.pitch_last_time = None
        self.pitch_angular_velocity_deg_s = 0.0
        self.encoder_angular_velocity_var.set("0.000 °/s")

        # 立即刷新界面
        self.encoder_single_angle_var.set("0.000 °")
        self.encoder_multi_angle_var.set("0.000 °")

        self.status_var.set(f"举升角度已置零，零点原始值: {self.lift_angle_zero_count}")

    def update_pitch_angular_velocity(self, current_angle_deg):
        """
        根据俯仰轴多圈角度计算角速度。
        current_angle_deg: 当前俯仰角度，单位 deg。
        """
        now = time.time()

        if self.pitch_last_angle_deg is None or self.pitch_last_time is None:
            self.pitch_last_angle_deg = current_angle_deg
            self.pitch_last_time = now
            self.pitch_angular_velocity_deg_s = 0.0
            return self.pitch_angular_velocity_deg_s

        dt = now - self.pitch_last_time
        if dt <= 0:
            return self.pitch_angular_velocity_deg_s

        raw_omega = (current_angle_deg - self.pitch_last_angle_deg) / dt

        alpha = self.pitch_velocity_filter_alpha
        self.pitch_angular_velocity_deg_s = (
            alpha * raw_omega + (1.0 - alpha) * self.pitch_angular_velocity_deg_s
        )

        self.pitch_last_angle_deg = current_angle_deg
        self.pitch_last_time = now

        return self.pitch_angular_velocity_deg_s
    
    def get_pitch_gain(self, angle_deg):
        """
        根据当前俯仰角查表，得到当前角度附近的机构增益。

        返回值：
            gain，单位 deg/s/rpm
            表示 1 rpm 电缸速度大约对应多少 deg/s 俯仰角速度。
        """
        table = sorted(self.pitch_gain_table, key=lambda x: x[0])

        if not table:
            return 0.0

        # 先按绝对角度查表
        # 如果后续发现负角度区间和正角度区间差异明显，再改成正负两张表
        angle = abs(angle_deg)

        if angle <= table[0][0]:
            return table[0][1]

        if angle >= table[-1][0]:
            return table[-1][1]

        for i in range(len(table) - 1):
            angle_0, gain_0 = table[i]
            angle_1, gain_1 = table[i + 1]

            if angle_0 <= angle <= angle_1:
                ratio = (angle - angle_0) / (angle_1 - angle_0)
                return gain_0 + ratio * (gain_1 - gain_0)

        return table[-1][1]

    def pitch_velocity_pid_to_lift_rpm(self, target_omega_deg_s):
        """
        俯仰轴角速度控制：
        角度相关前馈 + PID 修正。

        前馈：
            根据当前俯仰角的机构 gain，直接计算基础电缸 rpm。

        PID：
            根据目标角速度和实际角速度误差，只做小范围修正。
        """
        # 目标角速度接近 0 时，直接停止并清空 PID 状态
        if abs(target_omega_deg_s) < 1e-6:
            self.pitch_omega_integral = 0.0
            self.pitch_omega_last_error = 0.0
            self.pitch_omega_last_time = None
            return 0.0

        now = time.time()

        first_run = self.pitch_omega_last_time is None

        if first_run:
            dt = self.pitch_control_send_period
        else:
            dt = now - self.pitch_omega_last_time

        if dt <= 0:
            dt = self.pitch_control_send_period

        try:
            kp = float(self.pitch_omega_kp_var.get())
            ki = float(self.pitch_omega_ki_var.get())
            kd = float(self.pitch_omega_kd_var.get())
        except Exception:
            kp = 0.0
            ki = 0.0
            kd = 0.0

        actual_omega = self.pitch_angular_velocity_deg_s
        error = target_omega_deg_s - actual_omega

        # =====================================================
        # 1. 角度相关前馈
        # =====================================================
        current_angle = self.lift_multi_angle_deg
        gain = self.get_pitch_gain(current_angle)

        if gain <= 0:
            ff_rpm = 0.0
        else:
            # gain = deg/s/rpm
            # rpm = deg/s / gain
            ff_rpm = target_omega_deg_s / gain

        # =====================================================
        # 2. PID 修正
        # =====================================================
        self.pitch_omega_integral += error * dt
        self.pitch_omega_integral = max(-100.0, min(100.0, self.pitch_omega_integral))

        if first_run:
            derivative = 0.0
        else:
            derivative = (error - self.pitch_omega_last_error) / dt

        pid_rpm = (
            kp * error
            + ki * self.pitch_omega_integral
            + kd * derivative
        )

        # PID 只做修正，不让它抢前馈的主作用
        pid_rpm = max(
            -self.pitch_pid_correction_max_rpm,
            min(self.pitch_pid_correction_max_rpm, pid_rpm)
        )

        # =====================================================
        # 3. 前馈 + PID
        # =====================================================
        motor_rpm = ff_rpm + pid_rpm

        # 方向修正
        motor_rpm *= self.pitch_lift_direction

        # 总输出限幅
        motor_rpm = max(-self.MAX_MOTOR_RPM, min(self.MAX_MOTOR_RPM, motor_rpm))

        self.pitch_omega_last_error = error
        self.pitch_omega_last_time = now

        return motor_rpm

    def pitch_angular_velocity_to_lift_rpm(self, omega_deg_s):
        """
        把目标俯仰角速度 deg/s 转换成电动缸速度 r/min。
    
        这里不再用机构查表或固定 mm/deg，
        而是用角速度 PID 自动根据反馈调整电动缸速度。
        """
        return self.pitch_velocity_pid_to_lift_rpm(omega_deg_s)


    def send_pitch_angular_velocity(self, omega_deg_s):
        """
        俯仰轴角速度控制接口。
        输入目标角速度 deg/s，内部转换成电缸电机 r/min 并下发。
        """
        if not self.lift_connected or not self.lift_port:
            return False

        motor_rpm = self.pitch_angular_velocity_to_lift_rpm(omega_deg_s)
        return self.send_lift_cmd(motor_rpm)
    
    def start_lift_constant_speed_test(self):
        """
        开始电缸匀速开环测试：
        1. 不使用角速度 PID；
        2. 直接给电缸固定 rpm；
        3. 记录对应的俯仰角度和俯仰角速度。
        """
        if self.lift_speed_test_running:
            messagebox.showwarning("提示", "当前已经在电缸匀速测试中，请先停止测试")
            return

        if self.pitch_test_running:
            messagebox.showwarning("提示", "当前正在俯仰角速度测试，请先停止测试")
            return

        if self.pitch_velocity_control_enabled:
            messagebox.showwarning("提示", "当前正在俯仰角速度控制，请先停止角速度控制")
            return

        if not self.lift_connected or not self.lift_port:
            messagebox.showwarning("提示", "请先连接举升电动缸驱动器")
            return

        if not self.encoder_connected or not self.encoder_port:
            messagebox.showwarning("提示", "请先连接角度编码器")
            return

        if not self.is_polling:
            messagebox.showwarning("提示", "请先点击“开始读取”，否则无法记录测试数据")
            return

        if self.lift_angle_current_count is None:
            messagebox.showwarning("提示", "当前还没有读取到俯仰角度")
            return

        try:
            test_rpm = float(self.lift_speed_test_rpm_var.get())
        except Exception:
            messagebox.showwarning("输入错误", "电缸速度请输入有效数字，例如 30 或 -30")
            return

        if test_rpm == 0:
            messagebox.showwarning("输入错误", "测试电缸速度不能为 0")
            return

        if abs(test_rpm) > self.MAX_MOTOR_RPM:
            messagebox.showwarning(
                "速度超限",
                f"测试电缸速度 {test_rpm:.3f} r/min 超过限幅 {self.MAX_MOTOR_RPM:.3f} r/min"
            )
            return

        self.lift_speed_test_data = []
        self.lift_speed_test_start_time = None
        self.lift_speed_test_running = False

        ok = self.send_lift_cmd(test_rpm)

        if ok:
            self.lift_speed_test_start_time = time.time()
            self.lift_speed_test_running = True

            self.lift_last_send_var.set(f"电缸匀速测试: {test_rpm:.3f} r/min")
            self.lift_cmd_speed_var.set(f"{test_rpm:.3f} r/min")
            self.status_var.set(f"开始电缸匀速开环测试：{test_rpm:.3f} r/min")
        else:
            self.status_var.set("电缸匀速开环测试启动失败")


    def stop_lift_constant_speed_test(self):
        """
        停止电缸匀速开环测试：
        1. 停止记录；
        2. 停止电缸；
        3. 保存 CSV。
        """
        was_running = self.lift_speed_test_running
        self.lift_speed_test_running = False

        if self.lift_connected and self.lift_port:
            self.send_lift_cmd(0)

        if not was_running and not self.lift_speed_test_data:
            self.status_var.set("当前没有电缸匀速测试数据需要保存")
            self.lift_speed_test_start_time = None
            return

        if not self.lift_speed_test_data:
            self.status_var.set("电缸匀速测试已停止，但没有记录到有效数据")
            self.lift_speed_test_start_time = None
            return

        default_name = datetime.now().strftime("lift_constant_speed_test_%Y%m%d_%H%M%S.csv")

        file_path = filedialog.asksaveasfilename(
            title="保存电缸匀速开环测试数据",
            defaultextension=".csv",
            initialfile=default_name,
            filetypes=[("CSV 文件", "*.csv"), ("所有文件", "*.*")]
        )

        if not file_path:
            self.lift_speed_test_start_time = None
            self.status_var.set("电缸匀速测试已停止，未保存文件")
            return

        try:
            with open(file_path, "w", newline="", encoding="utf-8-sig") as f:
                writer = csv.writer(f)

                writer.writerow([
                    "time_s",
                    "test_lift_rpm",
                    "actual_omega_deg_s",
                    "multi_angle_deg",
                    "single_angle_deg",
                    "lift_cmd_rpm",
                    "lift_fb_rpm",
                    "lift_vel_mm_s",
                    "lift_pos_mm",
                    "lift_torque_pct",
                    "raw_count"
                ])

                for row in self.lift_speed_test_data:
                    writer.writerow([
                        f"{row['time_s']:.4f}",
                        f"{row['test_lift_rpm']:.6f}",
                        f"{row['actual_omega_deg_s']:.6f}",
                        f"{row['multi_angle_deg']:.6f}",
                        f"{row['single_angle_deg']:.6f}",
                        row["lift_cmd_rpm"],
                        row["lift_fb_rpm"],
                        f"{row['lift_vel_mm_s']:.6f}",
                        f"{row['lift_pos_mm']:.6f}",
                        f"{row['lift_torque_pct']:.6f}",
                        row["raw_count"]
                    ])

            self.lift_speed_test_start_time = None
            self.status_var.set(f"电缸匀速测试已停止并保存：{file_path}")

        except Exception as e:
            self.lift_speed_test_start_time = None
            messagebox.showerror("保存失败", f"电缸匀速测试数据保存失败：{e}")
            self.status_var.set("电缸匀速测试数据保存失败")


    def record_lift_constant_speed_test_data(self, actual_pos_count):
        """
        记录电缸匀速开环测试数据。
        在每次成功读取角度后调用。
        """
        if not self.lift_speed_test_running or self.lift_speed_test_start_time is None:
            return

        try:
            test_lift_rpm = float(self.lift_speed_test_rpm_var.get())
        except Exception:
            test_lift_rpm = 0.0

        elapsed_time = time.time() - self.lift_speed_test_start_time

        self.lift_speed_test_data.append({
            "time_s": elapsed_time,
            "test_lift_rpm": test_lift_rpm,
            "actual_omega_deg_s": self.pitch_angular_velocity_deg_s,
            "multi_angle_deg": self.lift_multi_angle_deg,
            "single_angle_deg": self.lift_single_angle_deg,
            "lift_cmd_rpm": self.lift_state["cmd_rpm"],
            "lift_fb_rpm": self.lift_state["vel_rpm"],
            "lift_vel_mm_s": self.lift_state["vel_mms"],
            "lift_pos_mm": self.lift_state["pos_mm"],
            "lift_torque_pct": self.lift_state["torque_pct"],
            "raw_count": actual_pos_count,
        })

    def start_pitch_velocity_test(self):
        """
        开始俯仰角速度测试：
        1. 设置目标角速度；
        2. 启动角速度闭环控制；
        3. 开始记录测试数据。
        """
        if self.pitch_test_running:
            messagebox.showwarning("提示", "当前已经在测试中，请先停止测试")
            return
        
        if self.lift_speed_test_running:
            messagebox.showwarning("提示", "当前正在电缸匀速开环测试，请先点击“停止测试并保存”")
            return

        if not self.lift_connected or not self.lift_port:
            messagebox.showwarning("提示", "请先连接举升电动缸驱动器")
            return

        if not self.encoder_connected or not self.encoder_port:
            messagebox.showwarning("提示", "请先连接角度编码器")
            return

        if not self.is_polling:
            messagebox.showwarning("提示", "请先点击“开始读取”，否则无法记录测试数据")
            return

        if self.lift_angle_current_count is None:
            messagebox.showwarning("提示", "当前还没有读取到俯仰角度")
            return

        try:
            test_omega = float(self.pitch_test_omega_var.get())
        except Exception:
            messagebox.showwarning("输入错误", "测试角速度请输入有效数字，例如 0.5")
            return

        if test_omega == 0:
            messagebox.showwarning("输入错误", "测试角速度不能为 0")
            return

        limit = abs(self.pitch_target_omega_limit_deg_s)
        if abs(test_omega) > limit:
            messagebox.showwarning(
                "速度超限",
                f"测试角速度 {test_omega:.3f} deg/s 超过限幅 {limit:.3f} deg/s"
            )
            return

        # 把测试角速度写入角速度控制输入框
        self.pitch_target_omega_var.set(f"{test_omega:.6f}")

        # 清空旧数据
        self.pitch_test_data = []
        self.pitch_test_start_time = None
        self.pitch_test_running = False

        # 启动角速度控制
        self.start_pitch_velocity_control()

        if self.pitch_velocity_control_enabled:
            self.pitch_test_start_time = time.time()
            self.pitch_test_running = True
            self.status_var.set(f"开始俯仰角速度测试：{test_omega:.3f} deg/s")
        else:
            self.status_var.set("俯仰角速度测试启动失败")


    def stop_pitch_velocity_test(self):
        """
        停止俯仰角速度测试：
        1. 停止记录；
        2. 停止角速度控制并停止电缸；
        3. 保存 CSV。
        """
        was_running = self.pitch_test_running
        self.pitch_test_running = False

        # 停止角速度控制，同时停止电缸
        self.stop_pitch_velocity_control()

        if not was_running and not self.pitch_test_data:
            self.status_var.set("当前没有测试数据需要保存")
            self.pitch_test_start_time = None
            return

        if not self.pitch_test_data:
            self.status_var.set("测试已停止，但没有记录到有效数据")
            self.pitch_test_start_time = None
            return

        default_name = datetime.now().strftime("pitch_velocity_test_%Y%m%d_%H%M%S.csv")

        file_path = filedialog.asksaveasfilename(
            title="保存俯仰角速度测试数据",
            defaultextension=".csv",
            initialfile=default_name,
            filetypes=[("CSV 文件", "*.csv"), ("所有文件", "*.*")]
        )

        if not file_path:
            self.pitch_test_start_time = None
            self.status_var.set("测试已停止，未保存文件")
            return

        try:
            with open(file_path, "w", newline="", encoding="utf-8-sig") as f:
                writer = csv.writer(f)

                writer.writerow([
                    "time_s",
                    "target_omega_deg_s",
                    "actual_omega_deg_s",
                    "multi_angle_deg",
                    "single_angle_deg",
                    "lift_cmd_rpm",
                    "lift_fb_rpm",
                    "lift_vel_mm_s",
                    "lift_pos_mm",
                    "lift_torque_pct",
                    "raw_count"
                ])

                for row in self.pitch_test_data:
                    writer.writerow([
                        f"{row['time_s']:.4f}",
                        f"{row['target_omega_deg_s']:.6f}",
                        f"{row['actual_omega_deg_s']:.6f}",
                        f"{row['multi_angle_deg']:.6f}",
                        f"{row['single_angle_deg']:.6f}",
                        row["lift_cmd_rpm"],
                        row["lift_fb_rpm"],
                        f"{row['lift_vel_mm_s']:.6f}",
                        f"{row['lift_pos_mm']:.6f}",
                        f"{row['lift_torque_pct']:.6f}",
                        row["raw_count"]
                    ])

            self.pitch_test_start_time = None
            self.status_var.set(f"测试已停止并保存：{file_path}")

        except Exception as e:
            self.pitch_test_start_time = None
            messagebox.showerror("保存失败", f"测试数据保存失败：{e}")
            self.status_var.set("测试数据保存失败")


    def record_pitch_velocity_test_data(self, actual_pos_count):
        """
        记录俯仰角速度测试数据。
        在每次成功读取角度后调用。
        """
        if not self.pitch_test_running or self.pitch_test_start_time is None:
            return

        try:
            target_omega = float(self.pitch_target_omega_var.get())
        except Exception:
            target_omega = 0.0

        elapsed_time = time.time() - self.pitch_test_start_time

        self.pitch_test_data.append({
            "time_s": elapsed_time,
            "target_omega_deg_s": target_omega,
            "actual_omega_deg_s": self.pitch_angular_velocity_deg_s,
            "multi_angle_deg": self.lift_multi_angle_deg,
            "single_angle_deg": self.lift_single_angle_deg,
            "lift_cmd_rpm": self.lift_state["cmd_rpm"],
            "lift_fb_rpm": self.lift_state["vel_rpm"],
            "lift_vel_mm_s": self.lift_state["vel_mms"],
            "lift_pos_mm": self.lift_state["pos_mm"],
            "lift_torque_pct": self.lift_state["torque_pct"],
            "raw_count": actual_pos_count,
        })

    def start_pitch_velocity_control(self):
        """
        开始俯仰角速度闭环控制。
        输入目标角速度 deg/s，角速度 PID 输出电缸电机速度 rpm。
        """
        if self.lift_speed_test_running:
            messagebox.showwarning("提示", "当前正在电缸匀速开环测试，请先点击“停止测试并保存”")
            return

        if not self.lift_connected or not self.lift_port:
            messagebox.showwarning("提示", "请先连接举升电动缸驱动器")
            return

        if not self.encoder_connected or not self.encoder_port:
            messagebox.showwarning("提示", "请先连接角度编码器")
            return

        if not self.is_polling:
            messagebox.showwarning("提示", "请先点击“开始读取”")
            return

        if self.lift_angle_current_count is None:
            messagebox.showwarning("提示", "当前还没有读取到俯仰角度")
            return

        try:
            float(self.pitch_target_omega_var.get())
            float(self.pitch_omega_kp_var.get())
            float(self.pitch_omega_ki_var.get())
            float(self.pitch_omega_kd_var.get())
        except Exception:
            messagebox.showwarning("输入错误", "目标角速度、速度PID参数都必须是有效数字")
            return

        # 启动角速度控制时，清空 PID 状态
        self.pitch_omega_integral = 0.0
        self.pitch_omega_last_error = 0.0
        self.pitch_omega_last_time = None

        self.pitch_velocity_control_enabled = True
        self.pitch_control_last_send_time = 0.0

        self.status_var.set("俯仰角速度控制已启动")


    def stop_pitch_velocity_control(self):
        """
        停止俯仰角速度闭环控制，并停止电缸。
        """
        if self.lift_speed_test_running:
            messagebox.showwarning("提示", "当前正在电缸匀速开环测试，请点击“停止测试并保存”")
            return
        
        if self.pitch_test_running:
            messagebox.showwarning("提示", "当前正在俯仰角速度测试，请点击“停止测试并保存”")
            return

        self.pitch_velocity_control_enabled = False

        self.pitch_omega_integral = 0.0
        self.pitch_omega_last_error = 0.0
        self.pitch_omega_last_time = None

        if self.lift_connected and self.lift_port:
            self.send_lift_cmd(0)

        self.status_var.set("俯仰角速度控制已停止")


    def update_pitch_velocity_control(self):
        """
        俯仰角速度控制循环。
        不再做目标角度位置控制，只跟踪目标角速度。
        """
        if not self.pitch_velocity_control_enabled:
            return

        if not self.lift_connected or not self.lift_port:
            self.pitch_velocity_control_enabled = False
            return

        now = time.time()
        if now - self.pitch_control_last_send_time < self.pitch_control_send_period:
            return

        try:
            target_omega_deg_s = float(self.pitch_target_omega_var.get())
        except Exception:
            self.pitch_velocity_control_enabled = False
            self.send_lift_cmd(0)
            return

        # 目标角速度限幅
        limit = abs(self.pitch_target_omega_limit_deg_s)

        # 根据当前角度和最大电缸速度，计算当前位置可实现的角速度上限
        current_gain = self.get_pitch_gain(self.lift_multi_angle_deg)

        # 0.9 表示保留 10% 电缸速度余量给 PID 修正和负载波动
        dynamic_limit = abs(current_gain * self.MAX_MOTOR_RPM * 0.9)

        # 取固定限幅和动态限幅中的较小值
        if dynamic_limit > 0:
            limit = min(limit, dynamic_limit)

        target_omega_deg_s = max(-limit, min(limit, target_omega_deg_s))

        # 目标角速度 -> 速度PID -> 电缸 rpm
        motor_rpm = self.pitch_angular_velocity_to_lift_rpm(target_omega_deg_s)
        self.send_lift_cmd(motor_rpm)

        self.pitch_control_last_send_time = now

    # =========================================================
    # 轮询
    # =========================================================
    def start_polling(self):
        if self.is_polling:
            return

        try:
            self.poll_period_ms = int(self.poll_period_var.get())
            if self.poll_period_ms <= 0:
                messagebox.showwarning("输入错误", "轮询周期必须大于 0")
                return
        except Exception:
            messagebox.showwarning("输入错误", "轮询周期请输入整数")
            return

        if not self.lift_connected and not self.encoder_connected:
            messagebox.showwarning("提示", "请至少连接举升电动缸或角度编码器")
            return

        self.stop_polling = False
        self.is_polling = True
        self.polling_thread = threading.Thread(target=self.polling_loop, daemon=True)
        self.polling_thread.start()

        self.status_var.set("开始轮询读取")

    def stop_polling_task(self):
        if not self.is_polling:
            self.status_var.set("当前未在轮询")
            return
        
        if self.lift_speed_test_running:
            messagebox.showwarning("提示", "当前正在电缸匀速开环测试，请先点击“停止测试并保存”")
            return

        if self.pitch_test_running:
            messagebox.showwarning("提示", "当前正在俯仰角速度测试，请先点击“停止测试并保存”")
            return

        if self.pitch_velocity_control_enabled:
            messagebox.showwarning("提示", "当前正在俯仰角速度控制，请先点击“停止角速度控制”")
            return

        self.stop_polling = True
        self.status_var.set("正在停止轮询...")

    def polling_loop(self):
        while not self.stop_polling:
            try:
                if self.lift_connected and self.lift_port:
                    self.root.after(0, self.update_lift_feedback)

                if self.encoder_connected and self.encoder_port:
                    encoder_data = self.read_angle_encoder_data()
                    if encoder_data is not None:
                        self.root.after(0, self.update_encoder_display, encoder_data)

                        # 只有角度读取正常时，才允许继续角速度闭环控制
                        if self.pitch_velocity_control_enabled:
                            if encoder_data.get("health") == "正常":
                                self.update_pitch_velocity_control()
                            else:
                                self.pitch_velocity_control_enabled = False
                                if self.lift_connected and self.lift_port:
                                    self.send_lift_cmd(0)
                                self.root.after(0, self.status_var.set, "角度读取失败，已停止俯仰角速度控制")

            except Exception as e:
                try:
                    self.root.after(0, self.status_var.set, f"读取异常: {e}")
                except Exception:
                    pass

            time.sleep(self.poll_period_ms / 1000.0)

        self.is_polling = False

        try:
            if self.root.winfo_exists():
                self.root.after(0, self.status_var.set, "已停止轮询")
        except Exception:
            pass

    # =========================================================
    # 显示清理
    # =========================================================
    def clear_display(self):
        self.lift_status_var.set("已连接" if self.lift_connected else "未连接")
        self.lift_cmd_speed_var.set("--- r/min")
        self.lift_pos_var.set("--- mm")
        self.lift_vel_var.set("--- r/min (--- mm/s)")
        self.lift_torque_var.set("--- %")

        if self.encoder_connected:
            self.encoder_status_var.set("已连接-协议待补" if not self.encoder_protocol_ready else "已连接")
        else:
            self.encoder_status_var.set("未连接")

        self.encoder_raw_var.set("---")
        self.encoder_single_angle_var.set("--- °")
        self.encoder_multi_angle_var.set("--- °")
        self.encoder_angular_velocity_var.set("--- °/s")
        self.encoder_health_var.set("---")

        self.status_var.set("显示已清空")

    # =========================================================
    # 关闭
    # =========================================================
    def on_closing(self):
        self.stop_polling = True

        self.lift_speed_test_running = False
        self.lift_speed_test_start_time = None

        self.pitch_test_running = False
        self.pitch_test_start_time = None

        self.pitch_velocity_control_enabled = False
        self.pitch_omega_integral = 0.0
        self.pitch_omega_last_error = 0.0
        self.pitch_omega_last_time = None

        try:
            if self.polling_thread and self.polling_thread.is_alive():
                self.polling_thread.join(timeout=0.3)
        except Exception:
            pass

        self.is_polling = False

        try:
            if self.lift_connected:
                self.send_lift_cmd(0)
        except Exception:
            pass

        try:
            with self.lift_lock:
                if self.lift_port:
                    self.lift_port.close()
                    self.lift_port = None
        except Exception:
            pass

        try:
            with self.encoder_lock:
                if self.encoder_port:
                    self.encoder_port.close()
                    self.encoder_port = None
        except Exception:
            pass

        self.lift_connected = False
        self.encoder_connected = False

        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = LiftSystemDebugMonitor(root)
    root.mainloop()

import serial               # 导入串口通信库，负责与硬件数据交互
import serial.tools.list_ports # 用于自动扫描电脑当前可用的 COM 端口
import tkinter as tk        # Python 自带的 GUI 库，用于创建窗口界面
from tkinter import ttk, messagebox, filedialog
import threading
import time
import struct               # 关键库：负责将 Python 的整数转换为 Modbus 协议要求的二进制格式
from datetime import datetime
import collections
import csv
import os
import random
import math


class TiltSensorMonitor:
    def __init__(self, root):
        self.root = root
        self.root.title("倾角传感器数据监控-ZQ-焦田亮 (手动调试+自动调平)")
        # ===== 根据屏幕分辨率自适应窗口大小 =====
        screen_w = self.root.winfo_screenwidth()
        screen_h = self.root.winfo_screenheight()

        win_w = min(1000, int(screen_w * 0.85))
        win_h = min(850, int(screen_h * 0.85))

        self.root.geometry(f"{win_w}x{win_h}")
        self.root.minsize(760, 600)
        
        # ================= 原有基础变量 =================
        self.serial_port = None            # 初始化串口对象变量
        self.is_connected = False          # 记录当前是否已连接电机
        self.receive_thread = None
        self.stop_receive = False
        self.data_count = 0
        self.last_update_time = time.time()
        self.frame_rate = 0
        self.data_buffer = collections.deque(maxlen=1024)
        self.is_saving = False
        self.csv_file = None
        self.csv_writer = None
        self.save_start_time = None
        self.save_data_count = 0

        # ===== 新增：传感器最新数值缓存，控制算法只读这里 =====
        self.sensor_lock = threading.Lock()
        self.latest_angles = {
            'x_angle': 0.0,
            'y_angle': 0.0,
            'z_angle': 0.0
        }

        # ===== 新增：传感器接收保护参数 =====
        self.sensor_rx_buffer = b""                 # 传感器串口本地缓冲
        self.sensor_max_frames_per_cycle = 3        # 每轮最多处理 3 帧，避免一轮吞几十帧
        self.sensor_backlog_drop_bytes = 2048       # 串口积压超过这个字节数时，直接丢弃旧数据
        self.sensor_local_buffer_keep = 64          # 本地 buffer 最多保留的尾部字节数
        self.sensor_max_abs_x = 90.0                # X 轴合法范围
        self.sensor_max_abs_y = 90.0                # Y 轴合法范围
        self.sensor_max_abs_z = 360.0               # Z 轴合法范围
        self.sensor_jump_reject_deg = 1.0           # 相邻有效帧单次跳变超过 1° 先丢掉
        self.last_valid_sensor_angles = {
            'x_angle': 0.0,
            'y_angle': 0.0,
            'z_angle': 0.0
        }
        self.last_sensor_frame_time = 0.0

        # ===== 新增：帧率统计 =====
        self.last_fps_time = time.time()
        self.last_fps_count = 0

        
        
        # ================= 新增：电机硬件控制变量 =================
        # self.motor_port = None          # 电机串口对象
        # self.is_motor_connected = False # 电机连接状态

        self.motor_ports = [None, None, None, None]          # 4个电机串口对象
        self.motor_connected = [False, False, False, False]  # 4个电机连接状态
        self.motor_port_vars = []                            # 4个端口 StringVar
        self.motor_baud_vars = []                            # 4个波特率 StringVar
        self.motor_combos = []                               # 4个端口下拉框
        self.motor_btns = []                                 # 4个连接按钮

        self.manual_pwm_vars = []          # 手动控制输入框变量列表，存放4条腿各自的速度/PWM设定值
        self.feedback_pos_vars = []         # 4条腿位置反馈显示变量
        self.feedback_vel_vars = []         # 4条腿速度反馈显示变量
        self.feedback_torque_vars = []      # 4条腿转矩反馈显示变量
        self.feedback_job = None            # 周期反馈任务
        self.lead_mm = 5.0                  # 丝杆导程，单位 mm
        self.reduction_ratio = 5.0          # 减速比（电机转5圈，丝杆转1圈）

        self.is_feedback_recording = False      # 是否正在记录反馈
        self.feedback_csv_file = None           # 反馈记录文件对象
        self.feedback_csv_writer = None         # 反馈记录 CSV 写入器
        self.feedback_record_start_time = None  # 反馈记录开始时间
        self.last_feedback_record_time = 0.0    # 上一次写入反馈的时间
        # self.feedback_record_interval = 0.2     # 反馈记录周期，单位秒

        # ===== 新增：两套周期 =====
        # 1) 控制输出周期
        self.output_period_ms = 50
        self.output_period_s = self.output_period_ms / 1000.0

        # 2) 反馈轮询/记录周期（两者统一）
        self.feedback_period_ms = 200
        self.feedback_period_s = self.feedback_period_ms / 1000.0
        self.feedback_record_interval = self.feedback_period_s

        # ===== 新增：界面变量 =====
        self.output_period_var = tk.StringVar(value=str(self.output_period_ms))
        self.feedback_period_var = tk.StringVar(value=str(self.feedback_period_ms))

        self.output_period_show_var = tk.StringVar(value=f"{self.output_period_ms} ms")
        self.feedback_period_show_var = tk.StringVar(value=f"{self.feedback_period_ms} ms")

        self.feedback_path_var = tk.StringVar(value="反馈文件：未选择")

        # ===== 新增：调平调试会话记录变量（独立于“开始记录反馈”功能） =====
        self.is_leveling_session_recording = False
        self.leveling_session_file = None
        self.leveling_session_writer = None
        self.leveling_session_start_time = None
        self.leveling_session_last_record_time = 0.0
        self.leveling_session_filename = ""
        self.leveling_stop_reason = ""
        self.leveling_entry_info = {
            "entry_time": "",
            "entry_x_angle": 0.0,
            "entry_y_angle": 0.0,
            "entry_z_angle": 0.0,
            "entry_ref_leg": 0,
            "entry_pos_1": 0.0,
            "entry_pos_2": 0.0,
            "entry_pos_3": 0.0,
            "entry_pos_4": 0.0,
        }
        
        # ================= TPCS2 算法核心配置参数 =================
        self.VEH_LEN = 1565   # 车辆长度 (前后支腿间距 mm)
        self.VEH_WID = 1215   # 车辆宽度 (左右支腿间距 mm)
        self.TH_AMP = 2.5       # [触地阈值] 电流超过此值(2.5A)认为接触地面
        self.PWM_FAST = 480      # [快速档] 阶段一寻找地面时的电机PWM
        self.PWM_SLOW = 300      # [慢速档] 阶段二同步顶升时的电机PWM
        self.LIFT_DIST = 30.0   # [顶升高度] 阶段二目标顶升距离 (mm)
        self.DEADBAND = 0.02    # [调平死区] 角度误差小于0.02度时停止调节
        
        self.K_P = 0.2                  # [比例控制增益] 将位置误差(mm)转换为期望物理速度(mm/s)

        # 驱动器允许的最大电机转速（你现在发送函数里实际上就是按 500 r/min 上限）
        self.MAX_MOTOR_RPM = 150.0

        # 由最大电机转速换算得到最大电缸速度
        self.MAX_CYLINDER_SPEED = self.MAX_MOTOR_RPM * (self.lead_mm / self.reduction_ratio) / 60.0

        self.K_TWIST = 0.1      # [抗扭增益] 用于消除车架扭曲应力的修正系数

        # ===== 新增：调平完成后维持参数 =====
        self.leveling_hold_time_s = 5.0          # 进入合适区间后，维持 5 秒再真正结束
        self.leveling_hold_start_time = None     # 开始维持的时刻
        self.leveling_hold_active = False        # 是否正在维持
        
        # ================= 调平系统状态变量 =================
        self.leveling_state = {
            "stage": "待机 (IDLE)",
            "ref_leg": "无",
            "leg_speeds": [0, 0, 0, 0]
        }
        self.sim_timer = None 
        self.algo_state = "IDLE"      
        self.ref_leg_id = 0           
        self.mask_timer = 0
        
        self.debug_mode = False      # 是否处于单阶段调试模式
        self.debug_stage = None      # 当前调试阶段：GROUNDING / PRE_LIFT / LEVELING
        
        self.legs = []
        for i in range(4):
            self.legs.append({
                'id': i,
                'pos': 0.0,                 # 当前相对位置，单位 mm
                'start_pos': 0.0,
                'amp': 0.0,                 # 这里暂时不是真实电流，后续如接入电流寄存器再替换
                'grounded': False,
                'pwm': 0,
                'encoder_accum_count': 0,   # 编码器累计 count
                'last_single_turn': None,   # 上一次单圈编码器读数
                'zero_encoder_count': None, # 相对零点
                'vel_rpm': 0,               # 电机速度 r/min
                'vel_mms': 0.0,             # 电动缸速度 mm/s
                'torque_pct': 0.0,          # 转矩 %
            })
        
        # self.virt_imu = {'x': 0.0, 'y': 0.0}
        self.setup_ui()                    # 初始化用户界面布局
        self.refresh_ports()               # 程序启动时自动扫描一次端口
        
    
    def setup_ui(self):
        # ===== 主界面滚动容器 =====
        outer_frame = ttk.Frame(self.root)
        outer_frame.pack(fill=tk.BOTH, expand=True)

        self.main_canvas = tk.Canvas(outer_frame, highlightthickness=0)
        self.main_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.main_scrollbar = ttk.Scrollbar(
            outer_frame,
            orient=tk.VERTICAL,
            command=self.main_canvas.yview
        )
        self.main_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        self.main_canvas.configure(yscrollcommand=self.main_scrollbar.set)

        main_frame = ttk.Frame(self.main_canvas, padding="10")
        self.main_canvas_window = self.main_canvas.create_window(
            (0, 0),
            window=main_frame,
            anchor="nw"
        )

        # 内容大小变化时，自动更新滚动区域
        def _on_frame_configure(event):
            self.main_canvas.configure(scrollregion=self.main_canvas.bbox("all"))

        main_frame.bind("<Configure>", _on_frame_configure)

        # 窗口宽度变化时，让内部 frame 跟着变宽
        def _on_canvas_configure(event):
            self.main_canvas.itemconfig(self.main_canvas_window, width=event.width)

        self.main_canvas.bind("<Configure>", _on_canvas_configure)

        # 鼠标滚轮滚动
        def _on_mousewheel(event):
            self.main_canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

        self.main_canvas.bind_all("<MouseWheel>", _on_mousewheel)
        
        # 1. 串口配置区域 (包含原有传感器和新增电机串口)
        config_frame = ttk.LabelFrame(main_frame, text="通信配置", padding="5")
        config_frame.pack(fill=tk.X, pady=5)
        
        # --- 传感器串口 ---
        ttk.Label(config_frame, text="传感器端口:").grid(row=0, column=0, sticky=tk.W)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(config_frame, textvariable=self.port_var, width=8)
        self.port_combo.grid(row=0, column=1, padx=5)
        self.baudrate_var = tk.StringVar(value="230400")
        ttk.Combobox(config_frame, textvariable=self.baudrate_var, values=["9600", "19200", "38400","57600", "115200", "230400", "460800", "921600"], width=8).grid(row=0, column=2, padx=5)
        self.connect_btn = ttk.Button(config_frame, text="连接传感器", command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=3, padx=5)

        # --- 4个电机各自串口 ---
        baud_values = ["9600", "19200", "38400", "57600", "115200", "230400", "460800", "921600"]

        for i in range(4):
            ttk.Label(config_frame, text=f"{i+1}号电机端口:").grid(row=i+1, column=0, sticky=tk.W, pady=5)

            port_var = tk.StringVar()
            baud_var = tk.StringVar(value="115200")

            combo = ttk.Combobox(config_frame, textvariable=port_var, width=8)
            combo.grid(row=i+1, column=1, padx=5)

            baud_combo = ttk.Combobox(config_frame, textvariable=baud_var, values=baud_values, width=8)
            baud_combo.grid(row=i+1, column=2, padx=5)

            btn = ttk.Button(config_frame, text=f"连接{i+1}号电机", command=lambda idx=i: self.toggle_motor_connection(idx))
            btn.grid(row=i+1, column=3, padx=5)

            self.motor_port_vars.append(port_var)
            self.motor_baud_vars.append(baud_var)
            self.motor_combos.append(combo)
            self.motor_btns.append(btn)

        ttk.Button(config_frame, text="刷新所有端口", command=self.refresh_ports).grid(row=0, column=4, rowspan=5, padx=10)
        
        # 2. 倾角数据显示区域 (保持不变)
        self.setup_angle_display_ui(main_frame)

        # 3. 调平系统监控面板 (包含原有进度条)
        self.setup_leveling_ui(main_frame)

        # 4. “手动控制”区域
        self.setup_manual_control_ui(main_frame)

        # 5. 数据保存与统计 (保持不变)
        self.setup_save_stats_ui(main_frame)
        
        self.refresh_ports()

    def setup_angle_display_ui(self, main_frame):
        data_frame = ttk.LabelFrame(main_frame, text="倾角数据", padding="10")
        data_frame.pack(fill=tk.X, pady=5)
        angles = [("X轴(俯仰角)", "x_angle", "red"), ("Y轴(滚转角)", "y_angle", "green"), ("Z轴(航向角)", "z_angle", "blue")]
        self.angle_vars = {}
        for i, (label, key, color) in enumerate(angles):
            angle_frame = ttk.LabelFrame(data_frame, text=label, padding="10")
            angle_frame.grid(row=0, column=i, sticky="nsew", padx=5, pady=5)
            self.angle_vars[key] = tk.StringVar(value="0.000°")
            tk.Label(angle_frame, textvariable=self.angle_vars[key], font=("Arial", 20, "bold"), fg=color).pack(expand=True)
            trend_label = tk.Label(angle_frame, text="●", font=("Arial", 12), fg="gray")
            setattr(self, f"{key}_trend", trend_label)
            trend_label.pack()
        data_frame.columnconfigure((0,1,2), weight=1)

    def setup_save_stats_ui(self, main_frame):
        save_frame = ttk.LabelFrame(main_frame, text="数据保存与统计", padding="5")
        save_frame.pack(fill=tk.X, pady=5)
        self.save_btn = ttk.Button(save_frame, text="开始保存", command=self.toggle_save_data)
        self.save_btn.grid(row=0, column=0, padx=5)
        self.save_status_var = tk.StringVar(value="未保存")
        ttk.Label(save_frame, textvariable=self.save_status_var).grid(row=0, column=1, padx=5)

        self.feedback_save_btn = ttk.Button(save_frame, text="开始记录反馈", command=self.toggle_feedback_recording)
        self.feedback_save_btn.grid(row=1, column=0, padx=5, pady=5)

        self.feedback_save_status_var = tk.StringVar(value="反馈未记录")
        ttk.Label(save_frame, textvariable=self.feedback_save_status_var).grid(row=1, column=1, padx=5, pady=5)

        # ===== 新增：输出周期设置 =====
        ttk.Label(save_frame, text="输出周期(ms):").grid(row=2, column=0, padx=5, pady=5, sticky=tk.W)
        ttk.Entry(save_frame, textvariable=self.output_period_var, width=8).grid(row=2, column=1, padx=5, pady=5, sticky=tk.W)
        ttk.Button(save_frame, text="更新输出周期", command=self.update_output_period).grid(row=2, column=2, padx=5, pady=5)
        ttk.Label(save_frame, text="当前输出周期:").grid(row=2, column=3, padx=5, pady=5, sticky=tk.E)
        ttk.Label(save_frame, textvariable=self.output_period_show_var).grid(row=2, column=4, padx=5, pady=5, sticky=tk.W)

        # ===== 新增：反馈/记录周期设置 =====
        ttk.Label(save_frame, text="反馈/记录周期(ms):").grid(row=3, column=0, padx=5, pady=5, sticky=tk.W)
        ttk.Entry(save_frame, textvariable=self.feedback_period_var, width=8).grid(row=3, column=1, padx=5, pady=5, sticky=tk.W)
        ttk.Button(save_frame, text="更新反馈周期", command=self.update_feedback_period).grid(row=3, column=2, padx=5, pady=5)
        ttk.Label(save_frame, text="当前反馈周期:").grid(row=3, column=3, padx=5, pady=5, sticky=tk.E)
        ttk.Label(save_frame, textvariable=self.feedback_period_show_var).grid(row=3, column=4, padx=5, pady=5, sticky=tk.W)

        # ===== 新增：反馈记录文件完整路径显示 =====
        ttk.Label(save_frame, textvariable=self.feedback_path_var).grid(
            row=4, column=0, columnspan=5, padx=5, pady=5, sticky=tk.W
        )

        self.fps_var = tk.StringVar(value="0 Hz")
        ttk.Label(save_frame, textvariable=self.fps_var).grid(row=0, column=2, padx=20)
        self.count_var = tk.StringVar(value="0 帧")
        ttk.Label(save_frame, textvariable=self.count_var).grid(row=0, column=3, padx=5)
        self.status_var = tk.StringVar(value="就绪")
        ttk.Label(self.root, textvariable=self.status_var, relief=tk.SUNKEN, anchor=tk.W).pack(side=tk.BOTTOM, fill=tk.X)

    def setup_leveling_ui(self, parent_frame):
        level_frame = ttk.LabelFrame(parent_frame, text="自动调平系统状态 (硬件输出)", padding="10")
        level_frame.pack(fill=tk.X, pady=5)
        info_frame = ttk.Frame(level_frame)
        info_frame.pack(fill=tk.X, pady=(0, 10))
        ttk.Label(info_frame, text="当前阶段:", font=("Arial", 11)).pack(side=tk.LEFT)
        self.stage_var = tk.StringVar(value="待机 (IDLE)")
        tk.Label(info_frame, textvariable=self.stage_var, font=("Arial", 12, "bold"), fg="blue", bg="#f0f0f0", width=18).pack(side=tk.LEFT, padx=5)

        # self.auto_btn = ttk.Button(info_frame, text="启动调平", command=self.toggle_auto_leveling)
        # self.auto_btn.pack(side=tk.RIGHT, padx=5)

        # 暂时屏蔽
        self.auto_btn = ttk.Button(
            info_frame,
            text="启动调平(暂不可用)",
            command=self.toggle_auto_leveling,
            state=tk.DISABLED
        )
        self.auto_btn.pack(side=tk.RIGHT, padx=5)


        self.stop_btn = ttk.Button(info_frame, text="全部停止", command=self.stop_all_motors)
        self.stop_btn.pack(side=tk.RIGHT, padx=5)

        legs_frame = ttk.Frame(level_frame)
        legs_frame.pack(fill=tk.X)
        self.leg_bars, self.leg_labels = [], []
        for i in range(4):
            leg_col = ttk.Frame(legs_frame)
            leg_col.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=5)
            ttk.Label(leg_col, text=f"{i+1}号腿速度", font=("Arial", 10)).pack(anchor="w")
            bar = ttk.Progressbar(leg_col, orient=tk.HORIZONTAL, length=100, mode='determinate')
            bar.pack(fill=tk.X, pady=2)
            self.leg_bars.append(bar); lbl = tk.StringVar(value="0 (r/min)")
            ttk.Label(leg_col, textvariable=lbl, font=("Arial", 9)).pack(anchor="e")
            self.leg_labels.append(lbl)

        debug_frame = ttk.Frame(level_frame)
        debug_frame.pack(fill=tk.X, pady=(8, 0))

        ttk.Label(debug_frame, text="单阶段调试:").pack(side=tk.LEFT, padx=5)

        # ttk.Button(
        #     debug_frame,
        #     text="调试触地",
        #     command=lambda: self.start_stage_debug("GROUNDING")
        # ).pack(side=tk.LEFT, padx=5)

        # ttk.Button(
        #     debug_frame,
        #     text="调试顶升",
        #     command=lambda: self.start_stage_debug("PRE_LIFT")
        # ).pack(side=tk.LEFT, padx=5)

        # ttk.Button(
        #     debug_frame,
        #     text="调试调平",
        #     command=lambda: self.start_stage_debug("LEVELING")
        # ).pack(side=tk.LEFT, padx=5)

        # 暂时屏蔽
        self.debug_ground_btn = ttk.Button(
            debug_frame,
            text="调试触地(暂不可用)",
            command=lambda: self.start_stage_debug("GROUNDING"),
            state=tk.DISABLED
        )
        self.debug_ground_btn.pack(side=tk.LEFT, padx=5)

        self.debug_lift_btn = ttk.Button(
            debug_frame,
            text="调试顶升(暂不可用)",
            command=lambda: self.start_stage_debug("PRE_LIFT"),
            state=tk.DISABLED
        )
        self.debug_lift_btn.pack(side=tk.LEFT, padx=5)

        self.debug_level_btn = ttk.Button(
            debug_frame,
            text="调试调平",
            command=lambda: self.start_stage_debug("LEVELING")
        )
        self.debug_level_btn.pack(side=tk.LEFT, padx=5)

    def setup_manual_control_ui(self, parent_frame):
        wrap_frame = ttk.Frame(parent_frame)
        wrap_frame.pack(fill=tk.X, pady=5)

        # 左侧：手动控制
        manual_frame = ttk.LabelFrame(wrap_frame, text="手动控制", padding="10")
        manual_frame.pack(side=tk.LEFT, fill=tk.X, expand=True)

        header = ["腿号", "速度/(r/min)", "上行", "下行", "停止"]
        for c, text in enumerate(header):
            ttk.Label(manual_frame, text=text, font=("Arial", 10, "bold")).grid(row=0, column=c, padx=8, pady=5)

        for i in range(4):
            ttk.Label(manual_frame, text=f"{i+1}号腿").grid(row=i+1, column=0, padx=8, pady=5)

            pwm_var = tk.StringVar(value="30")
            self.manual_pwm_vars.append(pwm_var)
            ttk.Entry(manual_frame, textvariable=pwm_var, width=8).grid(row=i+1, column=1, padx=8, pady=5)

            ttk.Button(
                manual_frame,
                text="上行",
                command=lambda idx=i: self.manual_move_leg(idx, 1)
            ).grid(row=i+1, column=2, padx=8, pady=5)

            ttk.Button(
                manual_frame,
                text="下行",
                command=lambda idx=i: self.manual_move_leg(idx, -1)
            ).grid(row=i+1, column=3, padx=8, pady=5)

            ttk.Button(
                manual_frame,
                text="停止",
                command=lambda idx=i: self.manual_stop_leg(idx)
            ).grid(row=i+1, column=4, padx=8, pady=5)

        # 同时控制按钮
        ttk.Button(
            manual_frame,
            text="同时上行",
            command=lambda: self.manual_move_all(1)
        ).grid(row=5, column=0, columnspan=2, pady=8, padx=5, sticky="ew")

        ttk.Button(
            manual_frame,
            text="同时下行",
            command=lambda: self.manual_move_all(-1)
        ).grid(row=5, column=2, columnspan=2, pady=8, padx=5, sticky="ew")

        ttk.Button(
            manual_frame,
            text="全部停止",
            command=self.stop_all_motors
        ).grid(row=5, column=4, pady=8, padx=5, sticky="ew")

        # 右侧：反馈区域
        feedback_frame = ttk.LabelFrame(wrap_frame, text="反馈", padding="10")
        feedback_frame.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(12, 0))

        fb_header = ["腿号", "位置", "速度", "转矩"]
        for c, text in enumerate(fb_header):
            ttk.Label(feedback_frame, text=text, font=("Arial", 10, "bold")).grid(row=0, column=c, padx=8, pady=5)

        for i in range(4):
            ttk.Label(feedback_frame, text=f"{i+1}号腿").grid(row=i+1, column=0, padx=8, pady=5)

            pos_var = tk.StringVar(value="--- mm")
            vel_var = tk.StringVar(value="--- r/min (--- mm/s)")
            torque_var = tk.StringVar(value="--- %")

            self.feedback_pos_vars.append(pos_var)
            self.feedback_vel_vars.append(vel_var)
            self.feedback_torque_vars.append(torque_var)

            ttk.Label(feedback_frame, textvariable=pos_var, width=12, anchor="center").grid(row=i+1, column=1, padx=8, pady=5)
            ttk.Label(feedback_frame, textvariable=vel_var, width=16, anchor="center").grid(row=i+1, column=2, padx=8, pady=5)
            ttk.Label(feedback_frame, textvariable=torque_var, width=8, anchor="center").grid(row=i+1, column=3, padx=8, pady=5)

        # 反馈区按钮
        ttk.Button(
            feedback_frame,
            text="重置反馈区",
            command=self.reset_feedback_area
        ).grid(row=5, column=1, columnspan=3, pady=8, padx=5, sticky="ew")

    # ================================================================
    # 新增：Modbus RTU 输出逻辑
    # ================================================================
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
        return struct.pack('<H', crc)

    def send_motor_cmd(self, leg_id, pwm_val):
        """
        下发电机转速指令
        现在 pwm_val 的物理含义已经改成：目标电机转速 r/min
        """
        if leg_id < 0 or leg_id >= 4:
            return

        port = self.motor_ports[leg_id]
        if not self.motor_connected[leg_id] or not port:
            return

        try:
            # 现在内部量直接就是目标转速 r/min
            phys_speed = int(pwm_val)

            # 限制在驱动器允许的最大转速范围内
            phys_speed = max(-int(self.MAX_MOTOR_RPM), min(int(self.MAX_MOTOR_RPM), phys_speed))

            # 如果你的实际接法是“每个串口各接一个驱动，且每个驱动站号都设为01”
            # 那这里就固定写 0x01
            payload = bytearray([0x01, 0x10, 0x33, 0x08, 0x00, 0x02, 0x08])

            # 速度按 16 位有符号整数、大端格式写入
            payload.extend(struct.unpack('BB', struct.pack('>h', phys_speed)))

            # 按你现在驱动协议的写法补后面 6 个字节
            if phys_speed < 0:
                payload.extend(b'\xFF\xFF\xFF\xFF\xFF\xFF')
            else:
                payload.extend(b'\x00\x00\x00\x00\x00\x00')

            # CRC 校验并发送
            full_packet = payload + self.calculate_crc16(payload)
            port.write(full_packet)

        except Exception as e:
            print(f"Motor Send Err: {e}")

    def read_motor_holding_registers(self, leg_id, start_addr, reg_count):
        """读取指定电机的 Modbus 保持寄存器"""
        if leg_id < 0 or leg_id >= 4:
            return None
        port = self.motor_ports[leg_id]
        if not self.motor_connected[leg_id] or not port:
            return None

        try:
            # slave_id = leg_id + 1
            slave_id = 0x01
            packet = bytearray([
                slave_id, 0x03,
                (start_addr >> 8) & 0xFF, start_addr & 0xFF,
                (reg_count >> 8) & 0xFF, reg_count & 0xFF
            ])
            full_packet = packet + self.calculate_crc16(packet)

            port.reset_input_buffer()
            port.write(full_packet)

            expected_len = 3 + reg_count * 2 + 2
            response = port.read(expected_len)
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
        except Exception:
            return None


    def read_leg_encoder_single_turn(self, leg_id):
        raw = self.read_motor_holding_registers(leg_id, 0x4202, 2)
        if raw is None or len(raw) != 4:
            return None

        # 实测取法：L=raw[0], M=raw[1], H=raw[3]
        byte_l = raw[0]
        byte_m = raw[1]
        byte_h = raw[3]
        value = (byte_h << 16) | (byte_m << 8) | byte_l
        return value & 0x7FFFFF


    def update_leg_encoder_accum(self, leg_id):
        single_turn = self.read_leg_encoder_single_turn(leg_id)
        if single_turn is None:
            return None

        leg = self.legs[leg_id]
        if leg['last_single_turn'] is None:
            leg['last_single_turn'] = single_turn
            leg['encoder_accum_count'] = 0
            return 0

        delta = single_turn - leg['last_single_turn']
        one_turn = 1 << 23
        half_turn = one_turn // 2
        if delta > half_turn:
            delta -= one_turn
        elif delta < -half_turn:
            delta += one_turn

        leg['encoder_accum_count'] += delta
        leg['last_single_turn'] = single_turn
        return leg['encoder_accum_count']


    def count_to_mm(self, count_value):
        mm_per_motor_rev = self.lead_mm / self.reduction_ratio
        mm_per_count = mm_per_motor_rev / (1 << 23)
        return count_value * mm_per_count


    def rpm_to_mms(self, rpm_value):
        mm_per_motor_rev = self.lead_mm / self.reduction_ratio
        return rpm_value * mm_per_motor_rev / 60.0


    def read_leg_velocity_rpm(self, leg_id):
        raw = self.read_motor_holding_registers(leg_id, 0x4025, 1)
        if raw is None or len(raw) != 2:
            return None
        return struct.unpack('>h', raw)[0]


    def read_leg_torque_pct(self, leg_id):
        raw = self.read_motor_holding_registers(leg_id, 0x6025, 1)
        if raw is None or len(raw) != 2:
            return None
        return struct.unpack('>h', raw)[0] / 10.0


    def start_feedback_polling(self):
        if self.feedback_job is None:
            self._feedback_loop()


    def stop_feedback_polling(self):
        if self.feedback_job is not None:
            self.root.after_cancel(self.feedback_job)
            self.feedback_job = None

    def _feedback_loop(self):
        self.update_leg_feedback()
        self.feedback_job = self.root.after(self.feedback_period_ms, self._feedback_loop)
    
    def stop_all_motors(self, stop_reason="manual_stop"):
        if self.sim_timer:
            self.root.after_cancel(self.sim_timer)
            self.sim_timer = None

        self.algo_state = "IDLE"
        self.debug_mode = False
        self.debug_stage = None
        self.leveling_hold_start_time = None
        self.leveling_hold_active = False

        for i in range(4):
            self.legs[i]['pwm'] = 0
            try:
                self.send_motor_cmd(i, 0)
            except:
                pass
    
            if i < len(self.leg_bars):
                self.leg_bars[i]['value'] = 0
            if i < len(self.leg_labels):
                self.leg_labels[i].set("0 (r/min)")

        if hasattr(self, "stage_var"):
            self.stage_var.set("待机 (IDLE)")
        if hasattr(self, "auto_btn"):
            self.auto_btn.config(text="启动调平(暂不可用)")

        self.stop_leveling_session_recording(stop_reason)

        if any(self.motor_connected) and self.feedback_job is None:
            self.start_feedback_polling()

    def manual_move_leg(self, leg_id, direction):
        if self.sim_timer:
            messagebox.showwarning("自动运行中", "请先停止自动调平/阶段调试")
            return

        if leg_id < 0 or leg_id >= 4:
            return

        if not self.motor_connected[leg_id]:
            messagebox.showwarning("电机未连接", f"{leg_id+1}号电机未连接")
            return

        try:
            # 现在输入框中的值，直接按“目标电机转速 r/min”处理
            cmd_rpm = int(self.manual_pwm_vars[leg_id].get())
        except:
            messagebox.showwarning("输入错误", f"{leg_id+1}号腿速度请输入整数 r/min")
            return

        # 先取绝对值，再限制到驱动器允许的最大转速范围内
        cmd_rpm = max(0, min(int(self.MAX_MOTOR_RPM), abs(cmd_rpm)))

        # 根据“上行/下行”按钮决定正负方向
        cmd_rpm = cmd_rpm if direction > 0 else -cmd_rpm

        # 虽然变量名还叫 pwm，但现在它的物理含义已经改成“指令转速 r/min”
        self.legs[leg_id]['pwm'] = cmd_rpm

        # 直接把目标转速下发给驱动器
        self.send_motor_cmd(leg_id, cmd_rpm)

        # 进度条按最大转速做百分比显示
        percent = int((abs(cmd_rpm) / self.MAX_MOTOR_RPM) * 100)
        self.leg_bars[leg_id]['value'] = percent

        # 标签显示当前下发的目标转速
        self.leg_labels[leg_id].set(f"{cmd_rpm} (r/min)")

    def manual_move_all(self, direction):
        """
        按4个输入框中的速度值，同时启动4条腿
        direction = 1  表示同时上行
        direction = -1 表示同时下行
        """
        if self.sim_timer:
            messagebox.showwarning("自动运行中", "请先停止自动调平/阶段调试")
            return

        cmd_list = []

        # 先统一检查，确保4条腿都满足条件后再一起启动
        for leg_id in range(4):
            if not self.motor_connected[leg_id]:
                messagebox.showwarning("电机未连接", f"{leg_id+1}号电机未连接，无法同时启动")
                return

            try:
                cmd_rpm = int(self.manual_pwm_vars[leg_id].get())
            except:
                messagebox.showwarning("输入错误", f"{leg_id+1}号腿速度请输入整数 r/min")
                return

            # 限幅到允许范围
            cmd_rpm = max(0, min(int(self.MAX_MOTOR_RPM), abs(cmd_rpm)))

            # 根据方向加正负号
            cmd_rpm = cmd_rpm if direction > 0 else -cmd_rpm

            cmd_list.append(cmd_rpm)

        # 全部检查通过后，再统一下发
        for leg_id in range(4):
            cmd_rpm = cmd_list[leg_id]

            self.legs[leg_id]['pwm'] = cmd_rpm
            self.send_motor_cmd(leg_id, cmd_rpm)

            percent = int((abs(cmd_rpm) / self.MAX_MOTOR_RPM) * 100)
            self.leg_bars[leg_id]['value'] = percent
            self.leg_labels[leg_id].set(f"{cmd_rpm} (r/min)")

    def manual_stop_leg(self, leg_id):
        if leg_id < 0 or leg_id >= 4:
            return

        self.legs[leg_id]['pwm'] = 0
        self.send_motor_cmd(leg_id, 0)
        self.leg_bars[leg_id]['value'] = 0
        self.leg_labels[leg_id].set("0 (r/min)")

    def toggle_motor_connection(self, leg_id):
        if not self.motor_connected[leg_id]:
            try:
                port_name = self.motor_port_vars[leg_id].get()
                if not port_name:
                    messagebox.showwarning("端口未选择", f"请先选择{leg_id+1}号电机串口")
                    return

                baudrate = int(self.motor_baud_vars[leg_id].get())
                self.motor_ports[leg_id] = serial.Serial(port_name, baudrate, timeout=0.03)
                self.motor_connected[leg_id] = True
                self.legs[leg_id]['encoder_accum_count'] = 0
                self.legs[leg_id]['last_single_turn'] = None
                self.legs[leg_id]['zero_encoder_count'] = None
                self.motor_btns[leg_id].config(text=f"断开{leg_id+1}号电机")
                self.start_feedback_polling()
            except Exception as e:
                messagebox.showerror("电机连接错误", f"{leg_id+1}号电机: {e}")
        else:
            try:
                self.send_motor_cmd(leg_id, 0)
            except:
                pass

            self.legs[leg_id]['pwm'] = 0
            self.motor_connected[leg_id] = False

            if self.motor_ports[leg_id]:
                self.motor_ports[leg_id].close()
                self.motor_ports[leg_id] = None

            self.motor_btns[leg_id].config(text=f"连接{leg_id+1}号电机")
            self.leg_bars[leg_id]['value'] = 0
            self.leg_labels[leg_id].set("0 (r/min)")
            self.legs[leg_id]['encoder_accum_count'] = 0
            self.legs[leg_id]['last_single_turn'] = None
            self.legs[leg_id]['zero_encoder_count'] = None
            self.legs[leg_id]['pos'] = 0.0
            self.legs[leg_id]['vel_rpm'] = 0
            self.legs[leg_id]['vel_mms'] = 0.0
            self.legs[leg_id]['torque_pct'] = 0.0

            if leg_id < len(self.feedback_pos_vars):
                self.feedback_pos_vars[leg_id].set("--- mm")
                self.feedback_vel_vars[leg_id].set("--- r/min (--- mm/s)")
                self.feedback_torque_vars[leg_id].set("--- %")

            if not any(self.motor_connected):
                self.stop_feedback_polling()


    # ================================================================
    # 核心算法：保持逻辑不变，增加 send_motor_cmd 调用
    # ================================================================
    def run_algorithm_loop(self):
        self.update_leg_feedback()

        # if not self.is_connected:
        #     self.stop_all_motors()
        #     self.algo_state = "IDLE"
        #     self.stage_var.set("传感器未连接")
        #     self.sim_timer = None
        #     self.auto_btn.config(text="启动调平")
        #     return
        if self.algo_state == "LEVELING" and not self.is_connected:
            self.stop_all_motors(stop_reason="sensor_disconnect")
            self.stage_var.set("传感器未连接")
            return

        with self.sensor_lock:
            curr_ax = self.latest_angles['x_angle']
            curr_ay = self.latest_angles['y_angle']    

        state = self.algo_state
        if state == "GROUNDING":
            all_done = True
            if self.mask_timer < 10: self.mask_timer += 1
            for leg in self.legs:
                if not leg['grounded']:
                    if self.mask_timer >= 10 and leg['amp'] > self.TH_AMP:
                        leg['grounded'] = True; leg['pwm'] = 0
                    else:
                        leg['pwm'] = self.PWM_FAST; all_done = False
                else: leg['pwm'] = 0
            if all_done:
                self.ref_leg_id = self.find_highest_leg(curr_ax, curr_ay)
                for leg in self.legs:
                    leg['start_pos'] = leg['pos']

                if self.debug_mode and self.debug_stage == "GROUNDING":
                    self.finish_stage_debug("寻找接地点完成")
                    return
                else:
                    self.algo_state = "PRE_LIFT"

        elif state == "PRE_LIFT":
            all_done = True
            for leg in self.legs:
                if (leg['pos'] - leg['start_pos']) < self.LIFT_DIST:
                    leg['pwm'] = self.PWM_SLOW; all_done = False
                else: leg['pwm'] = 0
            if all_done:
                if self.debug_mode and self.debug_stage == "PRE_LIFT":
                    self.finish_stage_debug("同步顶升完成")
                    return
                else:
                    self.algo_state = "LEVELING"

        elif state == "LEVELING":
            err_x, err_y = 0.0 - curr_ax, 0.0 - curr_ay
            if abs(err_x) < self.DEADBAND: err_x = 0.0
            if abs(err_y) < self.DEADBAND: err_y = 0.0
            v_pitch = math.tan(err_x * 0.01745) * self.VEH_LEN * 0.5
            v_roll  = math.tan(err_y * 0.01745) * self.VEH_WID * 0.5
            d = [l['pos'] - l['start_pos'] for l in self.legs]
            twist_err = (d[1] + d[3]) - (d[0] + d[2])
            v_twist = max(-50, min(50, twist_err * self.K_TWIST))
            
            if err_x == 0 and err_y == 0 and abs(twist_err) < 1.0:
                # 先全部停住，进入维持状态
                for leg in self.legs:
                    leg['pwm'] = 0

                now_time = time.time()

                # 第一次进入维持区间
                if not self.leveling_hold_active:
                    self.leveling_hold_active = True
                    self.leveling_hold_start_time = now_time

                hold_elapsed = now_time - self.leveling_hold_start_time

                # 在界面上显示当前正在维持
                self.stage_var.set(f"姿态调平完成，维持中 ({hold_elapsed:.1f}/{self.leveling_hold_time_s:.1f}s)")

                # 只有维持时间够了，才真正结束
                if hold_elapsed >= self.leveling_hold_time_s:
                    if self.debug_mode and self.debug_stage == "LEVELING":
                        self.finish_stage_debug("姿态调平完成")
                        return
            
            else:
                # 一旦跑出合适区间，取消维持，恢复正常调平
                self.leveling_hold_active = False
                self.leveling_hold_start_time = None

                v_req = [0.0]*4
                v_req[0] = ( v_pitch - v_roll) + v_twist # FR
                v_req[1] = ( v_pitch + v_roll) - v_twist # FL
                v_req[2] = (-v_pitch + v_roll) + v_twist # RL
                v_req[3] = (-v_pitch - v_roll) - v_twist # RR
                v_base = v_req[self.ref_leg_id]

                # 1. 计算每个腿的期望物理速度 (mm/s)
                raw_speeds = [0.0] * 4
                for i in range(4):
                    v_final = v_req[i] - v_base
                    if i == self.ref_leg_id: 
                        v_final = 0.0
                    # 比例控制：期望物理速度 = 位置偏差 * 比例控制增益 K_P
                    raw_speeds[i] = v_final * self.K_P
                
                # 2. 寻找当前四个腿中绝对速度最大的值
                max_abs_speed = max(abs(s) for s in raw_speeds)
                
                # 3. 计算等比例缩放系数 (保证四个腿协同运动，速度比例不变)
                if max_abs_speed > self.MAX_CYLINDER_SPEED:
                    scale_factor = self.MAX_CYLINDER_SPEED / max_abs_speed
                else:
                    scale_factor = 1.0
                    
                # 4. 执行限幅并转换为 PWM 硬件输出
                for i in range(4):
                    # limited_speed 单位：mm/s
                    limited_speed = raw_speeds[i] * scale_factor

                    # 直接把 mm/s 转成 r/min
                    cmd_rpm = limited_speed * 60.0 * self.reduction_ratio / self.lead_mm

                    # 限幅到驱动允许范围
                    self.legs[i]['pwm'] = max(
                        -int(self.MAX_MOTOR_RPM),
                        min(int(self.MAX_MOTOR_RPM), int(cmd_rpm))
                    )

        # 更新 UI 并下发硬件指令
        for i, leg in enumerate(self.legs):
            self.send_motor_cmd(i, leg['pwm']) # 新增：硬件输出
            percent = int((abs(leg['pwm']) / self.MAX_MOTOR_RPM) * 100)
            self.leg_bars[i]['value'] = percent
            self.leg_labels[i].set(f"{leg['pwm']} (r/min)")

        state_map = {"GROUNDING": "寻找接地点", "PRE_LIFT": "同步顶升", "LEVELING": "姿态调平", "IDLE": "待机"}

        # 只有在“非维持中”时，才用通用阶段文字覆盖
        if not (self.algo_state == "LEVELING" and self.leveling_hold_active):
            self.stage_var.set(state_map.get(self.algo_state, self.algo_state))
        
        self.sim_timer = self.root.after(self.output_period_ms, self.run_algorithm_loop)


    def update_leg_feedback(self):
        """
        读取4个电机的真实反馈，并更新：
        - self.legs[i]['pos']       (mm)
        - self.legs[i]['vel_rpm']   (r/min)
        - self.legs[i]['vel_mms']   (mm/s)
        - self.legs[i]['torque_pct'] (%)
        同时刷新右侧反馈区。
        """
        for i in range(4):
            if not self.motor_connected[i] or not self.motor_ports[i]:
                continue

            leg = self.legs[i]

            accum_count = self.update_leg_encoder_accum(i)
            if accum_count is not None:
                if leg['zero_encoder_count'] is None:
                    leg['zero_encoder_count'] = accum_count
                rel_count = accum_count - leg['zero_encoder_count']
                leg['pos'] = self.count_to_mm(rel_count)
                if i < len(self.feedback_pos_vars):
                    self.feedback_pos_vars[i].set(f"{leg['pos']:.3f} mm")

            vel_rpm = self.read_leg_velocity_rpm(i)
            if vel_rpm is not None:
                leg['vel_rpm'] = vel_rpm
                leg['vel_mms'] = self.rpm_to_mms(vel_rpm)
                if i < len(self.feedback_vel_vars):
                    self.feedback_vel_vars[i].set(f"{vel_rpm} r/min ({leg['vel_mms']:.3f} mm/s)")

            torque_pct = self.read_leg_torque_pct(i)
            if torque_pct is not None:
                leg['torque_pct'] = torque_pct
                if i < len(self.feedback_torque_vars):
                    self.feedback_torque_vars[i].set(f"{torque_pct:.1f} %")

        self.record_feedback_snapshot()

    def reset_feedback_area(self):
        """
        重置右侧反馈区：
        1）把当前位置设为新的位置零点
        2）把显示刷新为 0
        注意：运行调平/阶段调试时不允许重置
        """
        if self.sim_timer:
            messagebox.showwarning("运行中", "请先停止调平/阶段调试后再重置反馈区")
            return

        for i in range(4):
            leg = self.legs[i]

            # 已连接电机：把当前位置设为新的相对零点
            if self.motor_connected[i] and self.motor_ports[i]:
                accum_count = self.update_leg_encoder_accum(i)
                if accum_count is not None:
                    leg['zero_encoder_count'] = accum_count

                leg['pos'] = 0.0
                leg['vel_rpm'] = 0
                leg['vel_mms'] = 0.0
                leg['torque_pct'] = 0.0

                if i < len(self.feedback_pos_vars):
                    self.feedback_pos_vars[i].set("0.000 mm")
                if i < len(self.feedback_vel_vars):
                    self.feedback_vel_vars[i].set("0 r/min (0.000 mm/s)")
                if i < len(self.feedback_torque_vars):
                    self.feedback_torque_vars[i].set("0.0 %")

            # 未连接电机：显示默认占位
            else:
                leg['pos'] = 0.0
                leg['vel_rpm'] = 0
                leg['vel_mms'] = 0.0
                leg['torque_pct'] = 0.0

                if i < len(self.feedback_pos_vars):
                    self.feedback_pos_vars[i].set("--- mm")
                if i < len(self.feedback_vel_vars):
                    self.feedback_vel_vars[i].set("--- r/min (--- mm/s)")
                if i < len(self.feedback_torque_vars):
                    self.feedback_torque_vars[i].set("--- %")

        self.status_var.set("反馈区已重置：位置零点已设为当前位置")

    # ================= 原有功能函数 (保持逻辑不动) =================

    def find_highest_leg(self, ax, ay):
        if ax>=0 and ay>=0: return 1
        if ax>=0 and ay<0:  return 0
        if ax<0  and ay>=0: return 2
        return 3

    def refresh_ports(self):
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combo['values'] = ports

        for combo in self.motor_combos:
            combo['values'] = ports

        # 传感器端口：允许自动选一个默认值
        if ports:
            if (not self.port_var.get()) or (self.port_var.get() not in ports):
                self.port_var.set(ports[0])
        else:
            self.port_var.set("")

        # 电机端口：不自动分配，避免4个都落到同一个 COM
        for i in range(4):
            if self.motor_port_vars[i].get() not in ports:
                self.motor_port_vars[i].set("")

    def toggle_connection(self):
        if not self.is_connected: self.connect_serial()
        else: self.disconnect_serial()

    def connect_serial(self):
        try:
            self.serial_port = serial.Serial(
                self.port_var.get(),
                int(self.baudrate_var.get()),
                timeout=0.01
            )

            # ===== 新增：连接后先清空历史残留 =====
            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()
            self.sensor_rx_buffer = b""
            self.last_sensor_frame_time = 0.0
            self.data_count = 0
            self.last_valid_sensor_angles = {
                'x_angle': 0.0,
                'y_angle': 0.0,
                'z_angle': 0.0
            }

            self.last_fps_time = time.time()
            self.last_fps_count = 0
            self.frame_rate = 0.0

            self.is_connected = True
            self.stop_receive = False
            self.connect_btn.config(text="断开")

            self.receive_thread = threading.Thread(target=self.receive_data, daemon=True)
            self.receive_thread.start()

        except Exception as e:
            messagebox.showerror("连接错误", str(e))

    def disconnect_serial(self):
        self.stop_receive, self.is_connected = True, False
        if self.serial_port: self.serial_port.close()
        self.connect_btn.config(text="连接传感器")

    # #
    def toggle_auto_leveling(self):

        # 暂时屏蔽
        messagebox.showinfo("暂不可用", "完整自动调平依赖支腿电流和位移反馈，当前暂未接入。")
        return

        if self.sim_timer:
            self.root.after_cancel(self.sim_timer)
            self.sim_timer = None
            self.algo_state = "IDLE"
            self.debug_mode = False
            self.debug_stage = None
            self.stop_all_motors()
            self.stage_var.set("待机 (IDLE)")
            self.auto_btn.config(text="启动调平")
        else:
            if not self.is_connected:
                messagebox.showwarning("传感器未连接", "请先连接倾角传感器")
                return

            self.algo_state = "GROUNDING"
            self.mask_timer = 0
            for leg in self.legs:
                leg['grounded'] = False
                leg['start_pos'] = leg['pos']

            self.auto_btn.config(text="停止调平")
            self.run_algorithm_loop()

    ##
    def start_stage_debug(self, stage):
        # 暂时屏蔽
        if stage in ("GROUNDING", "PRE_LIFT"):
            messagebox.showinfo("暂不可用", "触地和顶升阶段依赖支腿电流/位移反馈，当前暂未接入。")
            return

        # ===== 先做基本检查，再停轮询 =====
        if stage == "LEVELING":
            if not all(self.motor_connected):
                messagebox.showwarning("电机未连接", "调平阶段需要4个电机全部连接")
                return

            if not self.is_connected:
                messagebox.showwarning("传感器未连接", "调平阶段需要先连接倾角传感器")
                return
            
            session_filename = filedialog.asksaveasfilename(
                defaultextension=".csv",
                filetypes=[("CSV files", "*.csv")],
                title="保存调平调试记录",
                initialfile=f"leveling_debug_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
            )
            if not session_filename:
                return
        else:
            if not any(self.motor_connected):
                messagebox.showwarning("电机未连接", "请先连接至少一个电机")
                return

        # 先停掉已有循环
        if self.sim_timer:
            self.root.after_cancel(self.sim_timer)
            self.sim_timer = None

        # 再停反馈轮询，避免和调试循环重复读
        self.stop_feedback_polling()

        # 先停掉全部电机输出
        for i in range(4):
            self.legs[i]['pwm'] = 0
            try:
                self.send_motor_cmd(i, 0)
            except:
                pass

        # 进入调试前，刷新一次当前位置反馈
        self.update_leg_feedback()

        self.debug_mode = True
        self.debug_stage = stage
        self.mask_timer = 0

        # ===== 新增：每次进入阶段调试前，清空“维持”状态 =====
        self.leveling_hold_start_time = None
        self.leveling_hold_active = False

        if stage == "GROUNDING":
            self.algo_state = "GROUNDING"
            for leg in self.legs:
                leg['grounded'] = False
                leg['pwm'] = 0
            self.stage_var.set("寻找接地点 (单独调试)")

        elif stage == "PRE_LIFT":
            self.algo_state = "PRE_LIFT"
            for leg in self.legs:
                leg['grounded'] = True
                leg['start_pos'] = leg['pos']
                leg['pwm'] = 0
            self.stage_var.set("同步顶升 (单独调试)")

        elif stage == "LEVELING":
            self.algo_state = "LEVELING"
            with self.sensor_lock:
                curr_ax = self.latest_angles['x_angle']
                curr_ay = self.latest_angles['y_angle']
                curr_az = self.latest_angles['z_angle']

            self.ref_leg_id = self.find_highest_leg(curr_ax, curr_ay)

            self.leveling_entry_info["entry_time"] = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
            self.leveling_entry_info["entry_x_angle"] = curr_ax
            self.leveling_entry_info["entry_y_angle"] = curr_ay
            self.leveling_entry_info["entry_z_angle"] = curr_az
            self.leveling_entry_info["entry_ref_leg"] = self.ref_leg_id + 1
            self.leveling_entry_info["entry_pos_1"] = self.legs[0]['pos']
            self.leveling_entry_info["entry_pos_2"] = self.legs[1]['pos']
            self.leveling_entry_info["entry_pos_3"] = self.legs[2]['pos']
            self.leveling_entry_info["entry_pos_4"] = self.legs[3]['pos']

            print("=== 进入LEVELING ===")
            print(f"time={self.leveling_entry_info['entry_time']}")
            print(f"x={curr_ax:.3f}, y={curr_ay:.3f}, z={curr_az:.3f}")
            print(f"ref_leg={self.ref_leg_id + 1}")
            print(
                f"pos=[{self.legs[0]['pos']:.3f}, {self.legs[1]['pos']:.3f}, {self.legs[2]['pos']:.3f}, {self.legs[3]['pos']:.3f}]"
            )

            for leg in self.legs:
                leg['grounded'] = True
                leg['start_pos'] = leg['pos']
                leg['pwm'] = 0

            if not self.start_leveling_session_recording(session_filename):
                self.debug_mode = False
                self.debug_stage = None
                self.algo_state = "IDLE"
                self.stop_all_motors(stop_reason="record_file_error")
                return

            self.record_leveling_session_snapshot(force=True, session_active=1, stop_reason="")
            self.stage_var.set("姿态调平 (单独调试)")

        self.run_algorithm_loop()

    ##
    def finish_stage_debug(self, text):
        if self.sim_timer:
            self.root.after_cancel(self.sim_timer)
            self.sim_timer = None

        self.debug_mode = False
        self.debug_stage = None
        self.algo_state = "IDLE"
        self.leveling_hold_start_time = None
        self.leveling_hold_active = False

        for i in range(4):
            self.legs[i]['pwm'] = 0
            try:
                self.send_motor_cmd(i, 0)
            except:
                pass

            self.leg_bars[i]['value'] = 0
            self.leg_labels[i].set("0 (r/min)")

        self.stage_var.set(text)
        self.auto_btn.config(text="启动调平(暂不可用)")
        self.stop_leveling_session_recording("normal_finish")

        if any(self.motor_connected):
            self.start_feedback_polling()
    
    def toggle_save_data(self):
        if not self.is_saving: self.start_save_data()
        else: self.stop_save_data()

    def start_save_data(self):
        filename = filedialog.asksaveasfilename(defaultextension=".csv")
        if not filename: return
        self.csv_file = open(filename, 'w', newline='', encoding='utf-8')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['时间戳', 'X轴', 'Y轴', 'Z轴', '帧序号'])
        self.is_saving = True; self.save_btn.config(text="停止保存")

    def stop_save_data(self):
        self.is_saving = False
        if self.csv_file: self.csv_file.close()
        self.save_btn.config(text="开始保存")

    def update_output_period(self):
        """
        更新控制输出周期
        """
        try:
            new_period_ms = int(self.output_period_var.get())
            if new_period_ms <= 0:
                messagebox.showwarning("输入错误", "输出周期必须大于 0")
                return
    
            self.output_period_ms = new_period_ms
            self.output_period_s = self.output_period_ms / 1000.0
            self.output_period_show_var.set(f"{self.output_period_ms} ms")
            self.status_var.set(f"输出周期已更新为 {self.output_period_ms} ms")

        except Exception:
            messagebox.showwarning("输入错误", "请输入有效的输出周期整数（单位 ms）")

    def update_feedback_period(self):
        """
        更新反馈轮询周期和记录周期（两者统一）
        """
        try:
            new_period_ms = int(self.feedback_period_var.get())
            if new_period_ms <= 0:
                messagebox.showwarning("输入错误", "反馈周期必须大于 0")
                return

            self.feedback_period_ms = new_period_ms
            self.feedback_period_s = self.feedback_period_ms / 1000.0
            self.feedback_record_interval = self.feedback_period_s

            self.feedback_period_show_var.set(f"{self.feedback_period_ms} ms")
            self.status_var.set(f"反馈/记录周期已更新为 {self.feedback_period_ms} ms")

            # 如果反馈轮询正在运行，立即重启一次，让新周期生效
            if self.feedback_job is not None:
                self.stop_feedback_polling()
                self.start_feedback_polling()

        except Exception:
            messagebox.showwarning("输入错误", "请输入有效的反馈周期整数（单位 ms）")

    def toggle_feedback_recording(self):
        if not self.is_feedback_recording:
            self.start_feedback_recording()
        else:
            self.stop_feedback_recording()


    def start_feedback_recording(self):
        filename = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv")],
            title="保存反馈记录"
        )
        if not filename:
            return

        try:
            self.feedback_csv_file = open(filename, 'w', newline='', encoding='utf-8-sig')
            self.feedback_csv_writer = csv.writer(self.feedback_csv_file)

            # 表头
            header = [
                "timestamp",
                "elapsed_s",
                "stage",
                "x_angle_deg",
                "y_angle_deg",
                "z_angle_deg"
            ]

            for i in range(4):
                leg_no = i + 1
                header.extend([
                    f"leg{leg_no}_connected",
                    f"leg{leg_no}_cmd_rpm",
                    f"leg{leg_no}_pos_mm",
                    f"leg{leg_no}_vel_rpm",
                    f"leg{leg_no}_vel_mms",
                    f"leg{leg_no}_torque_pct"
                ])

            self.feedback_csv_writer.writerow(header)

            self.is_feedback_recording = True
            self.feedback_record_start_time = time.time()
            self.last_feedback_record_time = 0.0

            self.feedback_save_btn.config(text="停止记录反馈")
            self.feedback_save_status_var.set("反馈记录中")
            self.feedback_path_var.set(f"反馈文件：{filename}")
            self.status_var.set(f"反馈记录已开始: {os.path.basename(filename)}")

        except Exception as e:
            messagebox.showerror("记录失败", f"无法创建反馈记录文件: {e}")


    def stop_feedback_recording(self):
        self.is_feedback_recording = False

        if self.feedback_csv_file:
            try:
                self.feedback_csv_file.close()
            except:
                pass

        self.feedback_csv_file = None
        self.feedback_csv_writer = None
        self.feedback_record_start_time = None
        self.last_feedback_record_time = 0.0

        if hasattr(self, "feedback_save_btn"):
            self.feedback_save_btn.config(text="开始记录反馈")
        if hasattr(self, "feedback_save_status_var"):
            self.feedback_save_status_var.set("反馈未记录")
        if hasattr(self, "feedback_path_var"):
            self.feedback_path_var.set("反馈文件：未选择")

    def start_leveling_session_recording(self, filename):
        """开始本次“调试调平”会话记录（独立 CSV）"""
        try:
            self.leveling_session_file = open(filename, 'w', newline='', encoding='utf-8-sig')
            self.leveling_session_writer = csv.writer(self.leveling_session_file)

            header = [
                "timestamp",
                "elapsed_s",
                "stage",
                "x_angle_deg",
                "y_angle_deg",
                "z_angle_deg",
                "entry_time",
                "entry_x_angle_deg",
                "entry_y_angle_deg",
                "entry_z_angle_deg",
                "entry_ref_leg",
                "entry_pos_1_mm",
                "entry_pos_2_mm",
                "entry_pos_3_mm",
                "entry_pos_4_mm",
                "session_active",
                "stop_reason",
            ]

            for i in range(4):
                leg_no = i + 1
                header.extend([
                    f"leg{leg_no}_connected",
                    f"leg{leg_no}_cmd_rpm",
                    f"leg{leg_no}_pos_mm",
                    f"leg{leg_no}_vel_rpm",
                    f"leg{leg_no}_vel_mms",
                    f"leg{leg_no}_torque_pct",
                ])

            self.leveling_session_writer.writerow(header)
            self.is_leveling_session_recording = True
            self.leveling_session_start_time = time.time()
            self.leveling_session_last_record_time = 0.0
            self.leveling_session_filename = filename
            self.leveling_stop_reason = ""

            self.status_var.set(f"调平调试记录已开始: {os.path.basename(filename)}")
            self.feedback_path_var.set(f"调平调试文件：{filename}")
            print(f"[LEVELING_SESSION] 保存文件: {filename}")
            return True
        except Exception as e:
            self.leveling_session_file = None
            self.leveling_session_writer = None
            self.is_leveling_session_recording = False
            self.leveling_session_start_time = None
            self.leveling_session_last_record_time = 0.0
            self.leveling_session_filename = ""
            messagebox.showerror("调平记录失败", f"无法创建调平调试记录文件: {e}")
            return False


    def record_leveling_session_snapshot(self, force=False, session_active=1, stop_reason=""):
        """把本次调平调试会话数据写入独立 CSV"""
        if not self.is_leveling_session_recording or self.leveling_session_writer is None:
            return

        now = time.time()
        if not force and self.leveling_session_last_record_time != 0.0:
            if now - self.leveling_session_last_record_time < self.feedback_record_interval:
                return

        self.leveling_session_last_record_time = now

        with self.sensor_lock:
            x_angle = self.latest_angles['x_angle']
            y_angle = self.latest_angles['y_angle']
            z_angle = self.latest_angles['z_angle']

        elapsed_s = 0.0
        if self.leveling_session_start_time is not None:
            elapsed_s = now - self.leveling_session_start_time

        row = [
            datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3],
            f"{elapsed_s:.3f}",
            self.algo_state,
            f"{x_angle:.3f}",
            f"{y_angle:.3f}",
            f"{z_angle:.3f}",
            self.leveling_entry_info["entry_time"],
            f"{self.leveling_entry_info['entry_x_angle']:.3f}",
            f"{self.leveling_entry_info['entry_y_angle']:.3f}",
            f"{self.leveling_entry_info['entry_z_angle']:.3f}",
            self.leveling_entry_info["entry_ref_leg"],
            f"{self.leveling_entry_info['entry_pos_1']:.3f}",
            f"{self.leveling_entry_info['entry_pos_2']:.3f}",
            f"{self.leveling_entry_info['entry_pos_3']:.3f}",
            f"{self.leveling_entry_info['entry_pos_4']:.3f}",
            int(session_active),
            stop_reason,
        ]

        for i in range(4):
            leg = self.legs[i]
            row.extend([
                int(self.motor_connected[i]),
                leg['pwm'],
                f"{leg['pos']:.6f}",
                leg['vel_rpm'],
                f"{leg['vel_mms']:.6f}",
                f"{leg['torque_pct']:.3f}",
            ])

        self.leveling_session_writer.writerow(row)
        if self.leveling_session_file:
            self.leveling_session_file.flush()


    def stop_leveling_session_recording(self, stop_reason):
        """结束本次“调试调平”会话记录"""
        if not self.is_leveling_session_recording:
            return

        self.leveling_stop_reason = stop_reason
        self.record_leveling_session_snapshot(force=True, session_active=0, stop_reason=stop_reason)

        if self.leveling_session_file:
            try:
                self.leveling_session_file.close()
            except:
                pass

        filename = self.leveling_session_filename
        self.leveling_session_file = None
        self.leveling_session_writer = None
        self.is_leveling_session_recording = False
        self.leveling_session_start_time = None
        self.leveling_session_last_record_time = 0.0
        self.leveling_session_filename = ""

        self.status_var.set(f"调平调试记录已结束: {stop_reason} ({os.path.basename(filename)})")
        print(f"[LEVELING_SESSION] 结束原因: {stop_reason}")

    def record_feedback_snapshot(self):
        """
        把当前4条腿的反馈写入一行 CSV
        """
        self.record_leveling_session_snapshot()

        if not self.is_feedback_recording or self.feedback_csv_writer is None:
            return

        now = time.time()

        # 限制写入频率
        if self.last_feedback_record_time != 0.0:
            if now - self.last_feedback_record_time < self.feedback_record_interval:
                return

        self.last_feedback_record_time = now

        with self.sensor_lock:
            x_angle = self.latest_angles['x_angle']
            y_angle = self.latest_angles['y_angle']
            z_angle = self.latest_angles['z_angle']

        elapsed_s = 0.0
        if self.feedback_record_start_time is not None:
            elapsed_s = now - self.feedback_record_start_time

        row = [
            datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3],
            f"{elapsed_s:.3f}",
            self.algo_state,
            f"{x_angle:.3f}",
            f"{y_angle:.3f}",
            f"{z_angle:.3f}",
        ]

        for i in range(4):
            leg = self.legs[i]
            row.extend([
                int(self.motor_connected[i]),
                leg['pwm'],
                f"{leg['pos']:.6f}",
                leg['vel_rpm'],
                f"{leg['vel_mms']:.6f}",
                f"{leg['torque_pct']:.3f}",
            ])

        self.feedback_csv_writer.writerow(row)

        # 及时刷盘，避免测试中断后丢数据
        if self.feedback_csv_file:
            self.feedback_csv_file.flush()

    def receive_data(self):
        prev_angles = {'x_angle': 0.0, 'y_angle': 0.0, 'z_angle': 0.0}

        while not self.stop_receive and self.is_connected:
            try:
                if self.serial_port and self.serial_port.in_waiting:
                    waiting_now = self.serial_port.in_waiting

                    # ===== 1. 串口积压过大：直接丢弃旧数据 =====
                    if waiting_now > self.sensor_backlog_drop_bytes:
                        print(f"[SENSOR] backlog too large: {waiting_now} bytes, drop old data")
                        self.serial_port.reset_input_buffer()
                        self.sensor_rx_buffer = b""
                        time.sleep(0.005)
                        continue

                    # ===== 2. 把当前可读数据读进本地缓冲 =====
                    read_size = max(17, min(waiting_now, 256))
                    self.sensor_rx_buffer += self.serial_port.read(read_size)

                    # 本地缓冲过长时，只保留尾部，避免越来越大
                    if len(self.sensor_rx_buffer) > self.sensor_local_buffer_keep:
                        self.sensor_rx_buffer = self.sensor_rx_buffer[-self.sensor_local_buffer_keep:]

                    frames_parsed = 0
                    last_valid_res = None

                    # ===== 3. 每轮最多处理固定数量的帧 =====
                    while len(self.sensor_rx_buffer) >= 17 and frames_parsed < self.sensor_max_frames_per_cycle:
                        idx = self.sensor_rx_buffer.find(b'\x50\x03\x0c')
                        if idx == -1:
                            self.sensor_rx_buffer = self.sensor_rx_buffer[-16:]
                            break

                        if idx > 0:
                            self.sensor_rx_buffer = self.sensor_rx_buffer[idx:]
                            continue

                        frame = self.sensor_rx_buffer[:17]
                        self.sensor_rx_buffer = self.sensor_rx_buffer[17:]

                        res, msg = self.parse_sensor_data(frame)
                        if not res:
                            continue

                        # ===== 4. 相邻有效帧突变过滤 =====
                        dx = abs(res['x_angle'] - self.last_valid_sensor_angles['x_angle'])
                        dy = abs(res['y_angle'] - self.last_valid_sensor_angles['y_angle'])
                        dz = abs(res['z_angle'] - self.last_valid_sensor_angles['z_angle'])

                        if self.last_sensor_frame_time != 0.0:
                            if dx > self.sensor_jump_reject_deg or dy > self.sensor_jump_reject_deg or dz > self.sensor_jump_reject_deg:
                                print(
                                    f"[SENSOR] jump rejected: "
                                    f"dx={dx:.3f}, dy={dy:.3f}, dz={dz:.3f}, "
                                    f"x={res['x_angle']:.3f}, y={res['y_angle']:.3f}, z={res['z_angle']:.3f}"
                                )
                                continue

                        last_valid_res = res
                        frames_parsed += 1

                    # ===== 5. 一轮只用最后一帧有效值更新控制/显示 =====
                    if last_valid_res is not None:
                        self.data_count += 1

                        with self.sensor_lock:
                            self.latest_angles['x_angle'] = last_valid_res['x_angle']
                            self.latest_angles['y_angle'] = last_valid_res['y_angle']
                            self.latest_angles['z_angle'] = last_valid_res['z_angle']

                        self.last_valid_sensor_angles = {
                            'x_angle': last_valid_res['x_angle'],
                            'y_angle': last_valid_res['y_angle'],
                            'z_angle': last_valid_res['z_angle']
                        }
                        self.last_sensor_frame_time = time.time()

                        if self.is_saving:
                            self.save_data_to_csv(last_valid_res)

                        if self.data_count % 5 == 0:
                            px = prev_angles['x_angle']
                            py = prev_angles['y_angle']
                            pz = prev_angles['z_angle']

                            self.root.after(0, self.update_display, last_valid_res, "OK")
                            self.root.after(0, self.update_trend_indicator, 'x_angle', last_valid_res['x_angle'], px)
                            self.root.after(0, self.update_trend_indicator, 'y_angle', last_valid_res['y_angle'], py)
                            self.root.after(0, self.update_trend_indicator, 'z_angle', last_valid_res['z_angle'], pz)
                            self.root.after(0, self.update_statistics)

                        prev_angles = {
                            'x_angle': last_valid_res['x_angle'],
                            'y_angle': last_valid_res['y_angle'],
                            'z_angle': last_valid_res['z_angle']
                        }

                else:
                    time.sleep(0.001)

            except Exception as e:
                self.stop_receive = True
                self.root.after(0, self.status_var.set, f"传感器接收异常: {e}")

    def parse_sensor_data(self, data):
        if len(data) != 17 or data[0:3] != b'\x50\x03\x0c':
            return None, "Err"

        def b2a(b):
            return struct.unpack('>i', b[2:4] + b[0:2])[0] / 1000.0

        try:
            x_angle = b2a(data[3:7])
            y_angle = b2a(data[7:11])
            z_angle = b2a(data[11:15])
        except Exception:
            return None, "DecodeErr"

        # ===== 新增：合法范围过滤 =====
        if abs(x_angle) > self.sensor_max_abs_x:
            return None, "RangeErrX"
        if abs(y_angle) > self.sensor_max_abs_y:
            return None, "RangeErrY"
        if abs(z_angle) > self.sensor_max_abs_z:
            return None, "RangeErrZ"

        return {
            'x_angle': x_angle,
            'y_angle': y_angle,
            'z_angle': z_angle,
            'timestamp': datetime.now()
        }, "OK"

    def update_trend_indicator(self, key, cur, prev):
        lbl = getattr(self, f"{key}_trend")
        if cur > prev: lbl.config(text="▲", fg="red")
        elif cur < prev: lbl.config(text="▼", fg="green")
        else: lbl.config(text="●", fg="gray")

    def update_statistics(self):
        now = time.time()
        dt = now - self.last_fps_time

        if dt >= 0.5:
            self.frame_rate = (self.data_count - self.last_fps_count) / dt
            self.last_fps_time = now
            self.last_fps_count = self.data_count

        self.fps_var.set(f"{self.frame_rate:.1f} Hz")
        self.count_var.set(f"{self.data_count} 帧")

    def update_display(self, data, message):
        self.angle_vars['x_angle'].set(f"{data['x_angle']:+.3f}°")
        self.angle_vars['y_angle'].set(f"{data['y_angle']:+.3f}°")
        self.angle_vars['z_angle'].set(f"{data['z_angle']:+.3f}°")

    def save_data_to_csv(self, data):
        ts = data['timestamp'].strftime('%Y%m%d%H%M%S%f')[:-3]
        self.csv_writer.writerow([ts, data['x_angle'], data['y_angle'], data['z_angle'], self.save_data_count])
        self.save_data_count += 1

    def on_closing(self):
        if self.sim_timer:
            self.root.after_cancel(self.sim_timer)
            self.sim_timer = None

        self.algo_state = "IDLE"
        self.stop_feedback_polling()
        self.stop_feedback_recording()
        self.stop_leveling_session_recording("app_close")

        for i in range(4):
            try:
                self.send_motor_cmd(i, 0)
            except:
                pass

            self.motor_connected[i] = False
            if self.motor_ports[i]:
                try:
                    self.motor_ports[i].close()
                except:
                    pass
                self.motor_ports[i] = None

        self.disconnect_serial()
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk(); app = TiltSensorMonitor(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing); root.mainloop()
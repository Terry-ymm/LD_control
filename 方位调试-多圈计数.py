import serial
import serial.tools.list_ports
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import threading
import time
import csv
from datetime import datetime


class AzimuthControllerMonitor:
    def __init__(self, root):
        self.root = root
        self.root.title("方位电机与方位编码器监控界面（单控制器单USB）")

        # ===== 窗口大小 =====
        screen_w = self.root.winfo_screenwidth()
        screen_h = self.root.winfo_screenheight()
        win_w = min(950, int(screen_w * 0.85))
        win_h = min(720, int(screen_h * 0.85))
        self.root.geometry(f"{win_w}x{win_h}")
        self.root.minsize(760, 580)

        # ===== 串口对象 =====
        self.controller_port = None
        self.controller_connected = False

        # ===== 串口互斥锁：同一个控制器串口，读写都要共用 =====
        self.controller_lock = threading.Lock()

        # ===== 方位电机发送控制 =====
        self.motor_send_value_var = tk.StringVar(value="0.1")   # 默认输入值
        self.motor_send_mode_var = tk.StringVar(value="速度")  # 先留“速度”模式，后面协议可扩展
        self.motor_last_send_var = tk.StringVar(value="---")   # 最近一次发送内容

        # ===== 方位电机安全限幅 =====
        # 1 rpm = 每分钟 1 圈，首次实验低速测试用
        self.MAX_AZIMUTH_RPM = 1.0

        # ===== 方位位置/速度单位参数 =====
        # 编码器为 32 位绝对位置；实测确认一圈对应 2147463847 count
        self.azimuth_position_ppr = 2147463847

        # 速度命令单位同样按 2147463847 count/rev 换算
        self.azimuth_velocity_ppr = 2147463847

        # ===== Modbus 地址偏移 =====
        # 0：使用字典表中 RTU 指令里的地址，例如 F0DA、F01D、F010
        # 1：如果实测读写无响应，再改成 1，例如 F0DB、F01E、F011
        self.modbus_addr_offset = 0

        # ===== 轮询状态 =====
        self.polling_thread = None
        self.stop_polling = False
        self.is_polling = False

        # ===== 轮询周期 =====
        self.poll_period_ms = 100
        self.poll_period_var = tk.StringVar(value=str(self.poll_period_ms))

        # ===== 通信配置 =====
        self.controller_port_var = tk.StringVar()
        self.controller_baud_var = tk.StringVar(value="115200")

        # ===== 方位电机显示变量 =====
        self.motor_status_var = tk.StringVar(value="未连接")
        self.motor_enable_var = tk.StringVar(value="---")
        self.motor_cmd_speed_var = tk.StringVar(value="---")
        self.motor_fb_speed_var = tk.StringVar(value="---")
        self.motor_current_var = tk.StringVar(value="---")
        self.motor_fault_var = tk.StringVar(value="---")

        # ===== 方位编码器显示变量 =====
        self.encoder_status_var = tk.StringVar(value="未连接")
        self.encoder_raw_var = tk.StringVar(value="---")
        self.encoder_single_angle_var = tk.StringVar(value="--- °")
        self.encoder_multi_angle_var = tk.StringVar(value="--- °")
        self.encoder_health_var = tk.StringVar(value="---")

        # ===== 状态栏 =====
        self.status_var = tk.StringVar(value="就绪")

        # ===== 临时调试打印去重 =====
        self.last_status_val = None
        self.last_mode_display = None

        # ===== 方位角度软件置零 =====
        # 只用于界面显示和后续运算，不写入驱动器
        self.azimuth_zero_count = 0          # 置零时记录的原始位置
        self.current_pos_count = None        # 当前原始位置

        # ===== 方位位置软件累计，用于多圈角度 =====
        self.last_single_count = None        # 上一次单圈位置
        self.zero_single_count = 0           # 置零时的单圈位置
        self.azimuth_turn_count = 0          # 相对置零点累计圈数
        self.azimuth_relative_count = 0      # 相对置零点的位置 count

        # ===== 方位角度运算用变量 =====
        self.azimuth_single_angle_deg = 0.0  # 当前单圈角度，0~360°
        self.azimuth_multi_angle_deg = 0.0   # 当前多圈累计角度，可正可负

        # ===== 匀速测试数据记录 =====
        self.test_speed_deg_s_var = tk.StringVar(value="3.0")  # 测试速度，单位 deg/s
        self.test_running = False                              # 是否正在测试
        self.test_start_time = None                            # 测试开始时间
        self.test_data = []                                    # 测试数据列表

        self.setup_ui()
        self.refresh_ports()

        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    # =========================================================
    # UI 总入口
    # =========================================================
    def setup_ui(self):
        """
        【新增】
        整个界面的总入口。
        __init__() 里面调用 self.setup_ui()，
        所以这里必须存在，否则程序启动会报 AttributeError。
        """

        # ===== 主容器 =====
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)

        # =====================================================
        # 1. 通信配置区
        # =====================================================
        config_frame = ttk.LabelFrame(main_frame, text="通信配置", padding="10")
        config_frame.pack(fill=tk.X, pady=5)

        ttk.Label(config_frame, text="控制器端口:").grid(
            row=0, column=0, padx=5, pady=5, sticky=tk.W
        )

        # 【注意】
        # refresh_ports() 里面会用到 self.controller_port_combo，
        # 所以这里必须创建它。
        self.controller_port_combo = ttk.Combobox(
            config_frame,
            textvariable=self.controller_port_var,
            width=12
        )
        self.controller_port_combo.grid(row=0, column=1, padx=5, pady=5)

        ttk.Label(config_frame, text="波特率:").grid(
            row=0, column=2, padx=5, pady=5, sticky=tk.W
        )

        ttk.Combobox(
            config_frame,
            textvariable=self.controller_baud_var,
            values=[
                "9600",
                "19200",
                "38400",
                "57600",
                "115200",
                "230400",
                "460800",
                "921600"
            ],
            width=10
        ).grid(row=0, column=3, padx=5, pady=5)

        self.controller_connect_btn = ttk.Button(
            config_frame,
            text="连接控制器",
            command=self.toggle_controller_connection
        )
        self.controller_connect_btn.grid(row=0, column=4, padx=5, pady=5)

        ttk.Button(
            config_frame,
            text="刷新端口",
            command=self.refresh_ports
        ).grid(row=0, column=5, padx=5, pady=5)

        # =====================================================
        # 2. 控制区
        # =====================================================
        self.setup_control_ui(main_frame)

        # =====================================================
        # 3. 数据显示区
        # =====================================================
        self.setup_data_ui(main_frame)

        # =====================================================
        # 4. 状态栏
        # =====================================================
        ttk.Label(
            self.root,
            textvariable=self.status_var,
            relief=tk.SUNKEN,
            anchor=tk.W
        ).pack(side=tk.BOTTOM, fill=tk.X)

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
            command=self.set_azimuth_zero
        ).pack(side=tk.LEFT, padx=8)

        # ===== 第二行：方位电机发送控制 =====
        motor_ctrl_frame = ttk.LabelFrame(ctrl_frame, text="方位电机输入信号", padding="8")
        motor_ctrl_frame.pack(fill=tk.X, pady=6)

        ttk.Label(motor_ctrl_frame, text="输入模式:").grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)

        ttk.Combobox(
            motor_ctrl_frame,
            textvariable=self.motor_send_mode_var,
            values=["速度"],   # 后面可扩展：位置、角度、扭矩等
            width=10,
            state="readonly"
        ).grid(row=0, column=1, padx=5, pady=5)

        ttk.Label(motor_ctrl_frame, text="输入值:").grid(row=0, column=2, padx=5, pady=5, sticky=tk.W)
        ttk.Entry(motor_ctrl_frame, textvariable=self.motor_send_value_var, width=10).grid(row=0, column=3, padx=5, pady=5)

        ttk.Button(
            motor_ctrl_frame,
            text="正向发送",
            command=lambda: self.send_motor_input_signal(direction=1)
        ).grid(row=0, column=4, padx=8, pady=5)

        ttk.Button(
            motor_ctrl_frame,
            text="反向发送",
            command=lambda: self.send_motor_input_signal(direction=-1)
        ).grid(row=0, column=5, padx=8, pady=5)

        ttk.Button(
            motor_ctrl_frame,
            text="停止电机",
            command=self.send_motor_stop_signal
        ).grid(row=0, column=6, padx=8, pady=5)

        ttk.Button(
            motor_ctrl_frame,
            text="使能电机",
            command=self.send_motor_enable_signal
        ).grid(row=0, column=7, padx=8, pady=5)

        ttk.Button(
            motor_ctrl_frame,
            text="失能电机",
            command=self.send_motor_disable_signal
        ).grid(row=0, column=8, padx=8, pady=5)

        ttk.Button(
            motor_ctrl_frame,
            text="故障复位",
            command=self.send_fault_reset_signal
        ).grid(row=0, column=9, padx=8, pady=5)

        ttk.Label(motor_ctrl_frame, text="最近发送:").grid(row=1, column=0, padx=5, pady=5, sticky=tk.W)
        ttk.Label(motor_ctrl_frame, textvariable=self.motor_last_send_var, font=("Arial", 11, "bold")).grid(
            row=1, column=1, columnspan=9, padx=5, pady=5, sticky=tk.W
        )

        # ===== 第三行：匀速测试模块 =====
        test_frame = ttk.LabelFrame(ctrl_frame, text="匀速测试", padding="8")
        test_frame.pack(fill=tk.X, pady=6)

        ttk.Label(test_frame, text="测试速度(deg/s):").grid(
            row=0, column=0, padx=5, pady=5, sticky=tk.W
        )

        ttk.Entry(
            test_frame,
            textvariable=self.test_speed_deg_s_var,
            width=10
        ).grid(row=0, column=1, padx=5, pady=5)

        ttk.Button(
            test_frame,
            text="开始测试",
            command=self.start_constant_speed_test
        ).grid(row=0, column=2, padx=8, pady=5)

        ttk.Button(
            test_frame,
            text="停止测试并保存",
            command=self.stop_constant_speed_test
        ).grid(row=0, column=3, padx=8, pady=5)

    def setup_data_ui(self, parent):
        data_frame = ttk.Frame(parent)
        data_frame.pack(fill=tk.BOTH, expand=True, pady=5)

        # ===== 左侧：方位电机数据 =====
        motor_frame = ttk.LabelFrame(data_frame, text="方位电机数据", padding="10")
        motor_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))

        motor_items = [
            ("状态", self.motor_status_var),
            ("使能状态", self.motor_enable_var),
            ("指令速度", self.motor_cmd_speed_var),
            ("反馈速度", self.motor_fb_speed_var),
            ("电流/转矩", self.motor_current_var),
            ("故障码", self.motor_fault_var),
        ]

        for i, (label, var) in enumerate(motor_items):
            ttk.Label(motor_frame, text=f"{label}:").grid(row=i, column=0, padx=6, pady=8, sticky=tk.W)
            ttk.Label(motor_frame, textvariable=var, font=("Arial", 12, "bold")).grid(
                row=i, column=1, padx=6, pady=8, sticky=tk.W
            )

        # ===== 右侧：方位编码器数据 =====
        encoder_frame = ttk.LabelFrame(data_frame, text="方位编码器数据", padding="10")
        encoder_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(5, 0))

        encoder_items = [
            ("状态", self.encoder_status_var),
            ("原始值", self.encoder_raw_var),
            ("单圈角度", self.encoder_single_angle_var),
            ("多圈角度", self.encoder_multi_angle_var),
            ("状态字", self.encoder_health_var),
        ]

        for i, (label, var) in enumerate(encoder_items):
            ttk.Label(encoder_frame, text=f"{label}:").grid(row=i, column=0, padx=6, pady=8, sticky=tk.W)
            ttk.Label(encoder_frame, textvariable=var, font=("Arial", 12, "bold")).grid(
                row=i, column=1, padx=6, pady=8, sticky=tk.W
            )

    
    # =========================================================
    # 串口连接
    # =========================================================
    def refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.controller_port_combo["values"] = ports

        if ports:
            if not self.controller_port_var.get() or self.controller_port_var.get() not in ports:
                self.controller_port_var.set(ports[0])
        else:
            # 【新增】
            # 没有可用串口时，清空当前选择。
            self.controller_port_var.set("")

        self.status_var.set("端口已刷新")

    def toggle_controller_connection(self):
        if not self.controller_connected:
            try:
                port_name = self.controller_port_var.get()
                baud = int(self.controller_baud_var.get())

                if not port_name:
                    messagebox.showwarning("提示", "请选择控制器端口")
                    return

                self.controller_port = serial.Serial(
                    port_name,
                    baud,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=0.05
                )

                # 【新增】清掉串口残留数据。
                self.controller_port.reset_input_buffer()
                self.controller_port.reset_output_buffer()

                self.controller_connected = True
                self.controller_connect_btn.config(text="断开控制器")

                # 重新连接后，重置调试打印去重状态
                self.last_status_val = None
                self.last_mode_display = None

                # 重新连接后，重置位置累计状态
                self.azimuth_zero_count = 0
                self.current_pos_count = None
                self.last_single_count = None
                self.zero_single_count = 0
                self.azimuth_turn_count = 0
                self.azimuth_relative_count = 0
                self.azimuth_single_angle_deg = 0.0
                self.azimuth_multi_angle_deg = 0.0

                self.motor_status_var.set("已连接")
                self.encoder_status_var.set("已连接")
                self.status_var.set(f"控制器已连接: {port_name}")

            except Exception as e:
                try:
                    if self.controller_port:
                        self.controller_port.close()
                except Exception:
                    pass

                self.controller_port = None
                self.controller_connected = False
                self.controller_connect_btn.config(text="连接控制器")

                self.motor_status_var.set("未连接")
                self.encoder_status_var.set("未连接")

                messagebox.showerror("连接失败", f"控制器连接失败: {e}")
        else:
            # =====================================================
            # 断开控制器
            # =====================================================

            # 1. 先通知轮询线程停止
            self.stop_polling = True
            # 断开前先失能电机
            try:
                if self.controller_connected and self.controller_port:
                    self.send_azimuth_motor_command(mode="disable", value=0)
            except Exception:
                pass

            # 2. 等待轮询线程退出一小段时间
            # 这样可以降低“线程正在读串口，但主线程已经关闭串口”的风险。
            try:
                if self.polling_thread and self.polling_thread.is_alive():
                    self.polling_thread.join(timeout=0.3)
            except Exception:
                pass

            # 4. 加锁关闭串口
            # 读写串口都用同一把锁 controller_lock，
            # 关闭串口时也加锁，避免和读写操作抢资源。
            try:
                with self.controller_lock:
                    if self.controller_port:
                        self.controller_port.close()
                        self.controller_port = None
            except Exception:
                pass

            self.controller_connected = False
            self.is_polling = False

            # 5. 刷新界面状态
            self.controller_connect_btn.config(text="连接控制器")

            self.motor_status_var.set("未连接")
            self.encoder_status_var.set("未连接")
            self.status_var.set("控制器已断开")

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

        if not self.controller_connected or not self.controller_port:
            messagebox.showwarning("提示", "请先连接控制器")
            return

        self.stop_polling = False
        self.is_polling = True
        self.polling_thread = threading.Thread(target=self.polling_loop, daemon=True)
        self.polling_thread.start()
        self.status_var.set("开始轮询读取")

    def stop_polling_task(self):
        """
        停止轮询读取。
        这里只通知线程退出，不关闭串口。
        """
        if not self.is_polling:
            self.status_var.set("当前未在轮询")
            return

        if self.test_running:
            messagebox.showwarning("提示", "当前正在匀速测试，请先点击“停止测试并保存”")
            return

        self.stop_polling = True
        self.status_var.set("正在停止轮询...")

    def polling_loop(self):
        while not self.stop_polling:
            try:
                # 【新增】
                # 如果已经断开控制器，就退出轮询。
                if not self.controller_connected or not self.controller_port:
                    break

                data = self.read_controller_data()

                if data is not None:
                    self.root.after(0, self.update_display, data)

            except Exception as e:
                self.root.after(0, self.status_var.set, f"读取异常: {e}")

            time.sleep(self.poll_period_ms / 1000.0)

        # 【新增】
        # 线程退出时，同步轮询状态。
        self.is_polling = False

        # 如果窗口还存在，更新状态栏
        try:
            if self.root.winfo_exists():
                self.root.after(0, self.status_var.set, "已停止轮询")
        except Exception:
            pass

    def send_motor_input_signal(self, direction):
        """
        direction = 1  -> 正向
        direction = -1 -> 反向
        """
        if not self.controller_connected or not self.controller_port:
            messagebox.showwarning("提示", "请先连接控制器")
            return

        try:
            value = float(self.motor_send_value_var.get())
        except Exception:
            messagebox.showwarning("输入错误", "请输入有效的数值")
            return

        mode = self.motor_send_mode_var.get().strip()

        if mode != "速度":
            messagebox.showwarning("提示", f"当前暂未实现模式: {mode}")
            return

        # 先取绝对值，再限制到最大 1 rpm
        value = abs(value)
        value = max(0.0, min(self.MAX_AZIMUTH_RPM, value))

        # 根据按钮方向决定正负
        send_value = value if direction > 0 else -value

        # 直接发送速度命令
        ok = self.send_azimuth_motor_command(mode="speed", value=send_value)

        if ok:
            self.motor_last_send_var.set(f"速度命令: {send_value}")

            self.status_var.set(f"已发送方位电机速度命令: {send_value}")
        else:
            self.status_var.set("发送失败")


    def send_motor_stop_signal(self):
        """
        发送停止指令。
        """
        if not self.controller_connected or not self.controller_port:
            messagebox.showwarning("提示", "请先连接控制器")
            return

        ok = self.send_azimuth_motor_command(mode="stop", value=0)

        if ok:
            self.motor_last_send_var.set("停止命令")

            self.status_var.set("已发送停止命令")
        else:
            self.status_var.set("停止命令发送失败")

    def send_motor_enable_signal(self):
        """
        单独发送使能指令。
        """
        if not self.controller_connected or not self.controller_port:
            messagebox.showwarning("提示", "请先连接控制器")
            return

        ok = self.send_azimuth_motor_command(mode="enable", value=0)

        if ok:
            self.motor_last_send_var.set("使能命令")
            self.status_var.set("已发送使能命令")
        else:
            self.status_var.set("使能命令发送失败")


    def send_motor_disable_signal(self):
        """
        单独发送失能指令。
        """
        if not self.controller_connected or not self.controller_port:
            messagebox.showwarning("提示", "请先连接控制器")
            return

        ok = self.send_azimuth_motor_command(mode="disable", value=0)

        if ok:
            self.motor_last_send_var.set("失能命令")
            self.status_var.set("已发送失能命令")
        else:
            self.status_var.set("失能命令发送失败")

    def send_fault_reset_signal(self):
        """
       发送故障复位指令。
        """
        if not self.controller_connected or not self.controller_port:
            messagebox.showwarning("提示", "请先连接控制器")
            return

        ok = self.send_azimuth_motor_command(mode="fault_reset", value=0)

        if ok:
            self.motor_last_send_var.set("故障复位命令")
            self.status_var.set("已发送故障复位命令")
        else:
            self.status_var.set("故障复位发送失败")

    def start_constant_speed_test(self):
        """
        开始匀速测试：
        1. 清空旧测试数据；
        2. 记录开始时间；
        3. 按 deg/s 换算 rpm；
        4. 下发速度命令。
        """
        if not self.controller_connected or not self.controller_port:
            messagebox.showwarning("提示", "请先连接控制器")
            return
        
        if not self.is_polling:
            messagebox.showwarning("提示", "请先点击“开始读取”，否则无法记录测试数据")
            return
        
        if self.test_running:
            messagebox.showwarning("提示", "当前已经在测试中，请先停止测试")
            return

        try:
            speed_deg_s = float(self.test_speed_deg_s_var.get())
        except Exception:
            messagebox.showwarning("输入错误", "请输入有效的测试速度，例如 3.0")
            return

        if speed_deg_s == 0:
            messagebox.showwarning("输入错误", "测试速度不能为 0")
            return

        # deg/s -> rpm
        # 1 rpm = 360 deg / 60 s = 6 deg/s
        speed_rpm = speed_deg_s / 6.0

        # 受当前安全限幅限制
        if abs(speed_rpm) > self.MAX_AZIMUTH_RPM:
            messagebox.showwarning(
                "速度超限",
                f"当前测试速度 {speed_deg_s:.3f} deg/s = {speed_rpm:.3f} rpm，"
                f"超过限幅 {self.MAX_AZIMUTH_RPM:.3f} rpm"
            )
            return

        self.test_data = []

        ok = self.send_azimuth_motor_command(mode="speed", value=speed_rpm)

        if ok:
            self.test_start_time = time.time()
            self.test_running = True
            self.status_var.set(f"开始匀速测试：{speed_deg_s:.3f} deg/s ({speed_rpm:.3f} rpm)")
            self.motor_last_send_var.set(f"匀速测试: {speed_deg_s:.3f} deg/s")
        else:
            self.test_start_time = None
            self.test_running = False
            self.status_var.set("匀速测试启动失败")


    def stop_constant_speed_test(self):
        """
        停止匀速测试：
        1. 停止记录；
        2. 失能电机；
        3. 保存 CSV 文件。
        """
        was_running = self.test_running
        self.test_running = False

        # 停止测试时失能电机：先清目标速度，再写失能控制字
        if self.controller_connected and self.controller_port:
            self.send_azimuth_motor_command(mode="disable", value=0)

        if not was_running and not self.test_data:
            self.status_var.set("当前没有测试数据需要保存")
            return

        if not self.test_data:
            self.test_start_time = None
            self.status_var.set("测试已停止，但没有记录到有效数据")
            return

        default_name = datetime.now().strftime("azimuth_test_%Y%m%d_%H%M%S.csv")

        file_path = filedialog.asksaveasfilename(
            title="保存测试数据",
            defaultextension=".csv",
            initialfile=default_name,
            filetypes=[("CSV 文件", "*.csv"), ("所有文件", "*.*")]
        )

        if not file_path:
            self.test_start_time = None
            self.status_var.set("测试已停止，未保存文件")
            return

        try:
            with open(file_path, "w", newline="", encoding="utf-8-sig") as f:
                writer = csv.writer(f)
                writer.writerow([
                    "time_s",
                    "multi_angle_deg",
                    "single_angle_deg",
                    "fb_speed_rpm"
                ])

                for row in self.test_data:
                    fb_speed_rpm = row["fb_speed_rpm"]

                    writer.writerow([
                        f"{row['time_s']:.4f}",
                        f"{row['multi_angle_deg']:.6f}",
                        f"{row['single_angle_deg']:.6f}",
                        "" if fb_speed_rpm is None else f"{fb_speed_rpm:.6f}"
                    ])

            self.test_start_time = None
            self.status_var.set(f"测试已停止并保存：{file_path}")

        except Exception as e:
            messagebox.showerror("保存失败", f"测试数据保存失败：{e}")
            self.status_var.set("测试数据保存失败")

    def modbus_crc16(self, data):
        """标准 Modbus RTU CRC16，小端返回"""
        crc = 0xFFFF
        for pos in data:
            crc ^= pos
            for _ in range(8):
                if crc & 1:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return crc.to_bytes(2, byteorder="little")
    
    def get_modbus_addr(self, base_addr):
        """
        根据地址偏移返回实际 RTU 地址。
        默认 offset=0，若实测无响应，可把 self.modbus_addr_offset 改成 1。
        """
        return base_addr + self.modbus_addr_offset


    def read_holding_registers(self, addr, reg_count, slave_id=0x01):
        """
        03 功能码：读取保持寄存器
        addr: 起始地址
        reg_count: 读取寄存器数量
        返回: 数据区 bytes，不含从站地址、功能码、CRC
        """
        if not self.controller_connected or not self.controller_port:
            return None

        packet = bytearray([
            slave_id,
            0x03,
            (addr >> 8) & 0xFF,
            addr & 0xFF,
            (reg_count >> 8) & 0xFF,
            reg_count & 0xFF
        ])

        packet += self.modbus_crc16(packet)

        try:
            with self.controller_lock:
                self.controller_port.reset_input_buffer()
                self.controller_port.write(packet)
                self.controller_port.flush()

                expected_len = 3 + reg_count * 2 + 2
                response = self.controller_port.read(expected_len)

            if len(response) != expected_len:
                print(f"[READ_LEN_ERR] addr=0x{addr:04X}, response={response.hex(' ')}")
                return None

            if response[0] != slave_id or response[1] != 0x03:
                print(f"[READ_HEAD_ERR] addr=0x{addr:04X}, response={response.hex(' ')}")
                return None

            if response[2] != reg_count * 2:
                print(f"[READ_BYTECOUNT_ERR] addr=0x{addr:04X}, response={response.hex(' ')}")
                return None

            data_without_crc = response[:-2]
            recv_crc = response[-2:]
            calc_crc = self.modbus_crc16(data_without_crc)

            if recv_crc != calc_crc:
                print(f"[READ_CRC_ERR] addr=0x{addr:04X}, response={response.hex(' ')}")
                return None

            return response[3:-2]

        except Exception as e:
            print(f"[READ_ERR] addr=0x{addr:04X}, {e}")
            return None


    def bytes_to_int16(self, raw, signed=True):
        if raw is None or len(raw) != 2:
            return None
        return int.from_bytes(raw, byteorder="big", signed=signed)


    def bytes_to_int32_normal(self, raw, signed=True):
        """
        32 位数据正常顺序解析：Aa Bb Cc Dd
        用于速度目标值、反馈速度等先按厂家指令验证。
        """
        if raw is None or len(raw) != 4:
            return None
        return int.from_bytes(raw, byteorder="big", signed=signed)

    def ps_to_rpm(self, value_p_s):
        """
        P/s -> rpm
        速度单位单独使用 azimuth_velocity_ppr
        """
        return value_p_s * 60.0 / self.azimuth_velocity_ppr


    def count_to_angle_deg(self, count_value):
        """
        编码器累计脉冲 -> 多圈角度，单位 deg
        位置角度换算使用 azimuth_position_ppr
        """
        return count_value * 360.0 / self.azimuth_position_ppr


    def count_to_single_turn_deg(self, count_value):
        """
        编码器累计脉冲 -> 单圈角度，范围 0~360 deg
        """
        one_turn_count = self.azimuth_position_ppr
        single_count = count_value % one_turn_count
        return single_count * 360.0 / one_turn_count

    def build_write_single_register_packet(self, addr, value, slave_id=0x01):
        """
        06 功能码：写单个保持寄存器
        addr: Modbus 地址
        value: 16位写入值
        """
        value = int(value) & 0xFFFF

        packet = bytearray([
            slave_id,
            0x06,
            (addr >> 8) & 0xFF,
            addr & 0xFF,
            (value >> 8) & 0xFF,
            value & 0xFF
        ])

        packet += self.modbus_crc16(packet)
        return packet


    def build_write_two_registers_packet(self, addr, value, signed=True, slave_id=0x01):
        """
        10 功能码：写两个保持寄存器，也就是 32 位数据
        """
        value = int(value)

        # 先按标准大端生成 4 个字节
        data_bytes = value.to_bytes(
            4,
            byteorder="big",
            signed=signed
        )

        packet = bytearray([
            slave_id,
            0x10,
            (addr >> 8) & 0xFF,
            addr & 0xFF,
            0x00,
            0x02,
            0x04
        ]) + data_bytes

        packet += self.modbus_crc16(packet)
        return packet


    def build_motor_command_packets(self, mode, value):
        """
        构造方位驱动器 Modbus RTU 报文列表。

        mode:
            'fault_reset' -> 清目标速度并故障复位
            'enable'      -> 设置速度模式并使能运行
            'speed'       -> 写目标速度并刷新运行使能
            'stop'        -> 目标速度置 0
            'disable'     -> 目标速度置 0 并失能电机

        value:
            输入速度，单位 rpm
        """
        slave_id = 0x01
        encoder_ppr = self.azimuth_velocity_ppr

        # 地址偏移：
        # ===== 修改点1：统一使用全局地址偏移量，避免读写地址错位 =====
        addr_offset = self.modbus_addr_offset

        ADDR_CONTROLWORD = 0xF002 + addr_offset         # 0x6040 (控制字)
        ADDR_MODE = 0xF009 + addr_offset                # 0x6060 (运行模式)
        ADDR_TARGET_VEL = 0xF0DA + addr_offset          # 0x60FF (目标速度)

        packets = []

        if mode == "fault_reset":
            # 先清目标速度
            packets.append(
                self.build_write_two_registers_packet(
                    addr=ADDR_TARGET_VEL,
                    value=0,
                    signed=True,
                    slave_id=slave_id
                )
            )

            # 再故障复位
            packets.append(
               self.build_write_single_register_packet(
                    addr=ADDR_CONTROLWORD,
                    value=0x0080,
                    slave_id=slave_id
                )
            )

            return packets
        
        elif mode == "enable":
            # 1. 设置速度模式：Modes_of_operation = 3
            packets.append(
                self.build_write_single_register_packet(
                    addr=ADDR_MODE,
                    value=0x0003,
                    slave_id=slave_id
                )
            )

            # 2. Shutdown：进入 Ready to switch on
            packets.append(
                self.build_write_single_register_packet(
                    addr=ADDR_CONTROLWORD,
                    value=0x0006,
                    slave_id=slave_id
                )
            )

            # 3. Switch on
            packets.append(
                self.build_write_single_register_packet(
                    addr=ADDR_CONTROLWORD,
                    value=0x0007,
                    slave_id=slave_id
                )
            )

            # 4. Enable operation
            packets.append(
                self.build_write_single_register_packet(
                    addr=ADDR_CONTROLWORD,
                    value=0x000F,
                    slave_id=slave_id
                )
            )

            return packets

        elif mode == "speed":
            rpm = float(value)
            rpm = max(-self.MAX_AZIMUTH_RPM, min(self.MAX_AZIMUTH_RPM, rpm))

            # 1. 写目标速度：rpm -> P/s
            target_velocity_p_s = int(round(rpm * encoder_ppr / 60.0))
            packets.append(
                self.build_write_two_registers_packet(
                    addr=ADDR_TARGET_VEL,
                    value=target_velocity_p_s,
                    signed=True,
                    slave_id=slave_id
                )
            )

            # 写目标速度后，再刷新一次 Enable operation
            packets.append(
                self.build_write_single_register_packet(
                    addr=ADDR_CONTROLWORD,
                    value=0x000F,
                    slave_id=slave_id
                )
            )

            return packets

        elif mode == "stop":
            # 停止速度：只把目标速度置 0，不写 0x010F，避免进入 Halt 状态
            packets.append(
                self.build_write_two_registers_packet(
                    addr=ADDR_TARGET_VEL,
                    value=0,
                    signed=True,
                    slave_id=slave_id
                )
            )

            return packets

        elif mode == "disable":
            # 停止速度
            packets.append(
                self.build_write_two_registers_packet(
                    addr=ADDR_TARGET_VEL,
                    value=0,
                    signed=True,
                    slave_id=slave_id
                )
            )

            # 失能电机：Controlword = 6
            packets.append(
                self.build_write_single_register_packet(
                    addr=ADDR_CONTROLWORD,
                    value=0x0006,
                    slave_id=slave_id
                )
            )

            return packets

        else:
            print(f"未知模式: {mode}")
            return None


    def send_azimuth_motor_command(self, mode, value):
        """
        按速度模式发送方位驱动器命令。
        一次命令可能包含多帧 Modbus RTU 报文，逐帧发送。
        """
        if not self.controller_connected or not self.controller_port:
            print("控制器未连接")
            return False

        packets = self.build_motor_command_packets(mode, value)

        if not packets:
            print("构造报文失败")
            return False

        try:
            with self.controller_lock:
                for packet in packets:
                    self.controller_port.reset_input_buffer()
                    self.controller_port.write(packet)
                    self.controller_port.flush()

                    # 06 和 10 功能码正常写入应答都是 8 字节
                    response = self.controller_port.read(8)

                    if len(response) != 8:
                        print(f"[WRITE_LEN_ERR] send={packet.hex(' ')} resp={response.hex(' ')}")
                        return False

                    data_without_crc = response[:-2]
                    recv_crc = response[-2:]
                    calc_crc = self.modbus_crc16(data_without_crc)

                    if recv_crc != calc_crc:
                        print(f"[WRITE_CRC_ERR] send={packet.hex(' ')} resp={response.hex(' ')}")
                        return False
                    
                    # 确认应答对应本次写入请求
                    if response[0] != packet[0] or response[1] != packet[1]:
                        print(f"[WRITE_HEAD_ERR] send={packet.hex(' ')} resp={response.hex(' ')}")
                        return False

                    # 06 功能码：应答应回显地址和值
                    if packet[1] == 0x06:
                        if response[2:6] != packet[2:6]:
                            print(f"[WRITE_ECHO_ERR] send={packet.hex(' ')} resp={response.hex(' ')}")
                            return False

                    # 10 功能码：应答应回显地址和寄存器数量
                    elif packet[1] == 0x10:
                        if response[2:6] != packet[2:6]:
                            print(f"[WRITE_ECHO_ERR] send={packet.hex(' ')} resp={response.hex(' ')}")
                            return False

                    # 打印每一帧写入应答，确认驱动器真的回应了
                    print(f"[WRITE_OK] send={packet.hex(' ')} resp={response.hex(' ')}")

                    time.sleep(0.03)

            if mode == "speed":
                self.motor_cmd_speed_var.set(f"{value} rpm")
            elif mode == "stop":
                self.motor_cmd_speed_var.set("0 rpm")

            print(f"已发送 {mode} 命令: {value if mode == 'speed' else ''}")
            return True

        except Exception as e:
            print(f"发送异常: {e}")
            return False

    # 协议
    def read_controller_data(self):
        """
        读取方位驱动器数据：
        1. 目标速度 Target_velocity
        2. 反馈速度 Velocity_actual_value
        3. 编码器位置 Position_actual_value，并换算成角度
        4. 状态字 Statusword (用于判断使能和故障)
        """
        if not self.controller_connected or not self.controller_port:
            return None

        # ===== 地址来自 Modbus 字典 RTU 指令列 =====
        ADDR_TARGET_VEL = self.get_modbus_addr(0xF0DA)   # 0x60FF，速度目标值，长度2
        ADDR_ACTUAL_VEL = self.get_modbus_addr(0xF01D)   # 0x606C，当前速度，长度2
        ADDR_ACTUAL_POS = self.get_modbus_addr(0xF010)   # 0x6064，当前位置，长度2
        ADDR_STATUSWORD = self.get_modbus_addr(0xF003)   # 0x6041，状态字，长度1
        ADDR_MODE_DISPLAY = self.get_modbus_addr(0xF00A)   # 0x6061，实际运行模式显示

        # 目标速度、反馈速度：先按厂家指令正常字节序解析
        # 1. 读取目标速度
        raw_target_vel = self.read_holding_registers(ADDR_TARGET_VEL, 2)
        target_vel_p_s = self.bytes_to_int32_normal(raw_target_vel, signed=True)

        # 2. 读取反馈速度
        raw_actual_vel = self.read_holding_registers(ADDR_ACTUAL_VEL, 2)
        actual_vel_p_s = self.bytes_to_int32_normal(raw_actual_vel, signed=True)

        # 3. 读取编码器位置：按正常字节序解析，不交换高低位
        raw_actual_pos = self.read_holding_registers(ADDR_ACTUAL_POS, 2)
        actual_pos_count = self.bytes_to_int32_normal(raw_actual_pos, signed=True)

        if actual_pos_count is not None:
            self.current_pos_count = actual_pos_count

            one_turn_count = self.azimuth_position_ppr
            current_single_count = actual_pos_count % one_turn_count

            # 用 1/4 圈和 3/4 圈作为跨圈判断阈值
            low_threshold = one_turn_count * 0.25
            high_threshold = one_turn_count * 0.75

            if self.last_single_count is None:
                # 第一次读取只记录单圈位置，不判断跨圈
                self.last_single_count = current_single_count
            else:
                # 正向跨圈：例如 359° -> 0°
                if self.last_single_count > high_threshold and current_single_count < low_threshold:
                    self.azimuth_turn_count += 1

                # 反向跨圈：例如 0° -> 359°
                elif self.last_single_count < low_threshold and current_single_count > high_threshold:
                    self.azimuth_turn_count -= 1

                self.last_single_count = current_single_count

        # ===== 修改点3：读取并解析状态字 0x6041 =====
        raw_status = self.read_holding_registers(ADDR_STATUSWORD, 1)
        status_val = self.bytes_to_int16(raw_status, signed=False)

        if raw_status is not None and status_val is not None:
            if status_val != self.last_status_val:
                print(f"[STATUS] raw={raw_status.hex(' ')} value=0x{status_val:04X}")
                self.last_status_val = status_val

        raw_mode_display = self.read_holding_registers(ADDR_MODE_DISPLAY, 1)
        mode_display = self.bytes_to_int16(raw_mode_display, signed=True)

        if raw_mode_display is not None and mode_display is not None:
            if mode_display != self.last_mode_display:
                print(f"[MODE_DISPLAY] raw={raw_mode_display.hex(' ')} value={mode_display}")
                self.last_mode_display = mode_display

        enable_text = "读取失败"
        fault_text = "---"
        
        if status_val is not None:
            # 判断第3位 (Fault位)
            if (status_val & 0x0008) != 0:
                fault_text = "有故障 (Fault)"
            else:
                fault_text = "无故障"

            # 按照 CiA 402 掩码判断状态
            if (status_val & 0x004F) == 0x0008:
                enable_text = "故障状态 (Fault)"
            elif (status_val & 0x006F) == 0x0027:
                enable_text = "已使能 (Operation Enabled)"
            elif (status_val & 0x004F) == 0x0040:
                enable_text = "待机中 (Switch on disabled)"
            elif (status_val & 0x006F) == 0x0021:
                enable_text = "准备好 (Ready to switch on)"
            else:
                enable_text = f"状态码: 0x{status_val:04X}"


        # ===== 数据换算 =====
        if target_vel_p_s is not None:
            target_rpm = self.ps_to_rpm(target_vel_p_s)
            target_speed_text = f"{target_rpm:.3f} rpm ({target_vel_p_s} P/s)"
        else:
            target_speed_text = "---"

        if actual_vel_p_s is not None:
            actual_rpm = self.ps_to_rpm(actual_vel_p_s)
            actual_speed_text = f"{actual_rpm:.3f} rpm ({actual_vel_p_s} P/s)"
        else:
            actual_rpm = None
            actual_speed_text = "---"

        if actual_pos_count is not None:
            # 原始值仍然显示驱动器返回的绝对位置
            raw_pos_text = f"{actual_pos_count} P"

            one_turn_count = self.azimuth_position_ppr
            current_single_count = actual_pos_count % one_turn_count

            # 当前单圈相对置零点的位置
            relative_single_count = current_single_count - self.zero_single_count

            # 多圈累计位置 = 圈数累计 + 当前单圈相对零点位置
            relative_pos_count = self.azimuth_turn_count * one_turn_count + relative_single_count

            # 保存给后续运算调用
            self.azimuth_relative_count = relative_pos_count
            self.azimuth_multi_angle_deg = self.count_to_angle_deg(relative_pos_count)
            self.azimuth_single_angle_deg = self.count_to_single_turn_deg(relative_pos_count)

            # 匀速测试数据记录
            if self.test_running and self.test_start_time is not None:
                elapsed_time = time.time() - self.test_start_time

                self.test_data.append({
                    "time_s": elapsed_time,
                    "multi_angle_deg": self.azimuth_multi_angle_deg,
                    "single_angle_deg": self.azimuth_single_angle_deg,
                    "fb_speed_rpm": actual_rpm
                })

            single_angle_text = f"{self.azimuth_single_angle_deg:.3f} °"
            multi_angle_text = f"{self.azimuth_multi_angle_deg:.3f} °"
        else:
            raw_pos_text = "---"
            single_angle_text = "--- °"
            multi_angle_text = "--- °"

        return {
            "motor": {
                "status": "已连接",
                "enable": enable_text,
                "cmd_speed": target_speed_text,
                "fb_speed": actual_speed_text,
                "current": "待补电流",
                "fault": fault_text,
            },
            "encoder": {
                "status": "已连接",
                "raw": raw_pos_text,
                "single_angle": single_angle_text,
                "multi_angle": multi_angle_text,
                "health": "正常" if status_val is not None else "读取失败",
            }
        }

    # =========================================================
    # 刷新显示
    # =========================================================
    def update_display(self, data):
        motor = data.get("motor", {})
        encoder = data.get("encoder", {})

        self.motor_status_var.set(str(motor.get("status", "---")))
        self.motor_enable_var.set(str(motor.get("enable", "---")))
        self.motor_cmd_speed_var.set(str(motor.get("cmd_speed", "---")))
        self.motor_fb_speed_var.set(str(motor.get("fb_speed", "---")))
        self.motor_current_var.set(str(motor.get("current", "---")))
        self.motor_fault_var.set(str(motor.get("fault", "---")))

        self.encoder_status_var.set(str(encoder.get("status", "---")))
        self.encoder_raw_var.set(str(encoder.get("raw", "---")))
        self.encoder_single_angle_var.set(str(encoder.get("single_angle", "---")))
        self.encoder_multi_angle_var.set(str(encoder.get("multi_angle", "---")))
        self.encoder_health_var.set(str(encoder.get("health", "---")))

    def set_azimuth_zero(self):
        """
        方位角度软件置零。
        只记录当前原始位置作为零点，不写驱动器参数。
        """
        if self.current_pos_count is None:
            messagebox.showwarning("提示", "当前还没有读取到方位位置，无法置零")
            return

        # 当前原始位置作为新的软件零点
        self.azimuth_zero_count = self.current_pos_count

        # 记录置零时的单圈位置
        one_turn_count = self.azimuth_position_ppr
        self.zero_single_count = self.current_pos_count % one_turn_count

        # 置零后，从当前位置重新开始累计圈数
        self.last_single_count = self.zero_single_count
        self.azimuth_turn_count = 0
        self.azimuth_relative_count = 0
        self.azimuth_single_angle_deg = 0.0
        self.azimuth_multi_angle_deg = 0.0

        # 立即刷新界面显示
        self.encoder_single_angle_var.set("0.000 °")
        self.encoder_multi_angle_var.set("0.000 °")

        self.status_var.set(f"方位角度已置零，零点原始值: {self.azimuth_zero_count}")

    def clear_display(self):
        self.motor_status_var.set("已连接" if self.controller_connected else "未连接")
        self.motor_enable_var.set("---")
        self.motor_cmd_speed_var.set("---")
        self.motor_fb_speed_var.set("---")
        self.motor_current_var.set("---")
        self.motor_fault_var.set("---")

        self.encoder_status_var.set("已连接" if self.controller_connected else "未连接")
        self.encoder_raw_var.set("---")
        self.encoder_single_angle_var.set("--- °")
        self.encoder_multi_angle_var.set("--- °")
        self.encoder_health_var.set("---")

        self.status_var.set("显示已清空")

    def on_closing(self):
        """
        窗口关闭时的安全退出逻辑。
        重点：
        1. 先通知轮询线程退出；
        2. 再把连接状态置为 False；
        3. 尽量等待轮询线程结束；
        4. 最后加锁关闭串口；
        5. 销毁窗口。
        """

        self.stop_polling = True
        self.test_running = False

        # 1. 通知轮询线程退出
        # 关闭窗口前先失能电机
        try:
            if self.controller_connected and self.controller_port:
                self.send_azimuth_motor_command(mode="disable", value=0)
        except Exception:
            pass

        # 2. 等待轮询线程退出一小段时间
        try:
            if self.polling_thread and self.polling_thread.is_alive():
                self.polling_thread.join(timeout=0.3)
        except Exception:
            pass

        # 3. 加锁关闭串口，避免和读写线程抢同一个串口
        try:
            with self.controller_lock:
                if self.controller_port:
                    self.controller_port.close()
                    self.controller_port = None
        except Exception:
            pass

        self.controller_connected = False
        self.is_polling = False

        # 4. 销毁窗口
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = AzimuthControllerMonitor(root)
    root.mainloop()
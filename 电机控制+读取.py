import serial               # 导入串口通信库，负责与硬件数据交互
import serial.tools.list_ports # 用于自动扫描电脑当前可用的 COM 端口
import tkinter as tk        # Python 自带的 GUI 库，用于创建窗口界面
from tkinter import ttk, messagebox 
import struct               # 关键库：负责将 Python 的整数转换为 Modbus 协议要求的二进制格式

# ================================================================
# 核心类：电动缸控制器+调平系统
# ================================================================
class FullRangeMotorController:
    def __init__(self, root):
        self.root = root
        self.root.title("电动缸全速域控制终端 (自动填充规则)") # 界面标题
        self.root.geometry("500x600")                     # 窗口初始大小
        
        self.serial_port = None                    # 初始化串口对象变量
        self.is_connected = False          # 记录当前是否已连接电机
        self.zero_encoder_count = None   # 相对位置零点（软件累计计数）
        self.encoder_accum_count = 0     # 软件累计位置（跨圈累加）
        self.last_single_turn = None     # 上一次读取到的单圈值
        self.encoder_track_job = None    # 后台位置跟踪任务
        self.lead_mm = 5.0          # 丝杆导程，单位 mm
        self.reduction_ratio = 5.0  # 减速比
        
        self.setup_ui()                    # 初始化用户界面布局
        self.refresh_ports()               # 程序启动时自动扫描一次端口

        self.debug_modbus = False   # 是否打印Modbus读写日志

    # ----------------------------------------------------------------
    # 1. 界面布局部分 (UI Setup)
    # ----------------------------------------------------------------
    def setup_ui(self):
        """构建图形化操作界面"""
        # --- 通信配置区 ---
        conn_frame = ttk.LabelFrame(self.root, text="第一步：通信配置", padding="10")
        conn_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Label(conn_frame, text="串口选择:").grid(row=0, column=0)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(conn_frame, textvariable=self.port_var, width=12)
        self.port_combo.grid(row=0, column=1, padx=5)
        
        self.baud_var = tk.StringVar(value="115200") # 默认波特率 115200
        ttk.Combobox(conn_frame, textvariable=self.baud_var, values=["9600", "115200"], width=8).grid(row=0, column=2, padx=5)
        
        self.conn_btn = ttk.Button(conn_frame, text="连接电机", command=self.toggle_connection)
        self.conn_btn.grid(row=0, column=3, padx=5)

        # --- 速度控制区 ---
        ctrl_frame = ttk.LabelFrame(self.root, text="第二步：速度控制 (范围: ±500)", padding="15")
        ctrl_frame.pack(fill=tk.X, padx=10, pady=10)

        # 交互式滑块
        ttk.Label(ctrl_frame, text="滑动调节 (实时同步数值):").pack(anchor=tk.W)
        self.speed_slider = tk.Scale(ctrl_frame, from_=-500, to=500, orient=tk.HORIZONTAL, 
                                   length=400, tickinterval=250, command=self.sync_slider_to_entry)
        self.speed_slider.pack(pady=5)
        self.speed_slider.set(0) # 初始速度为 0

        # 精确输入与发送
        input_frame = ttk.Frame(ctrl_frame)
        input_frame.pack(pady=10)
        ttk.Label(input_frame, text="目标速度:").pack(side=tk.LEFT)
        self.speed_entry_var = tk.StringVar(value="0")
        self.speed_entry = ttk.Entry(input_frame, textvariable=self.speed_entry_var, width=10)
        self.speed_entry.pack(side=tk.LEFT, padx=5)
        
        ttk.Button(input_frame, text="下发指令", command=self.send_custom_speed).pack(side=tk.LEFT, padx=5)
        ttk.Button(input_frame, text="急停(0)", command=self.emergency_stop).pack(side=tk.LEFT, padx=5)

        # --- 报文监视区 (Debug 窗口) ---
        rule_frame = ttk.LabelFrame(self.root, text="第三步：报文监视 (Hex Log)", padding="10")
        rule_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        # 实时显示填充规则提醒
        rule_tips = "规则：负数 9-14位填充 FF | 正数/零 9-14位填充 00"
        ttk.Label(rule_frame, text=rule_tips, foreground="blue", font=("Arial", 8)).pack(anchor=tk.W)
        
        self.hex_log_var = tk.StringVar(value="等待发送...")
        # 使用只读 Entry 显示报文，方便用户直接复制核对
        log_entry = ttk.Entry(rule_frame, textvariable=self.hex_log_var, state='readonly', font=("Consolas", 9))
        log_entry.pack(fill=tk.X, pady=10)

        # ==============================
        # 【新增】反馈显示区
        # ==============================
        fb_frame = ttk.LabelFrame(self.root, text="第四步：状态反馈", padding="10")
        fb_frame.pack(fill=tk.X, padx=10, pady=5)

        # 显示相对位置（mm）
        self.rel_pos_var = tk.StringVar(value="相对位置: 未置零")
        ttk.Label(fb_frame, textvariable=self.rel_pos_var).pack(anchor=tk.W)

        # 显示转矩
        self.torque_var = tk.StringVar(value="转矩: ---")
        ttk.Label(fb_frame, textvariable=self.torque_var).pack(anchor=tk.W)

        # 显示实际速度（先保留你当前这个显示位）
        self.vel_var = tk.StringVar(value="速度: ---")
        ttk.Label(fb_frame, textvariable=self.vel_var).pack(anchor=tk.W)

        # 按钮区
        btn_frame = ttk.Frame(fb_frame)
        btn_frame.pack(pady=5)

        ttk.Button(btn_frame, text="读取反馈", command=self.update_feedback).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="当前位置置零", command=self.set_zero_position).pack(side=tk.LEFT, padx=5)

    # ----------------------------------------------------------------
    # 2. 核心算法部分 (Protocol Logic)
    # ----------------------------------------------------------------
    def calculate_crc(self, data):
        """
        [工业标准] Modbus RTU CRC16 计算
        该函数会根据输入的数据，自动生成 2 字节的小端格式校验码
        """
        crc = 0xFFFF
        for pos in data:
            crc ^= pos
            for _ in range(8):
                if (crc & 1) != 0:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return struct.pack('<H', crc) # 返回结果，如 0x878A -> \x8A\x87
    
    def read_modbus_holding_registers(self, start_addr, reg_count, slave_id=0x01):
        """
        【新增】Modbus RTU 读保持寄存器（03h）

        参数说明：
        start_addr : 起始寄存器地址（例如 0x600F）
        reg_count  : 读取寄存器数量
        slave_id   : 从站地址，当前默认 1

        返回：
        成功 -> 仅返回“数据区”字节（bytes）
        失败 -> None
        """
        if not self.is_connected:
            return None

        try:
            # --------------------------------
            # 1. 构造 Modbus 03h 读寄存器请求帧
            # 格式：
            # [从站地址][03][起始地址H][起始地址L][寄存器数H][寄存器数L][CRC_L][CRC_H]
            # --------------------------------
            packet = bytearray([
                slave_id,
                0x03,
                (start_addr >> 8) & 0xFF,
                start_addr & 0xFF,
                (reg_count >> 8) & 0xFF,
                reg_count & 0xFF
            ])

            crc = self.calculate_crc(packet)
            full_packet = packet + crc

            # --------------------------------
            # 2. 清空输入缓冲区，避免残留回包干扰
            # --------------------------------
            self.serial_port.reset_input_buffer()

            # --------------------------------
            # 3. 发送请求
            # --------------------------------
            self.serial_port.write(full_packet)

            # --------------------------------
            # 4. 读取响应
            # 正常响应长度：
            # [从站][功能码][字节数][数据...][CRC2字节]
            # 总长度 = 3 + (2*寄存器数) + 2
            # --------------------------------
            expected_len = 3 + reg_count * 2 + 2
            response = self.serial_port.read(expected_len)

            # 调试打印
            if self.debug_modbus:
                print("读取发送:", full_packet.hex())
                print("读取回包:", response.hex())

            if len(response) != expected_len:
                print("读取失败：回包长度不对")
                return None

            # --------------------------------
            # 5. 基本格式检查
            # --------------------------------
            if response[0] != slave_id:
                print("读取失败：从站地址不匹配")
                return None

            if response[1] != 0x03:
                print("读取失败：功能码不是 03h")
                return None

            byte_count = response[2]
            if byte_count != reg_count * 2:
                print("读取失败：数据字节数不匹配")
                return None

            # --------------------------------
            # 6. CRC 校验
            # --------------------------------
            data_without_crc = response[:-2]
            recv_crc = response[-2:]
            calc_crc = self.calculate_crc(data_without_crc)

            if recv_crc != calc_crc:
                print("读取失败：CRC 校验错误")
                return None

            # --------------------------------
            # 7. 返回纯数据区
            # --------------------------------
            return response[3:-2]

        except Exception as e:
            print("Modbus读取异常:", e)
            return None

    def read_encoder_single_turn(self):
        """
        读取编码器单圈数据（4202h）
        2个寄存器，包含 L/M/H 三个字节

        返回：
        int，范围 0 ~ 0x7FFFFF
        """
        raw = self.read_modbus_holding_registers(0x4202, 2)

        if raw is None or len(raw) != 4:
            print("单圈数据读取失败")
            return None

        # 按文档里的 L / M / H 顺序先组合：
        # 单圈数据 = H * 0x10000 + M * 0x100 + L
        # 先按 raw[0]=L, raw[1]=M, raw[2]=H 处理
        # 如果后续方向相反，再只做符号方向调整，不先改字节顺序
        byte_l = raw[0]
        byte_m = raw[1]
        byte_h = raw[3]

        value = (byte_h << 16) | (byte_m << 8) | byte_l

        # 单圈数据是23bit
        value &= 0x7FFFFF
        return value

    def update_encoder_accum(self):
        """
        根据单圈数据做软件跨圈累计
        适用于 Pr0.15=1 时不能可靠使用多圈数据的情况
        """
        single_turn = self.read_encoder_single_turn()
        if single_turn is None:
            return None

        # 第一次读取：初始化基准
        if self.last_single_turn is None:
            self.last_single_turn = single_turn
            self.encoder_accum_count = 0
            return self.encoder_accum_count

        delta = single_turn - self.last_single_turn

        # 单圈范围：23bit -> 0 ~ 2^23-1
        one_turn = 1 << 23
        half_turn = one_turn // 2

        # 处理跨圈跳变
        if delta > half_turn:
            delta -= one_turn
        elif delta < -half_turn:
            delta += one_turn

        self.encoder_accum_count += delta
        self.last_single_turn = single_turn

        return self.encoder_accum_count
    
    def start_encoder_tracking(self):
        """
        启动后台单圈位置跟踪
        """
        if self.encoder_track_job is None:
            self._encoder_track_loop()


    def stop_encoder_tracking(self):
        """
        停止后台单圈位置跟踪
        """
        if self.encoder_track_job is not None:
            self.root.after_cancel(self.encoder_track_job)
            self.encoder_track_job = None


    def _encoder_track_loop(self):
        """
        每隔一小段时间自动读取一次 4202h，
        用于避免手动点击间隔过长导致跨圈判断出错
        """
        if not self.is_connected:
            self.encoder_track_job = None
            return

        # 后台刷新累计位置
        self.update_encoder_accum()

        # 先用 20ms；如果后面你觉得太频繁，可以改成 30~50ms
        self.encoder_track_job = self.root.after(50, self._encoder_track_loop)

    def set_zero_position(self):
        """
        将当前位置设为相对零点
        """
        # 如果后台还没采到任何单圈值，先尝试采一次
        if self.last_single_turn is None:
            current = self.update_encoder_accum()
            if current is None:
                messagebox.showerror("置零失败", "无法读取编码器单圈数据")
                return

        self.zero_encoder_count = self.encoder_accum_count
        self.rel_pos_var.set("相对位置: 0 count")
        print(f"已置零，zero_encoder_count = {self.zero_encoder_count}")


    def read_relative_position(self):
        """
        读取相对位置（软件累计计数）
        """
        if self.zero_encoder_count is None:
            return None

        if self.last_single_turn is None:
            return None

        return self.encoder_accum_count - self.zero_encoder_count
    
    def count_to_mm(self, count_value):
        """
        编码器累计 count -> 电动缸位移 mm
        单圈 23bit = 2^23 count
        电机转1圈位移 = lead_mm / reduction_ratio
        """
        mm_per_motor_rev = self.lead_mm / self.reduction_ratio
        mm_per_count = mm_per_motor_rev / (1 << 23)
        return count_value * mm_per_count


    def rpm_to_mms(self, rpm_value):
        """
        电机速度 r/min -> 电动缸速度 mm/s
        """
        mm_per_motor_rev = self.lead_mm / self.reduction_ratio
        return rpm_value * mm_per_motor_rev / 60.0

    def read_torque(self):
        """
        【重写】通过 Modbus 读取内部指令转矩（力矩反馈）

        寄存器：
        6025h = Torque demand
        长度：1 个寄存器（2 字节）
        单位：0.1%

        返回：
        float（单位：%）
        """
        raw = self.read_modbus_holding_registers(0x6025, 1)

        if raw is None:
            return None

        if len(raw) != 2:
            print("转矩读取失败：数据长度不是2字节")
            return None

        # 16位有符号整数，单位 0.1%
        value_raw = struct.unpack('>h', raw)[0]

        # 转成百分比
        value_percent = value_raw / 10.0
        return value_percent
    
    def read_velocity(self):
        """
        通过 Modbus 读取电机实际速度

        寄存器：
        4025h = Velocity actual value
        长度：1 个寄存器（2 字节）
        单位：r/min
        """
        raw = self.read_modbus_holding_registers(0x4025, 1)

        if raw is None:
            return None

        if len(raw) != 2:
            print("速度读取失败：数据长度不是2字节")
            return None

        # 16位有符号整数，单位 r/min
        value = struct.unpack('>h', raw)[0]
        return value

    def send_custom_speed(self):
        """
        [最核心逻辑] 构造符合特定填充规则的报文并发送
        """
        if not self.is_connected:
            messagebox.showwarning("警告", "串口未连接，请先点击连接电机")
            return
        
        try:
            speed = int(self.speed_entry_var.get()) # 获取用户输入的速度
            
            # A. 构造报文头部 (站号1 | 写多寄存器 | 起始地址3308 | 寄存器数2 | 数据字节数8)
            packet = bytearray([0x01, 0x10, 0x33, 0x08, 0x00, 0x02, 0x08])
            
            # B. 加入速度值 (2字节大端格式有符号整数)
            # 例如: 500 -> 01 F4 | -500 -> FE 0C
            packet.extend(struct.pack('>h', speed))
            
            # C. [关键] 执行自动填充逻辑
            # 根据用户发现的规律：负数补 FF，正数及零补 00
            if speed < 0:
                packet.extend(b'\xFF\xFF\xFF\xFF\xFF\xFF') # 填充 6 个 FF
            else:
                packet.extend(b'\x00\x00\x00\x00\x00\x00') # 填充 6 个 00
            
            # D. 计算并添加 CRC 校验码
            crc = self.calculate_crc(packet)
            full_packet = packet + crc  # 拼接成最终的 17 字节指令
            
            # # E. 物理下发至串口
            # self.serial_port.write(full_packet)
            
            # # F. 更新界面日志 (显示十六进制大写字母)
            # self.hex_log_var.set(full_packet.hex(' ').upper())

            # E. 发送前先清空接收缓冲区
            # 目的：把上一次通信残留的回包清掉，避免污染本次测试
            self.serial_port.reset_input_buffer()

            # F. 物理下发至串口
            self.serial_port.write(full_packet)

            # G. 读取这次“速度写入”对应的回包
            # 这里只是为了调试先看驱动器回了什么，不代表最终协议已经完全定型
            reply = self.serial_port.read(32)

            # H. 在控制台打印十六进制，便于你发给我分析
            print("速度发送:", full_packet.hex())
            print("速度回包:", reply.hex())

            # I. 更新界面日志（显示发送报文）
            self.hex_log_var.set(full_packet.hex(' ').upper())
            
        except ValueError:
            messagebox.showerror("格式错误", "请输入有效的整数 (范围: -500 到 500)")

    # ----------------------------------------------------------------
    # 3. 辅助功能部分 (Utility Functions)
    # ----------------------------------------------------------------
    def refresh_ports(self):
        """自动刷新当前电脑可用的串口列表"""
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo['values'] = ports
        if ports: self.port_var.set(ports[0])

    def toggle_connection(self):
        """切换串口连接/断开状态"""
        if self.is_connected:
            self.stop_encoder_tracking()
            self.is_connected = False
            if self.serial_port:
                self.serial_port.close()
                self.serial_port = None
            self.zero_encoder_count = None
            self.encoder_accum_count = 0
            self.last_single_turn = None
            self.conn_btn.config(text="连接电机")
        else:
            try:
                # 以当前界面选择的波特率打开串口
                self.serial_port = serial.Serial(
                    self.port_var.get(),
                    int(self.baud_var.get()),
                    timeout=0.2
                )
                self.is_connected = True
                self.zero_encoder_count = None   # 新连接后清除旧零点
                self.encoder_accum_count = 0
                self.last_single_turn = None
                self.conn_btn.config(text="断开连接")
                self.start_encoder_tracking()
            except Exception as e:
                messagebox.showerror("连接失败", f"无法打开串口: {str(e)}")

    def sync_slider_to_entry(self, val):
        """滑块变动时，同步更新文本框数值"""
        self.speed_entry_var.set(val)

    def emergency_stop(self):
        """快速归零并停止"""
        self.speed_slider.set(0)
        self.speed_entry_var.set("0")
        self.send_custom_speed()

    def update_feedback(self):
        """
        刷新界面上的相对位置(mm)、转矩(%)、速度(r/min + mm/s)反馈
        """
        rel_pos_count = self.read_relative_position()
        torque = self.read_torque()
        vel_rpm = self.read_velocity()

        # -----------------------------
        # 更新相对位置显示（mm）
        # -----------------------------
        if self.zero_encoder_count is None:
            self.rel_pos_var.set("相对位置: 未置零")
        elif rel_pos_count is not None:
            rel_pos_mm = self.count_to_mm(rel_pos_count)
            self.rel_pos_var.set(f"相对位置: {rel_pos_mm:.6f} mm")
        else:
            self.rel_pos_var.set("相对位置: 读取失败")

        # -----------------------------
        # 更新转矩显示
        # -----------------------------
        if torque is not None:
            self.torque_var.set(f"转矩: {torque:.1f}%")
        else:
            self.torque_var.set("转矩: 读取失败")

        # -----------------------------
        # 更新速度显示（r/min + mm/s）
        # -----------------------------
        if vel_rpm is not None:
            vel_mms = self.rpm_to_mms(vel_rpm)
            self.vel_var.set(f"速度: {vel_rpm} r/min ({vel_mms:.4f} mm/s)")
        else:
            self.vel_var.set("速度: 读取失败")

# ================================================================
# 程序入口
# ================================================================
if __name__ == "__main__":
    root = tk.Tk()
    app = FullRangeMotorController(root)
    # 增加窗口关闭时的清理逻辑
    def on_closing():
        if app.is_connected:
            app.stop_encoder_tracking()
            if app.serial_port:
                app.serial_port.close()
                app.serial_port = None
            app.is_connected = False
        root.destroy()
    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()
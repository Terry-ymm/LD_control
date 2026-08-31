# -*- coding: utf-8 -*-
"""方位执行器：由“方位调试-多圈计数(9).py”提取到总控使用。

保留内容：
- 已验证的 CiA402/Modbus 速度模式收发；
- 已验证的 32 位绝对位置正常字节序解析；
- 软件置零与多圈累计角度；
- 方位速度反馈、状态字显示；
- 方向约定：逆时针为正，顺时针为负。

不包含内容：
- Tkinter 调试界面；
- 匀速测试 CSV 记录。
"""
from __future__ import annotations

import math
import threading
import time
from typing import Optional

import serial

from common import (
    bytes_to_int16,
    bytes_to_int32_normal,
    clamp,
    modbus_crc16,
)


class AzimuthExecutor:
    def __init__(self):
        self.port = None
        self.connected = False
        self.lock = threading.Lock()

        # 方位方向约定：正值 = 逆时针，负值 = 顺时针。
        self.positive_direction = "CCW"

        # 与调试成功版本保持一致。
        self.max_rpm = 1.0
        self.position_ppr = 2147463847
        self.velocity_ppr = 2147463847
        self.modbus_addr_offset = 0

        # 如果后续 MPC 输出的是方位角速度 rad/s，可用此传动比换算电机 rpm。
        # 当前默认 1.0，表示机构 1 rev = 电机 1 rev；实际可后续标定。
        self.motor_gear_ratio = 1.0

        # 调试打印去重。
        self.last_status_val = None
        # self.last_mode_display = None

        # 软件置零和多圈累计状态。
        self.azimuth_zero_count = 0
        self.current_pos_count = None
        self.last_single_count = None
        self.zero_single_count = 0
        self.azimuth_turn_count = 0
        self.azimuth_relative_count = 0
        self.azimuth_single_angle_deg = 0.0
        self.azimuth_multi_angle_deg = 0.0

        self.state = {
            "cmd_rpm": 0.0,
            "fb_rpm": 0.0,
            "target_p_s": 0,
            "actual_p_s": 0,
            "raw_pos": None,
            "relative_count": 0,
            "single_angle_deg": 0.0,
            "multi_angle_deg": 0.0,
            "statusword": None,
            "enable_text": "---",
            "fault_text": "---",
            "connected": False,
        }

    # =========================================================
    # 连接 / 断开
    # =========================================================
    def connect(self, port_name: str, baudrate: int = 115200):
        if not port_name:
            raise ValueError("方位控制器端口为空")

        self.port = serial.Serial(
            port_name,
            int(baudrate),
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.05,
        )
        self.port.reset_input_buffer()
        self.port.reset_output_buffer()

        self.connected = True
        self.state["connected"] = True
        self.state["enable_text"] = "已连接"
        self.last_status_val = None
        self.reset_angle_accumulator()

    def disconnect(self):
        try:
            if self.connected:
                self.send_motor_command("disable", 0.0)
        except Exception:
            pass

        try:
            with self.lock:
                if self.port:
                    self.port.close()
        finally:
            self.port = None
            self.connected = False
            self.state["connected"] = False
            self.state["cmd_rpm"] = 0.0
            self.state["enable_text"] = "未连接"

    def reset_angle_accumulator(self):
        """重新连接或需要重置缓存时使用；不会写驱动器。"""
        self.azimuth_zero_count = 0
        self.current_pos_count = None
        self.last_single_count = None
        self.zero_single_count = 0
        self.azimuth_turn_count = 0
        self.azimuth_relative_count = 0
        self.azimuth_single_angle_deg = 0.0
        self.azimuth_multi_angle_deg = 0.0
        self.state["raw_pos"] = None
        self.state["relative_count"] = 0
        self.state["single_angle_deg"] = 0.0
        self.state["multi_angle_deg"] = 0.0

    # =========================================================
    # Modbus 基础
    # =========================================================
    def az_addr(self, base_addr: int) -> int:
        return base_addr + self.modbus_addr_offset

    def build_write_single_register_packet(self, addr: int, value: int, slave_id: int = 0x01) -> bytes:
        value = int(value) & 0xFFFF
        packet = bytearray([
            slave_id,
            0x06,
            (addr >> 8) & 0xFF,
            addr & 0xFF,
            (value >> 8) & 0xFF,
            value & 0xFF,
        ])
        return bytes(packet + modbus_crc16(packet))

    def build_write_two_registers_packet(
        self,
        addr: int,
        value: int,
        signed: bool = True,
        slave_id: int = 0x01,
    ) -> bytes:
        data_bytes = int(value).to_bytes(4, byteorder="big", signed=signed)
        packet = bytearray([
            slave_id,
            0x10,
            (addr >> 8) & 0xFF,
            addr & 0xFF,
            0x00,
            0x02,
            0x04,
        ]) + data_bytes
        return bytes(packet + modbus_crc16(packet))

    def write_packet(self, packet: bytes) -> bool:
        if not self.connected or not self.port:
            return False

        try:
            with self.lock:
                self.port.reset_input_buffer()
                self.port.write(packet)
                self.port.flush()
                response = self.port.read(8)

            if len(response) != 8:
                print(f"[AZ_WRITE_LEN_ERR] send={packet.hex(' ')} resp={response.hex(' ')}")
                return False

            if response[-2:] != modbus_crc16(response[:-2]):
                print(f"[AZ_WRITE_CRC_ERR] send={packet.hex(' ')} resp={response.hex(' ')}")
                return False

            if response[0] != packet[0] or response[1] != packet[1]:
                print(f"[AZ_WRITE_HEAD_ERR] send={packet.hex(' ')} resp={response.hex(' ')}")
                return False

            if response[2:6] != packet[2:6]:
                print(f"[AZ_WRITE_ECHO_ERR] send={packet.hex(' ')} resp={response.hex(' ')}")
                return False

            return True
        except Exception as exc:
            print(f"[AZ_WRITE_ERR] {exc}")
            return False

    def read_holding_registers(self, addr: int, reg_count: int, slave_id: int = 0x01) -> Optional[bytes]:
        if not self.connected or not self.port:
            return None

        packet = bytearray([
            slave_id,
            0x03,
            (addr >> 8) & 0xFF,
            addr & 0xFF,
            (reg_count >> 8) & 0xFF,
            reg_count & 0xFF,
        ])
        packet += modbus_crc16(packet)

        try:
            with self.lock:
                self.port.reset_input_buffer()
                self.port.write(packet)
                self.port.flush()
                expected_len = 3 + reg_count * 2 + 2
                response = self.port.read(expected_len)

            if len(response) != expected_len:
                return None
            if response[0] != slave_id or response[1] != 0x03:
                return None
            if response[2] != reg_count * 2:
                return None
            if response[-2:] != modbus_crc16(response[:-2]):
                return None

            return response[3:-2]
        except Exception as exc:
            print(f"[AZ_READ_ERR] addr=0x{addr:04X}: {exc}")
            return None

    # =========================================================
    # 方位电机命令
    # =========================================================
    def build_motor_command_packets(self, mode: str, value: float):
        slave_id = 0x01
        encoder_ppr = self.velocity_ppr

        addr_controlword = self.az_addr(0xF002)   # 0x6040
        addr_mode = self.az_addr(0xF009)          # 0x6060
        addr_target_vel = self.az_addr(0xF0DA)    # 0x60FF

        packets = []

        if mode == "fault_reset":
            packets.append(self.build_write_two_registers_packet(addr_target_vel, 0, signed=True, slave_id=slave_id))
            packets.append(self.build_write_single_register_packet(addr_controlword, 0x0080, slave_id=slave_id))
            return packets

        if mode == "enable":
            # 与调试成功版本保持一致。
            packets.append(self.build_write_single_register_packet(addr_mode, 0x0003, slave_id=slave_id))
            packets.append(self.build_write_single_register_packet(addr_controlword, 0x0006, slave_id=slave_id))
            packets.append(self.build_write_single_register_packet(addr_controlword, 0x0007, slave_id=slave_id))
            packets.append(self.build_write_single_register_packet(addr_controlword, 0x000F, slave_id=slave_id))
            return packets

        if mode == "speed":
            rpm = clamp(float(value), -self.max_rpm, self.max_rpm)
            target_velocity_p_s = int(round(rpm * encoder_ppr / 60.0))
            packets.append(self.build_write_two_registers_packet(addr_target_vel, target_velocity_p_s, signed=True, slave_id=slave_id))
            packets.append(self.build_write_single_register_packet(addr_controlword, 0x000F, slave_id=slave_id))
            return packets

        if mode == "stop":
            # 调试成功版本：停止只清目标速度，不写 0x010F，避免进入 Halt 状态。
            packets.append(self.build_write_two_registers_packet(addr_target_vel, 0, signed=True, slave_id=slave_id))
            return packets

        if mode == "disable":
            packets.append(self.build_write_two_registers_packet(addr_target_vel, 0, signed=True, slave_id=slave_id))
            packets.append(self.build_write_single_register_packet(addr_controlword, 0x0006, slave_id=slave_id))
            return packets

        print(f"[AZ_UNKNOWN_MODE] {mode}")
        return None

    def send_motor_command(self, mode: str, value: float = 0.0) -> bool:
        packets = self.build_motor_command_packets(mode, value)
        if not packets:
            return False

        for packet in packets:
            if not self.write_packet(packet):
                return False
            time.sleep(0.03)

        if mode == "speed":
            self.state["cmd_rpm"] = clamp(float(value), -self.max_rpm, self.max_rpm)
        elif mode in ("stop", "disable"):
            self.state["cmd_rpm"] = 0.0

        return True

    def enable(self) -> bool:
        return self.send_motor_command("enable", 0.0)

    def disable(self) -> bool:
        return self.send_motor_command("disable", 0.0)

    def fault_reset(self) -> bool:
        return self.send_motor_command("fault_reset", 0.0)

    def set_speed(self, rpm: float) -> bool:
        """总控按 rpm 调用。rpm > 0 为逆时针，rpm < 0 为顺时针。"""
        if not self.connected:
            return False
        rpm = clamp(float(rpm), -self.max_rpm, self.max_rpm)
        ok = self.send_motor_command("speed", rpm)
        if ok:
            self.state["cmd_rpm"] = rpm
        return ok

    def set_azimuth_rate(self, azimuth_rate_rad_s: float) -> bool:
        """
        后续 MPC 接口用：方位角速度 rad/s -> 方位电机 rpm。
        正值为逆时针。
        """
        rpm = float(azimuth_rate_rad_s) * 60.0 / (2.0 * math.pi) * self.motor_gear_ratio
        return self.set_speed(rpm)

    def stop(self):
        if self.connected:
            self.send_motor_command("stop", 0.0)

    # =========================================================
    # 角度、多圈、置零
    # =========================================================
    def ps_to_rpm(self, value_p_s: int | float) -> float:
        return float(value_p_s) * 60.0 / self.velocity_ppr

    def count_to_angle_deg(self, count_value: int | float) -> float:
        return float(count_value) * 360.0 / self.position_ppr

    def count_to_single_turn_deg(self, count_value: int | float) -> float:
        single_count = float(count_value) % self.position_ppr
        return single_count * 360.0 / self.position_ppr

    def set_zero(self) -> bool:
        """方位角度软件置零，只影响显示和总控运算，不写驱动器。"""
        if self.current_pos_count is None:
            return False

        self.azimuth_zero_count = self.current_pos_count
        one_turn_count = self.position_ppr
        self.zero_single_count = self.current_pos_count % one_turn_count
        self.last_single_count = self.zero_single_count
        self.azimuth_turn_count = 0
        self.azimuth_relative_count = 0
        self.azimuth_single_angle_deg = 0.0
        self.azimuth_multi_angle_deg = 0.0

        self.state["relative_count"] = 0
        self.state["single_angle_deg"] = 0.0
        self.state["multi_angle_deg"] = 0.0
        return True

    def _update_multiturn_from_position(self, actual_pos_count: int):
        self.current_pos_count = actual_pos_count

        one_turn_count = self.position_ppr
        current_single_count = actual_pos_count % one_turn_count

        low_threshold = one_turn_count * 0.25
        high_threshold = one_turn_count * 0.75

        if self.last_single_count is None:
            self.last_single_count = current_single_count
        else:
            # 正向跨圈：359° -> 0°。
            if self.last_single_count > high_threshold and current_single_count < low_threshold:
                self.azimuth_turn_count += 1
            # 反向跨圈：0° -> 359°。
            elif self.last_single_count < low_threshold and current_single_count > high_threshold:
                self.azimuth_turn_count -= 1

            self.last_single_count = current_single_count

        relative_single_count = current_single_count - self.zero_single_count
        relative_pos_count = self.azimuth_turn_count * one_turn_count + relative_single_count

        self.azimuth_relative_count = relative_pos_count
        self.azimuth_multi_angle_deg = self.count_to_angle_deg(relative_pos_count)
        self.azimuth_single_angle_deg = self.count_to_single_turn_deg(relative_pos_count)

        self.state["raw_pos"] = actual_pos_count
        self.state["relative_count"] = relative_pos_count
        self.state["single_angle_deg"] = self.azimuth_single_angle_deg
        self.state["multi_angle_deg"] = self.azimuth_multi_angle_deg

    # =========================================================
    # 反馈读取
    # =========================================================
    def update_feedback(self):
        if not self.connected or not self.port:
            return

        addr_target_vel = self.az_addr(0xF0DA)     # 0x60FF
        addr_actual_vel = self.az_addr(0xF01D)     # 0x606C
        addr_actual_pos = self.az_addr(0xF010)     # 0x6064
        addr_statusword = self.az_addr(0xF003)     # 0x6041

        raw_target = self.read_holding_registers(addr_target_vel, 2)
        target_p_s = bytes_to_int32_normal(raw_target, signed=True)

        raw_actual_vel = self.read_holding_registers(addr_actual_vel, 2)
        actual_p_s = bytes_to_int32_normal(raw_actual_vel, signed=True)

        # 调试成功版本：当前位置按正常字节序解析，不交换高低字。
        raw_actual_pos = self.read_holding_registers(addr_actual_pos, 2)
        actual_pos_count = bytes_to_int32_normal(raw_actual_pos, signed=True)

        raw_status = self.read_holding_registers(addr_statusword, 1)
        status_val = bytes_to_int16(raw_status, signed=False)

        # raw_mode_display = self.read_holding_registers(addr_mode_display, 1)
        # mode_display = bytes_to_int16(raw_mode_display, signed=True)

        if target_p_s is not None:
            self.state["target_p_s"] = target_p_s

        if actual_p_s is not None:
            self.state["actual_p_s"] = actual_p_s
            self.state["fb_rpm"] = self.ps_to_rpm(actual_p_s)

        if actual_pos_count is not None:
            self._update_multiturn_from_position(actual_pos_count)

        if status_val is not None:
            self.state["statusword"] = status_val
            self.state["fault_text"] = "有故障(Fault)" if (status_val & 0x0008) else "无故障"

            if status_val != self.last_status_val:
                print(f"[AZ_STATUS] value=0x{status_val:04X}")
                self.last_status_val = status_val

            if (status_val & 0x004F) == 0x0008:
                enable_text = "故障状态(Fault)"
            elif (status_val & 0x006F) == 0x0027:
                enable_text = "已使能(Operation Enabled)"
            elif (status_val & 0x004F) == 0x0040:
                enable_text = "待机中(Switch on disabled)"
            elif (status_val & 0x006F) == 0x0021:
                enable_text = "准备好(Ready to switch on)"
            else:
                enable_text = f"状态码: 0x{status_val:04X}"
            self.state["enable_text"] = enable_text

        # if mode_display is not None:
        #     self.state["mode_display"] = mode_display
        #     if mode_display != self.last_mode_display:
        #         print(f"[AZ_MODE_DISPLAY] value={mode_display}")
        #         self.last_mode_display = mode_display

    def get_state(self) -> dict:
        out = dict(self.state)
        out["connected"] = self.connected
        out["positive_direction"] = self.positive_direction
        out["current_pos_count"] = self.current_pos_count
        out["azimuth_relative_count"] = self.azimuth_relative_count
        return out

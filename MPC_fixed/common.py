# -*- coding: utf-8 -*-
"""公共工具：Modbus CRC、限幅、单位换算、编码器累计等。"""
from __future__ import annotations

import struct
from typing import Optional


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


def modbus_crc16(data: bytes | bytearray) -> bytes:
    """标准 Modbus RTU CRC16，小端返回。"""
    crc = 0xFFFF
    for pos in data:
        crc ^= pos
        for _ in range(8):
            if crc & 1:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return struct.pack("<H", crc)


def build_cylinder_speed_packet(rpm_val: float, max_rpm: float) -> tuple[bytes, int]:
    """
    构造调平/举升电动缸速度帧。

    注意：这里沿用你实验中已经验证可用的 0x3308 速度帧，不按通用 Modbus
    写多寄存器格式重新解释。
    """
    phys_speed = int(rpm_val)
    phys_speed = int(clamp(phys_speed, -int(max_rpm), int(max_rpm)))

    payload = bytearray([0x01, 0x10, 0x33, 0x08, 0x00, 0x02, 0x08])
    payload.extend(struct.unpack("BB", struct.pack(">h", phys_speed)))
    if phys_speed < 0:
        payload.extend(b"\xFF\xFF\xFF\xFF\xFF\xFF")
    else:
        payload.extend(b"\x00\x00\x00\x00\x00\x00")

    return bytes(payload + modbus_crc16(payload)), phys_speed


def read_holding_registers(port, lock, connected: bool, start_addr: int, reg_count: int, slave_id: int = 0x01) -> Optional[bytes]:
    """03 功能码读取保持寄存器，返回数据区 bytes。"""
    if not connected or not port:
        return None

    packet = bytearray([
        slave_id,
        0x03,
        (start_addr >> 8) & 0xFF,
        start_addr & 0xFF,
        (reg_count >> 8) & 0xFF,
        reg_count & 0xFF,
    ])
    full_packet = packet + modbus_crc16(packet)

    try:
        with lock:
            port.reset_input_buffer()
            port.write(full_packet)
            expected_len = 3 + reg_count * 2 + 2
            response = port.read(expected_len)

        if len(response) != expected_len:
            return None
        if response[0] != slave_id or response[1] != 0x03 or response[2] != reg_count * 2:
            return None
        if response[-2:] != modbus_crc16(response[:-2]):
            return None
        return response[3:-2]
    except Exception as exc:
        print(f"[MODBUS_READ_ERR] addr=0x{start_addr:04X}: {exc}")
        return None


def parse_cylinder_single_turn(raw: Optional[bytes]) -> Optional[int]:
    """沿用已验证取法：L=raw[0], M=raw[1], H=raw[3]。"""
    if raw is None or len(raw) != 4:
        return None
    return ((raw[3] << 16) | (raw[1] << 8) | raw[0]) & 0x7FFFFF


def update_encoder_accum(state: dict, single_turn: Optional[int]) -> Optional[int]:
    """把 23 位单圈编码器值累计为多圈 count。state 需含 last_single_turn/encoder_accum_count。"""
    if single_turn is None:
        return None
    if state.get("last_single_turn") is None:
        state["last_single_turn"] = single_turn
        state["encoder_accum_count"] = 0
        return 0

    delta = single_turn - state["last_single_turn"]
    one_turn = 1 << 23
    half_turn = one_turn // 2
    if delta > half_turn:
        delta -= one_turn
    elif delta < -half_turn:
        delta += one_turn

    state["encoder_accum_count"] += delta
    state["last_single_turn"] = single_turn
    return state["encoder_accum_count"]


def count_to_mm(count_value: int | float, lead_mm: float = 5.0, reduction_ratio: float = 5.0) -> float:
    mm_per_motor_rev = lead_mm / reduction_ratio
    return float(count_value) * mm_per_motor_rev / (1 << 23)


def rpm_to_mms(rpm_value: int | float, lead_mm: float = 5.0, reduction_ratio: float = 5.0) -> float:
    mm_per_motor_rev = lead_mm / reduction_ratio
    return float(rpm_value) * mm_per_motor_rev / 60.0


def bytes_to_int16(raw: Optional[bytes], signed: bool = True) -> Optional[int]:
    if raw is None or len(raw) != 2:
        return None
    return int.from_bytes(raw, byteorder="big", signed=signed)


def bytes_to_int32_normal(raw: Optional[bytes], signed: bool = True) -> Optional[int]:
    if raw is None or len(raw) != 4:
        return None
    return int.from_bytes(raw, byteorder="big", signed=signed)


def bytes_to_int32_swap(raw: Optional[bytes], signed: bool = True) -> Optional[int]:
    if raw is None or len(raw) != 4:
        return None
    return int.from_bytes(raw[2:4] + raw[0:2], byteorder="big", signed=signed)


def normalize_angle_180(angle_deg: float) -> float:
    """归一化到 [-180, 180)。"""
    return (float(angle_deg) + 180.0) % 360.0 - 180.0

# -*- coding: utf-8 -*-
"""调平执行器：只保留姿态调平阶段，不包含触地/预顶升。"""
from __future__ import annotations

import math
import struct
import threading
import time
from typing import Optional

import serial

from common import (
    build_cylinder_speed_packet,
    clamp,
    count_to_mm,
    parse_cylinder_single_turn,
    read_holding_registers,
    rpm_to_mms,
    update_encoder_accum,
)


class LevelingExecutor:
    def __init__(self):
        self.ports = [None, None, None, None]
        self.connected = [False, False, False, False]
        self.locks = [threading.Lock() for _ in range(4)]

        self.lead_mm = 5.0
        self.reduction_ratio = 5.0
        self.max_motor_rpm = 150.0
        self.max_cylinder_speed = self.max_motor_rpm * (self.lead_mm / self.reduction_ratio) / 60.0

        # 调平阶段参数
        self.veh_len = 1565.0
        self.veh_wid = 1215.0
        self.deadband = 0.002  # deg；MPC/高精度调平时使用很小死区
        self.k_p = 0.2
        self.k_twist = 0.01
        self.hold_time_s = 5.0
        self.hold_start_time = None
        self.hold_active = False
        self.ref_leg_id = 0

        self.legs = []
        for i in range(4):
            self.legs.append(self._new_leg_state(i))

    def _new_leg_state(self, idx: int) -> dict:
        return {
            "id": idx,
            "cmd_rpm": 0,
            "pos": 0.0,
            "start_pos": 0.0,
            "encoder_accum_count": 0,
            "last_single_turn": None,
            "zero_encoder_count": None,
            "vel_rpm": 0,
            "vel_mms": 0.0,
            "torque_pct": 0.0,
        }

    def connect_leg(self, leg_id: int, port_name: str, baudrate: int = 115200):
        self._check_leg_id(leg_id)
        if not port_name:
            raise ValueError(f"{leg_id + 1}号腿端口为空")
        port = serial.Serial(port_name, int(baudrate), timeout=0.03)
        port.reset_input_buffer()
        port.reset_output_buffer()
        self.ports[leg_id] = port
        self.connected[leg_id] = True
        self.reset_leg_feedback_cache(leg_id)

    def disconnect_leg(self, leg_id: int):
        self._check_leg_id(leg_id)
        try:
            self.send_leg_cmd(leg_id, 0)
        except Exception:
            pass
        try:
            with self.locks[leg_id]:
                if self.ports[leg_id]:
                    self.ports[leg_id].close()
        finally:
            self.ports[leg_id] = None
            self.connected[leg_id] = False
            self.reset_leg_feedback_cache(leg_id)

    def _check_leg_id(self, leg_id: int):
        if leg_id < 0 or leg_id >= 4:
            raise ValueError("leg_id must be 0..3")

    def all_connected(self) -> bool:
        return all(self.connected)

    def send_leg_cmd(self, leg_id: int, rpm_val: float) -> bool:
        self._check_leg_id(leg_id)
        if not self.connected[leg_id] or not self.ports[leg_id]:
            return False
        try:
            packet, phys_speed = build_cylinder_speed_packet(rpm_val, self.max_motor_rpm)
            with self.locks[leg_id]:
                self.ports[leg_id].write(packet)
            self.legs[leg_id]["cmd_rpm"] = phys_speed
            return True
        except Exception as exc:
            print(f"[LEVEL_SEND_ERR] leg={leg_id + 1}: {exc}")
            return False

    def apply_leg_commands(self, leg_cmds) -> None:
        for i in range(4):
            rpm = leg_cmds[i] if i < len(leg_cmds) else 0
            self.send_leg_cmd(i, rpm)

    def tilt_rate_to_leg_rpm(self, x_rate_rad_s: float, y_rate_rad_s: float) -> list[int]:
        """
        MPC 系统级接口：调平 X/Y 倾角速度(rad/s) -> 四条腿 rpm。

        注意：此处只做速度分配，不做姿态误差计算；
        具体符号如果实机方向相反，只应在本函数或接线方向表里修正，不要改 MPC。
        """
        v_pitch_mms = 0.5 * self.veh_len * float(x_rate_rad_s)
        v_roll_mms = 0.5 * self.veh_wid * float(y_rate_rad_s)

        leg_mms = [
            v_pitch_mms - v_roll_mms,
            v_pitch_mms + v_roll_mms,
            -v_pitch_mms + v_roll_mms,
            -v_pitch_mms - v_roll_mms,
        ]

        cmd = []
        for v in leg_mms:
            rpm = v * 60.0 * self.reduction_ratio / self.lead_mm
            cmd.append(int(clamp(int(rpm), -int(self.max_motor_rpm), int(self.max_motor_rpm))))
        return cmd

    def set_tilt_rate_rad_s(self, x_rate_rad_s: float, y_rate_rad_s: float) -> list[int]:
        cmd = self.tilt_rate_to_leg_rpm(x_rate_rad_s, y_rate_rad_s)
        self.apply_leg_commands(cmd)
        return cmd

    def stop_all(self):
        for i in range(4):
            try:
                self.send_leg_cmd(i, 0)
            except Exception:
                pass
        self.hold_active = False
        self.hold_start_time = None

    def read_leg_registers(self, leg_id: int, start_addr: int, reg_count: int) -> Optional[bytes]:
        self._check_leg_id(leg_id)
        return read_holding_registers(self.ports[leg_id], self.locks[leg_id], self.connected[leg_id], start_addr, reg_count)

    def update_feedback(self):
        for i in range(4):
            if not self.connected[i] or not self.ports[i]:
                continue
            raw_single = self.read_leg_registers(i, 0x4202, 2)
            single_turn = parse_cylinder_single_turn(raw_single)
            accum = update_encoder_accum(self.legs[i], single_turn)
            if accum is not None:
                if self.legs[i]["zero_encoder_count"] is None:
                    self.legs[i]["zero_encoder_count"] = accum
                rel = accum - self.legs[i]["zero_encoder_count"]
                self.legs[i]["pos"] = count_to_mm(rel, self.lead_mm, self.reduction_ratio)

            # raw_vel = self.read_leg_registers(i, 0x4025, 1)
            # if raw_vel is not None and len(raw_vel) == 2:
            #     self.legs[i]["vel_rpm"] = struct.unpack(">h", raw_vel)[0]
            #     self.legs[i]["vel_mms"] = rpm_to_mms(self.legs[i]["vel_rpm"], self.lead_mm, self.reduction_ratio)

            # raw_torque = self.read_leg_registers(i, 0x6025, 1)
            # if raw_torque is not None and len(raw_torque) == 2:
            #     self.legs[i]["torque_pct"] = struct.unpack(">h", raw_torque)[0] / 10.0

    def reset_leg_feedback_cache(self, leg_id: int):
        self.legs[leg_id].update(self._new_leg_state(leg_id))

    def reset_zero_all(self):
        for i in range(4):
            if not self.connected[i]:
                continue
            raw_single = self.read_leg_registers(i, 0x4202, 2)
            single_turn = parse_cylinder_single_turn(raw_single)
            accum = update_encoder_accum(self.legs[i], single_turn)
            if accum is not None:
                self.legs[i]["zero_encoder_count"] = accum
            self.legs[i]["pos"] = 0.0
            self.legs[i]["start_pos"] = 0.0

    def find_highest_leg(self, ax: float, ay: float) -> int:
        if ax >= 0 and ay >= 0:
            return 1
        if ax >= 0 and ay < 0:
            return 0
        if ax < 0 and ay >= 0:
            return 2
        return 3

    def reset_start_pos(self, ax: float, ay: float) -> int:
        for leg in self.legs:
            leg["start_pos"] = leg["pos"]
        self.ref_leg_id = self.find_highest_leg(ax, ay)
        self.hold_active = False
        self.hold_start_time = None
        return self.ref_leg_id

    def compute_local_leveling_leg_rpm(self, curr_ax: float, curr_ay: float) -> tuple[list[int], str, bool]:
        """
        姿态调平阶段控制律。返回 (四腿rpm, 状态文本, 是否完成)。
        已删除触地和预顶升阶段。
        """
        err_x = 0.0 - curr_ax
        err_y = 0.0 - curr_ay
        if abs(err_x) < self.deadband:
            err_x = 0.0
        if abs(err_y) < self.deadband:
            err_y = 0.0

        v_pitch = math.tan(err_x * 0.01745) * self.veh_len * 0.5
        v_roll = math.tan(err_y * 0.01745) * self.veh_wid * 0.5
        d = [leg["pos"] - leg["start_pos"] for leg in self.legs]
        twist_err = (d[1] + d[3]) - (d[0] + d[2])
        v_twist = clamp(twist_err * self.k_twist, -50.0, 50.0)

        if err_x == 0.0 and err_y == 0.0 and abs(twist_err) < 1.0:
            now = time.time()
            if not self.hold_active:
                self.hold_active = True
                self.hold_start_time = now
            hold_elapsed = now - self.hold_start_time
            status = f"姿态调平完成，维持中 ({hold_elapsed:.1f}/{self.hold_time_s:.1f}s)"
            if hold_elapsed >= self.hold_time_s:
                return [0, 0, 0, 0], "本地调平完成", True
            return [0, 0, 0, 0], status, False

        self.hold_active = False
        self.hold_start_time = None

        v_req = [0.0] * 4
        v_req[0] = (v_pitch - v_roll) + v_twist   # FR
        v_req[1] = (v_pitch + v_roll) - v_twist   # FL
        v_req[2] = (-v_pitch + v_roll) + v_twist  # RL
        v_req[3] = (-v_pitch - v_roll) - v_twist  # RR
        v_base = v_req[self.ref_leg_id]

        raw_speeds = [0.0] * 4
        for i in range(4):
            v_final = v_req[i] - v_base
            if i == self.ref_leg_id:
                v_final = 0.0
            raw_speeds[i] = v_final * self.k_p

        max_abs_speed = max(abs(s) for s in raw_speeds)
        scale = self.max_cylinder_speed / max_abs_speed if max_abs_speed > self.max_cylinder_speed else 1.0

        cmd = [0, 0, 0, 0]
        for i in range(4):
            limited_speed = raw_speeds[i] * scale
            rpm = limited_speed * 60.0 * self.reduction_ratio / self.lead_mm
            cmd[i] = int(clamp(int(rpm), -int(self.max_motor_rpm), int(self.max_motor_rpm)))
        return cmd, "姿态调平中", False

    def get_state(self) -> dict:
        return {
            "connected": list(self.connected),
            "leg_pos_mm": [leg["pos"] for leg in self.legs],
            "leg_cmd_rpm": [leg["cmd_rpm"] for leg in self.legs],
            "ref_leg_id": self.ref_leg_id,
            "hold_active": self.hold_active,
        }
    # "leg_vel_rpm": [leg["vel_rpm"] for leg in self.legs],

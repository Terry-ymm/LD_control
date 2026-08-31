# -*- coding: utf-8 -*-
"""举升/俯仰执行器。由“举升系统调试-前馈pid(1).py”提取到总控使用。

保留内容：
- 已验证的举升电动缸 0x3308 速度帧；
- 电动缸位置/速度/转矩反馈；
- 与方位系统相同的 0x6064/F010 角度编码器读取；
- 软件置零、单圈角度、多圈累计角度；
- 俯仰角速度差分与滤波；
- 角度相关前馈表 + 角速度 PID 修正；
- 面向 MPC 的 set_lift_rate_rad_s / set_lift_rate_deg_s 接口。

不包含内容：
- 独立 Tkinter 调试界面；
- 单独测试 CSV 保存界面。
"""
from __future__ import annotations

import json
import math
import os
import struct
import threading
import time
from datetime import datetime
from typing import Optional

import serial

from common import (
    build_cylinder_speed_packet,
    bytes_to_int32_normal,
    count_to_mm,
    parse_cylinder_single_turn,
    read_holding_registers,
    rpm_to_mms,
    update_encoder_accum,
)
from lift_position_controller import LiftPositionController


# 最低限位持久化配置文件：放在 lift_executor.py 同目录，跟随代码走。
_MIN_LIMIT_CONFIG_FILENAME = "lift_min_limit.json"


class LiftExecutor:
    def __init__(self):
        # =====================================================
        # 串口对象
        # =====================================================
        self.lift_port = None
        self.lift_connected = False
        self.lift_lock = threading.Lock()

        self.encoder_port = None
        self.encoder_connected = False
        self.encoder_lock = threading.Lock()
        self.encoder_protocol_ready = True

        # =====================================================
        # 电动缸机械参数
        # =====================================================
        self.lead_mm = 5.0
        self.reduction_ratio = 5.0
        # 与前馈 PID 调试版本保持一致
        self.max_motor_rpm = 2200.0

        # =====================================================
        # 举升角度编码器参数：与方位系统角度编码器一致
        # =====================================================
        self.lift_angle_position_ppr = 2147463847
        self.lift_angle_zero_count = 0
        self.lift_angle_current_count = None
        self.lift_angle_last_single_count = None
        self.lift_angle_zero_single_count = 0
        self.lift_angle_turn_count = 0
        self.lift_angle_relative_count = 0
        self.lift_single_angle_deg = 0.0
        self.lift_multi_angle_deg = 0.0

        # =====================================================
        # 俯仰角速度估计
        # =====================================================
        self.pitch_last_angle_deg = None
        self.pitch_last_time = None
        self.pitch_angular_velocity_deg_s = 0.0
        self.pitch_velocity_filter_alpha = 0.5

        # =====================================================
        # 俯仰角速度控制参数
        # =====================================================
        self.pitch_control_send_period = 0.15
        self.pitch_control_last_send_time = 0.0
        # 如果“目标角度变大”时实际角度反而变小，把这里改成 -1
        self.pitch_lift_direction = 1
        # 固定目标角速度限幅，单位 deg/s
        self.pitch_target_omega_limit_deg_s = 4

        # PID 参数：目标角速度 deg/s - 实际角速度 deg/s -> 电动缸 rpm 修正
        self.pitch_omega_kp = 50.0
        self.pitch_omega_ki = 0.0
        self.pitch_omega_kd = 0.0
        self.pitch_omega_integral = 0.0
        self.pitch_omega_last_error = 0.0
        self.pitch_omega_last_time = None
        self.pitch_pid_correction_max_rpm = 50.0

        # 前馈表：gain = deg/s/rpm
        self.pitch_gain_table = [
            (0.0, 0.00498),
            (2.5, 0.00432),
            (5.0, 0.00383),
            (7.5, 0.00346),
            (10.0, 0.00317),
            (12.5, 0.00294),
            (15.0, 0.00275),
            (17.5, 0.00260),
            (20.0, 0.00243),
            (25.0, 0.00226),
            (30.0, 0.00214),
            (35.0, 0.00206),
            (40.0, 0.00200),
            (45.0, 0.00197),
            (50.0, 0.001958),
            (55.0, 0.001960),
            (60.0, 0.001972),
            (65.0, 0.001972),
        ]

        # =====================================================
        # 最低限位（软限位）：锚定硬件原始角度，与软件置零无关
        # =====================================================
        self.pitch_min_limit_raw_deg = None   # None = 未启用；单位 deg，基于原始 count
        self.pitch_min_limit_margin_deg = 0.05
        # 启动即加载已持久化的最低限位（文件缺失则保持未启用）。
        self.load_pitch_min_limit()

        self.lift_state = self._new_lift_state()
        self.pitch_state = self._new_pitch_state()

        # =====================================================
        # 点到点位置环（在速度环之上）
        # =====================================================
        self.position_controller = LiftPositionController()
        # 与 MPC 举升角软限位保持一致：-0.02° ~ 65°（可被 configure 覆盖）
        self.position_controller.pos_min_deg = -0.02
        self.position_controller.pos_max_deg = 65.0

    # =========================================================
    # 状态结构
    # =========================================================
    def _new_lift_state(self) -> dict:
        return {
            "cmd_rpm": 0,
            "pos_mm": 0.0,
            "vel_rpm": 0,
            "vel_mms": 0.0,
            "torque_pct": 0.0,
            "encoder_accum_count": 0,
            "last_single_turn": None,
            "zero_encoder_count": None,
        }

    def _new_pitch_state(self) -> dict:
        return {
            "angle_deg": 0.0,
            "single_angle_deg": 0.0,
            "angle_rate_deg_s": 0.0,
            "raw": "---",
            "raw_count": None,
            "relative_count": 0,
            "health": "未连接",
            "target_omega_deg_s": 0.0,
            "last_motor_rpm": 0.0,
            "gain_deg_s_per_rpm": 0.0,
            "pid_kp": self.pitch_omega_kp,
            "pid_ki": self.pitch_omega_ki,
            "pid_kd": self.pitch_omega_kd,
        }

    # =========================================================
    # 串口连接
    # =========================================================
    def connect_lift(self, port_name: str, baudrate: int = 115200):
        if not port_name:
            raise ValueError("举升电动缸端口为空")
        self.lift_port = serial.Serial(port_name, int(baudrate), timeout=0.05)
        self.lift_port.reset_input_buffer()
        self.lift_port.reset_output_buffer()
        self.lift_connected = True
        self.reset_lift_feedback_cache()

    def disconnect_lift(self):
        self.reset_pitch_pid()
        try:
            self.send_lift_cmd(0)
        except Exception:
            pass
        try:
            with self.lift_lock:
                if self.lift_port:
                    self.lift_port.close()
        finally:
            self.lift_port = None
            self.lift_connected = False

    def connect_encoder(self, port_name: str, baudrate: int = 115200):
        if not port_name:
            raise ValueError("俯仰角度编码器端口为空")
        self.encoder_port = serial.Serial(port_name, int(baudrate), timeout=0.05)
        self.encoder_port.reset_input_buffer()
        self.encoder_port.reset_output_buffer()
        self.encoder_connected = True
        self.reset_pitch_angle_cache()
        self.pitch_state["health"] = "已连接"

    def disconnect_encoder(self):
        self.reset_pitch_pid()
        try:
            with self.encoder_lock:
                if self.encoder_port:
                    self.encoder_port.close()
        finally:
            self.encoder_port = None
            self.encoder_connected = False
            self.pitch_state["health"] = "未连接"

    # =========================================================
    # 低层 Modbus / 电动缸协议
    # =========================================================
    def send_lift_cmd(self, rpm_val: float) -> bool:
        """下发举升电动缸电机转速 r/min。"""
        if not self.lift_connected or not self.lift_port:
            return False
        try:
            packet, phys_speed = build_cylinder_speed_packet(round(float(rpm_val)), self.max_motor_rpm)
            with self.lift_lock:
                self.lift_port.write(packet)
            self.lift_state["cmd_rpm"] = phys_speed
            self.pitch_state["last_motor_rpm"] = float(phys_speed)
            return True
        except Exception as exc:
            print(f"[LIFT_SEND_ERR] {exc}")
            return False

    def stop(self):
        self.reset_pitch_pid()
        self.send_lift_cmd(0)

    def read_lift_registers(self, start_addr: int, reg_count: int) -> Optional[bytes]:
        return read_holding_registers(
            self.lift_port,
            self.lift_lock,
            self.lift_connected,
            start_addr,
            reg_count,
        )

    def read_encoder_registers(self, start_addr: int, reg_count: int) -> Optional[bytes]:
        return read_holding_registers(
            self.encoder_port,
            self.encoder_lock,
            self.encoder_connected,
            start_addr,
            reg_count,
        )

    # =========================================================
    # 电动缸反馈
    # =========================================================
    def update_lift_feedback(self):
        if not self.lift_connected or not self.lift_port:
            return

        # raw_single = self.read_lift_registers(0x4202, 2)
        # single_turn = parse_cylinder_single_turn(raw_single)
        # accum = update_encoder_accum(self.lift_state, single_turn)
        # if accum is not None:
        #     if self.lift_state["zero_encoder_count"] is None:
        #         self.lift_state["zero_encoder_count"] = accum
        #     rel = accum - self.lift_state["zero_encoder_count"]
        #     self.lift_state["pos_mm"] = count_to_mm(rel, self.lead_mm, self.reduction_ratio)

        # raw_vel = self.read_lift_registers(0x4025, 1)
        # if raw_vel is not None and len(raw_vel) == 2:
        #     self.lift_state["vel_rpm"] = struct.unpack(">h", raw_vel)[0]
        #     self.lift_state["vel_mms"] = rpm_to_mms(
        #         self.lift_state["vel_rpm"], self.lead_mm, self.reduction_ratio
        #     )

        # raw_torque = self.read_lift_registers(0x6025, 1)
        # if raw_torque is not None and len(raw_torque) == 2:
        #     self.lift_state["torque_pct"] = struct.unpack(">h", raw_torque)[0] / 10.0

    def reset_lift_feedback_cache(self):
        self.lift_state.update(self._new_lift_state())

    def reset_zero(self) -> bool:
        """电动缸位置反馈清零。"""
        if not self.lift_connected:
            return False
        raw_single = self.read_lift_registers(0x4202, 2)
        single_turn = parse_cylinder_single_turn(raw_single)
        accum = update_encoder_accum(self.lift_state, single_turn)
        if accum is None:
            return False
        self.lift_state["zero_encoder_count"] = accum
        self.lift_state["pos_mm"] = 0.0
        return True

    # =========================================================
    # 角度编码器 / 多圈角度 / 角速度
    # =========================================================
    def reset_pitch_angle_cache(self):
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
        old_target = self.pitch_state.get("target_omega_deg_s", 0.0) if hasattr(self, "pitch_state") else 0.0
        self.pitch_state.update(self._new_pitch_state())
        self.pitch_state["target_omega_deg_s"] = old_target
        self.pitch_state["health"] = "已连接" if self.encoder_connected else "未连接"

    def lift_angle_count_to_deg(self, count_value: int | float) -> float:
        return float(count_value) * 360.0 / self.lift_angle_position_ppr

    def lift_angle_count_to_single_deg(self, count_value: int | float) -> float:
        one_turn_count = self.lift_angle_position_ppr
        single_count = float(count_value) % one_turn_count
        return single_count * 360.0 / one_turn_count

    def update_pitch_encoder_feedback(self):
        """读取 F010/0x6064 当前角度位置，正常 int32 解析，不交换高低位。"""
        if not self.encoder_connected or not self.encoder_port:
            return

        raw_actual_pos = self.read_encoder_registers(0xF010, 2)
        actual_pos_count = bytes_to_int32_normal(raw_actual_pos, signed=True)

        if actual_pos_count is None:
            self.pitch_state["health"] = "读取失败"
            self.pitch_state["raw"] = "---"
            return

        self.lift_angle_current_count = actual_pos_count
        one_turn_count = self.lift_angle_position_ppr
        current_single_count = actual_pos_count % one_turn_count
        low_threshold = one_turn_count * 0.25
        high_threshold = one_turn_count * 0.75

        if self.lift_angle_last_single_count is None:
            self.lift_angle_last_single_count = current_single_count
        else:
            if self.lift_angle_last_single_count > high_threshold and current_single_count < low_threshold:
                self.lift_angle_turn_count += 1
            elif self.lift_angle_last_single_count < low_threshold and current_single_count > high_threshold:
                self.lift_angle_turn_count -= 1
            self.lift_angle_last_single_count = current_single_count

        relative_single_count = current_single_count - self.lift_angle_zero_single_count
        relative_pos_count = self.lift_angle_turn_count * one_turn_count + relative_single_count

        self.lift_angle_relative_count = relative_pos_count
        self.lift_multi_angle_deg = self.lift_angle_count_to_deg(relative_pos_count)
        self.lift_single_angle_deg = self.lift_angle_count_to_single_deg(relative_pos_count)
        self.update_pitch_angular_velocity(self.lift_multi_angle_deg)

        self.pitch_state.update({
            "angle_deg": self.lift_multi_angle_deg,
            "single_angle_deg": self.lift_single_angle_deg,
            "angle_rate_deg_s": self.pitch_angular_velocity_deg_s,
            "raw": f"{actual_pos_count} P",
            "raw_count": actual_pos_count,
            "relative_count": relative_pos_count,
            "health": "正常",
        })

    def update_pitch_angular_velocity(self, current_angle_deg: float) -> float:
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

    def reset_pitch_zero(self) -> bool:
        """俯仰角度编码器软件置零。"""
        if not self.encoder_connected or not self.encoder_port:
            return False
        if self.lift_angle_current_count is None:
            # 先尝试读取一次
            self.update_pitch_encoder_feedback()
        if self.lift_angle_current_count is None:
            return False

        one_turn_count = self.lift_angle_position_ppr
        self.lift_angle_zero_count = self.lift_angle_current_count
        self.lift_angle_zero_single_count = self.lift_angle_current_count % one_turn_count
        self.lift_angle_last_single_count = self.lift_angle_zero_single_count
        self.lift_angle_turn_count = 0
        self.lift_angle_relative_count = 0
        self.lift_single_angle_deg = 0.0
        self.lift_multi_angle_deg = 0.0

        self.pitch_last_angle_deg = None
        self.pitch_last_time = None
        self.pitch_angular_velocity_deg_s = 0.0
        self.pitch_state.update({
            "angle_deg": 0.0,
            "single_angle_deg": 0.0,
            "angle_rate_deg_s": 0.0,
            "relative_count": 0,
            "health": "正常",
        })
        self.reset_pitch_pid()
        return True

    # =========================================================
    # 最低限位（软限位）
    # =========================================================
    def _current_raw_angle_deg(self) -> Optional[float]:
        """当前硬件原始俯仰角（deg），基于原始 count，与软件置零无关。"""
        if self.lift_angle_current_count is None:
            return None
        return self.lift_angle_count_to_deg(self.lift_angle_current_count)

    def _min_limit_config_path(self) -> str:
        """最低限位持久化文件路径（与 lift_executor.py 同目录）。"""
        return os.path.join(os.path.dirname(os.path.abspath(__file__)), _MIN_LIMIT_CONFIG_FILENAME)

    def load_pitch_min_limit(self) -> Optional[float]:
        """启动时从配置文件加载最低限位原始角。

        文件缺失 / 损坏 / 数值越界时保持未启用（None）并打印原因，不抛异常。
        """
        path = self._min_limit_config_path()
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
        except FileNotFoundError:
            return None
        except (json.JSONDecodeError, OSError) as exc:
            print(f"[MIN_LIMIT_LOAD] 读取最低限位配置失败，已忽略：{exc}")
            return None

        value = data.get("pitch_min_limit_raw_deg") if isinstance(data, dict) else None
        if not isinstance(value, (int, float)) or not math.isfinite(float(value)):
            print("[MIN_LIMIT_LOAD] 最低限位配置值无效，已忽略")
            return None
        value = float(value)
        if not (-1000.0 <= value <= 1000.0):
            print(f"[MIN_LIMIT_LOAD] 最低限位值 {value:.3f}° 超出合理范围，已忽略")
            return None

        self.pitch_min_limit_raw_deg = value
        return value

    def _save_pitch_min_limit(self) -> None:
        """把当前最低限位写入配置文件（仅按钮点击时低频写入）。"""
        data = {
            "pitch_min_limit_raw_deg": self.pitch_min_limit_raw_deg,
            "margin_deg": abs(self.pitch_min_limit_margin_deg),
            "set_at": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "note": "锚定绝对编码器原始角，软件置零不影响；改动机械/编码器后需重新设置",
        }
        try:
            with open(self._min_limit_config_path(), "w", encoding="utf-8") as f:
                json.dump(data, f, ensure_ascii=False, indent=2)
        except OSError as exc:
            print(f"[MIN_LIMIT_SAVE] 写入最低限位配置失败：{exc}")

    def set_pitch_min_limit_from_current(self) -> bool:
        """以当前硬件原始角度为基准，在其上方 margin 度设置最低限位（软限位），并持久化。"""
        raw = self._current_raw_angle_deg()
        if raw is None:
            return False
        self.pitch_min_limit_raw_deg = raw + abs(self.pitch_min_limit_margin_deg)
        self._save_pitch_min_limit()
        return True

    def clear_pitch_min_limit(self):
        """清除最低限位，并删除持久化配置文件。"""
        self.pitch_min_limit_raw_deg = None
        try:
            os.remove(self._min_limit_config_path())
        except FileNotFoundError:
            pass
        except OSError as exc:
            print(f"[MIN_LIMIT_CLEAR] 删除最低限位配置失败：{exc}")

    def is_below_pitch_min_limit(self) -> bool:
        """当前角度是否处于最低限位及以下（即应禁止继续下降）。"""
        if self.pitch_min_limit_raw_deg is None:
            return False
        raw = self._current_raw_angle_deg()
        if raw is None:
            return False
        return raw <= self.pitch_min_limit_raw_deg

    # =========================================================
    # 前馈 + PID 角速度控制
    # =========================================================
    def configure_pitch_pid(
        self,
        kp: float | None = None,
        ki: float | None = None,
        kd: float | None = None,
        correction_limit_rpm: float | None = None,
        target_limit_deg_s: float | None = None,
        direction: int | None = None,
    ):
        if kp is not None:
            self.pitch_omega_kp = float(kp)
        if ki is not None:
            self.pitch_omega_ki = float(ki)
        if kd is not None:
            self.pitch_omega_kd = float(kd)
        if correction_limit_rpm is not None:
            self.pitch_pid_correction_max_rpm = abs(float(correction_limit_rpm))
        if target_limit_deg_s is not None:
            self.pitch_target_omega_limit_deg_s = abs(float(target_limit_deg_s))
        if direction is not None:
            self.pitch_lift_direction = 1 if int(direction) >= 0 else -1
        self.pitch_state.update({
            "pid_kp": self.pitch_omega_kp,
            "pid_ki": self.pitch_omega_ki,
            "pid_kd": self.pitch_omega_kd,
        })

    def reset_pitch_pid(self):
        self.pitch_omega_integral = 0.0
        self.pitch_omega_last_error = 0.0
        self.pitch_omega_last_time = None
        self.pitch_control_last_send_time = 0.0
        if hasattr(self, "pitch_state"):
            self.pitch_state["target_omega_deg_s"] = 0.0
            self.pitch_state["last_motor_rpm"] = 0.0

    def get_pitch_gain(self, angle_deg: float) -> float:
        table = sorted(self.pitch_gain_table, key=lambda x: x[0])
        if not table:
            return 0.0
        angle = abs(float(angle_deg))
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

    def _limit_target_omega(self, target_omega_deg_s: float) -> float:
        limit = abs(self.pitch_target_omega_limit_deg_s)
        current_gain = self.get_pitch_gain(self.lift_multi_angle_deg)
        dynamic_limit = abs(current_gain * self.max_motor_rpm * 0.95)
        if dynamic_limit > 0:
            limit = min(limit, dynamic_limit)
        return max(-limit, min(limit, float(target_omega_deg_s)))

    def pitch_velocity_pid_to_lift_rpm(self, target_omega_deg_s: float) -> float:
        target_omega_deg_s = self._limit_target_omega(target_omega_deg_s)
        self.pitch_state["target_omega_deg_s"] = target_omega_deg_s

        if abs(target_omega_deg_s) < 1e-6:
            self.reset_pitch_pid()
            return 0.0

        now = time.time()
        first_run = self.pitch_omega_last_time is None
        dt = self.pitch_control_send_period if first_run else now - self.pitch_omega_last_time
        if dt <= 0:
            dt = self.pitch_control_send_period

        actual_omega = self.pitch_angular_velocity_deg_s
        error = target_omega_deg_s - actual_omega

        current_angle = self.lift_multi_angle_deg
        gain = self.get_pitch_gain(current_angle)
        self.pitch_state["gain_deg_s_per_rpm"] = gain
        ff_rpm = 0.0 if gain <= 0 else target_omega_deg_s / gain

        self.pitch_omega_integral += error * dt
        self.pitch_omega_integral = max(-100.0, min(100.0, self.pitch_omega_integral))
        derivative = 0.0 if first_run else (error - self.pitch_omega_last_error) / dt

        pid_rpm = (
            self.pitch_omega_kp * error
            + self.pitch_omega_ki * self.pitch_omega_integral
            + self.pitch_omega_kd * derivative
        )
        pid_rpm = max(-self.pitch_pid_correction_max_rpm, min(self.pitch_pid_correction_max_rpm, pid_rpm))

        motor_rpm = (ff_rpm + pid_rpm) * self.pitch_lift_direction
        motor_rpm = max(-self.max_motor_rpm, min(self.max_motor_rpm, motor_rpm))

        self.pitch_omega_last_error = error
        self.pitch_omega_last_time = now
        return motor_rpm

    def set_lift_rate_deg_s(self, omega_deg_s: float, force_send: bool = False) -> bool:
        """MPC/总控接口：输入俯仰角速度 deg/s，内部前馈+PID 转为电动缸 rpm。"""
        if not self.lift_connected or not self.lift_port:
            return False
        omega_deg_s = float(omega_deg_s)
        if self.is_below_pitch_min_limit() and omega_deg_s < 0.0:
            omega_deg_s = 0.0   # 软限位：挡住下降方向（角度减小）
        if self.encoder_connected and self.pitch_state.get("health") != "正常":
            # 尽量先刷新一次角度反馈，避免刚启动时无反馈。
            self.update_pitch_encoder_feedback()
        now = time.time()
        if (not force_send) and (now - self.pitch_control_last_send_time < self.pitch_control_send_period):
            return True
        motor_rpm = self.pitch_velocity_pid_to_lift_rpm(float(omega_deg_s))
        ok = self.send_lift_cmd(motor_rpm)
        if ok:
            self.pitch_control_last_send_time = now
        return ok

    def set_lift_rate_rad_s(self, rate_rad_s: float, force_send: bool = False) -> bool:
        """MPC/总控接口：输入俯仰角速度 rad/s。"""
        return self.set_lift_rate_deg_s(math.degrees(float(rate_rad_s)), force_send=force_send)

    # 兼容旧叫法
    def send_pitch_angular_velocity(self, omega_deg_s: float) -> bool:
        return self.set_lift_rate_deg_s(omega_deg_s, force_send=True)

    # =========================================================
    # 点到点位置驱动（位置环 -> 速度环 -> rpm）
    # =========================================================
    def configure_lift_position_control(self, **kwargs) -> None:
        """配置位置环参数，例如 max_vel_deg_s / accel_deg_s2 / kp / deadband_deg / pos_min_deg 等。"""
        self.position_controller.configure(**kwargs)

    def start_lift_position_move(self, target_deg: float) -> dict:
        """以当前俯仰角为起点，启动一次点到点位置运动。

        返回位置环本次规划摘要（目标、距离、曲线时长、峰值速度等）。
        注意：调用前需确保角度编码器已连接并正在刷新（lift_multi_angle_deg 有效）。
        """
        current = self.lift_multi_angle_deg
        return self.position_controller.start_move(current, float(target_deg))

    def update_lift_position_control(self) -> dict:
        """推进一个位置环周期并下发速度环。

        应在周期性循环（如 GUI 的 after 回调 / 总控周期）里调用。
        返回位置环当前结果（target_omega_deg_s / state / arrived 等）。
        """
        res = self.position_controller.step(self.lift_multi_angle_deg)
        if not res["active"]:
            return res
        if res["state"] == "fault":
            # 跟随误差保护触发，立即停机（stop 直接下发 0）
            self.stop()
            return res
        # 位置环输出目标角速度 deg/s，交给已有速度环下发 rpm；
        # 到位时强制立即下发 0，避免受发送周期限制延迟停机。
        self.set_lift_rate_deg_s(res["target_omega_deg_s"], force_send=res["arrived"])
        return res

    def stop_lift_position_move(self):
        """停止位置驱动并复位。"""
        self.position_controller.stop()
        self.stop()

    def is_lift_position_arrived(self) -> bool:
        return self.position_controller.is_arrived()

    def is_lift_position_fault(self) -> bool:
        return self.position_controller.is_fault()

    # =========================================================
    # 总反馈与状态
    # =========================================================
    def update_feedback(self):
        self.update_lift_feedback()
        self.update_pitch_encoder_feedback()

    def get_state(self) -> dict:
        return {
            "lift_connected": self.lift_connected,
            "encoder_connected": self.encoder_connected,
            "lift": dict(self.lift_state),
            "pitch": dict(self.pitch_state),
            "pitch_min_limit_raw_deg": self.pitch_min_limit_raw_deg,
            "position": self.position_controller.get_state(),
        }

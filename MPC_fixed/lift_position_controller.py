# -*- coding: utf-8 -*-
"""举升/俯仰角 点到点位置环（纯计算，无硬件依赖）。

用途：
    在已有的“前馈 + 角速度 PID”速度环（LiftExecutor）之上再套一层位置环，
    实现“输入目标俯仰角(deg) -> 自动规划梯形速度曲线 -> 驱动到位”。

    本模块不访问串口、不写硬件，只负责：
      1. 梯形速度曲线规划（点到点，含最大速度 / 加减速度）；
      2. 位置环 P(I)(D) 校正 + 速度前馈，输出“目标角速度 deg/s”；
      3. 到位判断（死区 + 连续采样）、跟随误差保护、目标软限位钳制。

    输出的“目标角速度 deg/s”交给 LiftExecutor.set_lift_rate_deg_s()
    继续走已有的速度环（前馈 + 角速度 PID -> rpm -> 0x3308 速度帧）。

约定：
    - 角度单位 deg，角速度单位 deg/s；
    - 正方向 = 角度增大方向（与 lift_executor 的 pitch_lift_direction 解耦，
      方向换算由执行器层统一处理）。
"""
from __future__ import annotations

import math
import time
from typing import Any, Dict, Optional


class LiftPositionController:
    """举升俯仰角点到点位置环。

    控制结构（级联）：
        位置环（本模块）：位置误差 -> 目标角速度 deg/s
              |
              v
        速度环（LiftExecutor.set_lift_rate_deg_s）：目标角速度 -> rpm -> 0x3308

    位置环输出 = 速度前馈(梯形曲线) + P * 跟踪误差 + I * 误差积分 + D * 误差微分。
    """

    def __init__(self):
        # =====================================================
        # 梯形速度曲线参数
        # =====================================================
        self.max_vel_deg_s = 0.5        # 曲线最大速度 deg/s（保守默认，首次上电建议）
        self.accel_deg_s2 = 1.0         # 加速度 deg/s^2
        self.decel_deg_s2 = 1.0         # 减速度 deg/s^2

        # =====================================================
        # 位置环 PID（输出单位 deg/s）
        # =====================================================
        self.kp = 5.0                   # 比例：deg/s per deg
        self.ki = 0.0                   # 积分：deg/s per deg*s（默认关闭）
        self.kd = 0.0                   # 微分：deg/s per deg/s
        self.integral_limit_deg_s = 2.0 # 积分抗饱和限幅 deg/s

        # =====================================================
        # 输出限幅与安全
        # =====================================================
        self.max_output_deg_s = 1.0     # 输出目标角速度限幅 deg/s（保守默认，可上调）
        self.deadband_deg = 0.05        # 到位死区 deg
        self.arrive_hold_samples = 5    # 连续 N 拍在死区内才判定到位
        self.follow_error_limit_deg = 3.0  # 跟踪误差保护 deg（超过则报障停机）

        # 目标软限位（可选，None = 不启用）。角度增大为正。
        self.pos_min_deg: Optional[float] = None
        self.pos_max_deg: Optional[float] = None

        self._reset_state()

    # =========================================================
    # 内部状态
    # =========================================================
    def _reset_state(self):
        self._active = False
        self._state = "idle"            # idle / moving / holding / fault
        self._target_deg: Optional[float] = None
        self._start_deg = 0.0
        self._distance_deg = 0.0
        self._profile: Optional[Dict[str, float]] = None

        self._elapsed = 0.0             # 累计时间（秒）
        self._last_step_time: Optional[float] = None   # 实时模式上次步进时刻（monotonic）
        self._last_error = 0.0
        self._integral = 0.0
        self._arrive_count = 0

        self._last_result = self._idle_result()

    def _idle_result(self, current_deg: float = 0.0) -> Dict[str, Any]:
        return {
            "active": False,
            "state": "idle",
            "target_deg": None,
            "current_deg": float(current_deg),
            "pos_error_deg": 0.0,
            "tracking_error_deg": 0.0,
            "ref_pos_deg": float(current_deg),
            "ref_vel_deg_s": 0.0,
            "target_omega_deg_s": 0.0,
            "arrived": False,
            "following_error": False,
        }

    # =========================================================
    # 参数配置
    # =========================================================
    def configure(self, **kwargs: Any) -> None:
        for name, value in kwargs.items():
            if not hasattr(self, name):
                raise AttributeError(f"LiftPositionController 没有参数: {name}")
            setattr(self, name, value)

    # =========================================================
    # 目标设定
    # =========================================================
    def start_move(self, current_deg: float, target_deg: float) -> Dict[str, Any]:
        """规划并启动一次点到点运动。

        返回本次规划的摘要（目标、距离、曲线时长等），供界面显示。
        """
        current_deg = float(current_deg)
        target_deg = float(target_deg)

        # 目标软限位钳制
        clamped = False
        if self.pos_min_deg is not None and target_deg < self.pos_min_deg:
            target_deg = self.pos_min_deg
            clamped = True
        if self.pos_max_deg is not None and target_deg > self.pos_max_deg:
            target_deg = self.pos_max_deg
            clamped = True

        distance = target_deg - current_deg
        self._reset_state()

        if abs(distance) < 1e-9:
            # 已在目标位置，直接判定到位
            self._target_deg = target_deg
            self._start_deg = current_deg
            self._distance_deg = 0.0
            self._profile = None
            self._active = True
            self._state = "holding"
            self._arrive_count = self.arrive_hold_samples
            self._last_result = self._idle_result(current_deg)
            self._last_result.update(
                active=True, state="holding", target_deg=target_deg,
                current_deg=current_deg, ref_pos_deg=target_deg, arrived=True,
            )
            return self._last_result.copy()

        self._target_deg = target_deg
        self._start_deg = current_deg
        self._distance_deg = distance
        self._profile = self._plan_profile(distance)
        self._active = True
        self._state = "moving"
        self._elapsed = 0.0
        self._last_step_time = None
        self._last_error = 0.0
        self._integral = 0.0
        self._arrive_count = 0

        summary = {
            "active": True,
            "state": "moving",
            "target_deg": target_deg,
            "start_deg": current_deg,
            "distance_deg": distance,
            "profile_total_s": self._profile["total"],
            "profile_v_peak_deg_s": self._profile["v_peak"],
            "clamped": clamped,
        }
        self._last_result = self._idle_result(current_deg)
        self._last_result.update(summary)
        return self._last_result.copy()

    def _plan_profile(self, distance: float) -> Dict[str, float]:
        """梯形速度曲线规划（距离为负时反向）。

        返回 profile 字典（均为正数幅度 + sign 方向）：
            t_acc / t_cruise / t_dec / total  各段时间
            v_peak                             峰值速度幅度
            d_acc / d_dec                      加减速段走过的距离幅度
        """
        D = abs(distance)
        sign = 1.0 if distance >= 0 else -1.0
        v_max = max(abs(self.max_vel_deg_s), 1e-6)
        a_acc = max(abs(self.accel_deg_s2), 1e-6)
        a_dec = max(abs(self.decel_deg_s2), 1e-6)

        d_acc = v_max * v_max / (2.0 * a_acc)
        d_dec = v_max * v_max / (2.0 * a_dec)

        if D >= d_acc + d_dec:
            # 梯形：能到最高速
            t_acc = v_max / a_acc
            t_dec = v_max / a_dec
            t_cruise = (D - d_acc - d_dec) / v_max
            v_peak = v_max
        else:
            # 三角形：距离太短到不了最高速
            v_peak = math.sqrt(2.0 * D * a_acc * a_dec / (a_acc + a_dec))
            t_acc = v_peak / a_acc
            t_dec = v_peak / a_dec
            t_cruise = 0.0
            d_acc = v_peak * v_peak / (2.0 * a_acc)
            d_dec = v_peak * v_peak / (2.0 * a_dec)

        return {
            "D": D,
            "sign": sign,
            "v_max": v_max,
            "v_peak": v_peak,
            "a_acc": a_acc,
            "a_dec": a_dec,
            "t_acc": t_acc,
            "t_cruise": t_cruise,
            "t_dec": t_dec,
            "total": t_acc + t_cruise + t_dec,
            "d_acc": d_acc,
            "d_dec": d_dec,
        }

    def _profile_ref(self) -> tuple[float, float]:
        """由累计时间求当前曲线参考（相对位移幅度 pos, 速度幅度 vel），未含符号。"""
        p = self._profile
        if p is None:
            return 0.0, 0.0
        t = self._elapsed
        if t <= 0.0:
            return 0.0, 0.0
        if t < p["t_acc"]:
            vel = p["a_acc"] * t
            pos = 0.5 * p["a_acc"] * t * t
        elif t < p["t_acc"] + p["t_cruise"]:
            vel = p["v_peak"]
            pos = p["d_acc"] + p["v_peak"] * (t - p["t_acc"])
        elif t < p["total"]:
            tt = t - (p["t_acc"] + p["t_cruise"])
            vel = p["v_peak"] - p["a_dec"] * tt
            pos = p["d_acc"] + p["v_peak"] * p["t_cruise"] + p["v_peak"] * tt - 0.5 * p["a_dec"] * tt * tt
        else:
            vel = 0.0
            pos = p["D"]
        return pos, vel

    # =========================================================
    # 单步计算
    # =========================================================
    def step(self, current_deg: float, dt: Optional[float] = None) -> Dict[str, Any]:
        """推进一个控制周期，返回目标角速度 deg/s 及状态。

        dt=None 时使用墙钟时间（实时硬件模式）；
        显式传入 dt 时使用确定性步长（离线仿真 / 自测）。
        """
        current_deg = float(current_deg)

        if not self._active or self._target_deg is None:
            self._last_result = self._idle_result(current_deg)
            return self._last_result.copy()

        # ---- 时间推进 ----
        if dt is None:
            # 实时模式：用墙钟时间计算本拍步长，再累加。
            now = time.monotonic()
            step_dt = 0.0 if self._last_step_time is None else (now - self._last_step_time)
            self._last_step_time = now
        else:
            # 确定性模式：显式步长（离线仿真 / 自测）。
            step_dt = max(float(dt), 0.0)
        self._elapsed += step_dt

        step_dt = max(step_dt, 1e-9)

        # ---- 曲线参考 ----
        if self._profile is not None:
            pos_amp, vel_amp = self._profile_ref()
            ref_pos = self._start_deg + pos_amp * self._profile["sign"]
            ref_vel = vel_amp * self._profile["sign"]
        else:
            # 无曲线（距离为 0 的到位态）
            ref_pos = self._target_deg
            ref_vel = 0.0

        tracking_error = ref_pos - current_deg
        pos_error = self._target_deg - current_deg

        # ---- 跟随误差保护 ----
        if abs(tracking_error) > abs(self.follow_error_limit_deg):
            self._active = False
            self._state = "fault"
            self._last_result = {
                "active": False,
                "state": "fault",
                "target_deg": self._target_deg,
                "current_deg": current_deg,
                "pos_error_deg": pos_error,
                "tracking_error_deg": tracking_error,
                "ref_pos_deg": ref_pos,
                "ref_vel_deg_s": ref_vel,
                "target_omega_deg_s": 0.0,
                "arrived": False,
                "following_error": True,
            }
            return self._last_result.copy()

        # ---- 位置环 PID（输出 deg/s）----
        error = tracking_error
        derivative = (error - self._last_error) / step_dt
        self._integral += error * step_dt
        self._integral = max(-self.integral_limit_deg_s, min(self.integral_limit_deg_s, self._integral))

        v_corr = self.kp * error + self.ki * self._integral + self.kd * derivative
        v_target = ref_vel + v_corr

        # ---- 到位判断（用目标误差，不是跟踪误差）----
        if abs(pos_error) <= abs(self.deadband_deg):
            self._arrive_count += 1
        else:
            self._arrive_count = 0

        arrived = self._arrive_count >= self.arrive_hold_samples

        if arrived:
            self._state = "holding"
            # 到位后停止输出，避免在死区内来回抖动；
            # 若因重力漂移出死区，下一拍自然重新进入 moving 继续校正。
            v_target = 0.0
            self._integral = 0.0
        else:
            self._state = "moving"

        # ---- 输出限幅 ----
        limit = abs(self.max_output_deg_s)
        v_target = max(-limit, min(limit, v_target))
        if not math.isfinite(v_target):
            v_target = 0.0

        self._last_error = error

        self._last_result = {
            "active": True,
            "state": self._state,
            "target_deg": self._target_deg,
            "current_deg": current_deg,
            "pos_error_deg": pos_error,
            "tracking_error_deg": tracking_error,
            "ref_pos_deg": ref_pos,
            "ref_vel_deg_s": ref_vel,
            "target_omega_deg_s": v_target,
            "arrived": arrived,
            "following_error": False,
        }
        return self._last_result.copy()

    # =========================================================
    # 状态查询
    # =========================================================
    def stop(self):
        """停止并复位（下次使用前需重新 start_move）。"""
        self._reset_state()

    def reset(self):
        self._reset_state()

    def is_arrived(self) -> bool:
        return bool(self._last_result.get("arrived", False))

    def is_fault(self) -> bool:
        return self._last_result.get("state") == "fault"

    def get_state(self) -> Dict[str, Any]:
        return dict(self._last_result)

    @property
    def target_deg(self) -> Optional[float]:
        return self._target_deg


if __name__ == "__main__":
    # =========================================================
    # 离线自测：虚拟电动缸（一阶速度环近似 + 前馈增益表）
    # 验证位置环能把角度驱动到目标并停在死区内。
    # =========================================================
    print("=" * 60)
    print("LiftPositionController 离线自测（无硬件）")
    print("=" * 60)

    ctrl = LiftPositionController()
    # 位置环参数
    ctrl.configure(max_vel_deg_s=2.0, accel_deg_s2=5.0, decel_deg_s2=5.0,
                   kp=5.0, ki=0.0, kd=0.0, deadband_deg=0.05,
                   follow_error_limit_deg=3.0, max_output_deg_s=4.0)

    # 与 lift_executor 相同的前馈增益表（gain = deg/s per rpm），用于换算 rpm 检查。
    gain_table = [
        (0.0, 0.00498), (10.0, 0.00317), (20.0, 0.00243),
        (30.0, 0.00214), (40.0, 0.00200), (50.0, 0.001958), (65.0, 0.001972),
    ]
    def gain_of(angle_deg):
        a = abs(angle_deg)
        xs = [g[0] for g in gain_table]
        ys = [g[1] for g in gain_table]
        if a <= xs[0]:
            return ys[0]
        if a >= xs[-1]:
            return ys[-1]
        for i in range(len(xs) - 1):
            if xs[i] <= a <= xs[i + 1]:
                r = (a - xs[i]) / (xs[i + 1] - xs[i])
                return ys[i] + r * (ys[i + 1] - ys[i])
        return ys[-1]

    dt = 0.05          # 50ms 控制周期
    tau = 0.15         # 速度环一阶滞后时间常数
    target = 20.0      # 目标角度 deg
    current = 0.0      # 初始角度 deg
    actual_omega = 0.0

    ctrl.start_move(current, target)
    print(f"\n目标 {target} deg，初始 {current} deg，dt={dt}s")

    max_rpm_seen = 0.0
    t = 0.0
    while True:
        res = ctrl.step(current, dt=dt)
        v_cmd = res["target_omega_deg_s"]

        # 虚拟电动缸：速度环近似为一阶滞后，角度积分。
        actual_omega += (v_cmd - actual_omega) * (1.0 - math.exp(-dt / tau))
        current += actual_omega * dt
        t += dt

        # 换算 rpm 检查是否超 2200
        rpm = abs(v_cmd) / gain_of(current) if gain_of(current) > 0 else 0.0
        max_rpm_seen = max(max_rpm_seen, rpm)

        if res["arrived"]:
            print(f"  t={t:5.2f}s  到达  角度={current:8.3f} deg  误差={res['pos_error_deg']:+.4f} deg")
            break
        if res["state"] == "fault":
            print(f"  t={t:5.2f}s  跟随误差故障！")
            break
        if t > 60.0:
            print("  超时未到位")
            break

        if int(t / dt) % 20 == 0:
            print(f"  t={t:5.2f}s  角度={current:8.3f} deg  v_cmd={v_cmd:+7.3f} deg/s  rpm≈{rpm:7.1f}")

    print(f"\n最终角度 {current:.3f} deg，到位误差 {current - target:+.4f} deg")
    print(f"过程中估算最大转速 ≈ {max_rpm_seen:.0f} rpm（限 2200）")
    print("自测结束。")

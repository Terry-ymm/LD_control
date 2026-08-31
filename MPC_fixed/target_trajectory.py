import math
import numpy as np


class TargetTrajectory:
    """
    MPC 目标曲线生成器。

    输出：
        target_sequence:
        [
            Rx1, Ry1, Rz1,
            Rx2, Ry2, Rz2,
            Rx3, Ry3, Rz3
        ]

    单位约定：
        a: 举升/俯仰目标角，deg
        b: 方位目标角，deg
        wa: 俯仰目标最大角速度，deg/s
        wb: 方位目标最大角速度，deg/s
        t:  当前实验时间，s
        dt: MPC 预测步长时间间隔，s
    """

    def __init__(self):
        self.Np = 3

        # ===== 目标曲线参数 =====
        self.wa = 3.4        # 俯仰最大角速度，deg/s
        self.wb = 3.0        # 方位最大角速度，deg/s

        self.Ta = 2.0        # 俯仰加速段时间，s
        self.Tb = 2.0        # 方位加速段时间，s

        self.tar_a = 60.0    # 俯仰目标角度，deg
        self.time_a = 20.0   # 俯仰阶段总时间，s；之后开始方位运动

        # 多项式系数
        self.a3 = self.wa / (self.Ta ** 2)
        self.a4 = -0.5 * self.wa / (self.Ta ** 3)

        self.b3 = self.wb / (self.Tb ** 2)
        self.b4 = -0.5 * self.wb / (self.Tb ** 3)

    def compute(self, t, dt=0.1):
        """
        根据当前时间生成 MPC 预测目标序列。

        参数：
            t: 当前实验时间，单位 s
            dt: 预测序列间隔，单位 s，一般等于 MPC 周期 0.1s

        返回：
            numpy.ndarray，长度 9
        """

        s = np.zeros(3 * self.Np, dtype=float)

        for i in range(self.Np):
            ti = t + i * dt

            a_deg, b_deg = self._target_angle_at_time(ti)

            rx, ry, rz = self._angle_to_vector(a_deg, b_deg)

            s[3 * i + 0] = rx
            s[3 * i + 1] = ry
            s[3 * i + 2] = rz

        return s

    def _target_angle_at_time(self, t):
        """
        对应 MATLAB 中每个 T(i) 下的 a、b 目标角。
        """

        if t < self.time_a:
            # ===== 俯仰阶段 =====
            if t < self.Ta:
                a = self.a3 * (t ** 3) + self.a4 * (t ** 4)

            elif t < self.tar_a / self.wa:
                a = 0.5 * self.Ta * self.wa + self.wa * (t - self.Ta)

            elif t < (self.tar_a / self.wa + self.Ta):
                tau = self.tar_a / self.wa + self.Ta - t
                a = self.tar_a - (self.a3 * (tau ** 3) + self.a4 * (tau ** 4))

            else:
                a = self.tar_a

            b = 0.0

        else:
            # ===== 方位阶段 =====
            a = self.tar_a

            tb = t - self.time_a

            if tb < self.Tb:
                b = self.b3 * (tb ** 3) + self.b4 * (tb ** 4)
            else:
                b = 0.5 * self.Tb * self.wb + self.wb * (tb - self.Tb)

        return a, b

    @staticmethod
    def _angle_to_vector(a_deg, b_deg):
        """
        对应 MATLAB:

            S = [0; 0; -1]
            Mb = [cosd(b) sind(b) 0;
                 -sind(b) cosd(b) 0;
                  0       0       1]

            Ma = [1 0 0;
                  0 cosd(a) sind(a);
                  0 -sind(a) cosd(a)]

            S1 = Mb.' * Ma.' * S
        """

        a = math.radians(a_deg)
        b = math.radians(b_deg)

        S = np.array([0.0, 0.0, -1.0])

        Mb = np.array([
            [math.cos(b), math.sin(b), 0.0],
            [-math.sin(b), math.cos(b), 0.0],
            [0.0, 0.0, 1.0],
        ])

        Ma = np.array([
            [1.0, 0.0, 0.0],
            [0.0, math.cos(a), math.sin(a)],
            [0.0, -math.sin(a), math.cos(a)],
        ])

        S1 = Mb.T @ Ma.T @ S

        return float(S1[0]), float(S1[1]), float(S1[2])
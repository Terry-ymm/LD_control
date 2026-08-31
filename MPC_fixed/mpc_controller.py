"""
mpc_controller.py

由 MATLAB/Simulink S-function `MPC_fun` 移植而来。

功能：
    1. 接收当前状态和目标序列；
    2. 求解 MPC 二次规划；
    3. 输出 4 个“系统级角速度控制量”；
    4. 不在这里转换成电机 rpm。

输出约定：
    leveling_x_rate_rad_s : 调平系统 X 轴倾角速度，单位 rad/s
    leveling_y_rate_rad_s : 调平系统 Y 轴倾角速度，单位 rad/s
    azimuth_rate_rad_s    : 方位角速度，单位 rad/s，逆时针为正
    lift_rate_rad_s       : 举升/俯仰角速度，单位 rad/s

注意：
    Rx, Ry, Rz 是 MPC 内部状态构造需要的中间量。
    实际总控不需要输出它们，所以 compute() 返回值里不包含 R。
"""

import math
from dataclasses import dataclass
from typing import Any, Dict, Iterable, Optional, Sequence, Tuple

import numpy as np
from scipy.optimize import minimize
import scipy.sparse as sp
import osqp


@dataclass
class MPCResult:
    """MPC 求解结果，主要用于调试。"""

    ok: bool
    message: str
    objective: float


class MPCController:
    """
    三系统总控 MPC。

    MATLAB 原始定义：
        Nx = 3   状态量：目标方向向量的 x/y/z 分量
        Nu = 4   控制量：4 个系统级角速度
        Np = 3   预测步长
        Nc = 2   控制步长

    控制量顺序：
        U[0] = 调平 X 轴倾角速度，rad/s
        U[1] = 调平 Y 轴倾角速度，rad/s
        U[2] = 方位角速度，rad/s，逆时针为正
        U[3] = 举升/俯仰角速度，rad/s
    """

    def __init__(self) -> None:
        # =====================================================
        # MPC 维度参数
        # =====================================================
        self.Nx = 3
        self.Nu = 4
        self.Np = 3
        self.Nc = 2

        # =====================================================
        # 采样周期
        # 单位：s
        # 与 MATLAB 代码中的 T = 0.1 保持一致。
        # 总控 MPC 调用周期也建议统一为 0.1 s。
        # =====================================================
        self.T = 0.20

        # =====================================================
        # MPC 内部控制量记忆
        # 单位：rad/s
        # 对应 MATLAB global U。
        # =====================================================
        self.U = np.zeros(self.Nu, dtype=float)

        # =====================================================
        # 权重参数
        # Q_weight：
        #     输出误差权重，作用于 Rx/Ry/Rz 预测误差。
        #
        # R0：
        #     控制增量权重，对应 Nc * Nu = 8 个 delta_u。
        #     单位上可以理解为对 (rad/s) 控制增量的惩罚系数。
        #
        # P0：
        #     位置/角度相关惩罚项权重，对应 Nc * Nu = 8 个量。
        #     目前只惩罚调平 x/y 两个角速度通道，方位和举升通道为 0。
        # =====================================================
        self.Q_weight = 1300.0
        self.R0 = np.array(
            [180.0, 180.0, 8.0, 8.0, 180.0, 180.0, 8.0, 8.0],
            dtype=float,
        )
        self.P0 = np.array(
            [1.0, 1.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0],
            dtype=float,
        )
        self.P_scale = 8.0

        # =====================================================
        # 物理速度限制
        # 单位：deg/s
        #
        # 顺序：
        #   [调平X倾角速度, 调平Y倾角速度, 方位角速度, 举升角速度]
        #
        # MATLAB：
        #   val_vel_max = [0.1/2; 0.16/2; 3.5; 3.9] * pi/180;
        # =====================================================
        self.vel_max_deg_s = np.array(
            [0.073 *1.3 / 2.0, 0.094 *1.3 / 2.0, 3.5, 3.9],
            dtype=float,
        )

        # =====================================================
        # 物理角度限制
        # 单位：deg
        #
        # 顺序：
        #   [gx, gy, b, a]
        #
        # gx/gy:
        #   调平倾角限制
        #
        # b:
        #   方位角默认不限制，所以使用 +/-inf
        #
        # a:
        #   举升/俯仰角限制，默认 -0.02° ~ 65°
        # =====================================================
        self.pos_max_deg = np.array(
            [2.0, 2.0, np.inf, 65.0],
            dtype=float,
        )
        self.pos_min_deg = np.array(
            [-2.0, -2.0, -np.inf, -0.02],
            dtype=float,
        )

        # =====================================================
        # 控制增量限制
        # 单位：deg/s
        #
        # MATLAB：
        #   lb = -1*pi/180*ones(Nc*Nu, 1)
        #   ub =  1*pi/180*ones(Nc*Nu, 1)
        #
        # 含义：
        #   每次 MPC 允许 U 的变化量不超过 +/-1 deg/s。
        # =====================================================
        self.delta_u_limit_deg_s = 1.5

        # =====================================================
        # 求解器参数
        # =====================================================
        self.solver_max_iter = 100
        self.solver_ftol = 1e-9

        # 最近一次求解信息
        self.last_result = MPCResult(ok=False, message="not_run", objective=0.0)
        self.last_output: Dict[str, Any] = {
            "leveling_x_rate_rad_s": 0.0,
            "leveling_y_rate_rad_s": 0.0,
            "azimuth_rate_rad_s": 0.0,
            "lift_rate_rad_s": 0.0,
            "ok": False,
        }

    # =========================================================
    # 对外接口
    # =========================================================
    def reset(self) -> None:
        """
        重置 MPC 内部状态。

        使用场景：
            1. 开始新实验前；
            2. 急停后重新启动前；
            3. 目标序列切换前。
        """
        self.U[:] = 0.0
        self.last_result = MPCResult(ok=False, message="reset", objective=0.0)
        self.last_output = {
            "leveling_x_rate_rad_s": 0.0,
            "leveling_y_rate_rad_s": 0.0,
            "azimuth_rate_rad_s": 0.0,
            "lift_rate_rad_s": 0.0,
            "ok": False,
        }

    def update_parameters(self, **kwargs: Any) -> None:
        """
        运行前或实验前动态修改 MPC 参数。

        示例：
            mpc.update_parameters(
                Q_weight=1200.0,
                delta_u_limit_deg_s=0.4,
                vel_max_deg_s=[0.2, 0.3, 1.0, 2.0],
                pos_max_deg=[2, 2, float("inf"), 65],
                pos_min_deg=[-2, -2, -float("inf"), 0],
            )
        """
        for name, value in kwargs.items():
            if not hasattr(self, name):
                raise AttributeError(f"MPCController 没有参数: {name}")

            if name in {"R0", "P0", "vel_max_deg_s", "pos_max_deg", "pos_min_deg"}:
                setattr(self, name, np.asarray(value, dtype=float))
            else:
                setattr(self, name, value)

    def compute(self, state: Dict[str, Any]) -> Dict[str, Any]:
        """
        计算 MPC 控制量。

        输入 state 推荐格式：
            {
                "x_angle_deg": 当前调平 X 轴倾角，deg
                "y_angle_deg": 当前调平 Y 轴倾角，deg
                "z_angle_deg": 当前 Z / 航向相关角度，deg

                "azimuth_angle_deg": 当前方位角 b，deg
                "lift_angle_deg": 当前举升/俯仰角 a，deg

                # 以下二选一：
                "target_vector": [Rx_ref, Ry_ref, Rz_ref]
                # 或：
                "target_sequence": [
                    Rx1, Ry1, Rz1,
                    Rx2, Ry2, Rz2,
                    Rx3, Ry3, Rz3
                ]
            }

        返回：
            {
                "leveling_x_rate_rad_s": ...,
                "leveling_y_rate_rad_s": ...,
                "azimuth_rate_rad_s": ...,
                "lift_rate_rad_s": ...,
                "ok": True/False,

                # 调试辅助，不参与执行器控制：
                "u_rad_s": [...],
                "u_deg_s": [...],
                "solver_message": "...",
            }
        """
        gx = math.radians(self._get_first(state, ["x_angle_deg", "gx_deg"], 0.0))
        gy = math.radians(self._get_first(state, ["y_angle_deg", "gy_deg"], 0.0))
        gz = math.radians(self._get_first(state, ["z_angle_deg", "gz_deg"], 0.0))
        b = math.radians(self._get_first(state, ["azimuth_angle_deg", "b_deg"], 0.0))
        a = math.radians(
            self._get_first(
                state,
                ["lift_angle_deg", "pitch_angle_deg", "elevation_angle_deg", "a_deg"],
                0.0,
            )
        )

        y_ref = self._build_reference(state)

        try:
            u_rad_s, ok, message, objective = self._solve_mpc(
                gx=gx,
                gy=gy,
                gz=gz,
                b=b,
                a=a,
                y_ref=y_ref,
            )
        except Exception as exc:
            # MPC 出现异常时，不继续沿用旧速度，直接输出 0。
            self.U[:] = 0.0
            u_rad_s = self.U.copy()
            ok = False
            message = f"exception: {exc}"
            objective = 0.0

        # 防御性检查，避免 NaN/inf 下发到执行器。
        if not np.all(np.isfinite(u_rad_s)):
            self.U[:] = 0.0
            u_rad_s = self.U.copy()
            ok = False
            message = "non_finite_output"
            objective = 0.0

        self.last_result = MPCResult(
            ok=bool(ok),
            message=str(message),
            objective=float(objective),
        )

        self.last_output = {
            "leveling_x_rate_rad_s": float(u_rad_s[0]),
            "leveling_y_rate_rad_s": float(u_rad_s[1]),
            "azimuth_rate_rad_s": float(u_rad_s[2]),
            "lift_rate_rad_s": float(u_rad_s[3]),
            "ok": bool(ok),
            "u_rad_s": u_rad_s.astype(float).tolist(),
            "u_deg_s": np.degrees(u_rad_s).astype(float).tolist(),
            "solver_message": str(message),
            "solver_objective": float(objective),
        }

        return self.last_output.copy()

    # =========================================================
    # MPC 核心
    # =========================================================
    def _solve_mpc(
        self,
        gx: float,
        gy: float,
        gz: float,
        b: float,
        a: float,
        y_ref: np.ndarray,
    ) -> Tuple[np.ndarray, bool, str, float]:
        Nx = self.Nx
        Nu = self.Nu
        Np = self.Np
        Nc = self.Nc
        T = self.T

        # -----------------------------------------------------
        # 1. 当前方向向量计算
        # -----------------------------------------------------
        S = np.array([0.0, 0.0, -1.0], dtype=float)

        Magx = np.array(
            [
                [1.0, 0.0, 0.0],
                [0.0, math.cos(gx), math.sin(gx)],
                [0.0, -math.sin(gx), math.cos(gx)],
            ],
            dtype=float,
        )

        Magy = np.array(
            [
                [math.cos(gy), 0.0, -math.sin(gy)],
                [0.0, 1.0, 0.0],
                [math.sin(gy), 0.0, math.cos(gy)],
            ],
            dtype=float,
        )

        Magz = np.array(
            [
                [math.cos(gz), math.sin(gz), 0.0],
                [-math.sin(gz), math.cos(gz), 0.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=float,
        )

        Mb = np.array(
            [
                [math.cos(b), math.sin(b), 0.0],
                [-math.sin(b), math.cos(b), 0.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=float,
        )

        Ma = np.array(
            [
                [1.0, 0.0, 0.0],
                [0.0, math.cos(a), math.sin(a)],
                [0.0, -math.sin(a), math.cos(a)],
            ],
            dtype=float,
        )

        # MATLAB:
        #   S1 = Magz.' * Magx.' * Magy.' * Mb.' * Ma.' * S;
        S1 = Magz.T @ Magx.T @ Magy.T @ Mb.T @ Ma.T @ S
        Rx, Ry, Rz = S1.tolist()

        # -----------------------------------------------------
        # 2. 离散模型矩阵 A1/B1
        # -----------------------------------------------------
        A1 = np.eye(Nx, dtype=float)

        M = np.array(
            [
                [
                    0.0,
                    math.sin(a) * math.sin(b) * math.sin(gy)
                    - math.cos(a) * math.cos(gy),
                    -math.sin(a) * math.cos(b) * math.cos(gy),
                    -math.cos(a) * math.sin(b) * math.cos(gy)
                    + math.sin(a) * math.sin(gy),
                ],
                [
                    -math.sin(a) * math.cos(b) * math.sin(gx)
                    - math.sin(a) * math.sin(b) * math.sin(gy) * math.cos(gx)
                    + math.cos(a) * math.cos(gy) * math.cos(gx),

                    -math.sin(a) * math.sin(b) * math.cos(gy) * math.sin(gx)
                    - math.cos(a) * math.sin(gy) * math.sin(gx),

                    -math.sin(a) * math.sin(b) * math.cos(gx)
                    - math.sin(a) * math.cos(b) * math.sin(gy) * math.sin(gx),

                    math.cos(a) * math.cos(b) * math.cos(gx)
                    - math.cos(a) * math.sin(b) * math.sin(gy) * math.sin(gx)
                    - math.sin(a) * math.cos(gy) * math.sin(gx),
                ],
                [
                    math.sin(a) * math.cos(b) * math.cos(gx)
                    - math.sin(a) * math.sin(b) * math.sin(gy) * math.sin(gx)
                    + math.cos(a) * math.cos(gy) * math.sin(gx),

                    math.sin(a) * math.sin(b) * math.cos(gy) * math.cos(gx)
                    + math.cos(a) * math.sin(gy) * math.cos(gx),

                    -math.sin(a) * math.sin(b) * math.sin(gx)
                    + math.sin(a) * math.cos(b) * math.sin(gy) * math.cos(gx),

                    math.cos(a) * math.cos(b) * math.sin(gx)
                    + math.cos(a) * math.sin(b) * math.sin(gy) * math.cos(gx)
                    + math.sin(a) * math.cos(gy) * math.cos(gx),
                ],
            ],
            dtype=float,
        )

        # MATLAB:
        #   B1 = T * Magz.' * M
        B1 = T * (Magz.T @ M)

        # -----------------------------------------------------
        # 3. 增广状态 kesi = [Rx Ry Rz U1 U2 U3 U4]'
        # -----------------------------------------------------
        kesi = np.zeros(Nx + Nu, dtype=float)
        kesi[0:3] = [Rx, Ry, Rz]
        kesi[3:7] = self.U

        # -----------------------------------------------------
        # 4. 权重矩阵
        # -----------------------------------------------------
        Q = self.Q_weight * np.eye(Nx * Np, dtype=float)
        R = np.diag(self.R0)
        P = self.P_scale * np.diag(self.P0)

        # -----------------------------------------------------
        # 5. 增广系统矩阵
        # -----------------------------------------------------
        A_aug = np.block(
            [
                [A1, B1],
                [np.zeros((Nu, Nx), dtype=float), np.eye(Nu, dtype=float)],
            ]
        )

        B_aug = np.vstack(
            [
                B1,
                np.eye(Nu, dtype=float),
            ]
        )

        C_aug = np.hstack(
            [
                np.eye(Nx, dtype=float),
                np.zeros((Nx, Nu), dtype=float),
            ]
        )

        # -----------------------------------------------------
        # 6. 预测矩阵 PHI / THETA
        # -----------------------------------------------------
        PHI = np.vstack(
            [
                C_aug @ np.linalg.matrix_power(A_aug, i)
                for i in range(1, Np + 1)
            ]
        )

        theta_rows = []
        for i in range(1, Np + 1):
            row_blocks = []
            for j in range(1, Nc + 1):
                if i >= j:
                    row_blocks.append(
                        C_aug
                        @ np.linalg.matrix_power(A_aug, i - j)
                        @ B_aug
                    )
                else:
                    row_blocks.append(np.zeros((Nx, Nu), dtype=float))
            theta_rows.append(row_blocks)

        THETA = np.block(theta_rows)

        # -----------------------------------------------------
        # 7. 控制增量累计矩阵 A_I
        # -----------------------------------------------------
        A_t = np.tril(np.ones((Nc, Nc), dtype=float))
        A_I = np.kron(A_t, np.eye(Nu, dtype=float))

        Ut_t = np.kron(np.ones(Nc, dtype=float), self.U)
        theta_pos = np.array([gx, gy, b, a], dtype=float)
        weizhi = np.kron(np.ones(Nc, dtype=float), theta_pos)

        # -----------------------------------------------------
        # 8. 二次规划目标函数
        #
        # MATLAB:
        #   H = 2*(THETA'*Q*THETA + R + (A_I*A_I)'*P*(A_I*A_I));
        #   error = PHI*kesi - Y_ref;
        #   Z = 2*error'*Q*THETA + 2*(weizhi + A_I*Ut_t)'*P*(A_I*A_I);
        #
        # Python 目标：
        #   min 0.5*x'H*x + f'x
        # -----------------------------------------------------
        AII = A_I @ A_I

        H = 2.0 * (
            THETA.T @ Q @ THETA
            + R
            + AII.T @ P @ AII
        )

        H = 0.5 * (H + H.T)  # 数值对称化

        error = PHI @ kesi - y_ref

        f = (
            2.0 * error.T @ Q @ THETA
            + 2.0 * (weizhi + A_I @ Ut_t).T @ P @ AII
        )

        # -----------------------------------------------------
        # 9. 动态速度约束
        #
        # 速度限制单位 rad/s：
        #   vel_max_deg_s * pi/180
        #
        # 角度限制单位 rad：
        #   pos_max_deg * pi/180
        # -----------------------------------------------------
        val_vel_max = np.radians(np.asarray(self.vel_max_deg_s, dtype=float))
        val_vel_min = -val_vel_max

        val_pos_max = np.radians(np.asarray(self.pos_max_deg, dtype=float))
        val_pos_min = np.radians(np.asarray(self.pos_min_deg, dtype=float))

        v_lim_upper = (val_pos_max - theta_pos) / T
        v_lim_lower = (val_pos_min - theta_pos) / T

        safe_vel_max = np.minimum(val_vel_max, v_lim_upper)
        safe_vel_min = np.maximum(val_vel_min, v_lim_lower)

        # 如果当前角度已经越界，可能出现下限大于上限。
        # 这里把上下限折中到同一个值，避免优化器直接崩掉。
        for k in range(Nu):
            if safe_vel_min[k] > safe_vel_max[k]:
                mid = 0.5 * (safe_vel_min[k] + safe_vel_max[k])
                safe_vel_min[k] = mid
                safe_vel_max[k] = mid

        Umax_vec = np.kron(np.ones(Nc, dtype=float), safe_vel_max)
        Umin_vec = np.kron(np.ones(Nc, dtype=float), safe_vel_min)

        # 约束：
        #   A_I * delta_U <= Umax - U_prev
        #  -A_I * delta_U <= -Umin + U_prev
        A_cons = np.vstack([A_I, -A_I])
        B_cons = np.hstack([Umax_vec - Ut_t, -Umin_vec + Ut_t])

        # -----------------------------------------------------
        # 10. 控制增量上下限
        # 单位 rad/s
        # -----------------------------------------------------
        delta_limit = math.radians(float(self.delta_u_limit_deg_s))
        lb = -delta_limit * np.ones(Nc * Nu, dtype=float)
        ub = delta_limit * np.ones(Nc * Nu, dtype=float)
        bounds = list(zip(lb, ub))

        # -----------------------------------------------------
        # 11. 求解二次规划
        # 使用 OSQP 替代 SLSQP。
        #
        # 目标：
        #   min 0.5*x'H*x + f'x
        #
        # 约束：
        #   A_cons*x <= B_cons
        #   lb <= x <= ub
        #
        # OSQP 标准形式：
        #   min 0.5*x'P*x + q'x
        #   s.t. l_osqp <= A_osqp*x <= u_osqp
        # -----------------------------------------------------

        n_var = Nc * Nu

        # 数值对称化，并加一个很小的正则项，避免半正定/病态矩阵导致数值问题
        P = 0.5 * (H + H.T)
        P = P + 1e-9 * np.eye(n_var)

        q = np.asarray(f, dtype=float).reshape(-1)

        # 线性不等式：A_cons*x <= B_cons
        # 写成：-inf <= A_cons*x <= B_cons
        A_ineq = A_cons
        l_ineq = -np.inf * np.ones(A_cons.shape[0])
        u_ineq = B_cons

        # 变量边界：lb <= x <= ub
        A_bound = np.eye(n_var)
        l_bound = lb
        u_bound = ub

        # 合并约束
        A_osqp = np.vstack([A_ineq, A_bound])
        l_osqp = np.hstack([l_ineq, l_bound])
        u_osqp = np.hstack([u_ineq, u_bound])

        # 转稀疏矩阵
        P_sparse = sp.csc_matrix(P)
        A_sparse = sp.csc_matrix(A_osqp)

        solver = osqp.OSQP()
        solver.setup(
            P=P_sparse,
            q=q,
            A=A_sparse,
            l=l_osqp,
            u=u_osqp,
            verbose=False,
            polish=True,
            eps_abs=1e-6,
            eps_rel=1e-6,
            max_iter=4000,
        )

        result = solver.solve()

        status = result.info.status.lower()

        if status in ("solved", "solved inaccurate"):
            x_opt = np.asarray(result.x, dtype=float)
            delta_u = x_opt[0:Nu]
            ok = True
            message = result.info.status
            objective_value = float(result.info.obj_val)
        else:
            # 这里才是真正 QP 求解失败。
            # 先按 MATLAB 逻辑处理：delta_u = 0，保持当前速度。
            delta_u = np.zeros(Nu, dtype=float)
            ok = False
            message = result.info.status
            objective_value = float(result.info.obj_val) if result.info.obj_val is not None else 0.0

        # -----------------------------------------------------
        # 12. 更新 U
        # -----------------------------------------------------
        self.U = self.U + delta_u

        # 数值保护：限制 U 不超过当前安全速度边界。
        self.U = np.minimum(np.maximum(self.U, safe_vel_min), safe_vel_max)

        return self.U.copy(), ok, message, objective_value

    # =========================================================
    # 输入处理
    # =========================================================
    def _build_reference(self, state: Dict[str, Any]) -> np.ndarray:
        """
        构造 Y_ref，长度 Nx*Np = 9。

        支持两种输入：

        1. target_sequence:
            直接给 9 个值：
                [Rx1,Ry1,Rz1, Rx2,Ry2,Rz2, Rx3,Ry3,Rz3]

        2. target_vector:
            只给一个目标向量：
                [Rx_ref,Ry_ref,Rz_ref]
            自动重复 Np 次。
        """
        if "target_sequence" in state and state["target_sequence"] is not None:
            seq = np.asarray(state["target_sequence"], dtype=float)

            if seq.shape == (self.Np, self.Nx):
                return seq.reshape(self.Np * self.Nx)

            if seq.size == self.Np * self.Nx:
                return seq.reshape(self.Np * self.Nx)

            raise ValueError(
                f"target_sequence 必须是长度 {self.Np * self.Nx}，"
                f"或形状 ({self.Np}, {self.Nx})"
            )

        target_vector = np.asarray(
            state.get("target_vector", [0.0, 0.0, -1.0]),
            dtype=float,
        )

        if target_vector.size != self.Nx:
            raise ValueError("target_vector 必须包含 3 个值：[Rx_ref, Ry_ref, Rz_ref]")

        return np.tile(target_vector.reshape(self.Nx), self.Np)

    @staticmethod
    def _get_first(data: Dict[str, Any], keys: Sequence[str], default: float) -> float:
        for key in keys:
            if key in data and data[key] is not None:
                return float(data[key])
        return float(default)


if __name__ == "__main__":
    # 简单自检：不连接硬件，只验证优化器能正常运行。
    mpc = MPCController()

    test_state = {
        "x_angle_deg": 0.5,
        "y_angle_deg": -0.3,
        "z_angle_deg": 0.0,
        "azimuth_angle_deg": 0.0,
        "lift_angle_deg": 10.0,
        "target_vector": [0.0, 0.0, -1.0],
    }

    for idx in range(5):
        cmd = mpc.compute(test_state)
        print(idx, cmd)

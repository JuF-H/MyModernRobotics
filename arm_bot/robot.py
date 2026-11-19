import numpy as np
import modern_robotics as mr


class Robot:
    """机械臂运动学核心：初始化、关节限位、正逆运动学与状态管理。

    本类不含绘图，可视化见 arm_bot.viz；
    具体机器人参数（螺丝轴、零位、连杆长度）见 arm_bot.arm_3R 等模型文件。
    """

    def __init__(self, name):
        self.name = name
        self.S_space = None      # 空间坐标系中的螺旋轴（零位）
        self.M_sb = None         # 机器人初始位姿（零位）
        self.q0 = None           # 初始关节角(rad)
        self.q_limit = None      # 关节角限位(rad), shape (2, N)

        self.T_c = None          # 机器人当前位姿矩阵
        self.thetalist_c = None  # 机器人当前关节角

    # ------------------------------------------------------------------
    # ---------- 限位相关工具 ----------

    def is_within_limit(self, q, tol=1e-9):
        """判断一组关节角是否都在限位内"""
        q = np.asarray(q, dtype=float)
        return bool(np.all(q >= self.q_limit[0] - tol) and
                    np.all(q <= self.q_limit[1] + tol))

    def clip_q(self, q):
        """把关节角裁剪到限位内"""
        q = np.asarray(q, dtype=float)
        return np.clip(q, self.q_limit[0], self.q_limit[1])

    def _wrap_to_limit(self, q, tol=1e-9):
        """
        把每个关节角卷绕到“限位内且离原值最近”的等效角（±2π 系列）。
        若某个关节不存在这样的等效角，返回 None。
        """
        q = np.asarray(q, dtype=float)
        out = np.empty_like(q)
        for i in range(q.size):
            lo, hi = self.q_limit[0, i], self.q_limit[1, i]
            # 把 q[i] 卷绕到 [lo, lo + 2π)
            base = lo + np.mod(q[i] - lo, 2.0 * np.pi)
            cands = [base - 2.0 * np.pi, base, base + 2.0 * np.pi]
            valid = [c for c in cands if lo - tol <= c <= hi + tol]
            if not valid:
                return None
            out[i] = min(valid, key=lambda c: abs(c - q[i]))
        return out

    # ------------------------------------------------------------------
    # ---------- 初始化与状态 ----------

    def robot_init(self, S_space, M_sb, q0, q_limit):
        """
        初始化机器人，设置初始位姿
        :param S_space: 空间坐标系中的螺旋轴（零位）, shape (6, N)
        :param M_sb: 机器人初始位姿（零位）
        :param q0: 初始关节角(rad)
        :param q_limit: 关节角限位(rad), shape (2, N)，第 0 行下界，第 1 行上界
        """
        S_space = np.asarray(S_space, dtype=float)
        n = S_space.shape[1]

        q_limit = np.asarray(q_limit, dtype=float)
        if q_limit.shape != (2, n):
            raise ValueError(f"q_limit 形状应为 (2, {n})，实际为 {q_limit.shape}")
        if np.any(q_limit[0] > q_limit[1]):
            raise ValueError("q_limit 下界必须 <= 上界")

        self.S_space = S_space
        self.M_sb = np.asarray(M_sb, dtype=float)
        self.q0 = np.asarray(q0, dtype=float).reshape(n)
        self.q_limit = q_limit

        # 初始位形必须合法
        if not self.is_within_limit(self.q0):
            raise ValueError(f"q0={np.round(self.q0, 4)} 超出关节限位 "
                             f"[{np.round(q_limit[0], 4)}, {np.round(q_limit[1], 4)}]")

        self.T_c = self.FK(self.q0)
        self.thetalist_c = self.q0.copy()

    def set_q(self, q, strict=True):
        """
        在当前关节角基础上叠加增量
        :param q: 关节角增量(rad)
        :param strict: True 时越界拒绝执行并返回 False；False 时裁剪到限位
        :return: 是否成功执行
        """
        q_new = self.thetalist_c + np.asarray(q, dtype=float)

        if not self.is_within_limit(q_new):
            if strict:
                print(f"超出关节限位，拒绝执行：q_new={np.round(q_new, 4)}, "
                      f"限位下界={np.round(self.q_limit[0], 4)}, "
                      f"限位上界={np.round(self.q_limit[1], 4)}")
                return False
            q_new = self.clip_q(q_new)
            print(f"超出关节限位，已裁剪至：{np.round(q_new, 4)}")

        self.thetalist_c = q_new
        self.T_c = self.FK(self.thetalist_c)
        return True

    def get_Tc(self):
        """获取当前位姿矩阵"""
        return self.T_c.copy()

    def print_Tc(self):
        """打印当前位姿矩阵"""
        print("当前位姿矩阵为：\n", np.round(self.T_c, 4))

    def print_q_z(self):
        """打印当前关节角,相对于零位"""
        print("当前关节角,相对于零位为：", np.round(self.thetalist_c, 4))

    def get_q_z(self):
        """获取当前关节角,相对于零位"""
        return self.thetalist_c.copy()

    def print_q(self):
        """打印当前关节角,相对于初始位姿"""
        print("当前关节角,相对于初始位姿为：",
              np.round(self.thetalist_c - self.q0, 4))

    def get_q(self):
        """获取当前关节角,相对于初始位姿"""
        return (self.thetalist_c - self.q0).copy()

    # ------------------------------------------------------------------
    # ---------- 正逆运动学 ----------

    def FK(self, thetalist):
        """
        前向运动学
        :param thetalist: 关节角度(rad)
        :return: 对应的位姿矩阵
        """
        T = mr.FKinSpace(self.M_sb, self.S_space, thetalist)
        return T

    def IK_n(self, thetalist0, T_target, eomg=1e-3, ev=1e-3):
        """
        逆向运动学（带限位）
        :param thetalist0: 初始关节角度(rad)
        :param T_target: 目标位姿矩阵
        :param eomg: 姿态误差
        :param ev: 位置误差
        :return: 成功且解在限位内返回关节角(rad)，否则返回 None
        """
        thetalist, success = mr.IKinSpace(
            self.S_space, self.M_sb, T_target, thetalist0, eomg, ev)

        if not success:
            return None

        # 卷绕到限位内且离原值最近的等效角
        q_wrapped = self._wrap_to_limit(thetalist)
        if q_wrapped is None:
            return None

        return q_wrapped

    def move_to_T(self, T_target):
        """
        移动到目标位姿矩阵（限位不满足时不执行）
        :param T_target: 目标位姿矩阵
        :return: 成功则返回新关节角，否则返回 None
        """
        thetalist = self.IK_n(self.thetalist_c, T_target, eomg=1e-3, ev=1e-3)
        if thetalist is None:
            print("IK失败或解超出关节限位，无法移动到目标位姿矩阵")
            return None

        self.thetalist_c = thetalist
        self.T_c = self.FK(thetalist)
        print("移动到目标位姿矩阵：\n", np.round(self.T_c, 4))
        return self.thetalist_c

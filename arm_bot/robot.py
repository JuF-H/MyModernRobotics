import numpy as np
import modern_robotics as mr
import expendlib as excore


class Robot:
    """机械臂运动学核心：初始化、关节限位、正逆运动学与状态管理。

    面向对象设计：
        - 所有状态（当前位姿、关节角、雅可比、速度）都存在对象属性里
        - update_*()  从当前状态重算派生量，无参数
        - move_*()    修改状态（先改关节角/目标，再 update_FK）
        - get_*/print_*  读取状态

    本类不含绘图，可视化见 arm_bot.viz；
    具体机器人参数（螺丝轴、零位、连杆长度）见 arm_bot.arm_3R 等模型文件。
    """

    def __init__(self, name):
        self.name = name
        self.S_space = None         # 空间坐标系中的螺旋轴（零位）
        self.M_sb = None            # 机器人初始位姿（零位）
        self.q0 = None              # 初始关节角(rad), shape (N,)
        self.q_limit = None         # 关节角限位(rad), shape (2, N)

        self.T_c = None             # 当前位姿矩阵, shape (4, 4)
        self.T_target = None        # 目标位姿矩阵, shape (4, 4)
        self.q_target = None        # 目标关节角（相对于初始位姿）, shape (N,)
        self.thetalist_c = None     # 当前关节角（相对于零位）, shape (N,)
        self.q_c = None             # 当前关节角（相对于初始位姿）, shape (N,)
        self.Js_c = None            # 空间雅可比矩阵, shape (6, N)
        self.Jb_c = None            # 物体雅可比矩阵, shape (6, N)
        self.Jg_c = None            # 几何雅可比矩阵, shape (6, N)
        self.dq_c = None            # 关节速度, shape (N,)
        self.ddq_c = None           # 关节加速度, shape (N,)
        self.Vs = None              # 空间运动旋量, shape (6,)
        self.Vb = None              # 本体运动旋量, shape (6,)
        self.Vg = None              # 几何速度旋量, shape (6,)
        self.eomg = 1e-3            # 逆运动学姿态容差
        self.ev = 1e-3              # 逆运动学位置容差

# **************************************************
# 初始化机器人
# **************************************************
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
        self.q_limit = q_limit
        # 统一成 (N,)：传 (N,1) 会在 is_within_limit 里与 q_limit[0] 错误广播成 (N,N)
        q0 = np.asarray(q0, dtype=float).reshape(n)

        # 初始位形必须合法
        if not excore.is_within_limit(q0, self.q_limit):
            raise ValueError(f"q0={np.round(q0, 4)} 超出关节限位 "
                             f"[{np.round(q_limit[0], 4)}, {np.round(q_limit[1], 4)}]")

        # 初始状态
        self.q0 = q0.copy()
        self.thetalist_c = q0.copy()
        self.q_c = np.zeros(n)
        self.dq_c = np.zeros(n)
        self.ddq_c = np.zeros(n)

        self.q_target = np.zeros(n)
        self.update_FK()
        self.T_target = self.T_c.copy()



# **************************************************
# 正运动学
# **************************************************
    def FK(self, thetalist):
        """
        正运动学（纯计算，不修改对象状态）
        :param thetalist: 关节角（相对于零位）, shape (N,)
        :return: 对应的位姿矩阵, shape (4, 4)
        """
        return mr.FKinSpace(self.M_sb, self.S_space, thetalist)

    def update_FK(self):
        """
        用当前关节角 thetalist_c 更新当前位姿 T_c
        :return: 当前位姿矩阵
        """
        self.T_c = self.FK(self.thetalist_c)
        return self.T_c.copy()

# **************************************************
# 逆运动学（数值解）
# **************************************************
    def IK_n(self):
        """
        逆向运动学（数值解，纯计算，不修改对象状态）。

        目标为 self.T_target，初值为当前关节角 thetalist_c，
        容差取 self.eomg / self.ev。
        :return: 成功且解在限位内返回关节角（相对于零位），否则返回 None
        """
        if self.T_target is None:
            return None

        thetalist, success = mr.IKinSpace(
            self.S_space, self.M_sb, self.T_target, self.thetalist_c, self.eomg, self.ev)

        if not success:
            return None

        # 卷绕到限位内且离原值最近的等效角
        q_wrapped = excore.wrap_to_limit(thetalist, self.q_limit)
        if q_wrapped is None:
            return None
        return q_wrapped

# **************************************************
# 雅可比
# **************************************************
    def update_Js(self):
        """
        更新空间雅可比
        :return: 对应的空间雅可比矩阵, shape (6, N)
        """
        self.Js_c = mr.JacobianSpace(self.S_space, self.thetalist_c)
        return self.Js_c.copy()

    def update_Jb(self):
        """
        更新物体雅可比
        :return: 对应的物体雅可比矩阵, shape (6, N)
        """
        self.update_FK()
        self.update_Js()
        self.Jb_c = excore.Js2Jb(self.Js_c, self.T_c)
        return self.Jb_c.copy()

    def update_Jg(self):
        """
        更新几何雅可比
        :return: 对应的几何雅可比矩阵, shape (6, N)
        """
        self.update_FK()
        self.update_Js()
        self.Jg_c = excore.Js2Jg(self.Js_c, self.T_c)
        return self.Jg_c.copy()

# **************************************************
# 速度
# **************************************************
    def update_Vs(self):
        """
        更新空间运动旋量 Vs = Js · dq
        :return: 空间运动旋量, shape (6,)
        """
        self.update_Js()
        self.Vs = self.Js_c @ self.dq_c
        return self.Vs.copy()

    def update_Vb(self):
        """
        更新本体运动旋量 Vb = Jb · dq
        :return: 本体运动旋量, shape (6,)
        """
        self.update_Jb()
        self.Vb = self.Jb_c @ self.dq_c
        return self.Vb.copy()

    def update_Vg(self):
        """
        更新几何速度 Vg = Jg · dq
        :return: 几何速度旋量, shape (6,)
        """
        self.update_Jg()
        self.Vg = self.Jg_c @ self.dq_c
        return self.Vg.copy()


# **************************************************
# 机械臂运动方法
# **************************************************
    def move(self, verbose=True):
        """
        移动到 self.T_target。

        只有成功才会修改状态；失败（IK 无解或解超限）时所有状态保持不变。
        :param verbose: 是否打印提示信息
        :return: 成功返回 True，失败返回 False
        """
        thetalist = self.IK_n()
        if thetalist is None:
            if verbose:
                print("IK失败或解超出关节限位，无法移动到目标位姿矩阵")
            return False

        self.thetalist_c = thetalist
        self.q_c = thetalist - self.q0
        self.q_target = self.q_c.copy()
        self.update_FK()
        if verbose:
            print("移动到目标位姿矩阵：\n", np.round(self.T_c, 4))
        return True

    def move_T(self, T_target, verbose=True):
        """
        移动到目标位姿矩阵。

        成功：更新所有状态参数（thetalist_c / q_c / q_target / T_c 等）；
        失败（目标超出工作空间、IK 无解或解超限）：所有状态参数保持
        移动之前的值，self.T_target 也回滚为原值。

        :param T_target: 目标位姿矩阵, shape (4, 4)
        :param verbose: 是否打印提示信息
        :return: 成功返回 True，失败返回 False
        """
        T_target = np.asarray(T_target, dtype=float)
        if T_target.shape != (4, 4):
            raise ValueError(f"T_target 形状应为 (4, 4)，实际为 {T_target.shape}")

        T_old = self.T_target              # 暂存，失败时回滚
        self.T_target = T_target

        ok = self.move(verbose=verbose)    # move() 只有成功才会改状态

        if not ok:
            self.T_target = T_old          # 移动失败，目标位姿还原
        return ok

    def move_q(self, q, strict=True):
        """
        移动到目标关节角，相对于初始关节角 q0
        :param q: 目标关节角（相对于初始关节角）, shape (N,)
        :param strict: True 时越界拒绝执行并返回 False；False 时裁剪到限位
        :return: 是否成功执行
        """
        q = np.asarray(q, dtype=float)
        if q.shape != self.q0.shape:
            raise ValueError(f"q 形状应为 {self.q0.shape}，实际为 {q.shape}")

        q_new = self.q0 + q

        if not excore.is_within_limit(q_new, self.q_limit):
            if strict:
                # 失败：不改任何状态（含 q_target）
                print(f"超出关节限位，拒绝执行：q_new={np.round(q_new, 4)}, "
                      f"限位下界={np.round(self.q_limit[0], 4)}, "
                      f"限位上界={np.round(self.q_limit[1], 4)}")
                return False
            q_new = excore.clip_q(q_new, self.q_limit)
            print(f"超出关节限位，已裁剪至（相对于零位）：{np.round(q_new, 4)}")

        # 成功才更新状态
        self.q_target = q.copy()
        self.thetalist_c = q_new
        self.q_c = q_new - self.q0      # 用实际生效的值，而非请求值
        self.update_FK()
        return True


# **************************************************
# 打印参数
# **************************************************
    def print_q_z(self):
        """打印当前关节角,相对于零位"""
        print("当前关节角,相对于零位为：", np.round(self.thetalist_c, 4))

    def print_q(self):
        """打印当前关节角,相对于初始位姿"""
        print("当前关节角,相对于初始位姿为：", np.round(self.q_c, 4))

    def print_Tc(self):
        """打印当前位姿矩阵"""
        print("当前位姿矩阵为：\n", np.round(self.T_c, 4))

    def print_Js(self):
        """打印当前空间雅可比矩阵"""
        print("当前空间雅可比矩阵为：\n", np.round(self.get_Js(), 4))

    def print_Jb(self):
        """打印当前物体雅可比矩阵"""
        print("当前物体雅可比矩阵为：\n", np.round(self.get_Jb(), 4))

    def print_Jg(self):
        """打印当前几何雅可比矩阵"""
        print("当前几何雅可比矩阵为：\n", np.round(self.get_Jg(), 4))


# **************************************************
# 获取参数
# **************************************************
    def get_q_z(self):
        """获取当前关节角,相对于零位"""
        return self.thetalist_c.copy()

    def get_q(self):
        """获取当前关节角,相对于初始位姿"""
        return self.q_c.copy()

    def get_Tc(self):
        """获取当前位姿矩阵"""
        return self.T_c.copy()

    def get_T_target(self):
        """获取目标位姿矩阵（未设置时返回 None）"""
        return None if self.T_target is None else self.T_target.copy()

    def get_Js(self):
        """获取当前空间雅可比矩阵"""
        return self.update_Js()

    def get_Jb(self):
        """获取当前物体雅可比矩阵"""
        return self.update_Jb()

    def get_Jg(self):
        """获取当前几何雅可比矩阵"""
        return self.update_Jg()

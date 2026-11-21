import numpy as np
import modern_robotics as mr


# ---------- 限位相关工具 ----------

def is_within_limit(q, q_limit, tol=1e-9):
    """
    判断一组关节角是否都在限位内
    :param q: 关节角度(rad)
    :param q_limit: 关节限位，维度为 2×n
    :param tol: 误差容限
    :return: 是否都在限位内
    """
    q = np.asarray(q, dtype=float)
    return bool(np.all(q >= q_limit[0] - tol) and
                np.all(q <= q_limit[1] + tol))


def clip_q(q, q_limit):
    """
    把关节角裁剪到限位内
    :param q: 关节角度(rad)
    :param q_limit: 关节限位，维度为 2×n
    :return: 裁剪后的关节角度(rad)
    """
    q = np.asarray(q, dtype=float)
    return np.clip(q, q_limit[0], q_limit[1])


def wrap_to_limit(q, q_limit, tol=1e-9):
    """
    把每个关节角卷绕到“限位内且离原值最近”的等效角（±2π 系列）。
    若某个关节不存在这样的等效角，返回 None。
    :param q: 关节角度(rad)
    :param q_limit: 关节限位，维度为 2×n
    :param tol: 误差容限
    :return: 卷绕后的关节角度(rad)
    """
    q = np.asarray(q, dtype=float)
    out = np.empty_like(q)
    for i in range(q.size):
        lo, hi = q_limit[0, i], q_limit[1, i]
        # 把 q[i] 卷绕到 [lo, lo + 2π)
        base = lo + np.mod(q[i] - lo, 2.0 * np.pi)
        cands = [base - 2.0 * np.pi, base, base + 2.0 * np.pi]
        valid = [c for c in cands if lo - tol <= c <= hi + tol]
        if not valid:
            return None
        out[i] = min(valid, key=lambda c: abs(c - q[i]))
    return out


# ---------- 逆运动学 ----------

def IK_space_damped(Slist, M, T, thetalist0, eomg, ev,
                    lam0=1e-2, lam_up=10.0, lam_down=0.5,
                    lam_min=1e-12, lam_max=1e8, maxiter=200):
    """
    空间雅可比下的逆运动学：阻尼最小二乘（Levenberg–Marquardt）。

    与 modern_robotics.IKinSpace（无阻尼牛顿法，步长 = pinv(J)·V）的区别：
    奇异位形附近 J 的最小奇异值趋于 0，伪逆步长会爆炸导致迭代发散。
    本函数用阻尼步长

        Δθ = Jᵀ (J Jᵀ + λ² I)⁻¹ V

    λ 自适应调节：误差下降就减小 λ（趋近牛顿法、收敛快），
    误差不降就增大 λ（步长受限、不会发散），从而在奇异位形下仍能收敛。

    收敛判定用真实位姿误差（末端到目标的平移距离 ‖p‖、旋转向量范数 ‖ω‖），
    而不是旋量分量范数——后者在存在姿态偏差时与真实位置误差可差数倍，
    会导致“已收敛”解的实际位置误差超过 ev。

    :param Slist: 螺旋轴矩阵（每列一个关节轴）, shape (6, N)
    :param M: 零位时的末端位姿, shape (4, 4)
    :param T: 目标位姿 Tsd, shape (4, 4)
    :param thetalist0: 初值关节角, shape (N,)
    :param eomg: 姿态误差容差（旋转向量范数）
    :param ev: 位置误差容差（末端到目标的欧氏距离）
    :param lam0: 初始阻尼系数
    :param lam_up: 误差不降时 λ 的放大倍数
    :param lam_down: 误差下降时 λ 的缩小倍数
    :param lam_min / lam_max: λ 的上下限（λ 到上限说明已无法继续下降）
    :param maxiter: 最大迭代次数
    :return: (thetalist, success)，success=False 表示未在容差内收敛
             （目标不可达，或迭代已停滞）
    """
    Slist = np.asarray(Slist, dtype=float)
    M = np.asarray(M, dtype=float)
    T = np.asarray(T, dtype=float)

    th = np.array(thetalist0, dtype=float).copy()
    lam = float(lam0)
    I6 = np.eye(6)

    def err_of(th_):
        """
        当前关节角下的误差：
        :return: (Vb, w, p)
            Vb 本体系旋量误差 [ω; v]，用于构造牛顿步长
            w  旋转向量误差（本体系）
            p  位置误差（本体系，范数 = 末端到目标的欧氏距离）
        """
        Tsb = mr.FKinSpace(M, Slist, th_)
        E = mr.TransInv(Tsb) @ T              # 目标相对当前位姿
        Vb = mr.se3ToVec(mr.MatrixLog6(E))
        return Vb, Vb[:3], E[:3, 3]

    Vb, w, p = err_of(th)
    cost = np.linalg.norm(w) / eomg + np.linalg.norm(p) / ev
    for _ in range(maxiter):
        if np.linalg.norm(w) <= eomg and np.linalg.norm(p) <= ev:
            return th, True

        # 空间雅可比对应空间旋量误差 V = Ad(Tsb)·Vb
        Tsb = mr.FKinSpace(M, Slist, th)
        Vs = mr.Adjoint(Tsb) @ Vb
        J = mr.JacobianSpace(Slist, th)
        # 阻尼最小二乘步长（λ 大时步长被压住，不会在奇异方向上炸开）
        dth = J.T @ np.linalg.solve(J @ J.T + (lam ** 2) * I6, Vs)

        Vb_new, w_new, p_new = err_of(th + dth)
        cost_new = np.linalg.norm(w_new) / eomg + np.linalg.norm(p_new) / ev
        if cost_new < cost:
            th = th + dth              # 接受步长
            Vb, w, p, cost = Vb_new, w_new, p_new, cost_new
            lam = max(lam * lam_down, lam_min)
        else:
            lam = min(lam * lam_up, lam_max)   # 拒绝步长，加大阻尼重试
            if lam >= lam_max:
                return th, False   # 已停滞：目标不可达或到达收敛极限

    return th, False


def Js2Jg(Js, T):
    """
    空间雅可比 --> 几何雅可比（实际广义速度映射矩阵）

    参数:
        Js: 空间雅可比矩阵，维度为 6×n
            
        T: T_sb，维度为 4×4

    返回:
        Jg: 几何雅可比，维度为 6×n

    """

    Js = np.array(Js).copy().astype(float)
    T = np.array(T).copy().astype(float)

    p = T[:3, 3]
    p_hat = mr.VecToso3(p)
    Jg = np.block([
        [np.eye(3), np.zeros((3, 3))],
        [-p_hat, np.eye(3)]
    ]) @ Js

    return Jg


def Jb2Jg(Jb, T):
    """
    物体雅可比 --> 几何雅可比（实际广义速度映射矩阵）

    参数:
        Jb: 物体雅可比矩阵，维度为 6×n

        T: T_sb，维度为 4×4

    返回:
        Jg: 几何雅可比，维度为 6×n
    """

    Jb = np.array(Jb).copy().astype(float)
    T = np.array(T).copy().astype(float)

    R_sb = T[:3, :3]
    Jg = np.block([
        [R_sb, np.zeros((3, 3))],
        [np.zeros((3, 3)), R_sb]
    ]) @ Jb

    return Jg


def Js2Jb(Js, T_sb):
    """
    空间雅可比 --> 物体雅可比

    参数:
        Js: 空间雅可比矩阵，维度为 6×n
        T_sb: 物体坐标系{B}相对于空间坐标系{S}的齐次变换矩阵，维度为 4×4

    返回:
        Jb: 物体雅可比矩阵，维度为 6×n
    """

    Js = np.array(Js).copy().astype(float)
    T_sb = np.array(T_sb).copy().astype(float)

    T_bs = mr.TransInv(T_sb)
    Ad_T_bs = mr.Adjoint(T_bs)

    Jb = Ad_T_bs @ Js

    return Jb


def Jb2Js(Jb, T_sb):
    """
    物体雅可比 --> 空间雅可比

    参数:
        Jb: 物体雅可比矩阵，维度为 6×n
        T_sb: 物体坐标系{B}相对于空间坐标系{S}的齐次变换矩阵，维度为 4×4

    返回:
        Js: 空间雅可比矩阵，维度为 6×n
    """
    Jb = np.array(Jb).copy().astype(float)
    T_sb = np.array(T_sb).copy().astype(float)

    Ad_T_sb = mr.Adjoint(T_sb)
    Js = Ad_T_sb @ Jb

    return Js


def A_of_r(r):
    """
    右雅可比矩阵

    参数：
        r: 指数坐标，维度为 3x1

    返回：
        A: 右雅可比矩阵，维度为 3x3
    """
    nr = np.linalg.norm(r)
    I3 = np.eye(3)
    if nr < 1e-10:
        # r=0 零旋转，A(r)≈I
        return I3
    r_hat = mr.VecToso3(r)
    term1 = (1 - np.cos(nr)) / (nr**2)
    term2 = (nr - np.sin(nr)) / (nr**3)
    A = I3 - term1 * r_hat + term2 * (r_hat @ r_hat)

    return A


def A_of_l(r):
    """
    左雅可比矩阵

    参数：
        r: 指数坐标，维度为 3x1

    返回：
        A: 左雅可比矩阵，维度为 3x3
    """
    nr = np.linalg.norm(r)
    I3 = np.eye(3)
    if nr < 1e-10:
        # r=0 零旋转，A(r)≈I
        return I3
    r_hat = mr.VecToso3(r)
    term1 = (1 - np.cos(nr)) / (nr**2)
    term2 = (nr - np.sin(nr)) / (nr**3)
    A = I3 + term1 * r_hat + term2 * (r_hat @ r_hat)
    
    return A


def Jb2Ja_r(Jb, T_sb):
    """
    物体雅可比 --> 解析雅可比(基于指数坐标)

    参数：
        Jb: 物体雅可比矩阵，维度为 6×n
        T_sb: 物体坐标系{B}相对于空间坐标系{S}的齐次变换矩阵，维度为 4×4

    返回：
        Ja: 解析雅可比矩阵，维度为 6×n

    """
    # 提取旋转矩阵R_sb
    R_sb = T_sb[0:3, 0:3]
    # 求旋转矩阵的指数坐标 r ∈ R^3
    r = mr.MatrixLog3(R_sb)
    r = mr.so3ToVec(r)  # 将反对称矩阵转换为旋转向量
    # 计算A(r) 并求逆 Ainv = A^{-1}(r)
    A = A_of_r(r)
    Ainv = np.linalg.inv(A)
    
    # 构造6×6分块对角矩阵
    M_block = np.block([
        [Ainv, np.zeros((3,3))],
        [np.zeros((3,3)), R_sb]
    ])
    # Ja = block @ Jb
    Ja = M_block @ Jb
    return Ja


def Js2Ja_r(Js, T_sb):
    """
    空间雅可比 --> 解析雅可比(基于指数坐标)

    参数：
        Js: 空间雅可比矩阵，维度为 6×n
        T_sb: 物体坐标系{B}相对于空间坐标系{S}的齐次变换矩阵，维度为 4×4

    返回：
        Ja: 解析雅可比矩阵，维度为 6×n

    """
    p = T_sb[0:3, 3]
    p_hat = mr.VecToso3(p)

    # 提取旋转矩阵R_sb
    R_sb = T_sb[0:3, 0:3]
    # 求旋转矩阵的指数坐标 r ∈ R^3
    r = mr.MatrixLog3(R_sb)
    r = mr.so3ToVec(r)  # 将反对称矩阵转换为旋转向量
    # 计算A(r) 并求逆 Ainv = A^{-1}(r)
    A = A_of_l(r)
    Ainv = np.linalg.inv(A)
    
    # 构造6×6分块对角矩阵
    M_block = np.block([
        [Ainv, np.zeros((3,3))],
        [-p_hat, np.eye(3)]
    ])
    # Ja = block @ Jb
    Ja = M_block @ Js
    return Ja

def mu_1_axis_ratio(J):
    """
    计算可操作度椭球的最长轴与最短轴半径之比 μ1(A)
    等价于雅可比矩阵的最大奇异值与最小奇异值之比
    
    参数:
        J: 雅可比矩阵 (numpy array, 维度 m x n)
    返回:
        μ1 值 (float)。如果矩阵奇异（最小奇异值为0），返回 np.inf。
    """
    # 计算雅可比矩阵的奇异值
    # compute_uv=False 表示只返回奇异值，不返回左右奇异向量，提高计算速度
    singular_values = np.linalg.svd(J, compute_uv=False)
    
    max_sv = np.max(singular_values)
    min_sv = np.min(singular_values)
    
    if min_sv < 1e-12:  # 处理奇异情况（避免除以0）
        return np.inf
    
    return max_sv / min_sv


def mu_2_condition_number(J):
    """
    计算矩阵 A = J*J^T 的条件数 μ2(A)
    由于 λ = σ^2，所以这也是最大奇异值与最小奇异值之比的平方。
    
    参数:
        J: 雅可比矩阵 (numpy array)
    返回:
        μ2 值 (float)。如果矩阵奇异，返回 np.inf。
    """
    singular_values = np.linalg.svd(J, compute_uv=False)
    
    max_sv = np.max(singular_values)
    min_sv = np.min(singular_values)
    
    if min_sv < 1e-12:
        return np.inf
    
    return (max_sv / min_sv) ** 2


def mu_3_volume(J):
    """
    计算可操作度椭球的体积度量 μ3(A)
    由于 det(J*J^T) = (det(J))^2 (对于方阵)，等价于所有奇异值的乘积。
    
    参数:
        J: 雅可比矩阵 (numpy array)
    返回:
        μ3 值 (float)。该值越大，表示椭球体积越大，操作能力越强。
    """
    singular_values = np.linalg.svd(J, compute_uv=False)
    
    # 椭球体积正比于特征值乘积的平方根，也就是奇异值的乘积
    volume_measure = np.prod(singular_values)
    
    return volume_measure
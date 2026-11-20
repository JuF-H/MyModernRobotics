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
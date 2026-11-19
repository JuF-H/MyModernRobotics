import numpy as np

def translation_matrix(tx, ty, tz):
    T = np.eye(4)
    T[:3, 3] = [tx, ty, tz]
    return T


def rotation_matrix_axis(axis, theta):
    """绕通过原点的任意轴旋转（Rodrigues 公式）"""
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c, s = np.cos(theta), np.sin(theta)
    C = 1 - c
    R = np.array([
        [x*x*C + c,   x*y*C - z*s, x*z*C + y*s],
        [y*x*C + z*s, y*y*C + c,   y*z*C - x*s],
        [z*x*C - y*s, z*y*C + x*s, z*z*C + c  ]
    ])
    T = np.eye(4)
    T[:3, :3] = R
    return T


def rotate_about_point(point, axis, theta):
    """
    绕空间中任意一点 point，沿方向 axis 旋转 theta 弧度
    :param point: 旋转轴上的一点 (px, py, pz)
    :param axis:  旋转轴方向向量 (ux, uy, uz)
    :param theta: 旋转角度（弧度）
    :return: 4x4 齐次变换矩阵
    """
    px, py, pz = point
    T_to_origin = translation_matrix(-px, -py, -pz)   # 1. 平移到原点
    R           = rotation_matrix_axis(axis, theta)   # 2. 旋转
    T_back      = translation_matrix(px, py, pz)      # 3. 移回原位

    return T_back @ R @ T_to_origin                   # 右→左作用

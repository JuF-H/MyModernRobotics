"""3R 平面机械臂模型。

零位构型：三节臂沿 x 轴伸展，各关节轴沿 z 方向。

连杆长度 L = (4, 3, 2)，三处必须保持一致：
    - LINK_LENGTHS      供可视化用
    - S 的 v 分量 y=-4/-7   = 关节 2/3 在零位的 x 坐标（连杆累加 4、4+3）
    - M 的末端 x=9         = 4 + 3 + 2
"""
import numpy as np

from .robot import Robot

# 各连杆长度
LINK_LENGTHS = (4.0, 3.0, 2.0)

# 空间坐标系中的螺旋轴（零位），shape (6, 3)
# 每列是一个螺丝轴 [ω; v]：ω = (0, 0, 1) 绕 z 轴，v = -ω × q_i
# q_i 为关节 i 在零位时轴上一点：关节1 (0,0)、关节2 (4,0)、关节3 (7,0)
S = np.column_stack([
    [0, 0, 1, 0, 0, 0],
    [0, 0, 1, 0, -4, 0],
    [0, 0, 1, 0, -7, 0],
]).astype(float)

# 零位末端位姿，末端在 (4+3+2, 0) = (9, 0)
M = np.array([
    [1, 0, 0, 9],
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1],
], dtype=float)

# 初始关节角(rad)
q0 = np.array([np.pi / 2, -np.pi / 2, -np.pi / 2])

# 关节角限位(rad), shape (2, 3)，第 0 行下界，第 1 行上界
q_limit = np.array([
    [-np.pi, -np.pi, -np.pi],
    [np.pi,  np.pi,  np.pi],
])


def make_robot(name="arm_3R"):
    """构造一个已完成初始化的 3R 机械臂。"""
    robot = Robot(name)
    robot.robot_init(S, M, q0, q_limit)
    return robot


if __name__ == "__main__":
    arm = make_robot()
    arm.print_q_z()
    arm.print_Tc()

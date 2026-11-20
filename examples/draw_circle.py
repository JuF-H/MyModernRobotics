"""画圆示例：3R 平面臂末端沿圆轨迹走一圈。

运行（项目根目录下）：python -m examples.draw_circle
"""
import numpy as np

from arm_bot import arm_3R, viz


def main():
    arm = arm_3R.make_robot()

    # 1) 末端姿态固定 = 初始姿态（否则姿态也变，IK 更容易失败）
    R = arm.get_Tc()[:3, :3].copy()

    # 2) 圆心、半径、帧数（中心 (4,0) 半径 2 完全在可达空间内）
    o = np.array([4.0, 0.0])
    r = 2.0
    N = 72

    # 3) 从当前末端所在的角度 π/2 出发，保证第一帧解 = q0
    angles = np.linspace(np.pi / 2, 2 * np.pi + np.pi / 2, N, endpoint=False)

    q_list = []

    for t in angles:
        x = o[0] + r * np.cos(t)
        y = o[1] + r * np.sin(t)

        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [x, y, 0.0]

        # 逐帧移动，状态自动更新，所以下一帧 IK 的初值就是上一帧的解。
        # verbose=False 关掉每帧的打印，避免 72 行刷屏。
        if arm.move_T(T, verbose=False):
            q_list.append(arm.get_q_z())   # 成功：记录移动后的关节角
        else:
            q_list.append(None)            # 失败帧放 None，动画里会跳过

    n_ok = sum(1 for q in q_list if q is not None)
    print(f"画圆：成功 {n_ok}/{N} 帧")

    # 4) 检查圆是否闭合（首尾关节角应接近）
    if q_list[0] is not None and q_list[-1] is not None:
        print("闭合误差:", np.round(q_list[-1] - q_list[0], 4))

    # 5) 动画（trail=True 会把末端轨迹画出来）
    viz.animate_trajectory(arm, q_list, arm_3R.LINK_LENGTHS,
                           interval=80, trail=True)


if __name__ == "__main__":
    main()

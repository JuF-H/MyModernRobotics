"""平面串联臂的 2D 可视化。

所有函数都通过传入的 robot（提供 FK / name）与 link_lengths 工作，
不绑定具体某个机器人模型。
"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


def planar_joint_positions(q, link_lengths):
    """
    平面串联臂各关节在基座坐标系下的 2D 位置。

    :param q: 关节角(rad), shape (N,)
    :param link_lengths: 各连杆长度, shape (N,)
    :return: 关节位置, shape (N+1, 2)，第 0 行为基座原点
    """
    q = np.asarray(q, dtype=float).ravel()
    L = np.asarray(link_lengths, dtype=float).ravel()
    if q.size != L.size:
        raise ValueError(f"q 长度({q.size}) 与 link_lengths 长度({L.size}) 不一致")

    x, y, a = 0.0, 0.0, 0.0
    pts = [[x, y]]
    for qi, Li in zip(q, L):
        a += qi
        x += Li * np.cos(a)
        y += Li * np.sin(a)
        pts.append([x, y])

    return np.array(pts)


def plot(robot, link_lengths, thetalist=None, ax=None,
         show_frame=True, label=None):
    """
    画出机器人当前（或指定）位形。

    :param robot: Robot 实例
    :param link_lengths: 各连杆长度, shape (N,)
    :param thetalist: 关节角，默认取机器人当前值
    :param ax: matplotlib Axes，为 None 时新建
    :param show_frame: 是否画末端坐标系
    :return: matplotlib Axes
    """
    if thetalist is None:
        thetalist = robot.thetalist_c
    thetalist = np.asarray(thetalist, dtype=float)

    pts = planar_joint_positions(thetalist, link_lengths)
    xs, ys = pts[:, 0], pts[:, 1]

    if ax is None:
        fig, ax = plt.subplots(figsize=(6, 6))

    ax.plot(xs, ys, '-o', lw=3, ms=6, color='tab:blue',
            label=label or robot.name)
    ax.plot(xs[0], ys[0], 's', ms=10, color='k')

    if show_frame:
        T = robot.FK(thetalist)
        origin = T[:3, 3]
        for axis, color in zip(range(2), ['r', 'g']):
            d = T[:3, axis] * 1.0
            ax.arrow(origin[0], origin[1], d[0], d[1],
                     head_width=0.15, color=color,
                     length_includes_head=True)

    ax.set_xlim(-11, 11)
    ax.set_ylim(-11, 11)
    ax.set_aspect('equal')
    ax.grid(True)
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_title(f"{robot.name}  thetalist={np.round(thetalist, 3)}")
    return ax


def animate_trajectory(robot, q_list, link_lengths, interval=200, trail=True):
    """
    播放关节角序列的动画。

    :param robot: Robot 实例（只用到 name）
    :param q_list: 关节角序列，元素为 (N,) 数组或 None（None 表示该帧无解/越限，跳过）
    :param link_lengths: 各连杆长度, shape (N,)
    :param interval: 帧间隔(ms)
    :param trail: 是否画末端轨迹
    :return: FuncAnimation 对象
    """
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.set_aspect('equal')
    ax.grid(True)
    ax.set_xlim(-11, 11)
    ax.set_ylim(-11, 11)

    line, = ax.plot([], [], '-o', lw=3, ms=6, color='tab:blue')
    trail_line, = ax.plot([], [], '-', lw=1, color='tab:red', alpha=0.5)
    end_dot, = ax.plot([], [], '*', ms=14, color='tab:red')
    skip_dot, = ax.plot([], [], 'x', ms=14, color='gray')  # 越限帧提示

    trail_xs, trail_ys = [], []

    def update(frame):
        q = q_list[frame]
        if q is None:
            skip_dot.set_data([], [])
            ax.set_title(f"{robot.name}  frame={frame}  [越限/无解，跳过]")
            return line, trail_line, end_dot, skip_dot

        pts = planar_joint_positions(q, link_lengths)
        line.set_data(pts[:, 0], pts[:, 1])
        end = pts[-1]
        end_dot.set_data([end[0]], [end[1]])
        skip_dot.set_data([], [])

        if trail:
            trail_xs.append(end[0])
            trail_ys.append(end[1])
            trail_line.set_data(trail_xs, trail_ys)

        ax.set_title(f"{robot.name}  frame={frame}  q={np.round(q, 3)}")
        return line, trail_line, end_dot, skip_dot

    anim = FuncAnimation(fig, update, frames=len(q_list),
                         interval=interval, blit=False, repeat=False)
    plt.show()
    return anim

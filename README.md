# arm_learn · project1

机器人学学习项目：平面机械臂的正逆运动学、雅可比矩阵转换与可操作度分析，
基于 [`modern_robotics`](https://github.com/MachineLearningWithRobotics/modern_robotics) 库。

## 目录结构

```
project1/
├── arm_bot/              机器人本体
│   ├── robot.py          Robot 类：初始化、关节限位、FK/IK、状态管理（无绘图依赖）
│   ├── viz.py            2D 可视化：plot / animate_trajectory（模型无关）
│   └── arm_3R.py         3R 平面臂模型参数（螺丝轴 S、零位 M、连杆长、限位）
│
├── expendlib/            数学工具库
│   ├── core.py           雅可比转换 Js/Jb/Jg/Ja + 可操作度 μ1/μ2/μ3
│   └── compute.py        齐次变换：平移、任意轴旋转、绕点旋转
│
├── docs/
│   └── jacobian.md       三种雅可比矩阵的转换（学习笔记）
│
├── examples/             演示脚本
│   ├── draw_circle.py    3R 臂末端画圆（IK + 动画）
│   ├── jacobian_check.py 雅可比转换与 μ 指标自检
│   └── transform_demo.py 绕空间任意点旋转
│
├── requirements.txt
└── README.md
```

## 安装

```bash
pip install -r requirements.txt
```

依赖：`numpy`、`modern-robotics`、`matplotlib`。

## 快速开始

```python
import numpy as np
from arm_bot import arm_3R, viz

# 一步拿到初始化好的 3R 机械臂
arm = arm_3R.make_robot()
arm.print_q_z()
arm.print_Tc()

# 正运动学
T = arm.FK(np.array([0.1, 0.2, 0.3]))

# 逆运动学：移动到目标位姿（多初值 + 阻尼最小二乘，奇异位形也能解）
# 成功返回 True 并更新全部状态；失败返回 False，所有状态保持不变
arm.moveT(T)
q = arm.get_q_z()          # 解出的关节角（相对于零位）

# 绘图
viz.plot(arm, arm_3R.LINK_LENGTHS)
```

## 运行示例

在**项目根目录**下用 `python -m` 运行，保证包导入路径正确：

```bash
python -m examples.draw_circle       # 画圆动画
python -m examples.jacobian_check    # 雅可比转换自检
python -m examples.transform_demo    # 绕点旋转
```

## 模块速查

| 想做的事 | 用什么 |
|----------|--------|
| 正/逆运动学、关节限位 | `arm_bot.Robot`（`FK` / `moveT` / `moveJ` / `IK_n`） |
| 画静态图、播放轨迹动画 | `arm_bot.viz.plot` / `viz.animate_trajectory` |
| 换一个机器人模型 | 新建 `arm_bot/xxx.py`，照抄 `arm_3R.py` 的结构 |
| 雅可比 Js↔Jb↔Jg↔Ja 互转 | `expendlib.Js2Jb` / `Js2Jg` / `Jb2Ja_r` … |
| 奇异位形下更稳的 IK 求解器 | `expendlib.IK_space_damped` |
| 可操作度 / 奇异性判断 | `expendlib.mu_1_axis_ratio` 等三个函数 |
| 齐次变换构造 | `expendlib.rotate_about_point` 等 |

转换关系的推导见 [docs/jacobian.md](docs/jacobian.md)。

## 设计约定

- **模型参数集中**：`S`、`M`、`q0`、`q_limit`、连杆长度只写在模型文件里，
  `Robot` 类和 `viz` 都不写死任何数值 —— 换模型只需新建一个模型文件。
- **库文件不带 `__main__`**：所有演示代码统一放 `examples/`。
- **`Robot` 不依赖 matplotlib**：绘图全部在 `viz.py`，纯运动学可无头环境运行。

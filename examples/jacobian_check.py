"""雅可比转换与可操作度指标的自检。

运行（项目根目录下）：python -m examples.jacobian_check
"""
import numpy as np

from expendlib import (
    Js2Jg,
    mu_1_axis_ratio,
    mu_2_condition_number,
    mu_3_volume,
)


def check_jacobian_conversion():
    print("=" * 50)
    print("1) 空间雅可比 -> 几何雅可比")
    print("=" * 50)

    Js = np.array([[0, 0],
                   [0, 0],
                   [1, 1],
                   [0, 0],
                   [0, -10],
                   [0, 0]])
    T = np.array([[1, 0, 0, 20],
                  [0, 1, 0, 0],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])

    Jc = Js2Jg(Js, T)
    print("输入 Js:\n", Js)
    print("转换后 Jg:\n", Jc)


def check_manipulability():
    print()
    print("=" * 50)
    print("2) 可操作度椭球三种度量")
    print("=" * 50)

    J_example = np.array([
        [1.0, 0.0, 0.0],
        [0.0, 2.0, 0.0],
        [0.0, 0.0, 0.5]
    ])

    print(f"雅可比矩阵 J:\n{J_example}\n")

    val_mu1 = mu_1_axis_ratio(J_example)
    val_mu2 = mu_2_condition_number(J_example)
    val_mu3 = mu_3_volume(J_example)

    print(f"μ1(A) 长短轴比 (越小越好, 越接近1越理想): {val_mu1:.4f}")
    print(f"μ2(A) 条件数   (越小越好, 越接近1越理想): {val_mu2:.4f}")
    print(f"μ3(A) 椭球体积 (越大越好, 代表灵活度高): {val_mu3:.4f}")


if __name__ == "__main__":
    check_jacobian_conversion()
    check_manipulability()

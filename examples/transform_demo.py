"""绕空间任意点旋转的示例。

运行（项目根目录下）：python -m examples.transform_demo
"""
import numpy as np

from expendlib import rotate_about_point


def main():
    T0 = np.array([[1, 0, 0, 1],
                   [0, 1, 0, 2],
                   [0, 0, 1, 3],
                   [0, 0, 0, 1]])
    T = rotate_about_point(point=(1, 2, 3),
                           axis=(1, 1, 1),
                           theta=2 * np.pi / 3)
    T1 = T @ T0

    print("旋转前的变换矩阵:\n", np.round(T0, 4))
    print("旋转后的变换矩阵:\n", np.round(T1, 4))

    # 扫一遍角度，看变换随 θ 的变化
    for theta in np.linspace(0, 2 * np.pi, 5):
        T = rotate_about_point(point=(1, 2, 3),
                               axis=(1, 1, 1),
                               theta=theta)
        T1 = T @ T0
        print(f"\n旋转角度: {theta:.2f} 弧度")
        print("旋转后的变换矩阵:\n", np.round(T1, 4))


if __name__ == "__main__":
    main()

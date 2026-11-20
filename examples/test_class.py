import numpy as np

from arm_bot import arm_3R, viz


def main():
    arm = arm_3R.make_robot()
    arm.print_Tc()
    arm.print_q_z()
    arm.print_q()

    print("*" * 20)

    q = np.array([np.pi/2, 0, 0])
    arm.move_q(q)
    arm.print_Tc()
    arm.print_q_z()
    arm.print_q()

    print("*" * 20)

    T = np.array([[1, 0, 0, 9],
                [0, 1, 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1]])
    arm.move_T(T)
    arm.print_Tc()
    arm.print_q_z()
    arm.print_q()

    print("*" * 20)

    q = np.array([-np.pi/2, np.pi/2, np.pi/2])
    arm.move_q(q)
    arm.print_Tc()
    arm.print_q_z()
    arm.print_q()

    arm.print_Js()
    arm.print_Jb()
    arm.print_Jg()

    # TODO: dq2Vs / dq2Vb / dq2Vg 待实现（dq、ddq 后续再做），先不测
    # dq = np.array([1, 0, 0])
    # print(arm.dq2Vs(dq))
    # print(arm.dq2Vb(dq))
    # print(arm.dq2Vg(dq))


if __name__ == "__main__":
    main()
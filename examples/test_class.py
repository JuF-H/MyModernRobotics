import numpy as np

from arm_bot import arm_3R, viz


def main():
    arm = arm_3R.make_robot()
    arm.info()


    print("**************移动到目标关节角**************")

    q = np.array([-np.pi/2, np.pi/2, np.pi/2])
    dq = np.array([1, 1, 1])
    arm.moveJ(q)
    arm.set_dq(dq)
    arm.info()

    print("**************移动到目标位姿**************")

    T = np.array([[1, 0, 0, 6],
                  [0, 1, 0, 1],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])
    arm.moveT(T)
    dq = np.array([1, 0, 0])
    arm.set_dq(dq)
    arm.info()

if __name__ == "__main__":
    main()
"""Demonstration script for NAOH25 kinematics.

Port of Cpp/NAOH25/Main.cpp.
"""

import math
from .robot_consts import (
    NUMOFJOINTS, HEAD, L_ARM, R_ARM, L_LEG, R_LEG,
    CHAIN_L_ARM, CHAIN_R_ARM, CHAIN_L_LEG, CHAIN_R_LEG,
    YAW, PITCH, SHOULDER_PITCH, SHOULDER_ROLL, ELBOW_YAW, ELBOW_ROLL, WRIST_YAW,
    HIP_YAW_PITCH, HIP_ROLL, HIP_PITCH, KNEE_PITCH, ANKLE_PITCH, ANKLE_ROLL,
)
from .nao_kinematics import NAOKinematics


def main():
    nkin = NAOKinematics()
    joints = [0.0] * NUMOFJOINTS

    # Left Hand
    joints[L_ARM + SHOULDER_PITCH] = 0.2
    joints[L_ARM + SHOULDER_ROLL] = 0.1
    joints[L_ARM + ELBOW_YAW] = 0.0
    joints[L_ARM + ELBOW_ROLL] = 0.0
    joints[L_ARM + WRIST_YAW] = 0.0

    # Right Hand
    joints[R_ARM + SHOULDER_PITCH] = math.pi / 2.0
    joints[R_ARM + SHOULDER_ROLL] = -math.pi / 4.0
    joints[R_ARM + ELBOW_YAW] = 0.0
    joints[R_ARM + ELBOW_ROLL] = 0.0
    joints[R_ARM + WRIST_YAW] = math.pi / 3535.0

    # Left Leg
    joints[L_LEG + HIP_YAW_PITCH] = 0.0
    joints[L_LEG + HIP_ROLL] = 0.0
    joints[L_LEG + HIP_PITCH] = 0.0
    joints[L_LEG + KNEE_PITCH] = math.pi / 4.0
    joints[L_LEG + ANKLE_PITCH] = -math.pi / 4.0
    joints[L_LEG + ANKLE_ROLL] = 0.0

    # Right Leg
    joints[R_LEG + HIP_YAW_PITCH] = 0.0
    joints[R_LEG + HIP_ROLL] = 0.0
    joints[R_LEG + HIP_PITCH] = 0.0
    joints[R_LEG + KNEE_PITCH] = 0.0
    joints[R_LEG + ANKLE_PITCH] = 0.0
    joints[R_LEG + ANKLE_ROLL] = 0.0

    # Head
    joints[HEAD + YAW] = math.pi / 2.0
    joints[HEAD + PITCH] = 0.0

    nkin.set_joints(joints)

    output1 = nkin.get_forward_effector(CHAIN_L_ARM)
    output2 = nkin.get_forward_effector(CHAIN_R_ARM)
    output3 = nkin.get_forward_effector(CHAIN_L_LEG)
    output4 = nkin.get_forward_effector(CHAIN_R_LEG)
    output5 = nkin.get_forward_effector(NAOKinematics.EFF_CAMERA_BOT)

    print(f"x = {output1[0, 3]} y = {output1[1, 3]} z = {output1[2, 3]}")
    print(f"x = {output2[0, 3]} y = {output2[1, 3]} z = {output2[2, 3]}")
    print(f"x = {output3[0, 3]} y = {output3[1, 3]} z = {output3[2, 3]}")
    print(f"x = {output4[0, 3]} y = {output4[1, 3]} z = {output4[2, 3]}")
    print(f"x = {output5[0, 3]} y = {output5[1, 3]} z = {output5[2, 3]}")

    result = nkin.inverse_left_hand(output1)
    if result:
        print("--Solution exists 1")
        print(" ".join(f"angle{j} = {result[0][j]}" for j in range(len(result[0]))) + " ")

    result = nkin.inverse_right_hand(output2)
    if result:
        print("--Solution exists 2")
        print(" ".join(f"angle{j} = {result[0][j]}" for j in range(len(result[0]))) + " ")

    result = nkin.inverse_left_leg(output3)
    if result:
        print("--Solution exists 3")
        print(" ".join(f"angle{j} = {result[0][j]}" for j in range(len(result[0]))) + " ")
        joints[L_LEG + HIP_YAW_PITCH] = 0.0
        joints[L_LEG + HIP_ROLL] = 0.0
        joints[L_LEG + HIP_PITCH] = 0.0
        joints[L_LEG + KNEE_PITCH] = 0.0
        joints[L_LEG + ANKLE_PITCH] = 0.0
        joints[L_LEG + ANKLE_ROLL] = 0.0
        nkin.set_joints(joints)
        result = nkin.jacobian_inverse_left_leg(output3)
        print(" ".join(f"angle{j} = {result[0][j]}" for j in range(len(result[0]))) + " ")

    result = nkin.inverse_right_leg(output4)
    if result:
        print("--Solution exists 4")
        print(" ".join(f"angle{j} = {result[0][j]}" for j in range(len(result[0]))) + " ")
        joints[R_LEG + HIP_YAW_PITCH] = 0.0
        joints[R_LEG + HIP_ROLL] = 0.0
        joints[R_LEG + HIP_PITCH] = 0.0
        joints[R_LEG + KNEE_PITCH] = 0.0
        joints[R_LEG + ANKLE_PITCH] = 0.0
        joints[R_LEG + ANKLE_ROLL] = 0.0
        nkin.set_joints(joints)
        result = nkin.jacobian_inverse_right_leg(output4)
        print(" ".join(f"angle{j} = {result[0][j]}" for j in range(len(result[0]))) + " ")


if __name__ == "__main__":
    main()

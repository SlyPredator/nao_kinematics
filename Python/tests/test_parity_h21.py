"""Parity tests between Cpp/NAOH21 and Python/NAOH21."""

import math
import numpy as np
from Python.NAOH21.nao_kinematics import NAOKinematics
from Python.NAOH21.robot_consts import (
    HEAD, L_ARM, R_ARM, L_LEG, R_LEG, NUMOFJOINTS,
    CHAIN_HEAD, CHAIN_L_ARM, CHAIN_L_LEG, CHAIN_R_LEG, CHAIN_R_ARM, CHAINS_SIZE,
    YAW, PITCH, SHOULDER_PITCH, SHOULDER_ROLL, ELBOW_YAW, ELBOW_ROLL,
    HIP_YAW_PITCH, HIP_ROLL, HIP_PITCH, KNEE_PITCH, ANKLE_PITCH, ANKLE_ROLL,
)
from Python.tests.cpp_runner import get_cpp_oracle_output


def parse_oracle_output(text):
    """Parse matrices and COMs from C++ oracle output."""
    lines = [line.strip() for line in text.strip().splitlines() if line.strip()]
    cases = {}
    current_case = None
    from_to_mat = None

    i = 0
    while i < len(lines):
        line = lines[i]
        if line.startswith("CASE:"):
            current_case = line.split(":", 1)[1].strip()
            cases[current_case] = {"matrices": [], "com": None}
            i += 1
            # 7 matrices: 5 chains + bottom camera + top camera
            for _ in range(7):
                nums = [float(x) for x in lines[i].split()]
                mat = np.array(nums, dtype=np.float64).reshape((4, 4))
                cases[current_case]["matrices"].append(mat)
                i += 1
            # COM line
            com_line = lines[i]
            assert com_line.startswith("COM:")
            com_nums = [float(x) for x in com_line.split()[1:]]
            cases[current_case]["com"] = np.array(com_nums, dtype=np.float64)
            i += 1
        elif line.startswith("FROM_TO_LLEG_RLEG:"):
            i += 1
            nums = [float(x) for x in lines[i].split()]
            from_to_mat = np.array(nums, dtype=np.float64).reshape((4, 4))
            i += 1
        else:
            i += 1

    return cases, from_to_mat


def test_h21_parity():
    cpp_out = get_cpp_oracle_output("NAOH21")
    cases, from_to_mat = parse_oracle_output(cpp_out)

    nkin = NAOKinematics()

    # Case 1: Zero joints
    j_zero = [0.0] * NUMOFJOINTS
    nkin.set_joints(j_zero)

    mats_py_zero = [
        nkin.get_forward_effector(CHAIN_HEAD).mat,
        nkin.get_forward_effector(CHAIN_L_ARM).mat,
        nkin.get_forward_effector(CHAIN_L_LEG).mat,
        nkin.get_forward_effector(CHAIN_R_LEG).mat,
        nkin.get_forward_effector(CHAIN_R_ARM).mat,
        nkin.get_forward_effector(NAOKinematics.EFF_CAMERA_BOT).mat,
        nkin.get_forward_effector(NAOKinematics.EFF_CAMERA_TOP).mat,
    ]
    com_py_zero = nkin.calculate_center_of_mass()

    for idx, (m_py, m_cpp) in enumerate(zip(mats_py_zero, cases["ZERO"]["matrices"])):
        diff = np.max(np.abs(m_py - m_cpp))
        assert diff < 1e-5, f"ZERO Case Mat {idx} diff {diff} exceeds tolerance: py=\n{m_py}\ncpp=\n{m_cpp}"

    com_diff = np.max(np.abs(com_py_zero - cases["ZERO"]["com"]))
    assert com_diff < 1e-4, f"ZERO COM diff {com_diff} exceeds tolerance: py={com_py_zero}, cpp={cases['ZERO']['com']}"

    # Case 2: Nominal joints
    j_nom = [0.0] * NUMOFJOINTS
    j_nom[L_ARM + SHOULDER_PITCH] = math.pi / 2.0
    j_nom[L_ARM + SHOULDER_ROLL] = math.pi / 4.0
    j_nom[R_ARM + SHOULDER_PITCH] = math.pi / 2.0
    j_nom[R_ARM + SHOULDER_ROLL] = -math.pi / 4.0
    j_nom[L_LEG + KNEE_PITCH] = math.pi / 4.0
    j_nom[L_LEG + ANKLE_PITCH] = -math.pi / 4.0
    j_nom[HEAD + YAW] = math.pi / 2.0
    nkin.set_joints(j_nom)

    mats_py_nom = [
        nkin.get_forward_effector(CHAIN_HEAD).mat,
        nkin.get_forward_effector(CHAIN_L_ARM).mat,
        nkin.get_forward_effector(CHAIN_L_LEG).mat,
        nkin.get_forward_effector(CHAIN_R_LEG).mat,
        nkin.get_forward_effector(CHAIN_R_ARM).mat,
        nkin.get_forward_effector(NAOKinematics.EFF_CAMERA_BOT).mat,
        nkin.get_forward_effector(NAOKinematics.EFF_CAMERA_TOP).mat,
    ]
    com_py_nom = nkin.calculate_center_of_mass()

    for idx, (m_py, m_cpp) in enumerate(zip(mats_py_nom, cases["NOMINAL"]["matrices"])):
        diff = np.max(np.abs(m_py - m_cpp))
        assert diff < 1e-5, f"NOMINAL Case Mat {idx} diff {diff} exceeds tolerance"

    com_diff = np.max(np.abs(com_py_nom - cases["NOMINAL"]["com"]))
    assert com_diff < 1e-4, f"NOMINAL COM diff {com_diff} exceeds tolerance"

    # Case 3: Arbitrary joints
    j_arb = [0.0] * NUMOFJOINTS
    j_arb[HEAD + YAW] = 0.3
    j_arb[HEAD + PITCH] = -0.2
    j_arb[L_ARM + SHOULDER_PITCH] = 0.5
    j_arb[L_ARM + SHOULDER_ROLL] = 0.4
    j_arb[L_ARM + ELBOW_YAW] = -0.6
    j_arb[L_ARM + ELBOW_ROLL] = -0.7
    j_arb[R_ARM + SHOULDER_PITCH] = -0.5
    j_arb[R_ARM + SHOULDER_ROLL] = -0.4
    j_arb[R_ARM + ELBOW_YAW] = 0.6
    j_arb[R_ARM + ELBOW_ROLL] = 0.7
    j_arb[L_LEG + HIP_YAW_PITCH] = -0.2
    j_arb[L_LEG + HIP_ROLL] = 0.1
    j_arb[L_LEG + HIP_PITCH] = -0.3
    j_arb[L_LEG + KNEE_PITCH] = 0.6
    j_arb[L_LEG + ANKLE_PITCH] = -0.3
    j_arb[L_LEG + ANKLE_ROLL] = 0.1
    j_arb[R_LEG + HIP_YAW_PITCH] = -0.2
    j_arb[R_LEG + HIP_ROLL] = -0.1
    j_arb[R_LEG + HIP_PITCH] = -0.3
    j_arb[R_LEG + KNEE_PITCH] = 0.6
    j_arb[R_LEG + ANKLE_PITCH] = -0.3
    j_arb[R_LEG + ANKLE_ROLL] = -0.1
    nkin.set_joints(j_arb)

    mats_py_arb = [
        nkin.get_forward_effector(CHAIN_HEAD).mat,
        nkin.get_forward_effector(CHAIN_L_ARM).mat,
        nkin.get_forward_effector(CHAIN_L_LEG).mat,
        nkin.get_forward_effector(CHAIN_R_LEG).mat,
        nkin.get_forward_effector(CHAIN_R_ARM).mat,
        nkin.get_forward_effector(NAOKinematics.EFF_CAMERA_BOT).mat,
        nkin.get_forward_effector(NAOKinematics.EFF_CAMERA_TOP).mat,
    ]
    com_py_arb = nkin.calculate_center_of_mass()

    for idx, (m_py, m_cpp) in enumerate(zip(mats_py_arb, cases["ARBITRARY"]["matrices"])):
        diff = np.max(np.abs(m_py - m_cpp))
        assert diff < 1e-5, f"ARBITRARY Case Mat {idx} diff {diff} exceeds tolerance"

    com_diff = np.max(np.abs(com_py_arb - cases["ARBITRARY"]["com"]))
    assert com_diff < 1e-4, f"ARBITRARY COM diff {com_diff} exceeds tolerance"

    # From-To relative transform
    from_to_py = nkin.get_forward_from_to(CHAIN_L_LEG, CHAIN_R_LEG).mat
    diff_from_to = np.max(np.abs(from_to_py - from_to_mat))
    assert diff_from_to < 1e-5, f"FromTo LLEG->RLEG diff {diff_from_to} exceeds tolerance"

    # Inverse Kinematics self-consistency validation
    # Right Arm IK
    r_arm_target = nkin.get_forward_effector(CHAIN_R_ARM)
    sols = nkin.inverse_right_hand(r_arm_target)
    assert len(sols) > 0, "Right Hand analytical IK should find at least 1 solution"
    nkin.set_chain(CHAIN_R_ARM, sols[0])
    test_r_arm = nkin.get_forward_effector(CHAIN_R_ARM)
    assert r_arm_target.almost_equal_to(test_r_arm, tol=0.5), "FK(IK(RightArm)) must match target"

    # Left Leg IK
    l_leg_target = nkin.get_forward_effector(CHAIN_L_LEG)
    sols_leg = nkin.inverse_left_leg(l_leg_target)
    assert len(sols_leg) > 0, "Left Leg analytical IK should find at least 1 solution"
    nkin.set_chain(CHAIN_L_LEG, sols_leg[0])
    test_l_leg = nkin.get_forward_effector(CHAIN_L_LEG)
    assert l_leg_target.almost_equal_to(test_l_leg, tol=0.5), "FK(IK(LeftLeg)) must match target"

    print("[OK] NAOH21 Parity and Kinematics Tests PASSED successfully!")


if __name__ == "__main__":
    test_h21_parity()

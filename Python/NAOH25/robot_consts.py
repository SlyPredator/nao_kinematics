"""Robot device lists and joint constants for NAO H25 (25-DOF)."""

from enum import IntEnum


class ChainHeadNames(IntEnum):
    YAW = 0
    PITCH = 1
    HEAD_SIZE = 2


YAW = ChainHeadNames.YAW
PITCH = ChainHeadNames.PITCH
HEAD_SIZE = ChainHeadNames.HEAD_SIZE


class ChainArmNames(IntEnum):
    SHOULDER_PITCH = 0
    SHOULDER_ROLL = 1
    ELBOW_YAW = 2
    ELBOW_ROLL = 3
    WRIST_YAW = 4
    ARM_SIZE = 5


SHOULDER_PITCH = ChainArmNames.SHOULDER_PITCH
SHOULDER_ROLL = ChainArmNames.SHOULDER_ROLL
ELBOW_YAW = ChainArmNames.ELBOW_YAW
ELBOW_ROLL = ChainArmNames.ELBOW_ROLL
WRIST_YAW = ChainArmNames.WRIST_YAW
ARM_SIZE = ChainArmNames.ARM_SIZE


class ChainLegNames(IntEnum):
    HIP_YAW_PITCH = 0
    HIP_ROLL = 1
    HIP_PITCH = 2
    KNEE_PITCH = 3
    ANKLE_PITCH = 4
    ANKLE_ROLL = 5
    LEG_SIZE = 6


HIP_YAW_PITCH = ChainLegNames.HIP_YAW_PITCH
HIP_ROLL = ChainLegNames.HIP_ROLL
HIP_PITCH = ChainLegNames.HIP_PITCH
KNEE_PITCH = ChainLegNames.KNEE_PITCH
ANKLE_PITCH = ChainLegNames.ANKLE_PITCH
ANKLE_ROLL = ChainLegNames.ANKLE_ROLL
LEG_SIZE = ChainLegNames.LEG_SIZE


class ChainsNames(IntEnum):
    CHAIN_HEAD = 0
    CHAIN_L_ARM = 1
    CHAIN_L_LEG = 2
    CHAIN_R_LEG = 3
    CHAIN_R_ARM = 4
    CHAINS_SIZE = 5


CHAIN_HEAD = ChainsNames.CHAIN_HEAD
CHAIN_L_ARM = ChainsNames.CHAIN_L_ARM
CHAIN_L_LEG = ChainsNames.CHAIN_L_LEG
CHAIN_R_LEG = ChainsNames.CHAIN_R_LEG
CHAIN_R_ARM = ChainsNames.CHAIN_R_ARM
CHAINS_SIZE = ChainsNames.CHAINS_SIZE


class JointNames(IntEnum):
    HEAD = 0
    L_ARM = HEAD_SIZE
    L_LEG = L_ARM + ARM_SIZE
    R_LEG = L_LEG + LEG_SIZE
    R_ARM = R_LEG + LEG_SIZE
    NUMOFJOINTS = R_ARM + ARM_SIZE


HEAD = JointNames.HEAD
L_ARM = JointNames.L_ARM
L_LEG = JointNames.L_LEG
R_LEG = JointNames.R_LEG
R_ARM = JointNames.R_ARM
NUMOFJOINTS = JointNames.NUMOFJOINTS

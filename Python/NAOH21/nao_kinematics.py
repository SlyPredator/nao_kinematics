"""NAO H21 Kinematics library in Python.

Direct port of Cpp/NAOH21/NAOKinematics.h, NAOKinematics.cpp,
and NAOKinematicsJacobian.cpp.
"""

import math
from enum import IntEnum
import numpy as np

from .robot_consts import (
    HEAD, L_ARM, L_LEG, R_LEG, R_ARM, NUMOFJOINTS,
    CHAIN_HEAD, CHAIN_L_ARM, CHAIN_L_LEG, CHAIN_R_LEG, CHAIN_R_ARM, CHAINS_SIZE,
    HEAD_SIZE, ARM_SIZE, LEG_SIZE,
    YAW, PITCH,
    SHOULDER_PITCH, SHOULDER_ROLL, ELBOW_YAW, ELBOW_ROLL,
    HIP_YAW_PITCH, HIP_ROLL, HIP_PITCH, KNEE_PITCH, ANKLE_PITCH, ANKLE_ROLL,
)
from .kinematics_defines import (
    ShoulderOffsetY, ShoulderOffsetZ, ElbowOffsetY, UpperArmLength,
    LowerArmLength, HandOffsetX, HandOffsetZ,
    HipOffsetZ, HipOffsetY, ThighLength, TibiaLength, FootHeight,
    NeckOffsetZ, CameraBotomX, CameraBotomZ, CameraTopX, CameraTopZ,
    HeadYawHigh, HeadYawLow, HeadPitchHigh, HeadPitchLow,
    LShoulderPitchHigh, LShoulderPitchLow, LShoulderRollHigh, LShoulderRollLow,
    LElbowYawHigh, LElbowYawLow, LElbowRollHigh, LElbowRollLow,
    RShoulderPitchHigh, RShoulderPitchLow, RShoulderRollHigh, RShoulderRollLow,
    RElbowYawHigh, RElbowYawLow, RElbowRollHigh, RElbowRollLow,
    LHipYawPitchHigh, LHipYawPitchLow, LHipRollHigh, LHipRollLow,
    LHipPitchHigh, LHipPitchLow, LKneePitchHigh, LKneePitchLow,
    LAnklePitchHigh, LAnklePitchLow, LAnkleRollHigh, LAnkleRollLow,
    RHipYawPitchHigh, RHipYawPitchLow, RHipRollHigh, RHipRollLow,
    RHipPitchHigh, RHipPitchLow, RKneePitchHigh, RKneePitchLow,
    RAnklePitchHigh, RAnklePitchLow, RAnkleRollHigh, RAnkleRollLow,
    TotalMassH21, TorsoMass, TorsoX, TorsoY, TorsoZ,
    BatteryMass, BatteryX, BatteryY, BatteryZ,
    HeadYawMass, HeadYawX, HeadYawY, HeadYawZ,
    HeadPitchMass, HeadPitchX, HeadPitchY, HeadPitchZ,
    RShoulderPitchMass, RShoulderPitchX, RShoulderPitchY, RShoulderPitchZ,
    RShoulderRollMass, RShoulderRollX, RShoulderRollY, RShoulderRollZ,
    RElbowYawMass, RElbowYawX, RElbowYawY, RElbowYawZ,
    RElbowRollMass, RElbowRollX, RElbowRollY, RElbowRollZ,
    RHipYawPitchMass, RHipYawPitchX, RHipYawPitchY, RHipYawPitchZ,
    RHipRollMass, RHipRollX, RHipRollY, RHipRollZ,
    RHipPitchMass, RHipPitchX, RHipPitchY, RHipPitchZ,
    RKneePitchMass, RKneePitchX, RKneePitchY, RKneePitchZ,
    RAnklePitchMass, RAnklePitchX, RAnklePitchY, RAnklePitchZ,
    RAnkleRollMass, RAnkleRollX, RAnkleRollY, RAnkleRollZ,
)
from .kmat import ATMatrix, Transformations


class FKvars:
    """Container for cartesian point p and Euler angles a."""

    def __init__(self, p=None, a=None):
        self.p = np.array(p, dtype=np.float64) if p is not None else np.zeros(3, dtype=np.float64)
        self.a = np.array(a, dtype=np.float64) if a is not None else np.zeros(3, dtype=np.float64)


class NAOKinematics:
    """Forward and Inverse Kinematics solver for NAO H21 robot."""

    class Effectors(IntEnum):
        EFF_START = 0
        EFF_CAMERA_BOT = CHAINS_SIZE
        EFF_CAMERA_TOP = CHAINS_SIZE + 1
        EFFECTOR_SIZE = CHAINS_SIZE + 2

    EFF_CAMERA_BOT = Effectors.EFF_CAMERA_BOT
    EFF_CAMERA_TOP = Effectors.EFF_CAMERA_TOP
    EFFECTOR_SIZE = Effectors.EFFECTOR_SIZE

    class Frames(IntEnum):
        FR_BASE = 0
        FR_BASE_T = NUMOFJOINTS
        FR_END_T = NUMOFJOINTS + CHAINS_SIZE
        FR_SIZE = NUMOFJOINTS + CHAINS_SIZE + CHAINS_SIZE + 2

    FR_BASE = Frames.FR_BASE
    FR_BASE_T = Frames.FR_BASE_T
    FR_END_T = Frames.FR_END_T
    FR_SIZE = Frames.FR_SIZE

    def __init__(self):
        self.PI = math.pi
        self.PI_2 = math.pi / 2.0

        fr_size = int(self.FR_SIZE)
        self.T = [ATMatrix.identity() for _ in range(fr_size)]
        self.Tjacobian = [np.eye(4, dtype=np.float64) for _ in range(fr_size)]
        self.joints = [0.0] * fr_size
        self.coms = [np.zeros(3, dtype=np.float64) for _ in range(fr_size)]
        self.masses = [0.0] * fr_size

        # Base and End transforms setup
        # Head
        self.T[self.FR_BASE_T + CHAIN_HEAD] = Transformations.make_translation(0.0, 0.0, NeckOffsetZ)
        self.T[self.FR_END_T + CHAIN_HEAD] = Transformations.make_rotation_xyz(self.PI_2, self.PI_2, 0.0)

        self.coms[HEAD + YAW] = np.array([HeadYawX, HeadYawY, HeadYawZ], dtype=np.float64)
        self.coms[HEAD + PITCH] = np.array([HeadPitchX, HeadPitchY, HeadPitchZ], dtype=np.float64)
        self.coms[HEAD + PITCH] = self.T[self.FR_END_T + CHAIN_HEAD].transform(self.coms[HEAD + PITCH])

        self.masses[HEAD + YAW] = HeadYawMass
        self.masses[HEAD + PITCH] = HeadPitchMass

        # Right Arm
        self.T[self.FR_BASE_T + CHAIN_R_ARM] = Transformations.make_translation(0.0, -ShoulderOffsetY, ShoulderOffsetZ)
        self.T[self.FR_END_T + CHAIN_R_ARM] = Transformations.make_rotation_xyz(0.0, 0.0, -self.PI_2)
        t1 = Transformations.make_translation(HandOffsetX + LowerArmLength, 0.0, -HandOffsetZ)
        self.T[self.FR_END_T + CHAIN_R_ARM] *= t1

        t1 = Transformations.make_rotation_xyz(self.PI_2, 0.0, 0.0)
        self.coms[R_ARM + SHOULDER_PITCH] = t1.transform(np.array([RShoulderPitchX, RShoulderPitchY, RShoulderPitchZ], dtype=np.float64))

        t1 = Transformations.make_rotation_xyz(0.0, 0.0, -self.PI_2)
        self.coms[R_ARM + SHOULDER_ROLL] = t1.transform(np.array([RShoulderRollX, RShoulderRollY, RShoulderRollZ], dtype=np.float64))

        t1 = Transformations.make_rotation_xyz(-self.PI_2, 0.0, -self.PI_2)
        self.coms[R_ARM + ELBOW_YAW] = t1.transform(np.array([RElbowYawX, RElbowYawY, RElbowYawZ], dtype=np.float64))

        t1 = Transformations.make_rotation_xyz(0.0, 0.0, -self.PI_2)
        self.coms[R_ARM + ELBOW_ROLL] = t1.transform(np.array([RElbowRollX, RElbowRollY, RElbowRollZ], dtype=np.float64))

        self.masses[R_ARM + SHOULDER_PITCH] = RShoulderPitchMass
        self.masses[R_ARM + SHOULDER_ROLL] = RShoulderRollMass
        self.masses[R_ARM + ELBOW_YAW] = RElbowYawMass
        self.masses[R_ARM + ELBOW_ROLL] = RElbowRollMass

        # Left Arm
        self.T[self.FR_BASE_T + CHAIN_L_ARM] = Transformations.make_translation(0.0, ShoulderOffsetY, ShoulderOffsetZ)
        self.T[self.FR_END_T + CHAIN_L_ARM] = self.T[self.FR_END_T + CHAIN_R_ARM].copy()

        for i in range(ARM_SIZE):
            self.coms[L_ARM + i] = self.coms[R_ARM + i].copy()
            self.masses[L_ARM + i] = self.masses[R_ARM + i]

        self.coms[L_ARM + SHOULDER_PITCH][2] = -self.coms[L_ARM + SHOULDER_PITCH][2]
        self.coms[L_ARM + SHOULDER_ROLL][0] = -self.coms[L_ARM + SHOULDER_ROLL][0]
        self.coms[L_ARM + ELBOW_YAW][0] = -self.coms[L_ARM + ELBOW_YAW][0]
        self.coms[L_ARM + ELBOW_ROLL][0] = -self.coms[L_ARM + ELBOW_ROLL][0]

        # Right Leg
        self.T[self.FR_BASE_T + CHAIN_R_LEG] = Transformations.make_translation(0.0, -HipOffsetY, -HipOffsetZ)
        self.RotRLeg = Transformations.make_rotation_zyx(self.PI, -self.PI_2, 0.0)
        self.T[self.FR_END_T + CHAIN_R_LEG] = self.RotRLeg.copy()
        t1 = Transformations.make_translation(0.0, 0.0, -FootHeight)
        self.T[self.FR_END_T + CHAIN_R_LEG] *= t1

        t1 = Transformations.make_rotation_xyz(-self.PI_2 / 2.0, 0.0, -self.PI_2)
        t1.fast_invert()
        self.coms[R_LEG + HIP_YAW_PITCH] = t1.transform(np.array([RHipYawPitchX, RHipYawPitchY, RHipYawPitchZ], dtype=np.float64))

        t1 = Transformations.make_rotation_xyz(self.PI, self.PI_2, 0.0)
        self.coms[R_LEG + HIP_ROLL] = t1.transform(np.array([RHipRollX, RHipRollY, RHipRollZ], dtype=np.float64))

        t1 = Transformations.make_rotation_xyz(self.PI_2, self.PI_2, 0.0)
        self.coms[R_LEG + HIP_PITCH] = t1.transform(np.array([RHipPitchX, RHipPitchY, RHipPitchZ], dtype=np.float64))

        t1 = Transformations.make_rotation_xyz(self.PI_2, self.PI_2, 0.0)
        self.coms[R_LEG + KNEE_PITCH] = t1.transform(np.array([RKneePitchX, RKneePitchY, RKneePitchZ], dtype=np.float64))

        t1 = Transformations.make_rotation_xyz(self.PI_2, self.PI_2, 0.0)
        self.coms[R_LEG + ANKLE_PITCH] = t1.transform(np.array([RAnklePitchX, RAnklePitchY, RAnklePitchZ], dtype=np.float64))

        t1 = Transformations.make_rotation_xyz(self.PI, self.PI_2, 0.0)
        self.coms[R_LEG + ANKLE_ROLL] = t1.transform(np.array([RAnkleRollX, RAnkleRollY, RAnkleRollZ], dtype=np.float64))

        self.masses[R_LEG + HIP_YAW_PITCH] = RHipYawPitchMass
        self.masses[R_LEG + HIP_ROLL] = RHipRollMass
        self.masses[R_LEG + HIP_PITCH] = RHipPitchMass
        self.masses[R_LEG + KNEE_PITCH] = RKneePitchMass
        self.masses[R_LEG + ANKLE_PITCH] = RAnklePitchMass
        self.masses[R_LEG + ANKLE_ROLL] = RAnkleRollMass

        # Left Leg
        self.T[self.FR_BASE_T + CHAIN_L_LEG] = Transformations.make_translation(0.0, HipOffsetY, -HipOffsetZ)
        self.TBaseLLegInv = self.T[self.FR_BASE_T + CHAIN_L_LEG].copy()
        self.TBaseLLegInv.fast_invert()
        self.RotFixLLeg = Transformations.make_rotation_xyz(self.PI_2 / 2.0, 0.0, 0.0)
        t1 = Transformations.make_translation(0.0, 0.0, -FootHeight)
        self.T[self.FR_END_T + CHAIN_L_LEG] = Transformations.make_rotation_zyx(self.PI, -self.PI_2, 0.0) * t1
        self.TEndLLegInv = t1.copy()
        self.TEndLLegInv.fast_invert()

        t1 = Transformations.make_rotation_xyz(-(3.0 * self.PI) / 4.0, 0.0, -self.PI_2)
        t1.fast_invert()
        self.coms[L_LEG + HIP_YAW_PITCH] = t1.transform(np.array([RHipYawPitchX, -RHipYawPitchY, RHipYawPitchZ], dtype=np.float64))

        t1 = Transformations.make_rotation_xyz(self.PI, self.PI_2, 0.0)
        self.coms[L_LEG + HIP_ROLL] = t1.transform(np.array([RHipRollX, -RHipRollY, RHipRollZ], dtype=np.float64))

        t1 = Transformations.make_rotation_xyz(self.PI_2, self.PI_2, 0.0)
        self.coms[L_LEG + HIP_PITCH] = t1.transform(np.array([RHipPitchX, -RHipPitchY, RHipPitchZ], dtype=np.float64))

        t1 = Transformations.make_rotation_xyz(self.PI_2, self.PI_2, 0.0)
        self.coms[L_LEG + KNEE_PITCH] = t1.transform(np.array([RKneePitchX, -RKneePitchY, RKneePitchZ], dtype=np.float64))

        t1 = Transformations.make_rotation_xyz(self.PI_2, self.PI_2, 0.0)
        self.coms[L_LEG + ANKLE_PITCH] = t1.transform(np.array([RAnklePitchX, -RAnklePitchY, RAnklePitchZ], dtype=np.float64))

        t1 = Transformations.make_rotation_xyz(self.PI, self.PI_2, 0.0)
        self.coms[L_LEG + ANKLE_ROLL] = t1.transform(np.array([RAnkleRollX, -RAnkleRollY, RAnkleRollZ], dtype=np.float64))

        self.masses[L_LEG + HIP_YAW_PITCH] = RHipYawPitchMass
        self.masses[L_LEG + HIP_ROLL] = RHipRollMass
        self.masses[L_LEG + HIP_PITCH] = RHipPitchMass
        self.masses[L_LEG + KNEE_PITCH] = RKneePitchMass
        self.masses[L_LEG + ANKLE_PITCH] = RAnklePitchMass
        self.masses[L_LEG + ANKLE_ROLL] = RAnkleRollMass

        # Cameras
        self.T[self.FR_END_T + self.EFF_CAMERA_BOT] = Transformations.make_translation(CameraBotomX, 0.0, CameraBotomZ)
        self.T[self.FR_END_T + self.EFF_CAMERA_TOP] = Transformations.make_translation(CameraTopX, 0.0, CameraTopZ)

        self.prepare_forward()

    def set_joints(self, joints_set):
        if len(joints_set) != NUMOFJOINTS:
            return False
        for i in range(NUMOFJOINTS):
            self.joints[i] = float(joints_set[i])
        self.prepare_forward()
        return True

    setJoints = set_joints

    def set_chain(self, ch, joints_set):
        if ch == CHAIN_HEAD:
            frstart = HEAD
            chsize = HEAD_SIZE
        elif ch == CHAIN_L_LEG:
            frstart = L_LEG
            chsize = LEG_SIZE
        elif ch == CHAIN_R_LEG:
            frstart = R_LEG
            chsize = LEG_SIZE
        elif ch == CHAIN_L_ARM:
            frstart = L_ARM
            chsize = ARM_SIZE
        elif ch == CHAIN_R_ARM:
            frstart = R_ARM
            chsize = ARM_SIZE
        else:
            return False

        if len(joints_set) != chsize:
            return False

        for i in range(chsize):
            self.joints[frstart + i] = float(joints_set[i])

        self.prepare_forward(ch)
        return True

    setChain = set_chain

    def prepare_forward(self, ch=CHAINS_SIZE):
        make_dh = Transformations.make_dh_transformation
        pi_2 = self.PI_2

        if ch in (CHAIN_L_ARM, CHAINS_SIZE):
            self.T[L_ARM + SHOULDER_PITCH] = make_dh(0.0, -pi_2, 0.0, self.joints[L_ARM + SHOULDER_PITCH])
            self.T[L_ARM + SHOULDER_ROLL] = make_dh(0.0, pi_2, 0.0, self.joints[L_ARM + SHOULDER_ROLL] + pi_2)
            self.T[L_ARM + ELBOW_YAW] = make_dh(ElbowOffsetY, pi_2, UpperArmLength, self.joints[L_ARM + ELBOW_YAW])
            self.T[L_ARM + ELBOW_ROLL] = make_dh(0.0, -pi_2, 0.0, self.joints[L_ARM + ELBOW_ROLL])

        if ch in (CHAIN_R_ARM, CHAINS_SIZE):
            self.T[R_ARM + SHOULDER_PITCH] = make_dh(0.0, -pi_2, 0.0, self.joints[R_ARM + SHOULDER_PITCH])
            self.T[R_ARM + SHOULDER_ROLL] = make_dh(0.0, pi_2, 0.0, self.joints[R_ARM + SHOULDER_ROLL] + pi_2)
            self.T[R_ARM + ELBOW_YAW] = make_dh(-ElbowOffsetY, pi_2, UpperArmLength, self.joints[R_ARM + ELBOW_YAW])
            self.T[R_ARM + ELBOW_ROLL] = make_dh(0.0, -pi_2, 0.0, self.joints[R_ARM + ELBOW_ROLL])

        if ch in (CHAIN_L_LEG, CHAINS_SIZE):
            self.T[L_LEG + HIP_YAW_PITCH] = make_dh(0.0, -3.0 * pi_2 / 2.0, 0.0, self.joints[L_LEG + HIP_YAW_PITCH] - pi_2)
            self.T[L_LEG + HIP_ROLL] = make_dh(0.0, -pi_2, 0.0, self.joints[L_LEG + HIP_ROLL] + pi_2 / 2.0)
            self.T[L_LEG + HIP_PITCH] = make_dh(0.0, pi_2, 0.0, self.joints[L_LEG + HIP_PITCH])
            self.T[L_LEG + KNEE_PITCH] = make_dh(-ThighLength, 0.0, 0.0, self.joints[L_LEG + KNEE_PITCH])
            self.T[L_LEG + ANKLE_PITCH] = make_dh(-TibiaLength, 0.0, 0.0, self.joints[L_LEG + ANKLE_PITCH])
            self.T[L_LEG + ANKLE_ROLL] = make_dh(0.0, -pi_2, 0.0, self.joints[L_LEG + ANKLE_ROLL])

        if ch in (CHAIN_R_LEG, CHAINS_SIZE):
            self.T[R_LEG + HIP_YAW_PITCH] = make_dh(0.0, -pi_2 / 2.0, 0.0, self.joints[R_LEG + HIP_YAW_PITCH] - pi_2)
            self.T[R_LEG + HIP_ROLL] = make_dh(0.0, -pi_2, 0.0, self.joints[R_LEG + HIP_ROLL] - pi_2 / 2.0)
            self.T[R_LEG + HIP_PITCH] = make_dh(0.0, pi_2, 0.0, self.joints[R_LEG + HIP_PITCH])
            self.T[R_LEG + KNEE_PITCH] = make_dh(-ThighLength, 0.0, 0.0, self.joints[R_LEG + KNEE_PITCH])
            self.T[R_LEG + ANKLE_PITCH] = make_dh(-TibiaLength, 0.0, 0.0, self.joints[R_LEG + ANKLE_PITCH])
            self.T[R_LEG + ANKLE_ROLL] = make_dh(0.0, -pi_2, 0.0, self.joints[R_LEG + ANKLE_ROLL])

        if ch in (CHAIN_HEAD, CHAINS_SIZE):
            self.T[HEAD + YAW] = make_dh(0.0, 0.0, 0.0, self.joints[HEAD + YAW])
            self.T[HEAD + PITCH] = make_dh(0.0, -pi_2, 0.0, self.joints[HEAD + PITCH] - pi_2)

    prepareForward = prepare_forward

    def get_forward_effector(self, ef):
        baseframe = ef
        if ef in (self.EFF_CAMERA_BOT, self.EFF_CAMERA_TOP):
            baseframe = CHAIN_HEAD
            frstart = HEAD
            chsize = HEAD_SIZE
        elif ef == CHAIN_HEAD:
            frstart = HEAD
            chsize = HEAD_SIZE
        elif ef == CHAIN_L_LEG:
            frstart = L_LEG
            chsize = LEG_SIZE
        elif ef == CHAIN_R_LEG:
            frstart = R_LEG
            chsize = LEG_SIZE
        elif ef == CHAIN_L_ARM:
            frstart = L_ARM
            chsize = ARM_SIZE
        elif ef == CHAIN_R_ARM:
            frstart = R_ARM
            chsize = ARM_SIZE
        else:
            return ATMatrix.identity()

        t = self.T[self.FR_BASE_T + baseframe].copy()
        for i in range(chsize):
            t *= self.T[self.FR_BASE + frstart + i]
        t *= self.T[self.FR_END_T + ef]
        return t

    getForwardEffector = get_forward_effector

    def get_forward_from_to(self, start, stop):
        t1 = self.get_forward_effector(start).copy()
        t1.fast_invert()
        t1 *= self.get_forward_effector(stop)
        return t1

    getForwardFromTo = get_forward_from_to

    def calculate_com_chain(self, start, base_frame, startdown):
        pp = np.zeros(3, dtype=np.float64)
        pm = 0.0

        for i in range(startdown - 1, -1, -1):
            idx = start + i
            pp = pp * pm + self.coms[idx] * self.masses[idx]
            pm += self.masses[idx]
            if pm > 0:
                pp = pp * (1.0 / pm)
            pp = self.T[idx].transform(pp)

        pp = self.T[base_frame].transform(pp)
        return pp, pm

    calculateCoMChain = calculate_com_chain

    def calculate_center_of_mass(self):
        summass = 0.0
        sumc = np.zeros(3, dtype=np.float64)

        p, m = self.calculate_com_chain(HEAD, self.FR_BASE_T + CHAIN_HEAD, HEAD_SIZE)
        sumc = p.copy()
        summass = m

        for chain, fr, size in [
            (L_ARM, self.FR_BASE_T + CHAIN_L_ARM, ARM_SIZE),
            (R_ARM, self.FR_BASE_T + CHAIN_R_ARM, ARM_SIZE),
            (L_LEG, self.FR_BASE_T + CHAIN_L_LEG, LEG_SIZE),
            (R_LEG, self.FR_BASE_T + CHAIN_R_LEG, LEG_SIZE),
        ]:
            p, m = self.calculate_com_chain(chain, fr, size)
            sumc = sumc * summass + p * m
            summass += m
            sumc *= 1.0 / summass

        p = np.array([TorsoX, TorsoY, TorsoZ], dtype=np.float64)
        m = TorsoMass
        sumc = sumc * summass + p * m
        summass += m
        sumc *= 1.0 / summass

        p = np.array([BatteryX, BatteryY, BatteryZ], dtype=np.float64)
        m = BatteryMass
        sumc = sumc * summass + p * m
        summass += m
        sumc *= 1.0 / summass

        return sumc

    calculateCenterOfMass = calculate_center_of_mass

    @staticmethod
    def get_transformation(s):
        if isinstance(s, ATMatrix):
            return s
        if isinstance(s, np.ndarray) and s.shape == (4, 4):
            return ATMatrix(s)
        return Transformations.make_transformation(s.p[0], s.p[1], s.p[2], s.a[0], s.a[1], s.a[2])

    @staticmethod
    def mirror_transformation(target_point):
        target_point = target_point.copy()
        target_point.mat[0, 1] = -target_point.mat[0, 1]
        target_point.mat[1, 0] = -target_point.mat[1, 0]
        target_point.mat[1, 2] = -target_point.mat[1, 2]
        target_point.mat[2, 1] = -target_point.mat[2, 1]
        target_point.mat[1, 3] = -target_point.mat[1, 3]
        return target_point

    mirrorTransformation = mirror_transformation

    # Analytical Inverse Kinematics
    def inverse_head(self, target_point, with_angles=False, top_camera=False):
        T = self.get_transformation(target_point).copy()
        return_result = []

        if T[0, 3] != 0.0 and with_angles:
            return return_result

        if with_angles:
            theta1 = math.atan2(T[1, 0], T[0, 0])
            theta2 = math.atan2(-T[2, 0], math.sqrt(T[2, 1] ** 2 + T[2, 2] ** 2))
            return [[float(theta1), float(theta2)]]

        up = -T[2, 3] + NeckOffsetZ
        if top_camera:
            downt2 = math.sqrt(CameraTopX ** 2 + CameraTopZ ** 2)
            psi = math.atan2(CameraTopX, CameraTopZ)
        else:
            downt2 = math.sqrt(CameraBotomX ** 2 + CameraBotomZ ** 2)
            psi = math.atan2(CameraBotomX, CameraBotomZ)

        ratio = up / downt2
        ratio = max(-1.0, min(1.0, ratio))
        temp_theta2 = math.asin(ratio)
        pos_or_neg_pi = self.PI if temp_theta2 >= 0 else -self.PI

        for j in range(2):
            if j == 0 and (temp_theta2 - psi > HeadYawHigh or temp_theta2 - psi < HeadYawLow):
                continue
            elif j == 1 and (pos_or_neg_pi - temp_theta2 - psi > HeadYawHigh or pos_or_neg_pi - temp_theta2 - psi < HeadYawLow):
                continue
            elif j == 1:
                theta2 = pos_or_neg_pi - temp_theta2 - psi
            else:
                theta2 = temp_theta2 - psi

            if top_camera:
                downt1 = CameraTopZ * math.cos(theta2) - CameraTopX * math.sin(theta2)
            else:
                downt1 = CameraBotomZ * math.cos(theta2) - CameraBotomX * math.sin(theta2)

            theta2 = theta2 + self.PI_2
            if abs(downt1) < 1e-12:
                continue
            ratio_t1 = T[0, 3] / downt1
            if ratio_t1 < -1.0 or ratio_t1 > 1.0:
                continue
            theta1 = math.acos(ratio_t1)

            for i in range(2):
                th1 = theta1 if i == 0 else -theta1
                if i == 0 and (th1 > HeadPitchHigh or th1 < HeadPitchLow):
                    continue
                elif i == 1 and (th1 > HeadPitchHigh or th1 < HeadPitchLow):
                    continue

                self.joints[HEAD + YAW] = th1
                self.joints[HEAD + PITCH] = theta2
                self.prepare_forward(CHAIN_HEAD)
                output_t = self.get_forward_effector(self.EFF_CAMERA_TOP if top_camera else self.EFF_CAMERA_BOT)
                diff = target_point.get_translation() - output_t.get_translation()
                if np.dot(diff, diff) < 3.0:
                    return_result.append([float(th1), float(theta2)])

        return return_result

    inverseHead = inverse_head

    def inverse_left_hand(self, target_point):
        target = self.get_transformation(target_point)
        return_result = []

        t_init = target.copy()
        try:
            t_init.fast_invert()
        except np.linalg.LinAlgError:
            return return_result

        temp = self.T[self.FR_END_T + CHAIN_L_ARM] * t_init * self.T[self.FR_BASE_T + CHAIN_L_ARM]
        t_orig = temp.copy()
        try:
            t_orig.fast_invert()
        except np.linalg.LinAlgError:
            return return_result

        ratio3 = temp[2, 3] / ElbowOffsetY
        if ratio3 < -1.0:
            theta3temp = -self.PI_2
        elif ratio3 > 1.0:
            theta3temp = self.PI_2
        else:
            theta3temp = math.asin(ratio3)

        pos_or_neg_pi = self.PI if theta3temp >= 0 else -self.PI
        theta3 = theta3temp

        for i in range(2):
            if i == 0 and (theta3 > LElbowYawHigh or theta3 < LElbowYawLow):
                continue
            elif i == 1 and (pos_or_neg_pi - theta3 > LElbowYawHigh or pos_or_neg_pi - theta3 < LElbowYawLow):
                continue
            elif i == 1:
                theta3 = pos_or_neg_pi - theta3

            if theta3 == self.PI_2:
                ratio4 = temp[1, 3] / UpperArmLength
            else:
                top = temp[1, 3] * UpperArmLength - temp[0, 3] * math.cos(theta3) * ElbowOffsetY
                bottom = UpperArmLength ** 2 + (ElbowOffsetY * math.cos(theta3)) ** 2
                ratio4 = top / bottom

            if ratio4 < -1.0 or ratio4 > 1.0:
                continue
            theta4 = math.acos(ratio4)

            t3 = Transformations.make_dh_transformation(ElbowOffsetY, self.PI_2, UpperArmLength, theta3)
            try:
                t3.fast_invert()
            except np.linalg.LinAlgError:
                continue

            for j in range(2):
                temp_step = t_orig.copy()
                th4 = theta4 if j == 0 else -theta4
                if j == 0 and (th4 > LElbowRollHigh or th4 < LElbowRollLow):
                    continue
                elif j == 1 and (th4 > LElbowRollHigh or th4 < LElbowRollLow):
                    continue

                t4 = Transformations.make_dh_transformation(0.0, -self.PI_2, 0.0, th4)
                try:
                    t4.fast_invert()
                except np.linalg.LinAlgError:
                    continue

                temp_step *= t4
                temp_step *= t3
                try:
                    temp_step.fast_invert()
                except np.linalg.LinAlgError:
                    continue

                theta2 = math.atan2(temp_step[0, 1], temp_step[1, 1]) - self.PI_2
                if theta2 < LShoulderRollLow or theta2 > LShoulderRollHigh:
                    continue

                theta1 = math.atan2(temp_step[2, 0], temp_step[2, 2])
                if theta1 < LShoulderPitchLow or theta1 > LShoulderPitchHigh:
                    continue

                self.joints[L_ARM + SHOULDER_PITCH] = theta1
                self.joints[L_ARM + SHOULDER_ROLL] = theta2
                self.joints[L_ARM + ELBOW_YAW] = theta3
                self.joints[L_ARM + ELBOW_ROLL] = th4
                self.prepare_forward(CHAIN_L_ARM)
                test = self.get_forward_effector(CHAIN_L_ARM)

                if test.almost_equal_to(target):
                    return_result.append([float(theta1), float(theta2), float(theta3), float(th4)])

        return return_result

    inverseLeftHand = inverse_left_hand

    def inverse_right_hand(self, target_point):
        target = self.get_transformation(target_point)
        target_mirrored = self.mirror_transformation(target)
        res = self.inverse_left_hand(target_mirrored)
        for r in res:
            r[SHOULDER_ROLL] = -r[SHOULDER_ROLL]
            r[ELBOW_ROLL] = -r[ELBOW_ROLL]
            r[ELBOW_YAW] = -r[ELBOW_YAW]
        return res

    inverseRightHand = inverse_right_hand

    def inverse_left_leg(self, target_point):
        target = self.get_transformation(target_point)
        return_result = []

        t_init = target.copy()
        base = self.TBaseLLegInv * t_init * self.TEndLLegInv
        rot = self.RotFixLLeg * base
        t_start = rot.copy()

        try:
            rot.fast_invert()
        except np.linalg.LinAlgError:
            return return_result

        T = rot
        side1 = ThighLength
        side2 = TibiaLength
        trans = T.get_translation()
        distancesqrd = float(np.dot(trans, trans))
        arg = (side1 ** 2 + side2 ** 2 - distancesqrd) / (2.0 * side1 * side2)
        if arg < -1.0 or arg > 1.0:
            return return_result
        theta4 = self.PI - math.acos(arg)

        if abs(T[2, 3]) < 1e-12:
            return return_result
        theta6 = math.atan(T[1, 3] / T[2, 3])
        if theta6 < LAnkleRollLow or theta6 > LAnkleRollHigh:
            return return_result

        t6i = Transformations.make_dh_transformation(0.0, -self.PI_2, 0.0, theta6) * self.RotRLeg
        try:
            t6i.fast_invert()
            t_start *= t6i
            t_temp_theta5 = t_start.copy()
            t_temp_theta5.fast_invert()
        except np.linalg.LinAlgError:
            return return_result

        for itter in range(2):
            th4 = theta4 if itter == 0 else -theta4
            if th4 < RKneePitchLow or th4 > RKneePitchHigh:
                continue

            t4i = Transformations.make_dh_transformation(-ThighLength, 0.0, 0.0, th4)
            up = t_temp_theta5[1, 3] * (TibiaLength + ThighLength * math.cos(th4)) + ThighLength * t_temp_theta5[0, 3] * math.sin(th4)
            down = (ThighLength * math.sin(th4)) ** 2 + (TibiaLength + ThighLength * math.cos(th4)) ** 2
            if abs(down) < 1e-12:
                continue

            ratio5 = -up / down
            if ratio5 < -1.0:
                theta5 = -self.PI_2
            elif ratio5 > 1.0:
                theta5 = self.PI_2
            else:
                theta5 = math.asin(ratio5)

            pos_or_neg_pit5 = self.PI if theta5 >= 0 else -self.PI

            for i in range(2):
                th5 = theta5 if i == 0 else (pos_or_neg_pit5 - theta5)
                if th5 > LAnklePitchHigh or th5 < LAnklePitchLow:
                    continue

                t5i = Transformations.make_dh_transformation(-TibiaLength, 0.0, 0.0, th5)
                ttemp = t4i * t5i
                try:
                    ttemp.fast_invert()
                except np.linalg.LinAlgError:
                    continue

                ttemp2 = t_start * ttemp
                val2 = ttemp2[1, 2]
                if val2 < -1.0 or val2 > 1.0:
                    continue
                temptheta2 = math.acos(val2)

                for l in range(2):
                    if l == 0:
                        if (temptheta2 - self.PI_2 / 2.0 > LHipRollHigh) or (temptheta2 - self.PI_2 / 2.0 < LHipRollLow):
                            continue
                        th2 = temptheta2 - self.PI_2 / 2.0
                    else:
                        if (-temptheta2 - self.PI_2 / 2.0 > LHipRollHigh) or (-temptheta2 - self.PI_2 / 2.0 < LHipRollLow):
                            continue
                        th2 = -temptheta2 - self.PI_2 / 2.0

                    denom = math.sin(th2 + self.PI_2 / 2.0)
                    if abs(denom) < 1e-12:
                        continue

                    ratio3 = ttemp2[1, 1] / denom
                    if ratio3 < -1.0:
                        theta3 = -self.PI_2
                    elif ratio3 > 1.0:
                        theta3 = self.PI_2
                    else:
                        theta3 = math.asin(ratio3)
                    pos_or_neg_pit3 = self.PI if theta3 >= 0 else -self.PI

                    for k in range(2):
                        th3 = theta3 if k == 0 else (pos_or_neg_pit3 - theta3)
                        if th3 > LHipPitchHigh or th3 < LHipPitchLow:
                            continue

                        ratio1 = ttemp2[0, 2] / denom
                        if ratio1 < -1.0 or ratio1 > 1.0:
                            temptheta1 = 0.0
                        else:
                            temptheta1 = math.acos(ratio1)

                        for p in range(2):
                            if p == 0:
                                if (temptheta1 + self.PI_2 > LHipYawPitchHigh) or (-temptheta1 + self.PI_2 < LHipYawPitchLow):
                                    continue
                                th1 = temptheta1 + self.PI_2
                            else:
                                if (-temptheta1 + self.PI_2 > LHipYawPitchHigh) or (-temptheta1 + self.PI_2 < LHipYawPitchLow):
                                    continue
                                th1 = -temptheta1 + self.PI_2

                            self.joints[L_LEG + HIP_YAW_PITCH] = th1
                            self.joints[L_LEG + HIP_ROLL] = th2
                            self.joints[L_LEG + HIP_PITCH] = th3
                            self.joints[L_LEG + KNEE_PITCH] = th4
                            self.joints[L_LEG + ANKLE_PITCH] = th5
                            self.joints[L_LEG + ANKLE_ROLL] = theta6

                            self.prepare_forward(CHAIN_L_LEG)
                            test = self.get_forward_effector(CHAIN_L_LEG)

                            if test.almost_equal_to(t_init):
                                return_result.append([float(th1), float(th2), float(th3), float(th4), float(th5), float(theta6)])

        return return_result

    inverseLeftLeg = inverse_left_leg

    def inverse_right_leg(self, target_point):
        target = self.get_transformation(target_point)
        target_mirrored = self.mirror_transformation(target)
        res = self.inverse_left_leg(target_mirrored)
        for r in res:
            r[HIP_ROLL] = -r[HIP_ROLL]
            r[ANKLE_ROLL] = -r[ANKLE_ROLL]
        return res

    inverseRightLeg = inverse_right_leg

    # Numerical Jacobian Inverse Kinematics
    def prepare_derivatives(self, ch=CHAINS_SIZE):
        make_dh_d = Transformations.make_dh_derivative
        pi_2 = self.PI_2

        if ch in (CHAIN_L_ARM, CHAINS_SIZE):
            self.Tjacobian[L_ARM + SHOULDER_PITCH] = make_dh_d(-pi_2, self.joints[L_ARM + SHOULDER_PITCH])
            self.Tjacobian[L_ARM + SHOULDER_ROLL] = make_dh_d(pi_2, self.joints[L_ARM + SHOULDER_ROLL] + pi_2)
            self.Tjacobian[L_ARM + ELBOW_YAW] = make_dh_d(pi_2, self.joints[L_ARM + ELBOW_YAW])
            self.Tjacobian[L_ARM + ELBOW_ROLL] = make_dh_d(-pi_2, self.joints[L_ARM + ELBOW_ROLL])

        if ch in (CHAIN_R_ARM, CHAINS_SIZE):
            self.Tjacobian[R_ARM + SHOULDER_PITCH] = make_dh_d(-pi_2, self.joints[R_ARM + SHOULDER_PITCH])
            self.Tjacobian[R_ARM + SHOULDER_ROLL] = make_dh_d(pi_2, self.joints[R_ARM + SHOULDER_ROLL] + pi_2)
            self.Tjacobian[R_ARM + ELBOW_YAW] = make_dh_d(pi_2, self.joints[R_ARM + ELBOW_YAW])
            self.Tjacobian[R_ARM + ELBOW_ROLL] = make_dh_d(-pi_2, self.joints[R_ARM + ELBOW_ROLL])

        if ch in (CHAIN_L_LEG, CHAINS_SIZE):
            self.Tjacobian[L_LEG + HIP_YAW_PITCH] = make_dh_d(-3.0 * pi_2 / 2.0, self.joints[L_LEG + HIP_YAW_PITCH] - pi_2)
            self.Tjacobian[L_LEG + HIP_ROLL] = make_dh_d(-pi_2, self.joints[L_LEG + HIP_ROLL] + pi_2 / 2.0)
            self.Tjacobian[L_LEG + HIP_PITCH] = make_dh_d(pi_2, self.joints[L_LEG + HIP_PITCH])
            self.Tjacobian[L_LEG + KNEE_PITCH] = make_dh_d(0.0, self.joints[L_LEG + KNEE_PITCH])
            self.Tjacobian[L_LEG + ANKLE_PITCH] = make_dh_d(0.0, self.joints[L_LEG + ANKLE_PITCH])
            self.Tjacobian[L_LEG + ANKLE_ROLL] = make_dh_d(-pi_2, self.joints[L_LEG + ANKLE_ROLL])

        if ch in (CHAIN_R_LEG, CHAINS_SIZE):
            self.Tjacobian[R_LEG + HIP_YAW_PITCH] = make_dh_d(-pi_2 / 2.0, self.joints[R_LEG + HIP_YAW_PITCH] - pi_2)
            self.Tjacobian[R_LEG + HIP_ROLL] = make_dh_d(-pi_2, self.joints[R_LEG + HIP_ROLL] - pi_2 / 2.0)
            self.Tjacobian[R_LEG + HIP_PITCH] = make_dh_d(pi_2, self.joints[R_LEG + HIP_PITCH])
            self.Tjacobian[R_LEG + KNEE_PITCH] = make_dh_d(0.0, self.joints[R_LEG + KNEE_PITCH])
            self.Tjacobian[R_LEG + ANKLE_PITCH] = make_dh_d(0.0, self.joints[R_LEG + ANKLE_PITCH])
            self.Tjacobian[R_LEG + ANKLE_ROLL] = make_dh_d(-pi_2, self.joints[R_LEG + ANKLE_ROLL])

        if ch in (CHAIN_HEAD, CHAINS_SIZE):
            self.Tjacobian[HEAD + YAW] = make_dh_d(0.0, self.joints[HEAD + YAW])
            self.Tjacobian[HEAD + PITCH] = make_dh_d(-pi_2, self.joints[HEAD + PITCH] - pi_2)

    prepareDerivatives = prepare_derivatives

    def _pseudo_invert_jacobian(self, J):
        """KMat pseudo_invert: (J * J^T)^(-1) * J."""
        JJT = J @ J.T
        JJT_inv = np.linalg.inv(JJT)
        return JJT_inv @ J

    def jacobian_inverse_head(self, target_point, top_camera=False):
        target = self.get_transformation(target_point)
        alpha = 0.8
        clamp_val = 10.0
        max_iter = 50
        tol = 0.01

        frstart = HEAD
        chsize = HEAD_SIZE
        baseframe = CHAIN_HEAD
        ef = self.EFF_CAMERA_TOP if top_camera else self.EFF_CAMERA_BOT

        mask = np.ones((4, 4), dtype=np.float64)
        error = np.zeros((4, 4), dtype=np.float64)
        error_vector = np.zeros(16, dtype=np.float64)
        jacobian = np.zeros((chsize, 16), dtype=np.float64)

        self.prepare_forward(CHAIN_HEAD)
        self.prepare_derivatives(CHAIN_HEAD)

        iterations = -1
        while True:
            iterations += 1

            target_jacobian = self.T[self.FR_BASE_T + baseframe].mat.copy()
            at_to_gen = []
            for i in range(chsize):
                mat_i = self.T[self.FR_BASE + frstart + i].mat
                target_jacobian = target_jacobian @ mat_i
                at_to_gen.append(mat_i)

            target_jacobian = target_jacobian @ self.T[self.FR_END_T + ef].mat

            # Calculate derivatives
            derivatives = []
            for j in range(chsize):
                d = self.T[self.FR_BASE_T + baseframe].mat.copy()
                for i in range(chsize):
                    if j == i:
                        d = d @ self.Tjacobian[self.FR_BASE + frstart + i]
                    else:
                        d = d @ at_to_gen[i]
                d = d @ self.T[self.FR_END_T + ef].mat
                derivatives.append(d)

            # Error calculation
            max_err = 0.0
            for i in range(4):
                for j in range(4):
                    if i < 3:
                        err = (target[i, j] - target_jacobian[i, j]) * mask[i, j]
                        error[i, j] = err
                        if abs(err) > max_err:
                            max_err = abs(err)
                    else:
                        error[i, j] = 0.0

            value = (clamp_val / max_err) if max_err > clamp_val else 1.0

            for j in range(4):
                for i in range(4):
                    error_vector[i + j * 4] = error[i, j] * value

            for k in range(chsize):
                for j in range(4):
                    for i in range(4):
                        jacobian[k, i + j * 4] = derivatives[k][i, j]

            jacobian_pseudo_inv = self._pseudo_invert_jacobian(jacobian)
            jacobian_result = (jacobian_pseudo_inv @ error_vector) * alpha

            quality = 0.0
            for i in range(3):
                for j in range(4):
                    quality += (target[i, j] - target_jacobian[i, j]) * mask[i, j]

            for i in range(chsize):
                self.joints[frstart + i] += jacobian_result[i]

            self.prepare_forward(CHAIN_HEAD)
            self.prepare_derivatives(CHAIN_HEAD)

            if abs(quality) <= tol or iterations > max_iter:
                break

        r = [float(self.joints[frstart + i]) for i in range(chsize)]
        return [r]

    jacobianInverseHead = jacobian_inverse_head

    def jacobian_inverse_hands(self, target_point, ch):
        target = self.get_transformation(target_point)
        alpha = 0.8
        clamp_val = 10.0
        max_iter = 50
        tol = 0.01

        frstart = L_ARM if ch == CHAIN_L_ARM else R_ARM
        chsize = ARM_SIZE
        baseframe = ch
        ef = ch

        mask = np.ones((4, 4), dtype=np.float64)
        error = np.zeros((4, 4), dtype=np.float64)
        error_vector = np.zeros(16, dtype=np.float64)
        jacobian = np.zeros((chsize, 16), dtype=np.float64)

        self.prepare_forward(ch)
        self.prepare_derivatives(ch)

        iterations = -1
        while True:
            iterations += 1

            target_jacobian = self.T[self.FR_BASE_T + baseframe].mat.copy()
            at_to_gen = []
            for i in range(chsize):
                mat_i = self.T[self.FR_BASE + frstart + i].mat
                target_jacobian = target_jacobian @ mat_i
                at_to_gen.append(mat_i)

            target_jacobian = target_jacobian @ self.T[self.FR_END_T + ef].mat

            derivatives = []
            for j in range(chsize):
                d = self.T[self.FR_BASE_T + baseframe].mat.copy()
                for i in range(chsize):
                    if j == i:
                        d = d @ self.Tjacobian[self.FR_BASE + frstart + i]
                    else:
                        d = d @ at_to_gen[i]
                d = d @ self.T[self.FR_END_T + ef].mat
                derivatives.append(d)

            max_err = 0.0
            for i in range(4):
                for j in range(4):
                    if i < 3:
                        err = (target[i, j] - target_jacobian[i, j]) * mask[i, j]
                        error[i, j] = err
                        if abs(err) > max_err:
                            max_err = abs(err)
                    else:
                        error[i, j] = 0.0

            value = (clamp_val / max_err) if max_err > clamp_val else 1.0

            for j in range(4):
                for i in range(4):
                    error_vector[i + j * 4] = error[i, j] * value

            for k in range(chsize):
                for j in range(4):
                    for i in range(4):
                        jacobian[k, i + j * 4] = derivatives[k][i, j]

            jacobian_pseudo_inv = self._pseudo_invert_jacobian(jacobian)
            jacobian_result = (jacobian_pseudo_inv @ error_vector) * alpha

            quality = 0.0
            for i in range(3):
                for j in range(4):
                    quality += (target[i, j] - target_jacobian[i, j]) * mask[i, j]

            for i in range(chsize):
                self.joints[frstart + i] += jacobian_result[i]

            self.prepare_forward(ch)
            self.prepare_derivatives(ch)

            if abs(quality) <= tol or iterations > max_iter:
                break

        r = [float(self.joints[frstart + i]) for i in range(chsize)]
        return [r]

    jacobianInverseHands = jacobian_inverse_hands

    def jacobian_inverse_left_hand(self, target_point):
        return self.jacobian_inverse_hands(target_point, CHAIN_L_ARM)

    jacobianInverseLeftHand = jacobian_inverse_left_hand

    def jacobian_inverse_right_hand(self, target_point):
        return self.jacobian_inverse_hands(target_point, CHAIN_R_ARM)

    jacobianInverseRightHand = jacobian_inverse_right_hand

    def jacobian_inverse_legs(self, target_point, ch):
        target = self.get_transformation(target_point)
        alpha = 0.8
        clamp_val = 10.0
        max_iter = 50
        tol = 0.01

        frstart = L_LEG if ch == CHAIN_L_LEG else R_LEG
        chsize = LEG_SIZE
        baseframe = ch
        ef = ch

        mask = np.ones((4, 4), dtype=np.float64)
        error = np.zeros((4, 4), dtype=np.float64)
        error_vector = np.zeros(16, dtype=np.float64)
        jacobian = np.zeros((chsize, 16), dtype=np.float64)

        self.prepare_forward(ch)
        self.prepare_derivatives(ch)

        iterations = -1
        while True:
            iterations += 1

            target_jacobian = self.T[self.FR_BASE_T + baseframe].mat.copy()
            at_to_gen = []
            for i in range(chsize):
                mat_i = self.T[self.FR_BASE + frstart + i].mat
                target_jacobian = target_jacobian @ mat_i
                at_to_gen.append(mat_i)

            target_jacobian = target_jacobian @ self.T[self.FR_END_T + ef].mat

            derivatives = []
            for j in range(chsize):
                d = self.T[self.FR_BASE_T + baseframe].mat.copy()
                for i in range(chsize):
                    if j == i:
                        d = d @ self.Tjacobian[self.FR_BASE + frstart + i]
                    else:
                        d = d @ at_to_gen[i]
                d = d @ self.T[self.FR_END_T + ef].mat
                derivatives.append(d)

            max_err = 0.0
            for i in range(4):
                for j in range(4):
                    if i < 3:
                        err = (target[i, j] - target_jacobian[i, j]) * mask[i, j]
                        error[i, j] = err
                        if abs(err) > max_err:
                            max_err = abs(err)
                    else:
                        error[i, j] = 0.0

            value = (clamp_val / max_err) if max_err > clamp_val else 1.0

            for j in range(4):
                for i in range(4):
                    error_vector[i + j * 4] = error[i, j] * value

            for k in range(chsize):
                for j in range(4):
                    for i in range(4):
                        jacobian[k, i + j * 4] = derivatives[k][i, j]

            jacobian_pseudo_inv = self._pseudo_invert_jacobian(jacobian)
            jacobian_result = (jacobian_pseudo_inv @ error_vector) * alpha

            quality = 0.0
            for i in range(3):
                for j in range(4):
                    quality += (target[i, j] - target_jacobian[i, j]) * mask[i, j]

            for i in range(chsize):
                self.joints[frstart + i] += jacobian_result[i]

            self.prepare_forward(ch)
            self.prepare_derivatives(ch)

            if abs(quality) <= tol or iterations > max_iter:
                break

        r = [float(self.joints[frstart + i]) for i in range(chsize)]
        return [r]

    jacobianInverseLegs = jacobian_inverse_legs

    def jacobian_inverse_left_leg(self, target_point):
        return self.jacobian_inverse_legs(target_point, CHAIN_L_LEG)

    jacobianInverseLeftLeg = jacobian_inverse_left_leg

    def jacobian_inverse_right_leg(self, target_point):
        return self.jacobian_inverse_legs(target_point, CHAIN_R_LEG)

    jacobianInverseRightLeg = jacobian_inverse_right_leg

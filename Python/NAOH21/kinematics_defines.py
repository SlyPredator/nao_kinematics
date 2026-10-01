"""Kinematics parameters, limits, and masses for NAO H21."""

# Lengths of arms, legs, offsets (in mm)
ShoulderOffsetY = 98.0
ShoulderOffsetZ = 100.0
ElbowOffsetY = 15.0
UpperArmLength = 105.0
LowerArmLength = 57.75
HandOffsetX = 55.95
HandOffsetZ = 12.31
HipOffsetZ = 85.0
HipOffsetY = 50.0
ThighLength = 100.0
TibiaLength = 102.9
FootHeight = 45.11
NeckOffsetZ = 126.5
CameraBotomX = 48.8
CameraBotomZ = 23.81
CameraTopX = 53.9
CameraTopZ = 67.9

# Head Limits (radians)
HeadYawHigh = 2.0857
HeadYawLow = -2.0857
HeadPitchHigh = 0.5149
HeadPitchLow = -0.6720

# Left Arm Limits
LShoulderPitchHigh = 2.0857
LShoulderPitchLow = -2.0857
LShoulderRollHigh = 1.3265
LShoulderRollLow = -0.3142
LElbowYawHigh = 2.0875
LElbowYawLow = -2.0875
LElbowRollHigh = 0.00001
LElbowRollLow = -1.5446

# Right Arm Limits
RShoulderPitchHigh = 2.0857
RShoulderPitchLow = -2.0857
RShoulderRollHigh = 0.3142
RShoulderRollLow = -1.3265
RElbowYawHigh = 2.0875
RElbowYawLow = -2.0875
RElbowRollHigh = 1.5446
RElbowRollLow = -0.00001

# Left Leg Limits
LHipYawPitchHigh = 0.7408
LHipYawPitchLow = -1.1453
LHipRollHigh = 0.7904
LHipRollLow = -0.3794
LHipPitchHigh = 0.4840
LHipPitchLow = -1.7739
LKneePitchHigh = 2.1125
LKneePitchLow = -0.0923
LAnklePitchHigh = 0.9227
LAnklePitchLow = -1.1895
LAnkleRollHigh = 0.7690
LAnkleRollLow = -0.3978

# Right Leg Limits
RHipYawPitchHigh = 0.7408
RHipYawPitchLow = -1.1453
RHipRollHigh = 0.4147
RHipRollLow = -0.7383
RHipPitchHigh = 0.4856
RHipPitchLow = -1.7723
RKneePitchHigh = 2.1201
RKneePitchLow = -0.1030
RAnklePitchHigh = 0.9320
RAnklePitchLow = -1.1864
RAnkleRollHigh = 0.3886
RAnkleRollLow = -1.1864

# Masses (kg) and Centers of Mass (mm)
TotalMassH21 = 4.879 + 0.345

# Torso
TorsoMass = 1.03948
TorsoX = -4.15
TorsoY = 0.07
TorsoZ = 42.58

# Battery
BatteryMass = 0.345
BatteryX = -30.00
BatteryY = 0.00
BatteryZ = 39.00

# Head
HeadYawMass = 0.05930
HeadYawX = -0.02
HeadYawY = 0.17
HeadYawZ = -25.56

HeadPitchMass = 0.52065
HeadPitchX = 1.20
HeadPitchY = -0.84
HeadPitchZ = 53.53

# Right Arm
RShoulderPitchMass = 0.06996
RShoulderPitchX = -1.78
RShoulderPitchY = 24.96
RShoulderPitchZ = 0.18

RShoulderRollMass = 0.12309
RShoulderRollX = 18.85
RShoulderRollY = -5.77
RShoulderRollZ = 0.65

RElbowYawMass = 0.05971
RElbowYawX = -25.60
RElbowYawY = 0.01
RElbowYawZ = -0.19

RElbowRollMass = 0.18500
RElbowRollX = 65.36
RElbowRollY = -0.34
RElbowRollZ = -0.02

# Right Leg
RHipYawPitchMass = 0.07117
RHipYawPitchX = -7.66
RHipYawPitchY = 12.00
RHipYawPitchZ = 27.17

RHipRollMass = 0.13530
RHipRollX = -16.49
RHipRollY = -0.29
RHipRollZ = -4.75

RHipPitchMass = 0.39421
RHipPitchX = 1.32
RHipPitchY = -2.35
RHipPitchZ = -53.52

RKneePitchMass = 0.29159
RKneePitchX = 4.22
RKneePitchY = -2.52
RKneePitchZ = -48.68

RAnklePitchMass = 0.13892
RAnklePitchX = 1.42
RAnklePitchY = -0.28
RAnklePitchZ = 6.38

RAnkleRollMass = 0.16175
RAnkleRollX = 25.40
RAnkleRollY = -3.32
RAnkleRollZ = -32.41

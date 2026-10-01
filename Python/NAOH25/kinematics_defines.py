"""Kinematics parameters, limits, and masses for NAO H25."""

# Lengths of arms, legs, offsets (in mm)
ShoulderOffsetY = 98.0
ElbowOffsetY = 15.0
UpperArmLength = 105.0
ShoulderOffsetZ = 100.0
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
LWristYawHigh = 1.8238
LWristYawLow = -1.8238

# Right Arm Limits
RShoulderPitchHigh = 2.0857
RShoulderPitchLow = -2.0857
RShoulderRollHigh = 0.3142
RShoulderRollLow = -1.3265
RElbowYawHigh = 2.0875
RElbowYawLow = -2.0875
RElbowRollHigh = 1.5446
RElbowRollLow = 0.00001
RWristYawHigh = 1.8238
RWristYawLow = -1.8238

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
TotalMassH25 = 5.182530 + 0.345

# Torso
TorsoMass = 1.04956
TorsoX = -4.13
TorsoY = 0.09
TorsoZ = 43.42

# Battery
BatteryMass = 0.345
BatteryX = -30.00
BatteryY = 0.00
BatteryZ = 39.00

# Head
HeadYawMass = 0.06442
HeadYawX = -0.01
HeadYawY = 0.14
HeadYawZ = -27.42

HeadPitchMass = 0.60533
HeadPitchX = -1.12
HeadPitchY = 0.0
HeadPitchZ = 52.58

# Right Arm
RShoulderPitchMass = 0.06996
RShoulderPitchX = -1.65
RShoulderPitchY = 26.63
RShoulderPitchZ = 0.14

RShoulderRollMass = 0.15794
RShoulderRollX = 24.29
RShoulderRollY = -9.52
RShoulderRollZ = 0.32

RElbowYawMass = 0.06483
RElbowYawX = -27.44
RElbowYawY = 0.00
RElbowYawZ = -0.14

RElbowRollMass = 0.07778
RElbowRollX = 25.52
RElbowRollY = -2.81
RElbowRollZ = 0.9

RWristYawMass = 0.18533
RWristYawX = LowerArmLength + 34.34
RWristYawY = -0.88
RWristYawZ = 3.08

# Right Leg
RHipYawPitchMass = 0.07118
RHipYawPitchX = -7.66
RHipYawPitchY = 12.00
RHipYawPitchZ = 27.16

RHipRollMass = 0.13053
RHipRollX = -15.49
RHipRollY = -0.29
RHipRollZ = -5.16

RHipPitchMass = 0.38976
RHipPitchX = 1.39
RHipPitchY = -2.25
RHipPitchZ = -53.74

RKneePitchMass = 0.29163
RKneePitchX = 3.94
RKneePitchY = -2.21
RKneePitchZ = -49.38

RAnklePitchMass = 0.13415
RAnklePitchX = 0.45
RAnklePitchY = -0.3
RAnklePitchZ = 6.84

RAnkleRollMass = 0.16171
RAnkleRollX = 25.42
RAnkleRollY = -3.32
RAnkleRollZ = -32.39

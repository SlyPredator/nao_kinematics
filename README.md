NAOKinematics
=============

# Original Repo README ([Source](https://github.com/kouretes/NAOKinematics))

Forward and Inverse Kinematics solver for the NAO robot in C++.  Details about the library can be found in the following two documents:

 * Kofinas N., Orfanoudakis E., Lagoudakis M.: [Complete Analytical Inverse Kinematics for NAO](http://www.nikofinas.com/Publications/Complete_Analytical_Inverse_Kinematics_for_NAO.pdf), Proceedings of the 13th International Conference on Autonomous Robot Systems and Competitions (ROBOTICA), Lisbon, Portugal, April 2013, pp. 1-6.
 *  Kofinas N.: [Forward and Inverse Kinematics for the NAO Humanoid Robot](http://www.nikofinas.com/Projects/KofinasThesis.pdf), Diploma Thesis, Department of Electronic and Computer Engineering, Technical University of Crete, July 2012.

# Added Changes

# NAOKinematics Python Library

A Python port of the C++ `NAOKinematics` library for the Aldebaran NAO humanoid robot, supporting both **NAOH21** (21-DOF) and **NAOH25** (25-DOF) configurations.

## Features

- **Exact Mathematical Parity**: Ported directly from the C++ library with identical DH parameters, base and end effector transforms, and joint limits.
- **Forward Kinematics (FK)**:
  - Head Chain & Cameras (Bottom Camera and Top Camera)
  - Left & Right Arms (4-DOF on H21, 5-DOF on H25 with WristYaw)
  - Left & Right Legs (6-DOF with ankle roll/pitch and hip joints)
  - Relative chain-to-chain transforms (`get_forward_from_to`)
- **Center of Mass (CoM)**:
  - Link-by-link CoM transformations and total body center of mass including torso and battery masses.
- **Analytical (Closed-Form) Inverse Kinematics (IK)**:
  - Geometric closed-form solutions for Head, Arms (H21 and H25), and Legs.
  - Automatic filtering against joint limits and forward verification.
- **Numerical Jacobian Inverse Kinematics**:
  - Analytical DH partial derivative matrices.
  - Pseudo-inverse with error clamping and convergence iterations.

## Directory Structure

```
Python/
├── NAOH21/                 # 21-DOF NAO kinematics
│   ├── __init__.py
│   ├── robot_consts.py     # Chains and joint enumerations
│   ├── kinematics_defines.py # Physical dimensions, joint limits, masses
│   ├── kmat.py             # 4x4 Affine transformations and matrix math
│   ├── nao_kinematics.py   # Main kinematics class
│   └── main.py             # Demo equivalent to Cpp/NAOH21/Main.cpp
├── NAOH25/                 # 25-DOF NAO kinematics
│   ├── __init__.py
│   ├── robot_consts.py
│   ├── kinematics_defines.py
│   ├── kmat.py
│   ├── nao_kinematics.py
│   └── main.py             # Demo equivalent to Cpp/NAOH25/Main.cpp
├── tests/                  # Verification & parity test suite
│   ├── test_parity_h21.py  # Validation against C++ for H21
│   ├── test_parity_h25.py  # Validation against C++ for H25
│   └── test_random_poses.py# Cross-validation over random joint poses
└── verify_parity.py        # Master verification test runner
```

## Quick Start

### Running Demos
```bash
# Run H21 demonstration
python -m Python.NAOH21.main

# Run H25 demonstration
python -m Python.NAOH25.main
```

### Basic Usage Example

```python
import math
from Python.NAOH21.nao_kinematics import NAOKinematics
from Python.NAOH21.robot_consts import (
    CHAIN_L_ARM, CHAIN_L_LEG, EFF_CAMERA_BOT,
    L_ARM, SHOULDER_PITCH, SHOULDER_ROLL, NUMOFJOINTS
)

# Initialize solver
nkin = NAOKinematics()

# Set joints
joints = [0.0] * NUMOFJOINTS
joints[L_ARM + SHOULDER_PITCH] = math.pi / 2.0
joints[L_ARM + SHOULDER_ROLL] = math.pi / 4.0
nkin.set_joints(joints)

# Compute Forward Kinematics
left_arm_pose = nkin.get_forward_effector(CHAIN_L_ARM)
print("Left arm end effector position:", left_arm_pose.get_translation())

# Compute Analytical Inverse Kinematics
solutions = nkin.inverse_left_hand(left_arm_pose)
print("IK Solutions found:", len(solutions))
for sol in solutions:
    print("Joint angles:", sol)

# Calculate Center of Mass
com = nkin.calculate_center_of_mass()
print("Robot Center of Mass (x, y, z):", com)
```

## Verification & Parity

To verify parity between the Python port and the original C++ library:

```bash
python -m Python.verify_parity
```

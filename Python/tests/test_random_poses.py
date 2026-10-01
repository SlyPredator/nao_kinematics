"""Random joint configurations parity test comparing C++ and Python."""

import os
import subprocess
import numpy as np
from pathlib import Path

from Python.NAOH21.nao_kinematics import NAOKinematics as NAOKinematicsH21
from Python.NAOH25.nao_kinematics import NAOKinematics as NAOKinematicsH25
from Python.NAOH21.robot_consts import NUMOFJOINTS as NUMOFJOINTS_H21, CHAINS_SIZE
from Python.NAOH25.robot_consts import NUMOFJOINTS as NUMOFJOINTS_H25

WORKSPACE = Path(__file__).resolve().parent.parent.parent


def test_random_poses(version="NAOH21", num_samples=10):
    np.random.seed(1234)
    if version == "NAOH21":
        nkin_py = NAOKinematicsH21()
        num_joints = NUMOFJOINTS_H21
    else:
        nkin_py = NAOKinematicsH25()
        num_joints = NUMOFJOINTS_H25

    cpp_dir = WORKSPACE / "Cpp" / version
    src_file = cpp_dir / f"temp_{version}.cpp"
    exe_file = cpp_dir / f"temp_{version}.exe"

    for sample_idx in range(num_samples):
        joints = np.random.uniform(-0.8, 0.8, size=num_joints).tolist()
        nkin_py.set_joints(joints)

        cpp_snippet = f"""
#include <iostream>
#include <vector>
#include <iomanip>
#include "NAOKinematics.h"

using namespace std;
using namespace KDeviceLists;

int main() {{
    cout << fixed << setprecision(10);
    NAOKinematics nkin;
    vector<float> j = {{{", ".join(str(x) for x in joints)}}};
    nkin.setJoints(j);
    for (int ch = 0; ch < CHAINS_SIZE; ++ch) {{
        NAOKinematics::kmatTable m = nkin.getForwardEffector((NAOKinematics::Effectors)ch);
        for(int r=0; r<3; ++r)
            for(int c=0; c<4; ++c)
                cout << m(r, c) << " ";
        cout << endl;
    }}
    KVecDouble3 com = nkin.calculateCenterOfMass();
    cout << com(0) << " " << com(1) << " " << com(2) << endl;
    return 0;
}}
"""
        with open(src_file, "w") as f:
            f.write(cpp_snippet)

        compile_cmd = [
            "g++", "-O2", "-fpermissive", f"-I{cpp_dir}",
            str(src_file), str(cpp_dir / "NAOKinematics.cpp"), str(cpp_dir / "NAOKinematicsJacobian.cpp"),
            "-o", str(exe_file)
        ]
        sub = subprocess.run(compile_cmd, capture_output=True, text=True, cwd=str(WORKSPACE))
        assert sub.returncode == 0, f"Compile error: {sub.stderr}"

        run_sub = subprocess.run([str(exe_file)], capture_output=True, text=True, cwd=str(WORKSPACE))
        assert run_sub.returncode == 0, f"Run error: {run_sub.stderr}"

        lines = run_sub.stdout.strip().splitlines()

        for ch in range(CHAINS_SIZE):
            nums = [float(x) for x in lines[ch].split()]
            m_cpp = np.array(nums).reshape((3, 4))
            m_py = nkin_py.get_forward_effector(ch).mat[:3, :]
            diff = np.max(np.abs(m_py - m_cpp))
            assert diff < 1e-5, f"{version} Sample {sample_idx} Chain {ch} diff {diff} > 1e-5"

        com_nums = [float(x) for x in lines[-1].split()]
        com_cpp = np.array(com_nums)
        com_py = nkin_py.calculate_center_of_mass()
        com_diff = np.max(np.abs(com_py - com_cpp))
        assert com_diff < 1e-4, f"{version} Sample {sample_idx} CoM diff {com_diff} > 1e-4"

        try:
            if src_file.exists():
                os.remove(src_file)
            if exe_file.exists():
                os.remove(exe_file)
        except OSError:
            pass

    print(f"[OK] Tested {num_samples} random {version} configurations against C++: ALL MATCHED perfectly!")


if __name__ == "__main__":
    test_random_poses("NAOH21", 5)
    test_random_poses("NAOH25", 5)

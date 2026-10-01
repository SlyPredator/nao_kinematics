"""Helper to compile and run C++ test oracles for parity testing."""

import os
import subprocess
from pathlib import Path

WORKSPACE_ROOT = Path(__file__).resolve().parent.parent.parent


def get_cpp_oracle_output(version="NAOH21"):
    """Compile and execute Cpp/{version}/test_oracle.cpp and return its stdout."""
    cpp_dir = WORKSPACE_ROOT / "Cpp" / version
    src_file = cpp_dir / "test_oracle.cpp"
    exe_file = cpp_dir / "test_oracle.exe"

    if not exe_file.exists():
        cmd = [
            "g++",
            "-O2",
            "-fpermissive",
            f"-I{cpp_dir}",
            str(src_file),
            str(cpp_dir / "NAOKinematics.cpp"),
            str(cpp_dir / "NAOKinematicsJacobian.cpp"),
            "-o",
            str(exe_file),
        ]
        res = subprocess.run(cmd, capture_output=True, text=True, cwd=str(WORKSPACE_ROOT))
        if res.returncode != 0:
            raise RuntimeError(f"Failed to compile {version} C++ oracle: {res.stderr}")

    res = subprocess.run([str(exe_file)], capture_output=True, text=True, cwd=str(WORKSPACE_ROOT))
    if res.returncode != 0:
        raise RuntimeError(f"Failed to run {version} C++ oracle: {res.stderr}")

    return res.stdout

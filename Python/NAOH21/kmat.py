"""Matrix and kinematics transformation utilities matching KMat.hpp.

This module provides 4x4 affine transformation matrices, rotation helpers,
DH transformation and derivative constructors, and pseudo-inverse operations.
"""

import math
import numpy as np


class ATMatrix:
    """4x4 Affine Transformation Matrix with homogeneous coordinates [0, 0, 0, 1] on row 3."""

    def __init__(self, data=None):
        if data is not None:
            self.mat = np.array(data, dtype=np.float64).copy()
        else:
            self.mat = np.eye(4, dtype=np.float64)

    @classmethod
    def identity(cls):
        return cls(np.eye(4, dtype=np.float64))

    @classmethod
    def zero(cls):
        return cls(np.zeros((4, 4), dtype=np.float64))

    def set_identity(self):
        self.mat = np.eye(4, dtype=np.float64)
        return self

    def set_zero(self):
        self.mat = np.zeros((4, 4), dtype=np.float64)
        return self

    def __getitem__(self, idx):
        return self.mat[idx]

    def __setitem__(self, idx, value):
        self.mat[idx] = value

    def __call__(self, r, c):
        return self.mat[r, c]

    def set(self, r, c, value):
        self.mat[r, c] = value

    def __mul__(self, other):
        if isinstance(other, ATMatrix):
            return ATMatrix(self.mat @ other.mat)
        elif isinstance(other, np.ndarray):
            if other.shape == (4, 4):
                return ATMatrix(self.mat @ other)
            elif other.ndim == 1 and len(other) == 3:
                # 3D vector transform [x, y, z, 1]
                v = np.append(other, 1.0)
                res = self.mat @ v
                return res[:3]
            elif other.ndim == 2 and other.shape[0] == 3:
                # 3xN coordinates
                ones = np.ones((1, other.shape[1]), dtype=np.float64)
                v = np.vstack([other, ones])
                res = self.mat @ v
                return res[:3, :]
            else:
                return self.mat @ other
        elif isinstance(other, (int, float)):
            return ATMatrix(self.mat * other)
        else:
            return NotImplemented

    def __imul__(self, other):
        if isinstance(other, ATMatrix):
            self.mat = self.mat @ other.mat
        elif isinstance(other, np.ndarray):
            self.mat = self.mat @ other
        elif isinstance(other, (int, float)):
            self.mat *= other
        else:
            return NotImplemented
        return self

    def copy(self):
        return ATMatrix(self.mat)

    def fast_invert(self):
        """Invert the affine transformation matrix in-place.

        Matches KMat's ATMatrix::fast_invert():
        For [A, B; 0, 1], A_inv = inv(A), B_inv = -A_inv * B.
        """
        A = self.mat[:3, :3]
        B = self.mat[:3, 3]
        A_inv = np.linalg.inv(A)
        B_inv = -A_inv @ B
        self.mat[:3, :3] = A_inv
        self.mat[:3, 3] = B_inv
        self.mat[3, :3] = 0.0
        self.mat[3, 3] = 1.0
        return self

    def get_translation(self):
        """Return the 3-element translation vector [px, py, pz]."""
        return self.mat[:3, 3].copy()

    def getTranslation(self):
        return self.get_translation()

    def get_euler_angles(self):
        """Return Euler angles [rx, ry, rz] matching KMat::getEulerAngles().

        rx = atan2(A(2,1), A(2,2))
        ry = atan2(-A(2,0), sqrt(A(2,1)^2 + A(2,2)^2))
        rz = atan2(A(1,0), A(0,0))
        """
        A = self.mat[:3, :3]
        rx = math.atan2(A[2, 1], A[2, 2])
        ry = math.atan2(-A[2, 0], math.sqrt(A[2, 1] ** 2 + A[2, 2] ** 2))
        rz = math.atan2(A[1, 0], A[0, 0])
        return np.array([rx, ry, rz], dtype=np.float64)

    def getEulerAngles(self):
        return self.get_euler_angles()

    def transform(self, point):
        """Transform 3D point (p' = R*p + t)."""
        p = np.asarray(point, dtype=np.float64)
        return (self.mat[:3, :3] @ p) + self.mat[:3, 3]

    def almost_equal_to(self, other, tol=0.5):
        """Check if this transform is within tolerance of other.

        Matches KMat::almostEqualTo: sqrt(norm2(t_diff)) + sqrt(norm2(r_diff)) < tol.
        """
        if isinstance(other, ATMatrix):
            other_mat = other
        else:
            other_mat = ATMatrix(other)

        t_diff = self.get_translation() - other_mat.get_translation()
        r_diff = self.get_euler_angles() - other_mat.get_euler_angles()

        t_norm = np.linalg.norm(t_diff)
        r_norm = np.linalg.norm(r_diff)
        return (t_norm + r_norm) < tol

    def almostEqualTo(self, other, tol=0.5):
        return self.almost_equal_to(other, tol)

    def __repr__(self):
        return f"ATMatrix(\n{np.array2string(self.mat, precision=4, suppress_small=True)}\n)"


class Transformations:
    """Transformation factory functions matching KMatTransf."""

    PI = math.pi
    PI_2 = math.pi / 2.0

    @staticmethod
    def make_translation(x, y, z):
        res = ATMatrix.identity()
        res.mat[0, 3] = float(x)
        res.mat[1, 3] = float(y)
        res.mat[2, 3] = float(z)
        return res

    makeTranslation = make_translation

    @staticmethod
    def make_rotation_x(theta):
        res = ATMatrix.identity()
        c = math.cos(theta)
        s = math.sin(theta)
        res.mat[1, 1] = c
        res.mat[1, 2] = -s
        res.mat[2, 1] = s
        res.mat[2, 2] = c
        return res

    makeRotationX = make_rotation_x

    @staticmethod
    def make_rotation_y(theta):
        res = ATMatrix.identity()
        c = math.cos(theta)
        s = math.sin(theta)
        res.mat[0, 0] = c
        res.mat[0, 2] = s
        res.mat[2, 0] = -s
        res.mat[2, 2] = c
        return res

    makeRotationY = make_rotation_y

    @staticmethod
    def make_rotation_z(theta):
        res = ATMatrix.identity()
        c = math.cos(theta)
        s = math.sin(theta)
        res.mat[0, 0] = c
        res.mat[0, 1] = -s
        res.mat[1, 0] = s
        res.mat[1, 1] = c
        return res

    makeRotationZ = make_rotation_z

    @staticmethod
    def make_rotation_xyz(x_angle, y_angle, z_angle):
        """Rx * Ry * Rz"""
        rx = Transformations.make_rotation_x(x_angle)
        ry = Transformations.make_rotation_y(y_angle)
        rz = Transformations.make_rotation_z(z_angle)
        return rx * ry * rz

    makeRotationXYZ = make_rotation_xyz

    @staticmethod
    def make_rotation_zyx(z_angle, y_angle, x_angle):
        """Rz * Ry * Rx"""
        rz = Transformations.make_rotation_z(z_angle)
        ry = Transformations.make_rotation_y(y_angle)
        rx = Transformations.make_rotation_x(x_angle)
        return rz * ry * rx

    makeRotationZYX = make_rotation_zyx

    @staticmethod
    def make_dh_transformation(a, alpha, d, theta):
        """Standard Denavit-Hartenberg transformation matrix."""
        res = ATMatrix()
        c_th = math.cos(theta)
        s_th = math.sin(theta)
        c_al = math.cos(alpha)
        s_al = math.sin(alpha)

        res.mat[0, 0] = c_th
        res.mat[0, 1] = -s_th
        res.mat[0, 2] = 0.0
        res.mat[0, 3] = float(a)

        res.mat[1, 0] = s_th * c_al
        res.mat[1, 1] = c_th * c_al
        res.mat[1, 2] = -s_al
        res.mat[1, 3] = -s_al * float(d)

        res.mat[2, 0] = s_th * s_al
        res.mat[2, 1] = c_th * s_al
        res.mat[2, 2] = c_al
        res.mat[2, 3] = c_al * float(d)

        res.mat[3, 0] = 0.0
        res.mat[3, 1] = 0.0
        res.mat[3, 2] = 0.0
        res.mat[3, 3] = 1.0
        return res

    makeDHTransformation = make_dh_transformation

    @staticmethod
    def make_dh_derivative(alpha, theta):
        """Derivative of DH transformation matrix with respect to theta."""
        res = np.zeros((4, 4), dtype=np.float64)
        c_th = math.cos(theta)
        s_th = math.sin(theta)
        c_al = math.cos(alpha)
        s_al = math.sin(alpha)

        res[0, 0] = -s_th
        res[0, 1] = -c_th
        res[1, 0] = c_th * c_al
        res[1, 1] = -s_th * c_al
        res[2, 0] = c_th * s_al
        res[2, 1] = -s_th * s_al
        return res

    makeDHDerivative = make_dh_derivative

    @staticmethod
    def make_transformation(px, py, pz, rx, ry, rz):
        """Construct full transformation matrix from position and Euler angles."""
        res = ATMatrix()
        cx = math.cos(rx)
        sx = math.sin(rx)
        cy = math.cos(ry)
        sy = math.sin(ry)
        cz = math.cos(rz)
        sz = math.sin(rz)

        res.mat[0, 0] = cz * cy
        res.mat[0, 1] = cz * sy * sx - sz * cx
        res.mat[0, 2] = cz * sy * cx + sz * sx
        res.mat[0, 3] = float(px)

        res.mat[1, 0] = sz * cy
        res.mat[1, 1] = sz * sy * sx + cz * cx
        res.mat[1, 2] = sz * sy * cx - cz * sx
        res.mat[1, 3] = float(py)

        res.mat[2, 0] = -sy
        res.mat[2, 1] = cy * sx
        res.mat[2, 2] = cy * cx
        res.mat[2, 3] = float(pz)

        res.mat[3, 0] = 0.0
        res.mat[3, 1] = 0.0
        res.mat[3, 2] = 0.0
        res.mat[3, 3] = 1.0
        return res

    makeTransformation = make_transformation

    @staticmethod
    def cast_to_gen_matrix(at):
        if isinstance(at, ATMatrix):
            return at.mat.copy()
        return np.array(at, dtype=np.float64).copy()

    castToGenMatrix = cast_to_gen_matrix


def pseudo_inverse(matrix):
    """Compute Moore-Penrose pseudo-inverse of a matrix."""
    return np.linalg.pinv(matrix)

"""姿勢の計算（[x, y, z, roll, pitch, yaw] と 4x4 同次変換、ROS のメッセージとの変換）.

localization_bringup の robot_localization_adapter.py にあったものを移した（2026-10-15、robot_localization を廃止したため）。
"""
import math

import numpy as np


def rpy_to_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]])


def matrix_to_rpy(r):
    roll = math.atan2(r[2, 1], r[2, 2])
    pitch = math.asin(max(-1.0, min(1.0, -r[2, 0])))
    yaw = math.atan2(r[1, 0], r[0, 0])
    return roll, pitch, yaw


def quat_to_matrix(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def vec_to_matrix(v):
    t = np.eye(4)
    t[:3, :3] = rpy_to_matrix(v[3], v[4], v[5])
    t[:3, 3] = v[:3]
    return t


def matrix_to_vec(t):
    return np.array([t[0, 3], t[1, 3], t[2, 3], *matrix_to_rpy(t[:3, :3])])


def pose_msg_to_matrix(p):
    t = np.eye(4)
    t[:3, :3] = quat_to_matrix(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
    t[:3, 3] = [p.position.x, p.position.y, p.position.z]
    return t


def transform_msg_to_matrix(tr):
    t = np.eye(4)
    q = tr.rotation
    t[:3, :3] = quat_to_matrix(q.x, q.y, q.z, q.w)
    t[:3, 3] = [tr.translation.x, tr.translation.y, tr.translation.z]
    return t

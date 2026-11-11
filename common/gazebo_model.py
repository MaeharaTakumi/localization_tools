"""Gazebo の車椅子モデル（URDF・diff_drive_controller）の定数.

Gazebo の録画の評価・再生・図で使う。実機の値は環境ファイル（localization_bringup の param/env/<色>.yaml）と録画の /tf_static から取る。
"""
from pose_math import vec_to_matrix

# 車輪（URDF の形状。diff_drive_controller の設定と同じ）[m]
WHEEL_RADIUS = 0.1325
WHEEL_SEPARATION = 0.492
# オドメトリの倍率 [v, omega]（真値 / オドメトリ。localization_bringup の gazebo.yaml の odom_scale）
ODOM_SCALE = (1.0, 0.922)

# 取付 base_footprint -> velodyne：URDF の base_footprint -> base_link -> velodyne_base_link -> velodyne（回転 0）
URDF_TFS = [
    {'parent': 'base_footprint', 'child': 'base_link', 'xyz': [0.0, 0.0, 0.10], 'rpy': [0.0, 0.0, 0.0]},
    {'parent': 'base_link', 'child': 'velodyne_base_link', 'xyz': [-0.17, 0.0, 1.0], 'rpy': [0.0, 0.0, 0.0]},
    {'parent': 'velodyne_base_link', 'child': 'velodyne', 'xyz': [0.0, 0.0, 0.0377], 'rpy': [0.0, 0.0, 0.0]},
]
# 同じ取付の 4x4（(-0.17, 0, 1.1377)、回転 0）
URDF_MOUNT = vec_to_matrix([-0.17, 0.0, 0.10 + 1.0 + 0.0377, 0.0, 0.0, 0.0])

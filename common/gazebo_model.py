"""Gazebo の車椅子モデル（URDF・diff_drive_controller）の定数.

Gazebo の録画の再生（eval/replay_outputs.py）と情報ファイル（eval/bag_info.py）で使う。実機の値は環境（localization_bringup の param/<色>/）と録画の /tf_static から取る。
"""
# 車輪（URDF の形状。diff_drive_controller の設定と同じ）[m]
WHEEL_RADIUS = 0.1325
WHEEL_SEPARATION = 0.492

# 取付 base_footprint -> velodyne：URDF の base_footprint -> base_link -> velodyne_base_link -> velodyne（回転 0）
URDF_TFS = [
    {'parent': 'base_footprint', 'child': 'base_link', 'xyz': [0.0, 0.0, 0.10], 'rpy': [0.0, 0.0, 0.0]},
    {'parent': 'base_link', 'child': 'velodyne_base_link', 'xyz': [-0.17, 0.0, 1.0], 'rpy': [0.0, 0.0, 0.0]},
    {'parent': 'velodyne_base_link', 'child': 'velodyne', 'xyz': [0.0, 0.0, 0.0377], 'rpy': [0.0, 0.0, 0.0]},
]
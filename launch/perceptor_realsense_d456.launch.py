# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0
#
# Top-level Isaac Perceptor launch file for RealSense D456.
# Starts: D456 camera driver + realsense_splitter + cuVSLAM + NvBlox + RViz2.
#
# Usage:
#   ros2 launch isaac_ros_perceptor_realsense_d456 perceptor_realsense_d456.launch.py
#   ros2 launch ... run_realsense:=False              # no camera (for testing)
#   ros2 launch ... rosbag:=/path/to/bag              # replay from rosbag
#   ros2 launch ... run_rviz:=False                   # headless mode
#   ros2 launch ... camera_serial_numbers:=<serial>   # specific D456 serial

from launch_ros.descriptions import ComposableNode

from isaac_ros_launch_utils.all_types import (
    IfCondition, LaunchDescription, Node, SetParameter, TimerAction,
    UnlessCondition,
)
import isaac_ros_launch_utils as lu
from nvblox_ros_python_utils.nvblox_constants import NVBLOX_CONTAINER_NAME


# Config file paths resolved at package install time
RVIZ_CONFIG = lu.get_path(
    'isaac_ros_perceptor_realsense_d456',
    'config/rviz/perceptor_realsense_d456.rviz')

NVBLOX_BASE_CONFIG = lu.get_path(
    'nvblox_examples_bringup',
    'config/nvblox/nvblox_base.yaml')

NVBLOX_REALSENSE_CONFIG = lu.get_path(
    'nvblox_examples_bringup',
    'config/nvblox/specializations/nvblox_realsense.yaml')

NVBLOX_D456_CONFIG = lu.get_path(
    'isaac_ros_perceptor_realsense_d456',
    'config/nvblox/nvblox_d456.yaml')

# NvBlox topic remappings for single RealSense (matches realsense_splitter outputs)
NVBLOX_REALSENSE_REMAPPINGS = [
    ('camera_0/depth/image',        '/camera0/realsense_splitter_node/output/depth'),
    ('camera_0/depth/camera_info',  '/camera0/depth/camera_info'),
    ('camera_0/color/image',        '/camera0/color/image_raw'),
    ('camera_0/color/camera_info',  '/camera0/color/camera_info'),
]


def generate_launch_description() -> LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg(
        'run_realsense', 'True',
        description='Launch RealSense driver. Set False to replay from rosbag.',
        cli=True)
    args.add_arg(
        'rosbag', 'None',
        description='Path to rosbag (runs on live sensor if not set).',
        cli=True)
    args.add_arg(
        'rosbag_args', '',
        description='Additional arguments for ros2 bag play.',
        cli=True)
    args.add_arg(
        'log_level', 'info',
        choices=['debug', 'info', 'warn'],
        cli=True)
    args.add_arg(
        'camera_serial_numbers', '',
        description='D456 serial number (empty = auto-detect first connected camera).',
        cli=True)
    args.add_arg(
        'run_rviz', 'True',
        description='Launch RViz2 for visualization.',
        cli=True)
    args.add_arg(
        'enable_imu_fusion', 'False',
        description='Enable IMU fusion in cuVSLAM (experimental with D456).',
        cli=True)
    args.add_arg(
        'enable_ground_constraint', 'False',
        description='Constrain cuVSLAM to 2D plane (useful for ground robots).',
        cli=True)
    args.add_arg(
        'container_name', NVBLOX_CONTAINER_NAME,
        description='Name of the component container.')

    actions = args.get_launch_actions()

    # Enable use_sim_time when replaying from rosbag
    actions.append(
        SetParameter('use_sim_time', True,
                     condition=IfCondition(lu.is_valid(args.rosbag))))

    # Component container — owns all composable nodes
    actions.append(
        lu.component_container(args.container_name, log_level=args.log_level))

    # D456 camera driver + realsense_splitter
    run_rs_driver = UnlessCondition(
        lu.OrSubstitution(lu.is_valid(args.rosbag), lu.is_false(args.run_realsense)))
    actions.append(
        lu.include(
            'isaac_ros_perceptor_realsense_d456',
            'launch/sensors/realsense_d456.launch.py',
            launch_arguments={
                'container_name': args.container_name,
                'camera_serial_numbers': args.camera_serial_numbers,
                'num_cameras': '1',
            },
            condition=run_rs_driver))

    # cuVSLAM — inline definition (replaces vslam.launch.py include)
    # to allow setting image_jitter_threshold_ms for 15fps emitter-split operation
    actions.append(lu.log_info(
        f'Starting cuVSLAM with base_frame: camera0_link'))
    vslam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        remappings=[
            ('visual_slam/camera_info_0', '/camera0/infra1/camera_info'),
            ('visual_slam/camera_info_1', '/camera0/infra2/camera_info'),
            ('visual_slam/image_0',
             '/camera0/realsense_splitter_node/output/infra_1'),
            ('visual_slam/image_1',
             '/camera0/realsense_splitter_node/output/infra_2'),
            ('visual_slam/imu', 'camera0/imu'),
        ],
        parameters=[{
            'num_cameras': 2,
            'min_num_images': 2,
            'enable_localization_n_mapping': False,
            'enable_rectified_pose': True,
            'enable_image_denoising': False,
            'rectified_images': True,
            'gyro_noise_density': 0.000244,
            'gyro_random_walk': 0.000019393,
            'accel_noise_density': 0.001862,
            'accel_random_walk': 0.003,
            'calibration_frequency': 200.0,
            'image_jitter_threshold_ms': 100.0,
            'rig_frame': 'base_link',
            'imu_frame': 'camera0_gyro_optical_frame',
            'enable_slam_visualization': True,
            'enable_landmarks_view': True,
            'enable_observations_view': True,
            'path_max_size': 200,
            'verbosity': 5,
            'enable_debug_mode': False,
            'debug_dump_path': '/tmp/cuvslam',
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'camera0_link',
            'camera_optical_frames': [
                'camera0_infra1_optical_frame',
                'camera0_infra2_optical_frame',
            ],
        },
            {'enable_ground_constraint_in_odometry': args.enable_ground_constraint},
            {'enable_imu_fusion': args.enable_imu_fusion},
        ])
    actions.append(
        TimerAction(period=1.0, actions=[
            lu.load_composable_nodes(args.container_name, [vslam_node])]))

    # NvBlox — inline definition to stack 3 config files: base + realsense + D456 overrides
    nvblox_node = ComposableNode(
        name='nvblox_node',
        package='nvblox_ros',
        plugin='nvblox::NvbloxNode',
        remappings=NVBLOX_REALSENSE_REMAPPINGS,
        parameters=[
            NVBLOX_BASE_CONFIG,
            NVBLOX_REALSENSE_CONFIG,
            NVBLOX_D456_CONFIG,
            {'num_cameras': 1, 'use_lidar': False},
        ])
    actions.append(lu.load_composable_nodes(args.container_name, [nvblox_node]))

    # Rosbag playback (conditional)
    actions.append(
        lu.play_rosbag(
            bag_path=args.rosbag,
            additional_bag_play_args=args.rosbag_args,
            condition=IfCondition(lu.is_valid(args.rosbag))))

    # RViz2
    actions.append(
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', str(RVIZ_CONFIG)],
            output='screen',
            condition=IfCondition(args.run_rviz)))

    return LaunchDescription(actions)

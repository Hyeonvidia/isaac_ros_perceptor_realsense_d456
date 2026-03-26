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
#
# Usage:
#   ros2 launch isaac_ros_perceptor_realsense_d456 perceptor_realsense_d456.launch.py
#   ros2 launch ... mode:=dynamic                              # dynamic scene
#   ros2 launch ... mode:=dynamic use_splitter:=False          # high-speed (30fps IR)
#   ros2 launch ... mode:=people_segmentation                  # people segmentation
#   ros2 launch ... mode:=people_detection                     # people detection
#   ros2 launch ... enable_ground_constraint:=True             # 2D plane constraint

from typing import List

from launch import Action
from launch_ros.descriptions import ComposableNode

from isaac_ros_launch_utils.all_types import (
    IfCondition, LaunchDescription, Node, SetParameter, TimerAction,
    UnlessCondition,
)
import isaac_ros_launch_utils as lu
from nvblox_ros_python_utils.nvblox_constants import NVBLOX_CONTAINER_NAME
from nvblox_ros_python_utils.nvblox_launch_utils import NvbloxMode, NvbloxPeopleSegmentation


# Config file paths
RVIZ_CONFIG = lu.get_path(
    'isaac_ros_perceptor_realsense_d456',
    'config/rviz/perceptor_realsense_d456.rviz')
NVBLOX_BASE_CONFIG = lu.get_path(
    'nvblox_examples_bringup', 'config/nvblox/nvblox_base.yaml')
NVBLOX_REALSENSE_CONFIG = lu.get_path(
    'nvblox_examples_bringup', 'config/nvblox/specializations/nvblox_realsense.yaml')
NVBLOX_D456_CONFIG = lu.get_path(
    'isaac_ros_perceptor_realsense_d456', 'config/nvblox/nvblox_d456.yaml')
NVBLOX_SEGMENTATION_CONFIG = lu.get_path(
    'nvblox_examples_bringup', 'config/nvblox/specializations/nvblox_segmentation.yaml')
NVBLOX_DETECTION_CONFIG = lu.get_path(
    'nvblox_examples_bringup', 'config/nvblox/specializations/nvblox_detection.yaml')
NVBLOX_DYNAMICS_CONFIG = lu.get_path(
    'nvblox_examples_bringup', 'config/nvblox/specializations/nvblox_dynamics.yaml')


def _get_nvblox_mode_config(mode: NvbloxMode):
    if mode is NvbloxMode.people_segmentation:
        return NVBLOX_SEGMENTATION_CONFIG
    elif mode is NvbloxMode.people_detection:
        return NVBLOX_DETECTION_CONFIG
    elif mode is NvbloxMode.dynamic:
        return NVBLOX_DYNAMICS_CONFIG
    return {}


def _get_nvblox_remappings(mode: NvbloxMode, use_splitter: bool):
    depth_topic = ('/camera0/realsense_splitter_node/output/depth'
                   if use_splitter else '/camera0/depth/image_rect_raw')
    remappings = [
        ('camera_0/depth/image', depth_topic),
        ('camera_0/depth/camera_info', '/camera0/depth/camera_info'),
    ]
    if mode is NvbloxMode.people_segmentation:
        remappings.extend([
            ('camera_0/color/image', '/camera0/segmentation/image_resized'),
            ('camera_0/color/camera_info', '/camera0/segmentation/camera_info_resized'),
            ('camera_0/mask/image', '/camera0/segmentation/people_mask'),
            ('camera_0/mask/camera_info', '/camera0/segmentation/camera_info_resized'),
        ])
    elif mode is NvbloxMode.people_detection:
        remappings.extend([
            ('camera_0/color/image', '/camera0/color/image_raw'),
            ('camera_0/color/camera_info', '/camera0/color/camera_info'),
            ('camera_0/mask/image', '/camera0/detection/people_mask'),
            ('camera_0/mask/camera_info', '/camera0/color/camera_info'),
        ])
    else:
        remappings.extend([
            ('camera_0/color/image', '/camera0/color/image_raw'),
            ('camera_0/color/camera_info', '/camera0/color/camera_info'),
        ])
    return remappings


def _get_vslam_remappings(use_splitter: bool):
    if use_splitter:
        return [
            ('visual_slam/image_0', '/camera0/realsense_splitter_node/output/infra_1'),
            ('visual_slam/image_1', '/camera0/realsense_splitter_node/output/infra_2'),
        ]
    else:
        return [
            ('visual_slam/image_0', '/camera0/infra1/image_rect_raw'),
            ('visual_slam/image_1', '/camera0/infra2/image_rect_raw'),
        ]


def add_perception_stack(args: lu.ArgumentContainer) -> List[Action]:
    """Opaque function: resolves mode/use_splitter at runtime, builds cuVSLAM + nvblox + DNN."""
    mode = NvbloxMode[args.mode]
    use_splitter = lu.is_true(args.use_splitter)
    actions = []

    # --- cuVSLAM ---
    vslam_remappings = [
        ('visual_slam/camera_info_0', '/camera0/infra1/camera_info'),
        ('visual_slam/camera_info_1', '/camera0/infra2/camera_info'),
        ('visual_slam/imu', 'camera0/imu'),
    ] + _get_vslam_remappings(use_splitter)

    jitter_ms = 100.0 if use_splitter else 34.0
    actions.append(lu.log_info(
        f"cuVSLAM: use_splitter={use_splitter}, jitter_threshold={jitter_ms}ms"))

    vslam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        remappings=vslam_remappings,
        parameters=[{
            'num_cameras': 2,
            'min_num_images': 2,
            'enable_localization_n_mapping': False,
            'enable_rectified_pose': True,
            'enable_image_denoising': not use_splitter,
            'rectified_images': True,
            'gyro_noise_density': 0.000244,
            'gyro_random_walk': 0.000019393,
            'accel_noise_density': 0.001862,
            'accel_random_walk': 0.003,
            'calibration_frequency': 200.0,
            'image_jitter_threshold_ms': jitter_ms,
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
    actions.append(TimerAction(period=1.0, actions=[
        lu.load_composable_nodes(args.container_name, [vslam_node])]))

    # --- People segmentation pipeline ---
    if mode is NvbloxMode.people_segmentation:
        actions.append(lu.include(
            'nvblox_examples_bringup',
            'launch/perception/segmentation.launch.py',
            launch_arguments={
                'container_name': args.container_name,
                'people_segmentation': args.people_segmentation,
                'namespace_list': ['camera0'],
                'input_topic_list': ['/camera0/color/image_raw'],
                'input_camera_info_topic_list': ['/camera0/color/camera_info'],
                'output_resized_image_topic_list': ['/camera0/segmentation/image_resized'],
                'output_resized_camera_info_topic_list': [
                    '/camera0/segmentation/camera_info_resized'],
                'num_cameras': '1',
                'one_container_per_camera': 'True',
            }))

    # --- People detection pipeline ---
    if mode is NvbloxMode.people_detection:
        actions.append(lu.include(
            'nvblox_examples_bringup',
            'launch/perception/detection.launch.py',
            launch_arguments={
                'namespace_list': ['camera0'],
                'input_topic_list': ['/camera0/color/image_raw'],
                'num_cameras': '1',
                'container_name': args.container_name,
                'one_container_per_camera': 'True',
            }))

    # --- NvBlox ---
    nvblox_remappings = _get_nvblox_remappings(mode, use_splitter)
    nvblox_mode_config = _get_nvblox_mode_config(mode)

    depth_rate = 15.0 if use_splitter else 30.0
    actions.append(lu.log_info(
        f"nvblox: mode='{mode}', depth_rate={depth_rate}Hz"))

    nvblox_node = ComposableNode(
        name='nvblox_node',
        package='nvblox_ros',
        plugin='nvblox::NvbloxNode',
        remappings=nvblox_remappings,
        parameters=[
            NVBLOX_BASE_CONFIG,
            nvblox_mode_config,
            NVBLOX_REALSENSE_CONFIG,
            NVBLOX_D456_CONFIG,
            {'num_cameras': 1, 'use_lidar': False,
             'integrate_depth_rate_hz': depth_rate},
        ])
    actions.append(lu.load_composable_nodes(args.container_name, [nvblox_node]))

    return actions


def generate_launch_description() -> LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg(
        'mode', default=NvbloxMode.static,
        choices=NvbloxMode.names(),
        description='NvBlox mapping mode.',
        cli=True)
    args.add_arg(
        'use_splitter', 'True',
        description='Use realsense_splitter (15fps clean IR). '
                    'Set False for high-speed (30fps IR with dot pattern).',
        cli=True)
    args.add_arg(
        'people_segmentation',
        default=NvbloxPeopleSegmentation.peoplesemsegnet_shuffleseg,
        choices=[str(NvbloxPeopleSegmentation.peoplesemsegnet_vanilla),
                 str(NvbloxPeopleSegmentation.peoplesemsegnet_shuffleseg)],
        description='People segmentation model (only with mode:=people_segmentation).',
        cli=True)
    args.add_arg('run_realsense', 'True', cli=True)
    args.add_arg('rosbag', 'None', cli=True)
    args.add_arg('rosbag_args', '', cli=True)
    args.add_arg('log_level', 'info', choices=['debug', 'info', 'warn'], cli=True)
    args.add_arg('camera_serial_numbers', '', cli=True)
    args.add_arg('run_rviz', 'True', cli=True)
    args.add_arg('enable_imu_fusion', 'False', cli=True)
    args.add_arg(
        'enable_ground_constraint', 'False',
        description='Constrain cuVSLAM to 2D plane (useful for vehicles).',
        cli=True)
    args.add_arg('container_name', NVBLOX_CONTAINER_NAME)

    # Register opaque function BEFORE get_launch_actions()
    args.add_opaque_function(add_perception_stack)

    actions = args.get_launch_actions()

    # use_sim_time for rosbag replay
    actions.append(
        SetParameter('use_sim_time', True,
                     condition=IfCondition(lu.is_valid(args.rosbag))))

    # Component container
    actions.append(
        lu.component_container(args.container_name, log_level=args.log_level))

    # D456 camera driver (+ splitter if use_splitter:=True)
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
                'use_splitter': args.use_splitter,
            },
            condition=run_rs_driver))

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

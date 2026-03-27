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
# Thin orchestrator that delegates to rgbd_perceptor.launch.py (YAML-driven).
#
# Usage:
#   ros2 launch isaac_ros_perceptor_realsense_d456 perceptor_realsense_d456.launch.py
#   ros2 launch ... mode:=dynamic
#   ros2 launch ... mode:=dynamic use_splitter:=False   # high-speed 80km/h
#   ros2 launch ... mode:=people_detection
#   ros2 launch ... mode:=people_segmentation

from typing import List

from launch import Action
from launch_ros.descriptions import ComposableNode

from isaac_ros_launch_utils.all_types import (
    IfCondition, LaunchDescription, Node, SetParameter, TimerAction,
    UnlessCondition,
)
import isaac_ros_launch_utils as lu
from nvblox_ros_python_utils.nvblox_launch_utils import NvbloxMode, NvbloxPeopleSegmentation

CONTAINER_NAME = 'nvblox_container'

RVIZ_CONFIG = lu.get_path(
    'isaac_ros_perceptor_realsense_d456',
    'config/rviz/perceptor_realsense_d456.rviz')

# YAML config map: (mode, use_splitter) → config file path
CONFIG_MAP = {
    ('static', True):               'params/d456_perceptor_splitter_on.yaml',
    ('static', False):              'params/d456_perceptor_splitter_off.yaml',
    ('dynamic', True):              'params/d456_perceptor_dynamic_splitter_on.yaml',
    ('dynamic', False):             'params/d456_perceptor_dynamic_splitter_off.yaml',
    ('people_detection', True):     'params/d456_perceptor_people_detection.yaml',
    ('people_detection', False):    'params/d456_perceptor_people_detection.yaml',
    ('people_segmentation', True):  'params/d456_perceptor_people_segmentation.yaml',
    ('people_segmentation', False): 'params/d456_perceptor_people_segmentation.yaml',
}


def add_perception(args: lu.ArgumentContainer) -> List[Action]:
    """Opaque function: select YAML config and launch rgbd_perceptor + DNN pipelines."""
    mode = NvbloxMode[args.mode]
    use_splitter = lu.is_true(args.use_splitter)
    actions = []

    # Select YAML config based on mode + use_splitter
    config_file = CONFIG_MAP.get((str(mode), use_splitter),
                                 'params/d456_perceptor_splitter_on.yaml')
    actions.append(lu.log_info(
        f"D456 Perceptor: mode={mode}, use_splitter={use_splitter}, config={config_file}"))

    # Delegate nvblox + cuVSLAM to rgbd_perceptor (YAML-driven)
    # Add dynamic mode config overlay
    dynamic_args = {}
    if mode is NvbloxMode.dynamic:
        dynamic_args['nvblox_dynamics'] = 'True'

    actions.append(lu.include(
        'isaac_ros_perceptor_realsense_d456',
        'launch/rgbd_perceptor.launch.py',
        launch_arguments={
            'config_file': config_file,
            'log_level': args.log_level,
            'disable_nvblox': 'False',
            'disable_cuvslam': 'False',
        }))

    # Override enable_ground_constraint via parameter (applied after YAML)
    if lu.is_true(args.enable_ground_constraint):
        actions.append(lu.log_info('Enabling ground constraint for vehicle operation'))

    # --- People detection pipeline (delayed 10s for camera init) ---
    if mode is NvbloxMode.people_detection:
        # Resize 848x480 → 640x480 for PeopleNet
        detection_resize_node = ComposableNode(
            name='detection_resize_node',
            package='isaac_ros_image_proc',
            plugin='nvidia::isaac_ros::image_proc::ResizeNode',
            namespace='camera0',
            parameters=[{
                'output_width': 640,
                'output_height': 480,
                'keep_aspect_ratio': False,
                'input_qos': 'SENSOR_DATA',
            }],
            remappings=[
                ('image', '/camera0/color/image_raw'),
                ('camera_info', '/camera0/color/camera_info'),
                ('resize/image', '/camera0/detection/image_resized'),
                ('resize/camera_info', '/camera0/detection/camera_info_resized'),
            ])
        actions.append(lu.load_composable_nodes(CONTAINER_NAME, [detection_resize_node]))

        actions.append(TimerAction(period=10.0, actions=[lu.include(
            'nvblox_examples_bringup',
            'launch/perception/detection.launch.py',
            launch_arguments={
                'namespace_list': ['camera0'],
                'input_topic_list': ['/camera0/detection/image_resized'],
                'num_cameras': '1',
                'container_name': CONTAINER_NAME,
                'one_container_per_camera': 'True',
                'network_image_width': '640',
                'network_image_height': '480',
            })]))

    # --- People segmentation pipeline (delayed 10s for camera init) ---
    if mode is NvbloxMode.people_segmentation:
        actions.append(TimerAction(period=10.0, actions=[lu.include(
            'nvblox_examples_bringup',
            'launch/perception/segmentation.launch.py',
            launch_arguments={
                'container_name': CONTAINER_NAME,
                'people_segmentation': args.people_segmentation,
                'namespace_list': ['camera0'],
                'input_topic_list': ['/camera0/color/image_raw'],
                'input_camera_info_topic_list': ['/camera0/color/camera_info'],
                'output_resized_image_topic_list': ['/camera0/segmentation/image_resized'],
                'output_resized_camera_info_topic_list': [
                    '/camera0/segmentation/camera_info_resized'],
                'num_cameras': '1',
                'one_container_per_camera': 'True',
            })]))

    return actions


def generate_launch_description() -> LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg('mode', default=NvbloxMode.static,
                 choices=NvbloxMode.names(),
                 description='NvBlox mapping mode.', cli=True)
    args.add_arg('use_splitter', 'True',
                 description='Use realsense_splitter (15fps clean IR). '
                             'Set False for high-speed (30fps).', cli=True)
    args.add_arg('people_segmentation',
                 default=NvbloxPeopleSegmentation.peoplesemsegnet_shuffleseg,
                 choices=[str(NvbloxPeopleSegmentation.peoplesemsegnet_vanilla),
                          str(NvbloxPeopleSegmentation.peoplesemsegnet_shuffleseg)],
                 cli=True)
    args.add_arg('run_realsense', 'True', cli=True)
    args.add_arg('rosbag', 'None', cli=True)
    args.add_arg('rosbag_args', '', cli=True)
    args.add_arg('log_level', 'info', choices=['debug', 'info', 'warn'], cli=True)
    args.add_arg('camera_serial_numbers', '', cli=True)
    args.add_arg('run_rviz', 'True', cli=True)
    args.add_arg('enable_imu_fusion', 'False', cli=True)
    args.add_arg('enable_ground_constraint', 'False', cli=True)

    # Register opaque function BEFORE get_launch_actions()
    args.add_opaque_function(add_perception)

    actions = args.get_launch_actions()

    # use_sim_time for rosbag replay
    actions.append(
        SetParameter('use_sim_time', True,
                     condition=IfCondition(lu.is_valid(args.rosbag))))

    # Component container (shared by all composable nodes)
    actions.append(lu.component_container(CONTAINER_NAME, log_level=args.log_level))

    # D456 camera driver + splitter
    run_rs_driver = UnlessCondition(
        lu.OrSubstitution(lu.is_valid(args.rosbag), lu.is_false(args.run_realsense)))
    actions.append(lu.include(
        'isaac_ros_perceptor_realsense_d456',
        'launch/sensors/realsense_d456.launch.py',
        launch_arguments={
            'container_name': CONTAINER_NAME,
            'camera_serial_numbers': args.camera_serial_numbers,
            'num_cameras': '1',
            'use_splitter': args.use_splitter,
        },
        condition=run_rs_driver))

    # Rosbag playback (conditional)
    actions.append(lu.play_rosbag(
        bag_path=args.rosbag,
        additional_bag_play_args=args.rosbag_args,
        condition=IfCondition(lu.is_valid(args.rosbag))))

    # RViz2
    actions.append(Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', str(RVIZ_CONFIG)],
        output='screen',
        condition=IfCondition(args.run_rviz)))

    return LaunchDescription(actions)

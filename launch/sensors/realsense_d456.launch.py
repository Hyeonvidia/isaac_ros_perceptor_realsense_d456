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
# RealSense D456 camera bringup launch file.
# Derived from nvblox_examples_bringup/launch/sensors/realsense.launch.py but
# substitutes D456-optimized config files (848x480x30 depth, 848x480x15 RGB).
#
# Behaviour:
#   - camera0 uses emitter_flashing config + realsense_splitter (for cuVSLAM)
#   - additional cameras (if any) use emitter_on config without splitter
#   - cameras after camera0 are started with a 10s delay each for USB stability

from typing import List, Optional

from isaac_ros_launch_utils.all_types import (
    ComposableNode, IfCondition, LaunchDescription, TimerAction,
)
import isaac_ros_launch_utils as lu
from nvblox_ros_python_utils.nvblox_constants import NVBLOX_CONTAINER_NAME


# D456-specific config paths (resolved at package install time)
EMITTER_FLASHING_CONFIG = lu.get_path(
    'isaac_ros_perceptor_realsense_d456',
    'config/sensors/realsense_d456_emitter_flashing.yaml')

EMITTER_ON_CONFIG = lu.get_path(
    'isaac_ros_perceptor_realsense_d456',
    'config/sensors/realsense_d456_emitter_on.yaml')


def get_camera_node(camera_name: str, config_file_path: str,
                    serial_number: Optional[str] = None) -> ComposableNode:
    parameters = [config_file_path, {'camera_name': camera_name}]
    if serial_number:
        parameters.append({'serial_no': str(serial_number)})
    return ComposableNode(
        namespace=camera_name,
        package='realsense2_camera',
        plugin='realsense2_camera::RealSenseNodeFactory',
        parameters=parameters)


def get_splitter_node(camera_name: str) -> ComposableNode:
    return ComposableNode(
        namespace=camera_name,
        name='realsense_splitter_node',
        package='realsense_splitter',
        plugin='nvblox::RealsenseSplitterNode',
        parameters=[{
            'input_qos': 'SENSOR_DATA',
            'output_qos': 'SYSTEM_DEFAULT',
        }],
        remappings=[
            ('input/infra_1',             f'/{camera_name}/infra1/image_rect_raw'),
            ('input/infra_1_metadata',    f'/{camera_name}/infra1/metadata'),
            ('input/infra_2',             f'/{camera_name}/infra2/image_rect_raw'),
            ('input/infra_2_metadata',    f'/{camera_name}/infra2/metadata'),
            ('input/depth',               f'/{camera_name}/depth/image_rect_raw'),
            ('input/depth_metadata',      f'/{camera_name}/depth/metadata'),
            ('input/pointcloud',          f'/{camera_name}/depth/color/points'),
            ('input/pointcloud_metadata', f'/{camera_name}/depth/metadata'),
        ])


def add_cameras(args: lu.ArgumentContainer) -> List:
    """Build composable nodes for each camera up to num_cameras."""
    serial_numbers_str = str(args.camera_serial_numbers)
    if serial_numbers_str == '':
        serial_numbers = [None]
    else:
        serial_numbers = serial_numbers_str.split(',')

    num_cameras = int(args.num_cameras)
    use_splitter = lu.is_true(args.use_splitter)
    actions = []

    for idx in range(num_cameras):
        camera_name = f'camera{idx}'
        # camera0 uses splitter (emitter flashing) only if use_splitter is True
        run_splitter = (idx == 0) and use_splitter
        config = EMITTER_FLASHING_CONFIG if run_splitter else EMITTER_ON_CONFIG
        serial = serial_numbers[idx] if idx < len(serial_numbers) else None

        nodes = [get_camera_node(camera_name, config, serial)]
        if run_splitter:
            nodes.append(get_splitter_node(camera_name))

        # Stagger camera startup to avoid RealSenseNodeFactory instability
        actions.append(
            TimerAction(
                period=float(idx * 10),
                actions=[lu.load_composable_nodes(args.container_name, nodes)]))
        actions.append(
            lu.log_info(
                f'Launching D456 camera: {camera_name}, splitter: {run_splitter}'))

    return actions


def generate_launch_description() -> LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg('container_name', NVBLOX_CONTAINER_NAME)
    args.add_arg('run_standalone', 'False')
    args.add_arg('camera_serial_numbers', '',
                 description='Comma-separated serial numbers (empty = auto-detect first).')
    args.add_arg('num_cameras', 1)
    args.add_arg('use_splitter', 'True',
                 description='Use realsense_splitter for emitter on/off separation.')

    args.add_opaque_function(add_cameras)
    actions = args.get_launch_actions()
    actions.append(
        lu.component_container(
            args.container_name,
            condition=IfCondition(lu.is_true(args.run_standalone))))
    return LaunchDescription(actions)

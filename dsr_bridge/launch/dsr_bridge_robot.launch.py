#!/usr/bin/env python3

# Copyright (c) 2023 Alberto J. Tudela Roldán
# Copyright (c) 2023 Grupo Avispa, DTE, Universidad de Málaga
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launches the DSR bridge in the robot for the CAMPERO project."""

import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Getting directories and launch-files
    dsr_bridge_dir = get_package_share_directory('dsr_bridge')
    dsr_util_dir = get_package_share_directory('dsr_util')
    default_params_file = os.path.join(dsr_bridge_dir, 'params', 'default_params.yaml')

    # Input parameters declaration
    params_file = LaunchConfiguration('params_file')
    log_level = LaunchConfiguration('log-level')

    declare_params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file with dsr agent configuration'
    )

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'dsr_input_file': os.path.join(dsr_util_dir, 'worlds', 'world_with_robot.json')
    }

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True
    )

    declare_log_level_arg = DeclareLaunchArgument(
        name='log-level',
        default_value='info',
        description='Logging level (info, debug, ...)'
    )

    # Prepare the node
    dsr_bridge_robot_node = Node(
        package='dsr_bridge',
        namespace='',
        executable='dsr_bridge_node',
        name='dsr_bridge_robot',
        parameters=[configured_params],
        emulate_tty=True,
        output='screen',
        arguments=[
            '--ros-args',
            '--log-level', ['dsr_bridge_robot:=', log_level]]
    )

    return LaunchDescription([
        declare_params_file_arg,
        declare_log_level_arg,
        dsr_bridge_robot_node
    ])

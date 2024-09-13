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

"""Launches the DSR agents for the CAMPERO project."""

import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Getting directories and launch-files
    dsr_agent_dir = get_package_share_directory('dsr_agents')
    default_params_file = os.path.join(dsr_agent_dir, 'params', 'campero_params.yaml')

    # Input parameters declaration
    params_file = LaunchConfiguration('params_file')
    log_level = LaunchConfiguration('log-level')

    declare_params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file with dsr agent configuration'
    )

    declare_log_level_arg = DeclareLaunchArgument(
        name='log-level',
        default_value='info',
        description='Logging level (info, debug, ...)'
    )

    # Prepare the nodes
    # tf_agent_node = Node(
    #     package='dsr_agents',
    #     namespace='',
    #     executable='tf_agent',
    #     name='tf_agent',
    #     parameters=[params_file],
    #     emulate_tty=True,
    #     output='screen',
    #     arguments=[
    #         '--ros-args',
    #         '--log-level', ['tf_agent:=', log_level]]
    # )

    nav_agent_node = Node(
        package='dsr_agents',
        namespace='',
        executable='nav_agent',
        name='nav_agent',
        parameters=[params_file],
        emulate_tty=True,
        output='screen',
        arguments=[
            '--ros-args',
            '--log-level', ['nav_agent:=', log_level]]
    )

    docking_agent_node = Node(
        package='dsr_agents',
        namespace='',
        executable='docking_agent',
        name='docking_agent',
        parameters=[params_file],
        emulate_tty=True,
        output='screen',
        arguments=[
            '--ros-args',
            '--log-level', ['docking_agent:=', log_level]]
    )

    semantic_nav_agent_node = Node(
        package='dsr_agents',
        namespace='',
        executable='semantic_nav_agent',
        name='semantic_nav_agent',
        parameters=[params_file],
        emulate_tty=True,
        output='screen',
        arguments=[
            '--ros-args',
            '--log-level', ['semantic_nav_agent:=', log_level]]
    )

    battery_agent_node = Node(
        package='dsr_agents',
        namespace='',
        executable='topic_agent',
        name='battery_agent',
        parameters=[params_file],
        emulate_tty=True,
        output='screen',
        arguments=[
            '--ros-args',
            '--log-level', ['battery_agent:=', log_level]]
    )

    person_agent_node = Node(
        package='dsr_agents',
        namespace='',
        executable='person_agent',
        name='person_agent',
        parameters=[params_file],
        emulate_tty=True,
        output='screen',
        arguments=[
            '--ros-args',
            '--log-level', ['person_agent:=', log_level]]
    )

    # whisper_agent_node = Node(
    #     package='dsr_agents',
    #     namespace='',
    #     executable='whisper_agent',
    #     name='whisper_agent',
    #     parameters=[params_file],
    #     emulate_tty=True,
    #     output='screen',
    #     arguments=[
    #         '--ros-args',
    #         '--log-level', ['whisper_agent:=', log_level]]
    # )

    return LaunchDescription([
        declare_params_file_arg,
        declare_log_level_arg,
        # tf_agent_node,
        nav_agent_node,
        docking_agent_node,
        semantic_nav_agent_node,
        battery_agent_node,
        person_agent_node,
        # whisper_agent_node
    ])

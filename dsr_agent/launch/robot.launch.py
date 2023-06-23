#!/usr/bin/env python3

'''
    Launches a DSR Agent node.
'''
import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Getting directories and launch-files
    dsr_agent_dir = get_package_share_directory('dsr_agent')
    default_params_file = os.path.join(dsr_agent_dir, 'params', 'robot_params.yaml')

    # Input parameters declaration
    params_file = LaunchConfiguration('params_file')

    declare_params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file with dsr agent configuration'
    )

    # Prepare the laser segmentation node.
    battery_agent_node = Node(
        package = 'dsr_agent',
        namespace = '',
        executable = 'generic_agent',
        name = 'battery_agent',
        parameters=[params_file],
        emulate_tty = True
    )

    laser_agent_node = Node(
        package = 'dsr_agent',
        namespace = '',
        executable = 'generic_agent',
        name = 'laser_agent',
        parameters=[params_file],
        emulate_tty = True
    )

    rgb_agent_node = Node(
        package = 'dsr_agent',
        namespace = '',
        executable = 'generic_agent',
        name = 'rgb_agent',
        parameters=[params_file],
        emulate_tty = True
    )

    depth_agent_node = Node(
        package = 'dsr_agent',
        namespace = '',
        executable = 'generic_agent',
        name = 'rgbd_agent',
        parameters=[params_file],
        emulate_tty = True
    )

    tf_agent_node = Node(
        package = 'dsr_agent',
        namespace = '',
        executable = 'tf_agent',
        name = 'tf_agent',
        parameters=[params_file],
        emulate_tty = True
    )

    return LaunchDescription([
        declare_params_file_arg,
        battery_agent_node,
        laser_agent_node,
        rgb_agent_node,
        depth_agent_node,
        tf_agent_node
    ])
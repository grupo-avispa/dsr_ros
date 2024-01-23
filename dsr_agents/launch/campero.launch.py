#!/usr/bin/env python3

'''
    Launches the DSR agents for the CAMPERO project.
'''
import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # Getting directories and launch-files
    dsr_agent_dir = get_package_share_directory('dsr_agents')
    default_params_file = os.path.join(dsr_agent_dir, 'params', 'campero_params.yaml')

    # Input parameters declaration
    params_file = LaunchConfiguration('params_file')
    log_level = LaunchConfiguration('log-level')

    declare_params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value = default_params_file,
        description = 'Full path to the ROS2 parameters file with dsr agent configuration'
    )

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'dsr_input_file': os.path.join(dsr_agent_dir, 'worlds', 'empty.json')
    }

    configured_params = RewrittenYaml(
        source_file = params_file,
        root_key = '',
        param_rewrites = param_substitutions,
        convert_types = True
    )

    declare_log_level_arg = DeclareLaunchArgument(
        name = 'log-level',
        default_value = 'info',
        description = 'Logging level (info, debug, ...)'
    )

    # Prepare the nodes
    tf_agent_node = Node(
        package = 'dsr_agents',
        namespace = '',
        executable = 'tf_agent',
        name = 'tf_agent',
        parameters = [configured_params],
        emulate_tty = True,
        output = 'screen', 
        arguments = [
            '--ros-args', 
            '--log-level', ['tf_agent:=', LaunchConfiguration('log-level')]]
    )

    nav_agent_node = Node(
        package = 'dsr_agents',
        namespace = '',
        executable = 'nav_agent',
        name = 'nav_agent',
        parameters = [configured_params],
        emulate_tty = True,
        output = 'screen', 
        arguments = [
            '--ros-args', 
            '--log-level', ['nav_agent:=', LaunchConfiguration('log-level')]]
    )

    battery_agent_node = Node(
        package = 'dsr_agents',
        namespace = '',
        executable = 'topic_agent',
        name = 'battery_agent',
        parameters = [configured_params],
        emulate_tty = True,
        output = 'screen', 
        arguments = [
            '--ros-args', 
            '--log-level', ['battery_agent:=', LaunchConfiguration('log-level')]]
    )

    person_agent_node = Node(
        package = 'dsr_agents',
        namespace = '',
        executable = 'person_agent',
        name = 'person_agent',
        parameters = [configured_params],
        emulate_tty = True,
        output = 'screen', 
        arguments = [
            '--ros-args', 
            '--log-level', ['person_agent:=', LaunchConfiguration('log-level')]]
    )

    return LaunchDescription([
        declare_params_file_arg,
        declare_log_level_arg,
        tf_agent_node,
        nav_agent_node,
        battery_agent_node,
        person_agent_node
    ])
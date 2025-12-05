#!/usr/bin/env python

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare launch arguments
    name_skill_arg = DeclareLaunchArgument(
        'name_skill', 
        default_value='skill', 
        description='Name of the skill to execute'
    )
    localize_box_arg = DeclareLaunchArgument(
        'localize_box', 
        default_value='true', 
        description='Enable or disable box localization'
    )
    localize_template_arg = DeclareLaunchArgument(
        'name_template', 
        default_value='sponge_template', 
        description='Template name'
    )

    localize_template_arg = DeclareLaunchArgument(
        'name_storage', 
        default_value='sponge_template', 
        description='Storage name'
    )

    # Define the execute_node
    execute_node = Node(
        package='skills_manager',
        executable='play_skill',
        name='execute_node',
        output='screen',
        parameters=[{
            'name_skill': LaunchConfiguration('name_skill'),
            'localize_box': LaunchConfiguration('localize_box'),
            'name_template': LaunchConfiguration('name_template'),
            'name_storage': LaunchConfiguration('name_storage'),
        }],
        remappings=[],
    )

    # Return LaunchDescription
    return LaunchDescription([
        name_skill_arg,
        localize_box_arg,
        localize_template_arg,
        execute_node,
    ])

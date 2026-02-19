#!/usr/bin/env python3

"""
Launch file for turtlebot3_navigation.launch.py
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os 
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    slam = LaunchConfiguration('slam', default='false')
    turtlebot3_world_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch', 'turtlebot3_world.launch.py')
        )
    )

    turtlebot3_navigation_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('turtlebot3_navigation2'), 'launch', 'navigation2.launch.py')
        ),
        launch_arguments={
              'use_sim_time': use_sim_time, 
              'slam': slam}.items()
    )


    return LaunchDescription(
        [
            SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'),

            DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
            DeclareLaunchArgument(
            'slam',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
            turtlebot3_world_launch_file,
            turtlebot3_navigation_launch_file
        ]
    )

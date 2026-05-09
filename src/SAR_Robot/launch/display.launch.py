#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('SAR_Robot')
    
    # Path to URDF file
    urdf_file = os.path.join(pkg_dir, 'urdf', 'robot.urdf.xacro')
    
    # Process xacro to get robot description
    robot_desc = xacro.process_file(urdf_file).toxml()

    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        
        # Joint State Publisher (publishes zeros for all joints, no GUI)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),
        
        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),
    ])
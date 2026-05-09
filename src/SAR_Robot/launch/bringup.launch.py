#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg = get_package_share_directory('SAR_Robot')

    # 1. Process the xacro into a URDF string
    urdf_path = os.path.join(pkg, 'urdf', 'robot.urdf.xacro')
    robot_desc = xacro.process_file(urdf_path).toxml()

    # 2. Path to the world file
    world_path = os.path.join(pkg, 'worlds', 'assignment_world.sdf')

    # 3. Start Gazebo with the world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),
    )

    # 4. Robot State Publisher (publishes TF from URDF)
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
    )

    # 4b. Joint State Publisher - publishes zero joint states for the wheels.
    #     Required because gz-sim's DiffDrive plugin doesn't publish
    #     /joint_states by default, and robot_state_publisher needs them
    #     to compute transforms for the continuous wheel joints. Wheels
    #     won't visibly rotate in RViz but will render in their correct
    #     positions. Cosmetic only - real wheel motion happens inside
    #     Gazebo via the DiffDrive plugin.
    jsp = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # 5. Spawn the robot in Gazebo at (0, 0)
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'walle',
            '-topic', 'robot_description',
            '-x', '0', '-y', '0', '-z', '0.05',
        ],
        output='screen',
    )

    # 6. Bridge: connects Gazebo topics <-> ROS 2 topics
    #    [ = Gazebo to ROS    (sensor data flowing in)
    #    ] = ROS to Gazebo    (commands flowing out)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        ],
        output='screen',
    )

    # 7. Static TF bridges: Gazebo collapses fixed joints, so it reports
    #    sensor frame_ids as scoped names like 'walle/base_link/lidar'
    #    instead of the URDF link names. These zero-offset static
    #    transforms glue the scoped Gazebo frames onto the URDF frames so
    #    RViz (and anything else using TF) can resolve laser/camera data.
    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_frame_bridge',
        arguments=['--frame-id', 'lidar_link',
                   '--child-frame-id', 'walle/base_link/lidar'],
        output='screen',
    )
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_frame_bridge',
        arguments=['--frame-id', 'camera_link',
                   '--child-frame-id', 'walle/base_link/camera'],
        output='screen',
    )

    return LaunchDescription([gz_sim, rsp, jsp, spawn, bridge, lidar_tf, camera_tf])

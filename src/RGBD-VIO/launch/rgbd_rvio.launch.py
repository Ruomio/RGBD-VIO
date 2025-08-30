#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('rgbd_vio')

    # Define the path to the config file
    config_file = os.path.join(pkg_share, 'config', 'downsample.yaml')

    # Create the nodes
    feature_tracker_node = Node(
        package='rgbd_vio',
        executable='feature_tracker_rgbd_vio',
        name='feature_tracker',
        output='screen',
        parameters=[
            {'config_file': config_file},
            {'depth_interpolate': True},

        ],
        remappings=[
            ("/cam0/color", "/oakd/rgb/preview/image_raw"),
        ],
        # prefix=['gdb -ex run --args']
    )

    rvio_node = Node(
        package='rgbd_vio',
        executable='rgbd_vio_node',
        name='rvio_node',
        output='screen',
        parameters=[{'config_file': config_file}],
        remappings=[
            ("/imu0", "/imu"),
            ("/feature_tracker/feature", "feature"),
            ("cam0/depth", "/oakd/rgb/preview/depth")
        ],
        # prefix=['gdb -ex run --args']
    )
    run_dvio_node = Node(
        package='rgbd_vio',
        executable='run_dvio_node',
        name='run_dvio_node',
        output='screen',
        parameters=[{'config_file': config_file}],
        remappings=[
            ("/imu0", "/imu"),
            ("/feature_tracker/feature", "feature"),
            ("cam0/depth", "/oakd/rgb/preview/depth")
        ],
        # prefix=['gdb -ex run --args']
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(pkg_share, 'launch_rvio', 'rvio_demo.rviz')],
        # prefix=['nice']
    )

    stf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '0.0', '--y', '0.0', '--z', '0.0',
                   '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                   '--frame_id', 'imu_link',
                   '--child_frame-id', 'world'
        ],
        output='screen'
    )

    return LaunchDescription([
        feature_tracker_node,
        run_dvio_node,
        stf_node,
        # rvio_node,
        # rviz_node
    ])

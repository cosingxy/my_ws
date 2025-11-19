#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_gui = LaunchConfiguration('use_gui')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_gui', default_value='false',
            description='Use joint_state_publisher_gui if true.'
        ),
        OpaqueFunction(function=_launch_setup, args=[use_gui]),
    ])


def _launch_setup(context, use_gui):
    pkg_share = get_package_share_directory('rml_63_ee_description')
    urdf_path = os.path.join(pkg_share, 'urdf', 'rml_63_ee.urdf')

    with open(urdf_path, 'r') as f:
        robot_description_content = f.read()

    params = {'robot_description': robot_description_content}

    nodes = []

    if use_gui.perform(context).lower() in ['true', '1', 'yes']:
        nodes.append(Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            parameters=[params]
        ))
    else:
        nodes.append(Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[params]
        ))

    nodes.append(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[params]
    ))

    nodes.append(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2'
    ))

    return nodes

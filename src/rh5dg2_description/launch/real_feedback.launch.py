"""Render measured joint states. No sliders, synthetic feedback, or driver startup."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def _display(context):
    side = LaunchConfiguration('side').perform(context)
    if side not in ('left', 'right'):
        raise RuntimeError('side must be left or right')
    share = get_package_share_directory('rh5dg2_description')
    description = xacro.process_file(os.path.join(share, 'urdf', f'rh5dg2_{side}.xacro')).toxml()
    namespace = f'/inspire/{side}'
    return [
        LogInfo(msg=f'Waiting for {namespace}/joint_states. ' +
                ('Left reference is mirrored from the right model. ' if side == 'left' else '') +
                'Enable driver joint states only after verifying joint mapping and calibration.'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', namespace=namespace, output='screen',
             parameters=[{'robot_description': description}],
             remappings=[('joint_states', f'{namespace}/joint_states')]),
        Node(package='rviz2', executable='rviz2', name='rviz2', namespace=namespace,
             arguments=['-d', os.path.join(share, 'rviz', f'{side}.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('side', default_value='right', choices=['left', 'right']),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=_display),
    ])

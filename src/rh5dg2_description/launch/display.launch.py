"""Preview right vendor or left mirrored reference model, without a driver."""

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
    namespace = f'/rh5dg2_preview/{side}'
    tf_remaps = [('/tf', f'{namespace}/tf'), ('/tf_static', f'{namespace}/tf_static')]
    return [
        LogInfo(msg=('Left reference derived by mirroring the vendor right model. ' if side == 'left' else '') +
                'Preview only: joint mapping is not calibrated to hardware.'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', namespace=namespace, output='screen',
             parameters=[{'robot_description': description}], remappings=tf_remaps),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             name='joint_state_publisher_gui', namespace=namespace,
             parameters=[{'robot_description': description}],
             condition=IfCondition(LaunchConfiguration('gui'))),
        Node(package='rviz2', executable='rviz2', name='rviz2', namespace=namespace,
             arguments=['-d', os.path.join(share, 'rviz', f'{side}.rviz')],
             remappings=tf_remaps, condition=IfCondition(LaunchConfiguration('rviz'))),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('side', default_value='right', choices=['left', 'right']),
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=_display),
    ])

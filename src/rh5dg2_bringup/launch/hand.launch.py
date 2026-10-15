"""Start one hand. Explicit launch arguments override the side's YAML."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_hand(context):
    side = LaunchConfiguration('side').perform(context)
    if side not in ('left', 'right'):
        raise RuntimeError('side must be left or right')
    params_file = os.path.join(
        get_package_share_directory('rh5dg2_bringup'), 'config', f'{side}.yaml')
    overrides = {'side': side}
    for name in ('port', 'baudrate', 'hand_id', 'enable_write',
                 'publish_joint_states', 'joint_mapping_confirmed'):
        value = LaunchConfiguration(name).perform(context)
        if not value:
            continue
        if name in ('baudrate', 'hand_id'):
            value = int(value)
        elif name != 'port':
            if value not in ('true', 'false'):
                raise RuntimeError(f'{name} must be true or false')
            value = value == 'true'
        overrides[name] = value
    return [Node(
        package='rh5dg2_driver', executable='hand_node', name='driver',
        namespace=f'/inspire/{side}', output='screen',
        parameters=[params_file, overrides],
    )]


def generate_launch_description():
    arguments = [DeclareLaunchArgument('side', default_value='left', choices=['left', 'right'])]
    for name in ('port', 'baudrate', 'hand_id'):
        arguments.append(DeclareLaunchArgument(
            name, default_value='', description='Empty uses the selected side YAML'))
    for name in ('enable_write', 'publish_joint_states', 'joint_mapping_confirmed'):
        arguments.append(DeclareLaunchArgument(
            name, default_value='', choices=['', 'true', 'false'],
            description='Empty uses the selected side YAML'))
    return LaunchDescription(arguments + [OpaqueFunction(function=_launch_hand)])

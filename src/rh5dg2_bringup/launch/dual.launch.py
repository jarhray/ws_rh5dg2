"""Start two hands on distinct serial adapters."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _launch_pair(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    share = get_package_share_directory('rh5dg2_bringup')
    ports = {}
    for side in ('left', 'right'):
        with open(os.path.join(share, 'config', f'{side}.yaml'), encoding='utf-8') as stream:
            config = yaml.safe_load(stream)[f'/inspire/{side}/driver']['ros__parameters']
        ports[side] = value(f'{side}_port') or config['port']
    if not all(ports.values()):
        raise RuntimeError('Both serial ports must be specified')
    if os.path.realpath(ports['left']) == os.path.realpath(ports['right']):
        raise RuntimeError('Dual-hand mode requires distinct ports; '
                           'two drivers must not share a serial adapter')
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'hand.launch.py')),
        launch_arguments={
            'side': side, 'port': ports[side],
            'baudrate': value('baudrate'), 'hand_id': value(f'{side}_hand_id'),
            'enable_write': value('enable_write'),
            'publish_joint_states': value('publish_joint_states'),
            'joint_mapping_confirmed': value('joint_mapping_confirmed'),
        }.items(),
    ) for side in ('left', 'right')]


def generate_launch_description():
    arguments = [DeclareLaunchArgument('baudrate', default_value='',
                                       description='Empty uses each side YAML')]
    for name in ('enable_write', 'publish_joint_states', 'joint_mapping_confirmed'):
        arguments.append(DeclareLaunchArgument(
            name, default_value='', choices=['', 'true', 'false'],
            description='Empty uses each side YAML'))
    for side in ('left', 'right'):
        for name in ('port', 'hand_id'):
            arguments.append(DeclareLaunchArgument(
                f'{side}_{name}', default_value='',
                description=f'Empty uses {side}.yaml'))
    return LaunchDescription(arguments + [OpaqueFunction(function=_launch_pair)])

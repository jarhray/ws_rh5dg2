"""Check launch configuration without opening devices or starting drivers."""

import importlib.util
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.logging import launch_config


ROOT = Path(__file__).resolve().parents[1]


def load(filename):
    spec = importlib.util.spec_from_file_location(filename, ROOT / 'launch' / filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def context_for(module, **overrides):
    context = LaunchContext()
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    context.launch_configurations.update(overrides)
    return context


class LaunchGuards(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.log_directory = tempfile.TemporaryDirectory(prefix='rh5dg2-launch-test-')
        cls.addClassCleanup(cls.log_directory.cleanup)
        launch_config.log_dir = cls.log_directory.name

    def single(self, **overrides):
        module = load('hand.launch.py')
        with patch.object(module, 'get_package_share_directory', return_value=str(ROOT)), \
                patch.object(module, 'Node') as node:
            module._launch_hand(context_for(module, **overrides))
        return node.call_args.kwargs

    def test_defaults_inherit_each_side_configuration(self):
        for side in ('left', 'right'):
            with self.subTest(side=side):
                arguments = self.single(side=side)
                filename, overrides = arguments['parameters']
                self.assertEqual(arguments['namespace'], f'/inspire/{side}')
                self.assertEqual(Path(filename).name, f'{side}.yaml')
                # No default launch argument may overwrite a user's YAML value.
                self.assertEqual(overrides, {'side': side})
                config = yaml.safe_load(Path(filename).read_text())
                parameters = config[f'/inspire/{side}/driver']['ros__parameters']
                self.assertEqual(parameters['port'], f'/dev/rh5dg2_{side}')
                self.assertEqual(parameters['baudrate'], 115200)
                self.assertEqual(parameters['hand_id'], 1)
                self.assertTrue(parameters['enable_write'])
                self.assertFalse(parameters['publish_joint_states'])
                self.assertFalse(parameters['joint_mapping_confirmed'])
                self.assertEqual(parameters['angle_write_ack'], 32 if side == 'left' else 1)
                self.assertEqual(parameters['stop_write_ack'], 2 if side == 'left' else 1)
                self.assertEqual(parameters['single_angle_write_ack'], 2 if side == 'left' else 1)

    def test_explicit_overrides_have_ros_parameter_types(self):
        arguments = self.single(
            side='right', port='/dev/custom', baudrate='57600', hand_id='2',
            enable_write='true', publish_joint_states='true', joint_mapping_confirmed='false')
        self.assertEqual(arguments['parameters'][1], {
            'side': 'right', 'port': '/dev/custom', 'baudrate': 57600, 'hand_id': 2,
            'enable_write': True, 'publish_joint_states': True, 'joint_mapping_confirmed': False,
        })

    def test_launch_surface_has_only_supported_options(self):
        module = load('hand.launch.py')
        self.assertEqual(set(context_for(module).launch_configurations), {
            'side', 'port', 'baudrate', 'hand_id', 'enable_write',
            'publish_joint_states', 'joint_mapping_confirmed',
        })

    def test_dual_hand_rejects_aliases_of_one_physical_port(self):
        module = load('dual.launch.py')
        with tempfile.TemporaryDirectory() as temporary:
            device = Path(temporary) / 'adapter'
            device.touch()
            alias = Path(temporary) / 'other-name'
            alias.symlink_to(device)
            with patch.object(module, 'get_package_share_directory', return_value=str(ROOT)), \
                    self.assertRaisesRegex(RuntimeError, 'distinct ports'):
                module._launch_pair(context_for(
                    module, left_port=str(device), right_port=str(alias)))

    def test_dual_hand_checks_ports_in_yaml(self):
        module = load('dual.launch.py')
        with tempfile.TemporaryDirectory() as temporary:
            config_dir = Path(temporary) / 'config'
            config_dir.mkdir()
            for side in ('left', 'right'):
                (config_dir / f'{side}.yaml').write_text(yaml.safe_dump({
                    f'/inspire/{side}/driver': {'ros__parameters': {'port': '/dev/shared'}}}))
            with patch.object(module, 'get_package_share_directory', return_value=temporary), \
                    self.assertRaisesRegex(RuntimeError, 'distinct ports'):
                module._launch_pair(context_for(module))

    def test_dual_hand_forwards_explicit_values_and_inherits_other_yaml_values(self):
        module = load('dual.launch.py')
        with patch.object(module, 'get_package_share_directory', return_value=str(ROOT)):
            actions = module._launch_pair(context_for(
                module, left_port='/dev/left-adapter', right_hand_id='3', enable_write='true'))
        self.assertEqual(len(actions), 2)
        for side, action in zip(('left', 'right'), actions):
            self.assertEqual(dict(action.launch_arguments), {
                'side': side,
                'port': '/dev/left-adapter' if side == 'left' else '/dev/rh5dg2_right',
                'baudrate': '', 'hand_id': '' if side == 'left' else '3',
                'enable_write': 'true', 'publish_joint_states': '', 'joint_mapping_confirmed': '',
            })


if __name__ == '__main__':
    unittest.main()

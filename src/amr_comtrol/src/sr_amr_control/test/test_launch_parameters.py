import importlib.util
import sys
import types
import unittest
from pathlib import Path


class LaunchDescription:
  def __init__(self, entities):
    self.entities = entities


class DeclareLaunchArgument:
  def __init__(self, name, default_value=None):
    self.name = name
    self.default_value = default_value


class LaunchConfiguration:
  def __init__(self, variable_name):
    self.variable_name = variable_name


class Node:
  def __init__(self, **kwargs):
    self.node_kwargs = kwargs


class ParameterValue:
  def __init__(self, value, value_type=None):
    self.value = value
    self.value_type = value_type


def _load_launch_module():
  launch = types.ModuleType('launch')
  launch.LaunchDescription = LaunchDescription
  launch_actions = types.ModuleType('launch.actions')
  launch_actions.DeclareLaunchArgument = DeclareLaunchArgument
  launch_substitutions = types.ModuleType('launch.substitutions')
  launch_substitutions.LaunchConfiguration = LaunchConfiguration
  launch_ros = types.ModuleType('launch_ros')
  launch_ros_actions = types.ModuleType('launch_ros.actions')
  launch_ros_actions.Node = Node
  launch_ros_parameter_descriptions = types.ModuleType(
    'launch_ros.parameter_descriptions'
  )
  launch_ros_parameter_descriptions.ParameterValue = ParameterValue

  launch_file = Path(__file__).resolve().parents[1] / 'launch' / 'amr_control.launch.py'
  spec = importlib.util.spec_from_file_location('amr_control_launch', launch_file)
  launch_module = importlib.util.module_from_spec(spec)

  original_modules = sys.modules.copy()
  try:
    sys.modules.update(
      {
        'launch': launch,
        'launch.actions': launch_actions,
        'launch.substitutions': launch_substitutions,
        'launch_ros': launch_ros,
        'launch_ros.actions': launch_ros_actions,
        'launch_ros.parameter_descriptions': launch_ros_parameter_descriptions,
      }
    )
    spec.loader.exec_module(launch_module)
  finally:
    sys.modules.clear()
    sys.modules.update(original_modules)

  return launch_module


class TestLaunchParameters(unittest.TestCase):
  def test_launch_parameters_are_declared_and_passed_to_control_node(self):
    launch_module = _load_launch_module()

    launch_description = launch_module.generate_launch_description()
    entities = list(launch_description.entities)

    declared_arg_names = {
      entity.name for entity in entities if isinstance(entity, DeclareLaunchArgument)
    }
    self.assertIn('parent_frame_id', declared_arg_names)
    self.assertIn('lidar_points_frame_id', declared_arg_names)
    self.assertIn('lidar', declared_arg_names)

    lidar_arg = next(
      entity
      for entity in entities
      if isinstance(entity, DeclareLaunchArgument) and entity.name == 'lidar'
    )
    self.assertEqual(lidar_arg.default_value, 'false')

    control_node = next(entity for entity in entities if isinstance(entity, Node))
    node_parameters = control_node.node_kwargs['parameters'][0]

    self.assertIn('parent_frame_id', node_parameters)
    self.assertEqual(
      node_parameters['parent_frame_id'].variable_name, 'parent_frame_id'
    )
    self.assertIn('lidar_points_frame_id', node_parameters)
    self.assertEqual(
      node_parameters['lidar_points_frame_id'].variable_name,
      'lidar_points_frame_id',
    )
    self.assertIn('lidar', node_parameters)
    self.assertIs(node_parameters['lidar'].value_type, bool)
    self.assertEqual(node_parameters['lidar'].value.variable_name, 'lidar')


if __name__ == '__main__':
  unittest.main()

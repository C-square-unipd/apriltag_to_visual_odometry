import os

import ament_index_python.packages

from launch import LaunchDescription
import launch_ros.actions

import yaml


def generate_launch_description():
	share_dir = ament_index_python.packages.get_package_share_directory('apriltag_to_visual_odometry')
	# Passing parameters to a composed node must be done via a dictionary of
	# key -> value pairs.  Here we read in the data from the configuration file
	# and create a dictionary of it that the ComposableNode will accept.
	params_file = os.path.join(share_dir, 'config', 'apriltag_config_3.yaml')
	with open(params_file, 'r') as f:
		params = yaml.safe_load(f)['apriltag_to_visual_odometry']['ros__parameters']
	return LaunchDescription([
		launch_ros.actions.Node(
			package='apriltag_to_visual_odometry',
			executable='apriltag_to_visual_odometry',
			output='screen',
			remappings=[('/fmu/vehicle_visual_odometry_in','/fmu/vehicle_visual_odometry/in_3')],
			parameters=[params]
		),
	])

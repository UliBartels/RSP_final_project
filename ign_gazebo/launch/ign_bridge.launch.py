import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext, LaunchService
from launch.actions import DeclareLaunchArgument, GroupAction, ExecuteProcess, SetLaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import TextSubstitution

import xacro

# convert ns from launchconfiguration type to string type

def generate_launch_description():

	def render_ns(context: LaunchContext, ns):
		ns_str = context.perform_substitution(ns)

		return [SetLaunchConfiguration('ns', ns_str)]

	ns_arg = DeclareLaunchArgument('ns', default_value='new')

	ns_str_op = OpaqueFunction(function=render_ns, args=[LaunchConfiguration('ns')])

	SetEnvironmentVariable(name='TURTLEBOT3_MODEL', value='waffle')

# joint states bridge

	def create_joint_state_bridge(context):
		print('Creating joint state bridge node')
		joint_state_bridge = Node(
			package='ros_ign_bridge',
			executable='parameter_bridge',
			namespace=context.launch_configurations['ns'],
			output='screen',
			arguments=['/world/empty/model/' + context.launch_configurations['ns'] + '/joint_state@sensor_msgs/msg/JointState@ignition.msgs.Model']
		)

		return [joint_state_bridge]

	create_joint_state_bridge_def = OpaqueFunction(function=create_joint_state_bridge)

# laser scan bridge

	def create_laser_scan_bridge(context):
		print('Creating laser scan bridge node')
		laser_scan_bridge = Node(
			package='ros_ign_bridge',
			executable='parameter_bridge',
			#namespace=context.launch_configurations['ns'],
			output='screen',
			arguments=[context.launch_configurations['ns'] + '_lidar@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan'],
		)

		return [laser_scan_bridge]

	create_laser_scan_bridge_def = OpaqueFunction(function=create_laser_scan_bridge)

# cmd vel bridge

	def create_cmd_vel_bridge(context):
		print('Creating cmd vel bridge node')
		cmd_vel_bridge = Node(
			package='ros_ign_bridge',
			executable='parameter_bridge',
			#namespace=context.launch_configurations['ns'],
			output='screen',
			arguments=[context.launch_configurations['ns'] + '/cmd_vel@geometry_msgs/msg/Twist[ignition.msgs.Twist'],
		)

		return [cmd_vel_bridge]

	create_cmd_vel_bridge_def = OpaqueFunction(function=create_cmd_vel_bridge)

# odom bridge

	def create_odom_bridge(context):
		print('Creating odom bridge node')
		odom_bridge = Node(
			package='ros_ign_bridge',
			executable='parameter_bridge',
			#namespace=context.launch_configurations['ns'],
			output='screen',
			arguments=['odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry'],
		)

		return [odom_bridge]

	create_odom_bridge_def = OpaqueFunction(function=create_odom_bridge)

# tf bridge

	def create_tf_bridge(context):
		print('Creating tf bridge node')
		tf_bridge = Node(
			package='ros_ign_bridge',
			executable='parameter_bridge',
			#namespace=context.launch_configurations['ns'],
			output='screen',
			arguments=['tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V'],
		)

		return [tf_bridge]

	create_tf_bridge_def = OpaqueFunction(function=create_tf_bridge)


	return LaunchDescription([
		ns_arg,
		ns_str_op,
		create_joint_state_bridge_def,
		create_cmd_vel_bridge_def,
		create_laser_scan_bridge_def,
		create_odom_bridge_def,
		create_tf_bridge_def
		])
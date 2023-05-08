import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, ExecuteProcess, SetLaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable

import xacro

# This file creates the robot_description topics for both waffle and burger

def generate_launch_description():

# get the right urdfs from .urdf.xacro for both the robots

	def render_str_waffle(context):
		pkg_path = os.path.join(get_package_share_directory('final_project'))
		waffle_xacro_file = os.path.join(pkg_path,'urdf','waffle.urdf.xacro')
		waffle_description_config = xacro.process_file(
			waffle_xacro_file
		)
		waffle_desc = waffle_description_config.toprettyxml(indent=' ')

		waffle_file = open(pkg_path+'/urdf/waffle.urdf','w')
		waffle_file.write(waffle_desc)
		waffle_file.close()

		return [SetLaunchConfiguration('waffle_desc', waffle_desc)]

	create_waffle_description_arg = OpaqueFunction(function=render_str_waffle)


	def render_str_burger(context):
		pkg_path = os.path.join(get_package_share_directory('final_project'))
		burger_xacro_file = os.path.join(pkg_path,'urdf','burger.urdf.xacro')
		burger_description_config = xacro.process_file(
			burger_xacro_file
		)
		burger_desc = burger_description_config.toprettyxml(indent=' ')

		burger_file = open(pkg_path+'/urdf/burger.urdf','w')
		burger_file.write(burger_desc)
		burger_file.close()

		return [SetLaunchConfiguration('burger_desc', burger_desc)]

	create_burger_description_arg = OpaqueFunction(function=render_str_burger)


# access all packages and urdfs required to spawn stuff to ignition

	main_pkg = get_package_share_directory('final_project')

	waffle_urdf = os.path.join(main_pkg,'urdf','waffle.urdf')

	burger_urdf = os.path.join(main_pkg,'urdf','burger.urdf')

# declare launch arguments and set their default values

	entity1_name = LaunchConfiguration('entity1_name')
	entity2_name = LaunchConfiguration('entity2_name')

	ns1 = LaunchConfiguration('ns1')
	ns2 = LaunchConfiguration('ns2')

	SetEnvironmentVariable(name='TURTLEBOT3_MODEL', value='waffle')

	entity1_name_arg = DeclareLaunchArgument(
		'entity1_name',
		default_value='waffle'
	)

	entity2_name_arg = DeclareLaunchArgument(
		'entity2_name',
		default_value='burger'
	)

	ns1_arg = DeclareLaunchArgument(
		'ns1',
		default_value='waffle'
	)

	ns2_arg = DeclareLaunchArgument(
		'ns2',
		default_value='burger'
	)

# run robot_state_publishers 

	robot_state_pub_node_waffle = Node(

		package='robot_state_publisher',
		executable='robot_state_publisher',
		namespace=ns1,
		output='screen',
		parameters=[{'robot_description' : LaunchConfiguration('waffle_desc')}]
	)

	robot_state_pub_node_burger = Node(

		package='robot_state_publisher',
		executable='robot_state_publisher',
		namespace=ns2,
		output='screen',
		parameters=[{'robot_description' : LaunchConfiguration('burger_desc')}]
	)

	return LaunchDescription([
		create_burger_description_arg,
		create_waffle_description_arg,
		entity1_name_arg,
		entity2_name_arg,
		ns1_arg,
		ns2_arg,
		robot_state_pub_node_waffle,
		robot_state_pub_node_burger
		])

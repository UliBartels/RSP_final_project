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

# This file launches ignition gazebo and spawn the robots - note that spawing is done using 
# urdf files created using robot_state_pub.launch.py. It is necessary to run that launch file 
# if this has to work

def generate_launch_description():

# access all packages and urdfs required to spawn stuff to ignition

	main_pkg = get_package_share_directory('final_project')
	world_file = os.path.join(main_pkg,'world','template.sdf')

	waffle_urdf = os.path.join(main_pkg,'urdf','waffle.urdf')

	burger_urdf = os.path.join(main_pkg,'urdf','burger.urdf')

# declare launch arguments and set their default values

	entity1_name = LaunchConfiguration('entity1_name')
	entity2_name = LaunchConfiguration('entity2_name')

	ns1 = LaunchConfiguration('ns1')
	ns2 = LaunchConfiguration('ns2')

	ign_args = LaunchConfiguration('ign_args')

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

	ign_launch_arg = DeclareLaunchArgument(
		'ign_args',
		default_value='--render-engine ogre '+world_file + ' -v 4'
	)

# and bam! launch and spawn everything ignition 

	gazebo_launch = IncludeLaunchDescription(
		PythonLaunchDescriptionSource([os.path.join(
			get_package_share_directory('ros_ign_gazebo'),'launch','ign_gazebo.launch.py'
			)]), launch_arguments={'ign_args': ign_args}.items()	
	)

	spawn_waffle = Node(
		package='ros_ign_gazebo',
		namespace=ns1,
		executable='create',
		arguments=['-file', waffle_urdf, '-name', entity1_name, '-z', "-0.11"],
		output='screen'
	)


	spawn_burger = Node(
		package='ros_ign_gazebo',
		namespace=ns2,
		executable='create',
		arguments=['-file', burger_urdf, '-name', entity2_name, '-z', "-0.11", '-x', "1.0"],
		output='screen'
	)

# call the ros_ign_bridge 

	ign_waffle_bridge = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(get_package_share_directory('ign_gazebo'),'launch','ign_bridge.launch.py')
		),
		launch_arguments={
			'ns':ns1}.items()
	)

	ign_burger_bridge = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(get_package_share_directory('ign_gazebo'),'launch','ign_bridge.launch.py')
		),
		launch_arguments={
			'ns':ns2}.items()
	)

	return LaunchDescription([
		entity1_name_arg,
		entity2_name_arg,
		ns1_arg,
		ns2_arg,
		ign_launch_arg,
		gazebo_launch,
		spawn_waffle,
		spawn_burger,
		ign_waffle_bridge,
		ign_burger_bridge
		])









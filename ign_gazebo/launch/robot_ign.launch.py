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

	def world_str(context):
		pkg_path = os.path.join(get_package_share_directory('ign_gazebo'))
		world_xacro_file = os.path.join(pkg_path,'urdf','bot_world.urdf.xacro')
		world_description_config = xacro.process_file(
			world_xacro_file
		)
		world_desc = world_description_config.toprettyxml(indent=' ')

		file = open(pkg_path+'/urdf/bot_world.urdf','w')
		file.write(world_desc)
		file.close()

		return [SetLaunchConfiguration('world_desc', world_desc)]

	create_world_description_arg = OpaqueFunction(function=world_str)


# access all packages and urdfs required to spawn stuff to ignition

	main_pkg = get_package_share_directory('final_project')
	#world_file = os.path.join(main_pkg,'world','template.sdf')

	waffle_urdf = os.path.join(main_pkg,'urdf','waffle.urdf')

	burger_urdf = os.path.join(main_pkg,'urdf','burger.urdf')


	share_pkg_path_world = os.path.join(get_package_share_directory('ign_gazebo'))
	worlds_file = os.path.join(share_pkg_path_world,'worlds','sample_world.sdf')
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
		default_value='--render-engine ogre empty.sdf -v 4'
	)

# and bam! launch and spawn everything ignition 

	gazebo_launch = IncludeLaunchDescription(
		PythonLaunchDescriptionSource([os.path.join(
			get_package_share_directory('ros_ign_gazebo'),'launch','ign_gazebo.launch.py'
			)]), launch_arguments={'ign_args': ign_args}.items()	
	)

# spawn the world first
	urdf_file_world = os.path.join(share_pkg_path_world,'urdf','bot_world.urdf')

	spawn_world = Node(

		package='ros_ign_gazebo',
		executable='create',
		arguments=[ '-file', urdf_file_world, '-name', "tags"],
		output='screen'
		)


	spawn_waffle = Node(
		package='ros_ign_gazebo',
		namespace=ns1,
		executable='create',
		arguments=['-file', waffle_urdf, '-name', entity1_name, '-y', "1", '-z', "-0.11"],
		output='screen'
	)


	spawn_burger = Node(
		package='ros_ign_gazebo',
		namespace=ns2,
		executable='create',
		arguments=['-file', burger_urdf, '-name', entity2_name, '-x', "1.0"],
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
		create_world_description_arg,
		entity1_name_arg,
		entity2_name_arg,
		ns1_arg,
		ns2_arg,
		ign_launch_arg,
		gazebo_launch,
		spawn_world,
		spawn_waffle,
		spawn_burger,
		ign_waffle_bridge,
		ign_burger_bridge
		])









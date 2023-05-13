import os

from ament_index_python.packages import get_package_share_directory

from launch.actions import GroupAction, ExecuteProcess, SetLaunchConfiguration, IncludeLaunchDescription
from launch_ros.actions import PushRosNamespace
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import xacro

def generate_launch_description():

	# def world_str(context):
	# 	pkg_path = os.path.join(get_package_share_directory('ign_gazebo'))
	# 	world_xacro_file = os.path.join(pkg_path,'urdf','bot_world.urdf.xacro')
	# 	world_description_config = xacro.process_file(
	# 		world_xacro_file
	# 	)
	# 	world_desc = world_description_config.toprettyxml(indent=' ')

	# 	file = open(pkg_path+'/urdf/bot_world.urdf','w')
	# 	file.write(world_desc)
	# 	file.close()

	# 	return [SetLaunchConfiguration('world_desc', world_desc)]

	# create_world_description_arg = OpaqueFunction(function=world_str)

	# share_pkg_path_world = os.path.join(get_package_share_directory('ign_gazebo'))
	# worlds_file = os.path.join(share_pkg_path_world,'worlds','sample_world.sdf')
	# ign_args = LaunchConfiguration('ign_args')

	# # node_publisher_world = Node(
	# # 	package='robot_state_publisher',
	# # 	executable='robot_state_publisher',
	# # 	output='screen',
	# # 	parameters=[{'robot_description' : LaunchConfiguration('world_desc')}]
	# # )

	# share_pkg_path_burger = os.path.join(get_package_share_directory('final_project'))
	# burger_urdf = os.path.join(share_pkg_path_burger,'urdf','waffle.urdf')

	# ign_launch_arg = DeclareLaunchArgument(
	# 	'ign_args',
	# 	default_value='--render-engine ogre '+worlds_file + ' -v 4'
	# ) 

	# gazebo_launch = IncludeLaunchDescription(
	# 	PythonLaunchDescriptionSource([os.path.join(
	# 		get_package_share_directory('ros_ign_gazebo'),'launch','ign_gazebo.launch.py'
	# 		)]), launch_arguments={'ign_args': ign_args}.items()	
	# )

	# spawn_burger = Node(
	# 	package='ros_ign_gazebo',
	# 	namespace='waffle',
	# 	executable='create',
	# 	arguments=['-file', burger_urdf, '-name', 'waffle', '-x', "1.0"],
	# 	output='screen'
	# )


	# urdf_file_world = os.path.join(share_pkg_path_world,'urdf','bot_world.urdf')

	# spawn_world = Node(

	# 	package='ros_ign_gazebo',
	# 	executable='create',
	# 	arguments=[ '-file', urdf_file_world, '-name', "tags"],
	# 	output='screen'
	# 	)


	def render_str_waffle(context):
		pkg_path = os.path.join(get_package_share_directory('final_project'))
		waffle_xacro_file = os.path.join(pkg_path,'urdf','waffle.urdf.xacro')
		waffle_description_config = xacro.process_file(
			waffle_xacro_file,
			mappings={
				'entity_name': 'waffle', 
			}
		)
		waffle_desc = waffle_description_config.toprettyxml(indent=' ')

		waffle_file = open(pkg_path+'/urdf/waffle.urdf','w')
		waffle_file.write(waffle_desc)
		waffle_file.close()

		return [SetLaunchConfiguration('waffle_desc', waffle_desc)]

	create_waffle_description_arg = OpaqueFunction(function=render_str_waffle)

	rviz_launch = Node(
		package='rviz2',
		executable='rviz2',
		name='rviz2',
		output='screen'
		)

	robot_state_pub_node_waffle = Node(

		package='robot_state_publisher',
		executable='robot_state_publisher',
		namespace='waffle',
		output='screen',
		parameters=[{'robot_description' : LaunchConfiguration('waffle_desc')}]
	)
	node_joint_state_publisher_gui = Node(

		package='joint_state_publisher_gui',
		executable='joint_state_publisher_gui',
		namespace='waffle',
		output='screen'
		)

	return LaunchDescription([

		#create_world_description_arg,
		#node_publisher_world,
		# ign_launch_arg,
		# gazebo_launch,
		# spawn_burger
		# #spawn_world
		create_waffle_description_arg,
		rviz_launch,
		node_joint_state_publisher_gui,
			TimerAction(period = 5.0,
		actions=[robot_state_pub_node_waffle])

		])


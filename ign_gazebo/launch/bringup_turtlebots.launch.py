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
from launch.substitutions import EnvironmentVariable

import xacro


# This file launches the simulation in ignition, launches rviz, creates robot_state_publisher description for 
# both the robots and passes the right namespace and entity names to other launch files


def generate_launch_description():

# access all packages and urdfs required to spawn stuff to ignition

	main_pkg = get_package_share_directory('final_project')

	world_file = os.path.join(main_pkg,'world','template.sdf')

	waffle_urdf = os.path.join(main_pkg,'urdf','waffle.urdf')

	burger_urdf = os.path.join(main_pkg,'urdf','burger.urdf')

# declare launch arguments and set their default values

	entity1_name = LaunchConfiguration('entity1_name')
	entity2_name = LaunchConfiguration('entity2_name')

	waffle_desc = LaunchConfiguration('waffle_desc')
	burger_desc = LaunchConfiguration('burger_desc')

	ns1 = LaunchConfiguration('ns1')
	ns2 = LaunchConfiguration('ns2')

	print('printing env variable: ')
	#SetEnvironmentVariable(name='TURTLEBOT3_MODEL', value='waffle'),
	#SetEnvironmentVariable(name='TURTLEBOT3_MODEL', value='waffle')
	#model = ExecuteProcess(cmd=['echo', EnvironmentVariable('TURTLEBOT3_MODEL')], output='screen')

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

# calls the launch file to create and publish robot descriptions

	robot_state_publisher_call = IncludeLaunchDescription(
		PythonLaunchDescriptionSource([os.path.join(
			get_package_share_directory('ign_gazebo'),'launch','robot_state_pub.launch.py'
			)]), launch_arguments={'entity1_name' : entity1_name, 'entity2_name' : entity2_name,
		'ns1' : ns1, 'ns2' : ns2}.items()
	)

# calls the launch file to simulate everything in gazebo-ignition environment

	ignition_launch = IncludeLaunchDescription(
		PythonLaunchDescriptionSource([os.path.join(
			get_package_share_directory('ign_gazebo'),'launch','robot_ign.launch.py'
			)]), launch_arguments={'entity1_name' : entity1_name, 'entity2_name' : entity2_name,
		'ns1' : ns1, 'ns2' : ns2}.items()

	)

	nav_launch = IncludeLaunchDescription(
		PythonLaunchDescriptionSource([os.path.join(
			main_pkg,'launch','navigation','bringup_launch.py'
			)])
	)

# visualize in rviz

	rviz_launch = Node(
		package='rviz2',
		executable='rviz2',
		name='rviz2',
		arguments=['-d'+os.path.join(get_package_share_directory('final_project'),
			'rviz','slam.rviz')]
		)

	teleop_burger = ExecuteProcess(
		cmd=[
		    'gnome-terminal -e', '--', 'bash -c "export TURTLEBOT3_MODEL=burger"']
	)



	# nav_launch =  IncludeLaunchDescription(
	# 	PythonLaunchDescriptionSource([os.path.join(
	# 		get_package_share_directory('final_project'),'launch','navigation','navigation2.launch.py'
	# 		)])
	# ) 


	# slam_launch = Node(
	# 	package='slam_toolbox',
	# 	executable='async_slam_toolbox_node',
	# 	remappings=[
	# 		("scan", "waffle/waffle_lidar")
	# 	]
	# )


	return LaunchDescription([
		ns1_arg,
		ns2_arg,
		entity1_name_arg,
		entity2_name_arg,
		nav_launch,
		rviz_launch,
		robot_state_publisher_call,
		ignition_launch,

		# teleop_burger
		#nav_launch,
		#slam_launch
		#teleop_waffle
		])
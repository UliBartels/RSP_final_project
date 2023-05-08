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

		print("Hello World! am in render_str_waffle")
		print(pkg_path + '/urdf/waffle.urdf')

		return [SetLaunchConfiguration('waffle_desc', waffle_desc)]

	create_waffle_description_arg = OpaqueFunction(function=render_str_waffle)


	pkg = get_package_share_directory('final_project')
	world_file = os.path.join(pkg,'world','template.sdf')

	waffle_urdf = os.path.join(pkg,'urdf','waffle.urdf')
	
	print(waffle_urdf)

	ign_args = LaunchConfiguration('ign_args')


	ign_launch_arg = DeclareLaunchArgument(
		'ign_args',
		default_value='--render-engine ogre '+ world_file + ' -v 4'
	)

# and bam! launch and spawn everything ignition 

	gazebo_launch = IncludeLaunchDescription(
		PythonLaunchDescriptionSource([os.path.join(
			get_package_share_directory('ros_ign_gazebo'),'launch','ign_gazebo.launch.py'
			)]), launch_arguments={'ign_args': ign_args}.items()	
	)

	spawn_waffle = Node(
		package='ros_ign_gazebo',
		executable='create',
		arguments=['-file', waffle_urdf, '-name', 'waffle_bot', '-y', "1", '-z', "-0.11"],
		output='screen'
	)

	return LaunchDescription([
		#create_waffle_description_arg,
		ign_launch_arg,
		gazebo_launch,
		spawn_waffle
		])

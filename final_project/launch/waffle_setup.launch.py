
from ament_index_python.packages import get_package_share_directory
from launch.actions import OpaqueFunction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch import LaunchContext
from launch_ros.actions import Node



def generate_launch_description():
    ns = "/waffle" 

    def call_turtlebot_launch_file(context):
        robot_launch_instance = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [get_package_share_directory('turtlebot3_bringup'), '/launch/robot.launch.py']
            )
        )
        return [robot_launch_instance]
    call_turtlebot_launch_file_arg = OpaqueFunction(function=call_turtlebot_launch_file)

    return LaunchDescription([
        call_turtlebot_launch_file_arg
    ])

    

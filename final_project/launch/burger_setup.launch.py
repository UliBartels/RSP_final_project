
from ament_index_python.packages import get_package_share_directory
from launch.actions import OpaqueFunction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch import LaunchContext
from launch_ros.actions import Node



def generate_launch_description():
    ns = "/burger" 

    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        output="screen",
        namespace=ns,
        #parameters=[{"camera_calibration_file": "file:///home/spragunr/.ros/camera_info/camera.yaml"}],
        #remappings=[
        #    ('/camera/camera_info', '/camera/camera_info'),
        #    ('/camera/image', '/camera/image_raw')]
    )

    aruco_node = Node(
        package='ros2_aruco',
        executable='aruco_node',
        namespace=ns,
        parameters=[{"image_topic": "/burger/image_raw"},{"camera_info_topic": "/burger/camera_info"}]
    )
 
    def call_turtlebot_launch_file(context):
        robot_launch_instance = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [get_package_share_directory('turtlebot3_bringup'), '/launch/robot.launch.py']
            )
        )
        return [robot_launch_instance]
    call_turtlebot_launch_file_arg = OpaqueFunction(function=call_turtlebot_launch_file)

    return LaunchDescription([
        camera_node,
        aruco_node,
        call_turtlebot_launch_file_arg
    ])

    

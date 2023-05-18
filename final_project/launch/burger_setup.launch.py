
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch_ros.actions import Node
from launch.actions import OpaqueFunction, SetEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchContext



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
        # namespace=ns,
        parameters=[{"image_topic": "/burger/image_raw"},{"camera_info_topic": "/burger/camera_info"}]
    )

    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    LDS_MODEL = os.environ['LDS_MODEL']
    LDS_LAUNCH_FILE = '/hlds_laser.launch.py'

    usb_port = LaunchConfiguration('usb_port', default='/dev/ttyACM0')

    tb3_param_dir = LaunchConfiguration(
        'tb3_param_dir',
        default=os.path.join(
            get_package_share_directory('turtlebot3_bringup'),
            'param',
            TURTLEBOT3_MODEL + '.yaml'))

    robot_pub_dir = LaunchConfiguration(
        'robot_pub_dir',
        default=os.path.join(
            get_package_share_directory('turtlebot3_bringup'),
            'launch'
            ))

    if LDS_MODEL == 'LDS-01':
        lidar_pkg_dir = LaunchConfiguration(
            'lidar_pkg_dir',
            default=os.path.join(get_package_share_directory('hls_lfcd_lds_driver'), 'launch'))
    elif LDS_MODEL == 'LDS-02':
        lidar_pkg_dir = LaunchConfiguration(
            'lidar_pkg_dir',
            default=os.path.join(get_package_share_directory('ld08_driver'), 'launch'))
        LDS_LAUNCH_FILE = '/ld08.launch.py'
    else:
        lidar_pkg_dir = LaunchConfiguration(
            'lidar_pkg_dir',
            default=os.path.join(get_package_share_directory('hls_lfcd_lds_driver'), 'launch'))

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'usb_port',
            default_value=usb_port,
            description='Connected USB port with OpenCR'),

        DeclareLaunchArgument(
            'tb3_param_dir',
            default_value=tb3_param_dir,
            description='Full path to turtlebot3 parameter file to load'),

        DeclareLaunchArgument(
            'robot_pub_dir',
            default_value=robot_pub_dir,
            description='Full path to turtlebot3 robot state publisher launch file to load'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [robot_pub_dir, '/turtlebot3_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time, 'frame_prefix': 'burger/'}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([lidar_pkg_dir, LDS_LAUNCH_FILE]),
            launch_arguments={'port': '/dev/ttyUSB0', 'frame_id': 'base_scan'}.items(),
        ),

        Node(
            package='turtlebot3_node',
            executable='turtlebot3_ros',
            parameters=[tb3_param_dir],
            arguments=['-i', usb_port],
            remappings = [('/odom','/burger/odom'),('/joint_states','/burger/joint_states'),('/imu','/burger/imu'),('/cmd_vel','/burger/cmd_vel')],
            output='screen'),

        camera_node,
        aruco_node,
    ])

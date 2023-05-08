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


# This file is used to spawn waffle/burger via xacro files

def generate_launch_description():
    declared_arguments = []
    # Declare general arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="final_project",
            description="Description package with robot URDF/XACRO files.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="waffle.urdf.xacro",
            description="XACRO description file for waffle/burger.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "entity_name",
            default_value="waffle",
            description="Turtlebot name: waffle or burger",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "namespace",
            default_value="waffle",
            description="Namespace used for spawn multiple robots",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "X",
            default_value="0",
            description="X offset when spawn a robot",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "Y",
            default_value="0",
            description="Y offset when spawn a robot",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "Z",
            default_value="0",
            description="Z offset when spawn a robot",
        )
    )

    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    entity_name = LaunchConfiguration("entity_name")
    ns = LaunchConfiguration("namespace")
    offset_X = LaunchConfiguration("X")
    offset_Y = LaunchConfiguration("Y")
    offset_Z = LaunchConfiguration("Z")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(FindPackageShare(description_package), "urdf", description_file),
        ]
    )

    robot_description = {"robot_description": robot_description_content}
    remapping = [('joint_states', '/world/empty/model/' + entity_name + '/joint_state'),
                 ('robot_description', '/' + entity_name + '/robot_description')]

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=ns,
        output="both",
        parameters=[robot_description],
        remappings=remapping
    )

    ign_gazebo_node = Node(
        package="ros_ign_gazebo",
        executable="create",
        arguments=['-topic', robot_description, '-x', offset_X, '-y', offset_Y, '-z', offset_Z],
        namespace=ns,
    )

    return LaunchDescription(declared_arguments +
                             [robot_state_publisher_node,
                              ign_gazebo_node,
                              ])

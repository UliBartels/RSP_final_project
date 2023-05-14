from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, LogInfo, RegisterEventHandler, TimerAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get Turtlebot3 Model Name
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "entity_name_1",
            description="Robot 1 Name.",
            default_value="waffle",
        )
    )

    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "entity_name_2",
    #         description="Robot 2 Name.",
    #         default_value="burger",
    #     )
    # )

    declared_arguments.append(
        DeclareLaunchArgument(
            "namespace_1",
            description="Namespace for the first robot entity.",
            default_value="ns_waffle",
        )
    )

    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "namespace_2",
    #         description="Namespace for the second robot entity.",
    #         default_value="ns_burger",
    #     )
    # )
    GroupAction(
        actions = [
            robot_description_content = Command(
                [
                    PathJoinSubstitution([FindExecutable(name="xacro")]),
                    " ",
                    PathJoinSubstitution([FindPackageShare(final_project), "urdf", "waffle.urdf.xacro"]),
                    " ",
                    "entity_name:=",
                    entity_name_1,
                ]
            )

            robot_state_publisher_node = Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="both",
                parameters=[robot_description],
            )
        ]
    )
    return LaunchDescription()

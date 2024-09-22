from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
import launch_ros
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):

    ur_type = LaunchConfiguration("ur_type")
    world_file = LaunchConfiguration("world_file")

    ur_sim_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ur_simulation_gz"),
                    "launch",
                    "ur_sim_moveit.launch.py",
                ]
            )
        ),
        launch_arguments={
            "ur_type": ur_type,
            "world_file": world_file,
        }.items(),
    )
    octomap_server_node = TimerAction(
        period=0.0,
        actions=[
            Node(
                package="octomap_server",
                executable="octomap_server_node",
                name="octomap_server",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "resolution": 0.1,
                        "frame_id": "world",
                    }
                ],
                remappings=[("/cloud_in", "/rgbd_camera/points")],
            )
        ],
    )

    ace_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="automatic_cell_explorer",
                executable="ace",
                output="screen",
            )
        ],
    )

    moveit_demo_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="automatic_cell_explorer",
                executable="moveit_demo",
                output="screen",
            )
        ],
    )

    nodes_to_launch = [ur_sim_moveit_launch, octomap_server_node]

    return nodes_to_launch


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            description="Type/series of used UR robot.",
            choices=[
                "ur3",
                "ur3e",
                "ur5",
                "ur5e",
                "ur10",
                "ur10e",
                "ur16e",
                "ur20",
                "ur30",
            ],
            default_value="ur5e",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "world_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("automatic_cell_explorer"),
                    "worlds",
                    "cell.sdf",
                ]
            ),
            description="Gazebo world file (absolute path or filename from the gazebosim worlds collection) containing a custom world.",
        )
    )

    declared_arguments.append(
        launch_ros.actions.SetParameter(name="use_sim_time", value=True)
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )

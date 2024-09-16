from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
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
                    }  # maybe should be "rgbd_camera frame"
                ],
                remappings=[("/cloud_in", "/rgbd_camera/points")],
            )
        ],
    )

    octomap_processor_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="automatic_cell_explorer",
                executable="octomap_processor",
                name="octomap_processor",
                output="screen",
            )
        ],
    )

    # commander_node = TimerAction(
    #     period=20.0,
    #     actions=[
    #         Node(
    #             package="automatic_cell_explorer",
    #             executable="commander",
    #             name="commander",
    #             output="screen",
    #         )
    #     ],
    # )
    planner_node = TimerAction(
        period=6.0,
        actions=[
            Node(
                package="automatic_cell_explorer",
                executable="planner",
                name="planner",
                output="screen",
            )
        ],
    )
    move_robot_node = TimerAction(
        period=6.0,
        actions=[
            Node(
                package="automatic_cell_explorer",
                executable="move_robot_service_node",
                name="move_robot_nodeS",
                output="screen",
            )
        ],
    )

    nodes_to_launch = [
        ur_sim_moveit_launch,
        octomap_server_node,
        # octomap_processor_node,
        # planner_node,
        move_robot_node,
        # commander_node,
    ]

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

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )

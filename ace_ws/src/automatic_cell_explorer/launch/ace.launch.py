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
from ament_index_python.packages import get_package_share_directory


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
                        "frame_id": "world",  # or "rgbd_camera??"
                    }
                ],
                remappings=[("/cloud_in", "/rgbd_camera/points")],
            )
        ],
    )

    parameters = [
        {"use_sim_time": True}
        # {
        #     "sensors": ["rgbd_camera"],
        #     "rgbd_camera.filtered_cloud_topic": "/filtered_cloud",
        #     "rgbd_camera.max_range": 5.0,
        #     "rgbd_camera.max_update_rate": 1.0,
        #     "rgbd_camera.padding_offset": 0.1,
        #     "rgbd_camera.padding_scale": 1.0,
        #     "rgbd_camera.point_cloud_topic": "/rgbd_camera/points",
        #     "rgbd_camera.point_subsample": 1,
        #     "rgbd_camera.sensor_plugin": "occupancy_map_monitor/PointCloudOctomapUpdater",
        #     # specify frame_id???
        # }
    ]

    ace_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="automatic_cell_explorer",
                executable="ace",
                output="screen",
                parameters=parameters,
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

    nodes_to_launch = [ur_sim_moveit_launch, octomap_server_node, ace_node]

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

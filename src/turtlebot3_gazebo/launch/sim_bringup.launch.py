#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory("turtlebot3_gazebo")

    model = LaunchConfiguration("model").perform(context)
    urdf_path = os.path.join(pkg_share, "urdf", f"{model}.urdf")

    with open(urdf_path, "r", encoding="utf-8") as urdf_file:
        robot_description = urdf_file.read()

    model_sdf = PathJoinSubstitution(
        [FindPackageShare("turtlebot3_gazebo"), "models", model, "model.sdf"]
    )

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[
                {
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "robot_description": robot_description,
                }
            ],
        ),
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=[
                "-entity",
                model,
                "-file",
                model_sdf,
                "-x",
                LaunchConfiguration("x_pose"),
                "-y",
                LaunchConfiguration("y_pose"),
                "-z",
                LaunchConfiguration("z_pose"),
                "-Y",
                LaunchConfiguration("yaw"),
            ],
            output="screen",
        ),
    ]


def generate_launch_description():
    world = LaunchConfiguration("world")
    rviz = LaunchConfiguration("rviz")

    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")

    default_world = PathJoinSubstitution(
        [
            FindPackageShare("turtlebot3_gazebo"),
            "worlds",
            "turtlebot3_world.world",
        ]
    )
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("turtlebot3_gazebo"), "rviz", "tb3_gazebo.rviz"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=default_world,
                description="Full path to the Gazebo world file.",
            ),
            DeclareLaunchArgument(
                "model",
                default_value="turtlebot3_waffle_pi",
                description="TurtleBot3 model folder and URDF prefix.",
            ),
            DeclareLaunchArgument("x_pose", default_value="-2.0"),
            DeclareLaunchArgument("y_pose", default_value="-0.5"),
            DeclareLaunchArgument("z_pose", default_value="0.05"),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use Gazebo /clock for all simulation nodes.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Start RViz with the TurtleBot3 Gazebo view.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_gazebo_ros, "launch", "gzserver.launch.py")
                ),
                launch_arguments={"world": world}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_gazebo_ros, "launch", "gzclient.launch.py")
                )
            ),
            OpaqueFunction(function=launch_setup),
            Node(
                condition=IfCondition(rviz),
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
                output="screen",
            ),
        ]
    )

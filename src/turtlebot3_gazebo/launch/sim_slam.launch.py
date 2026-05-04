#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory("turtlebot3_gazebo")

    use_sim_time = LaunchConfiguration("use_sim_time")

    default_world = PathJoinSubstitution(
        [
            FindPackageShare("turtlebot3_gazebo"),
            "worlds",
            "turtlebot3_world.world",
        ]
    )
    default_slam_params = PathJoinSubstitution(
        [FindPackageShare("turtlebot3_gazebo"), "config", "slam_sim.yaml"]
    )
    default_rviz_config = PathJoinSubstitution(
        [FindPackageShare("turtlebot3_gazebo"), "rviz", "tb3_slam_map.rviz"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=default_world,
                description="Full path to the Gazebo world file.",
            ),
            DeclareLaunchArgument(
                "slam_params_file",
                default_value=default_slam_params,
                description="SLAM Toolbox parameter file for simulation.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use Gazebo /clock for SLAM.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Start RViz for SLAM visualization.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz_config,
                description="RViz config file for SLAM mode.",
            ),
            DeclareLaunchArgument(
                "rviz_delay",
                default_value="3.0",
                description="Reserved for compatibility; RViz starts directly.",
            ),
            GroupAction(
                scoped=True,
                actions=[
                    SetLaunchConfiguration("rviz", "false"),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(pkg_share, "launch", "sim_bringup.launch.py")
                        ),
                        launch_arguments={
                            "world": LaunchConfiguration("world"),
                            "use_sim_time": use_sim_time,
                        }.items(),
                    ),
                ],
            ),
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[
                    LaunchConfiguration("slam_params_file"),
                    {"use_sim_time": use_sim_time},
                ],
            ),
            ExecuteProcess(
                condition=IfCondition(LaunchConfiguration("rviz")),
                cmd=[
                    "/opt/ros/humble/bin/rviz2",
                    "-d",
                    LaunchConfiguration("rviz_config"),
                    "--ros-args",
                    "-p",
                    ["use_sim_time:=", use_sim_time],
                ],
                output="screen",
            ),
        ]
    )

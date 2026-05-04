from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution([FindPackageShare("turtlebot3_drl_local_planner"), "config", "gap_sac.yaml"])
    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_config),
            DeclareLaunchArgument("checkpoint", default_value=""),
            DeclareLaunchArgument("output", default_value="evaluation_metrics.csv"),
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "run",
                    "turtlebot3_drl_local_planner",
                    "eval_policy",
                    "--config",
                    LaunchConfiguration("config"),
                    "--checkpoint",
                    LaunchConfiguration("checkpoint"),
                    "--output",
                    LaunchConfiguration("output"),
                ],
                output="screen",
            ),
        ]
    )

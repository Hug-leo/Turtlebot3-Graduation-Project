from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _build_train_command(context):
    cmd = [
        "ros2",
        "run",
        "turtlebot3_drl_local_planner",
        "train_gap_sac",
        "--config",
        LaunchConfiguration("config").perform(context),
    ]
    episodes = LaunchConfiguration("episodes").perform(context).strip()
    if episodes:
        cmd.extend(["--episodes", episodes])
    resume_checkpoint = LaunchConfiguration("resume_checkpoint").perform(context).strip()
    if resume_checkpoint:
        cmd.extend(["--resume-checkpoint", resume_checkpoint])
    curriculum = LaunchConfiguration("curriculum").perform(context).strip()
    if curriculum:
        cmd.extend(["--curriculum", curriculum])
    start_stage = LaunchConfiguration("start_stage").perform(context).strip()
    if start_stage:
        cmd.extend(["--start-stage", start_stage])
    return [ExecuteProcess(cmd=cmd, output="screen")]


def generate_launch_description():
    default_config = PathJoinSubstitution([FindPackageShare("turtlebot3_drl_local_planner"), "config", "gap_sac.yaml"])
    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_config),
            DeclareLaunchArgument("episodes", default_value=""),
            DeclareLaunchArgument("resume_checkpoint", default_value=""),
            DeclareLaunchArgument("curriculum", default_value=""),
            DeclareLaunchArgument("start_stage", default_value=""),
            SetEnvironmentVariable("CUDA_VISIBLE_DEVICES", ""),
            OpaqueFunction(function=_build_train_command),
        ]
    )

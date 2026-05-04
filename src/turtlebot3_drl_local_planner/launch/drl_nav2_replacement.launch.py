from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution(
        [FindPackageShare("turtlebot3_drl_local_planner"), "config", "gap_sac.yaml"]
    )
    default_nav2_params = PathJoinSubstitution(
        [FindPackageShare("turtlebot3_gazebo"), "config", "nav2_sim_params.yaml"]
    )
    default_world = PathJoinSubstitution(
        [FindPackageShare("turtlebot3_gazebo"), "worlds", "turtlebot3_world.world"]
    )
    default_map = [EnvironmentVariable("HOME"), "/maps/my_map.yaml"]

    return LaunchDescription(
        [
            DeclareLaunchArgument("map", default_value=default_map),
            DeclareLaunchArgument("world", default_value=default_world),
            DeclareLaunchArgument("params_file", default_value=default_nav2_params),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("policy_checkpoint", default_value=""),
            DeclareLaunchArgument("config_file", default_value=default_config),
            DeclareLaunchArgument("lookahead_distance_m", default_value="0.4"),
            DeclareLaunchArgument("safety_stop_distance", default_value="0.18"),
            DeclareLaunchArgument("heading_guard_enabled", default_value="true"),
            DeclareLaunchArgument("goal_stop_distance_m", default_value="0.35"),
            DeclareLaunchArgument("speed_governor_enabled", default_value="true"),
            DeclareLaunchArgument("speed_slow_distance_m", default_value="0.45"),
            DeclareLaunchArgument("speed_min_distance_m", default_value="0.22"),
            DeclareLaunchArgument("speed_min_scale", default_value="0.15"),
            DeclareLaunchArgument("recovery_enabled", default_value="true"),
            DeclareLaunchArgument("recovery_linear_velocity", default_value="-0.035"),
            DeclareLaunchArgument("recovery_angular_velocity", default_value="0.55"),
            DeclareLaunchArgument("recovery_duration_sec", default_value="0.8"),
            DeclareLaunchArgument("narrow_passage_enabled", default_value="true"),
            DeclareLaunchArgument("narrow_passage_clearance_m", default_value="0.42"),
            DeclareLaunchArgument("narrow_passage_front_clearance_m", default_value="0.24"),
            DeclareLaunchArgument("narrow_passage_linear_velocity", default_value="0.025"),
            DeclareLaunchArgument("narrow_passage_centering_gain", default_value="1.4"),
            DeclareLaunchArgument("narrow_passage_max_angular_velocity", default_value="0.45"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("turtlebot3_gazebo"), "launch", "sim_nav2.launch.py"]
                    )
                ),
                launch_arguments={
                    "map": LaunchConfiguration("map"),
                    "world": LaunchConfiguration("world"),
                    "params_file": LaunchConfiguration("params_file"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "rviz": LaunchConfiguration("rviz"),
                    "nav2_cmd_vel_topic": "/cmd_vel_nav2",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("turtlebot3_drl_local_planner"),
                            "launch",
                            "drl_controller.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "config_file": LaunchConfiguration("config_file"),
                    "policy_checkpoint": LaunchConfiguration("policy_checkpoint"),
                    "publish_directly_to_cmd_vel": "true",
                    "plan_target_mode": "lookahead",
                    "lookahead_distance_m": LaunchConfiguration("lookahead_distance_m"),
                    "prefer_plan_goal": "true",
                    "safety_stop_distance": LaunchConfiguration("safety_stop_distance"),
                    "heading_guard_enabled": LaunchConfiguration("heading_guard_enabled"),
                    "goal_stop_distance_m": LaunchConfiguration("goal_stop_distance_m"),
                    "speed_governor_enabled": LaunchConfiguration("speed_governor_enabled"),
                    "speed_slow_distance_m": LaunchConfiguration("speed_slow_distance_m"),
                    "speed_min_distance_m": LaunchConfiguration("speed_min_distance_m"),
                    "speed_min_scale": LaunchConfiguration("speed_min_scale"),
                    "recovery_enabled": LaunchConfiguration("recovery_enabled"),
                    "recovery_linear_velocity": LaunchConfiguration("recovery_linear_velocity"),
                    "recovery_angular_velocity": LaunchConfiguration("recovery_angular_velocity"),
                    "recovery_duration_sec": LaunchConfiguration("recovery_duration_sec"),
                    "narrow_passage_enabled": LaunchConfiguration("narrow_passage_enabled"),
                    "narrow_passage_clearance_m": LaunchConfiguration("narrow_passage_clearance_m"),
                    "narrow_passage_front_clearance_m": LaunchConfiguration("narrow_passage_front_clearance_m"),
                    "narrow_passage_linear_velocity": LaunchConfiguration("narrow_passage_linear_velocity"),
                    "narrow_passage_centering_gain": LaunchConfiguration("narrow_passage_centering_gain"),
                    "narrow_passage_max_angular_velocity": LaunchConfiguration("narrow_passage_max_angular_velocity"),
                }.items(),
            ),
        ]
    )

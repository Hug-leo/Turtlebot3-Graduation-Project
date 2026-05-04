import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 1. Declare package paths
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    pkg_share = get_package_share_directory("agv_controller")

    # 2. Declare configuration file paths
    default_map = os.path.join(os.path.expanduser("~"), "my_room_map.yaml")
    my_params_file = os.path.join(pkg_share, "config", "my_nav2_params.yaml")
    ekf_params_file = os.path.join(
        pkg_share, "config", "ekf.yaml"
    )  # Added EKF config

    # 3. Launch argument — map selection from outside
    map_arg = DeclareLaunchArgument(
        "map",
        default_value=default_map,
        description="Full path to the map yaml file to load",
    )
    map_yaml_file = LaunchConfiguration("map")

    return LaunchDescription(
        [
            map_arg,
            # --- 1. START LIDAR ---
            Node(
                package="hls_lfcd_lds_driver",
                executable="hlds_laser_publisher",
                name="hlds_laser_publisher",
                parameters=[
                    {"port": "/dev/ttyUSB0", "frame_id": "laser", "use_sim_time": False}
                ],
                output="screen",
            ),
            # --- 2. TF TREE ---
            # TF frame: base_link -> laser (LiDAR position on the robot)
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_tf_laser",
                arguments=["0", "0", "0.1", "0", "0", "0", "base_link", "laser"],
            ),
            # TF frame: base_link -> base_footprint (Projection of the robot onto the ground)
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_tf_footprint",
                arguments=["0", "0", "0", "0", "0", "0", "base_link", "base_footprint"],
            ),
            # TF frame: base_link -> imu_link (Actual position of BNO055 on the robot)
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_tf_imu",
                arguments=[
                    "0.095",
                    "-0.07",
                    "0.02",
                    "0",
                    "0",
                    "0",
                    "base_link",
                    "imu_link",
                ],
            ),
            # --- 4. EKF FILTER (SENSOR FUSION) ---
            # Fuse /odom and /imu/data, publish TF: odom -> base_link smoothly
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                parameters=[ekf_params_file, {"use_sim_time": False}],
            ),
            # --- 5. NAV2 NAVIGATION SYSTEM ---
            # Run AMCL, Planner, Controller (RPP), Behavior...
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, "launch", "bringup_launch.py")
                ),
                launch_arguments={
                    "map": map_yaml_file,
                    "use_sim_time": "False",
                    "params_file": my_params_file,
                    "autostart": "True",
                }.items(),
            ),
        ]
    )

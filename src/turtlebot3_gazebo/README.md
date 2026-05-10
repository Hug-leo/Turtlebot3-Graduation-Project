# TurtleBot3 Gazebo SLAM/Nav2 Baseline

This package is the Gazebo-side baseline for the warehouse AMR project using a TurtleBot3 Waffle Pi. The real robot pipeline already runs ROS 2, SLAM, Nav2, and autonomous warehouse navigation. This package keeps the simulation path stable while the DRL local-planner bypass is trained and evaluated.

## Current Goal

The immediate goal is not DRL. The immediate goal is to make simulation behave like the real robot navigation pipeline:

1. Spawn `turtlebot3_waffle_pi` in Gazebo.
2. Verify `/scan`, `/odom`, `/cmd_vel`, `/tf`, and `/tf_static`.
3. Run SLAM Toolbox with simulation time.
4. Save a map.
5. Load the saved map.
6. Run Nav2 with AMCL and the default local controller.
7. Send a goal in RViz and verify the robot navigates.

After this baseline is stable, DRL local planning is evaluated against a known-good Nav2 stack. The current best DRL simulation checkpoint is:

```text
/home/hug/tb3_sim_ws/runs/gap_sac_20260506_230839/deployment_checkpoint.pt
```

## Package Structure

Important paths:

```text
turtlebot3_gazebo/
├── config/
│   ├── ekf.yaml
│   ├── my_nav2_params.yaml
│   ├── nav2_sim_params.yaml
│   ├── slam_handheld.yaml
│   └── slam_sim.yaml
├── launch/
│   ├── handheld_slam.launch.py
│   ├── nav2_custom.launch.py
│   ├── sim_bringup.launch.py
│   ├── sim_nav2.launch.py
│   └── sim_slam.launch.py
├── models/
├── rviz/
├── urdf/
├── worlds/
├── CMakeLists.txt
├── package.xml
└── README.md
```

The real-robot-derived files are preserved:

```text
launch/handheld_slam.launch.py
launch/nav2_custom.launch.py
config/ekf.yaml
config/my_nav2_params.yaml
config/slam_handheld.yaml
```

The simulation-specific files are:

```text
launch/sim_bringup.launch.py
launch/sim_slam.launch.py
launch/sim_nav2.launch.py
config/slam_sim.yaml
config/nav2_sim_params.yaml
```

## Why SLAM/Nav2 First

DRL local planning should not be debugged at the same time as Gazebo spawn, TF, odometry, LiDAR, map loading, AMCL, and Nav2 lifecycle issues. The baseline must first prove that the standard ROS 2 navigation stack works in simulation. Then a DRL controller can be evaluated as a local planner replacement while keeping map server, localization, BT navigation, and global planning stable.

## Build

From the workspace root:

```bash
cd ~/turtlebot3_ws
colcon build --packages-select turtlebot3_gazebo
source install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
```

The new simulation launch files do not require `TURTLEBOT3_MODEL`, because they default to `turtlebot3_waffle_pi`. The export is still useful for compatibility with upstream TurtleBot3 launch files.

## Launch Files

`sim_bringup.launch.py` starts Gazebo, spawns the Waffle Pi model, starts `robot_state_publisher`, and optionally starts RViz.

Main arguments:

```text
world
model
x_pose
y_pose
z_pose
yaw
use_sim_time
rviz
```

Defaults:

```text
model:=turtlebot3_waffle_pi
x_pose:=-2.0
y_pose:=-0.5
z_pose:=0.05
yaw:=0.0
use_sim_time:=true
rviz:=true
```

The default spawn pose is intentionally away from the central obstacle in `turtlebot3_world.world` so Gazebo starts without contact instability.

`sim_slam.launch.py` includes `sim_bringup.launch.py`, starts SLAM Toolbox with `config/slam_sim.yaml`, and optionally starts RViz.

`sim_nav2.launch.py` includes `sim_bringup.launch.py`, loads a saved map, starts Nav2 bringup with `config/nav2_sim_params.yaml`, and optionally starts RViz.

## Config Files

`config/slam_sim.yaml` is for Gazebo mapping. It uses:

```yaml
use_sim_time: true
map_frame: map
odom_frame: odom
base_frame: base_link
scan_topic: /scan
```

`config/nav2_sim_params.yaml` is for Gazebo navigation. It uses:

```yaml
use_sim_time: true
global_frame: map
odom_frame_id: odom
base_frame_id: base_link
scan_topic: /scan
odom_topic: /odom
```

`config/my_nav2_params.yaml`, `config/slam_handheld.yaml`, and `config/ekf.yaml` are retained as real-robot-derived configs. They may use real hardware topics, EKF settings, or `use_sim_time: false`, so do not use them as the default simulation configs unless intentionally testing robot-specific behavior.

## Run Gazebo Only

```bash
ros2 launch turtlebot3_gazebo sim_bringup.launch.py
```

Use another world:

```bash
ros2 launch turtlebot3_gazebo sim_bringup.launch.py \
  world:=$(ros2 pkg prefix turtlebot3_gazebo)/share/turtlebot3_gazebo/worlds/turtlebot3_house.world
```

Verify core topics:

```bash
ros2 topic list
ros2 topic echo /scan --once
ros2 topic echo /odom --once
ros2 topic info /cmd_vel
```

Teleop:

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

The robot should move in Gazebo when `/cmd_vel` is published.

## Run SLAM

```bash
ros2 launch turtlebot3_gazebo sim_slam.launch.py
```

Drive the robot around:

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

Check map output:

```bash
ros2 topic echo /map --once
ros2 topic echo /map_metadata --once
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
```

## Save A Map

Create a map directory and save the current SLAM map:

```bash
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/warehouse_sim
```

Expected output files:

```text
~/maps/warehouse_sim.yaml
~/maps/warehouse_sim.pgm
```

Inspect the YAML if Nav2 later fails to load the map:

```bash
cat ~/maps/warehouse_sim.yaml
```

The `image:` path inside the YAML must point to the saved image file.

## Run Nav2 With A Saved Map

```bash
ros2 launch turtlebot3_gazebo sim_nav2.launch.py map:=$HOME/maps/warehouse_sim.yaml
```

If you saved the map to the default path above, this also works:

```bash
ros2 launch turtlebot3_gazebo sim_nav2.launch.py
```

In RViz:

1. Set the initial pose with `2D Pose Estimate`.
2. Confirm `/amcl_pose` is publishing.
3. Send a goal with `2D Nav Goal`.
4. Confirm `/plan` and `/cmd_vel` activity.

Useful checks:

```bash
ros2 lifecycle nodes
ros2 topic echo /amcl_pose --once
ros2 topic echo /plan --once
ros2 topic echo /cmd_vel
```

## Required Topics

Gazebo only:

```text
/scan
/odom
/cmd_vel
/tf
/tf_static
```

SLAM:

```text
/map
/map_metadata
```

Nav2:

```text
/goal_pose
/amcl_pose
/plan
/cmd_vel
```

## Required TF Tree

For Gazebo bringup:

```text
odom -> base_footprint -> base_link -> base_scan
```

For SLAM:

```text
map -> odom -> base_footprint -> base_link -> base_scan
```

For Nav2 localization:

```text
map -> odom -> base_footprint -> base_link -> base_scan
```

Debug TF:

```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map odom
```

## Common Problems And Fixes

No `/scan`:

```bash
ros2 topic list | grep scan
ros2 topic echo /scan --once
```

If missing, confirm the Waffle Pi SDF spawned and Gazebo is not paused.

No robot motion:

```bash
ros2 topic echo /cmd_vel
ros2 topic echo /odom --once
```

If `/cmd_vel` exists but `/odom` does not change, the Gazebo diff-drive plugin is not active or the entity did not spawn correctly.

RViz shows TF errors:

```bash
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map odom
```

During Gazebo-only testing, `map -> odom` is not expected. During SLAM or Nav2, it is required.

Nav2 lifecycle nodes are inactive:

```bash
ros2 lifecycle nodes
ros2 lifecycle get /controller_server
```

Check the map path and make sure `use_sim_time:=true` is active.

AMCL does not localize:

```bash
ros2 topic echo /initialpose --once
ros2 topic echo /amcl_pose --once
ros2 topic echo /scan --once
```

Set the initial pose in RViz after Nav2 starts.

Map server fails:

```bash
ls -l ~/maps/warehouse_sim.yaml ~/maps/warehouse_sim.pgm
cat ~/maps/warehouse_sim.yaml
```

The YAML path and image path must be valid from the machine running Nav2.

## Future DRL Local Planner

The DRL work targets only the local planning/controller layer. Nav2 still handles map loading, AMCL, global planning, BT navigation, and user goals.

Keep these Nav2 components:

```text
map_server
amcl
planner_server
bt_navigator
mission logic
global planner
```

Replace or shadow this layer:

```text
controller_server / local planner
```

Expected DRL observations:

```text
LiDAR scan
odometry
current robot velocity
local goal or path segment
distance to goal
heading error
nearest obstacle distance and angle
```

Expected DRL action:

```text
linear velocity v
angular velocity w
```

Possible implementation paths:

1. Active path: a separate ROS 2 node publishing `/cmd_vel` in bypass mode, first used in shadow mode for logging and safety validation.
2. Paused path: a proper Nav2 controller plugin implementing `nav2_core::Controller`.

Training should happen in Gazebo first. Real-robot deployment must keep the default Nav2 controller available as a fallback and must include safety limits for velocity, obstacle distance, watchdog timeout, and emergency stop behavior.

Current DRL runtime behavior includes final-orientation alignment. After the robot reaches the final goal position, the controller can rotate in place to match the RViz goal yaw before publishing a final zero command.

## Maintenance Notes

Keep real-robot configs and simulation configs separate. The simulation launch files should default to `use_sim_time:=true`, `/scan`, `/odom`, and `base_link`.

Do not add hardware drivers such as the physical LDS driver to simulation launch files. Gazebo already publishes the simulated laser scan.

Do not add EKF to the default simulation path unless there is a specific reason. Gazebo already publishes odometry and `odom -> base_footprint`; adding EKF can duplicate TF.

Before changing Nav2 controller parameters for DRL experiments, first confirm the baseline still works with:

```bash
ros2 launch turtlebot3_gazebo sim_nav2.launch.py map:=$HOME/maps/warehouse_sim.yaml
```

If a later experiment breaks navigation, revert to `config/nav2_sim_params.yaml` and verify SLAM/Nav2 again before debugging DRL.

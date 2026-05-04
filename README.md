# TurtleBot3 DRL Simulation Workspace

This workspace contains the active Gazebo and DRL development packages for the TurtleBot3 local-planner replacement research.

The main goal is to keep a working Nav2 simulation baseline while training and evaluating a Deep Reinforcement Learning local planner candidate in Gazebo.

## Workspace Layout

```text
tb3_sim_ws/
├── src/
│   ├── turtlebot3_drl_local_planner/
│   ├── turtlebot3_drl_nav2_controller/
│   └── turtlebot3_gazebo/
├── runs/
│   └── gap_sac_<timestamp>/
├── docs/
│   ├── DRL_LOCAL_PLANNER_REPORT_DRAFT.md
│   ├── PROJECT_CONTEXT_FOR_AI.md
│   └── MAINTENANCE_LOG.md
├── build/
├── install/
└── log/
```

## Current Status

- Gazebo SLAM/Nav2 baseline is available through `src/turtlebot3_gazebo`.
- DRL training and bypass replacement are available through `src/turtlebot3_drl_local_planner`.
- Native Nav2 plugin work exists in `src/turtlebot3_drl_nav2_controller`, but the active direction is currently better policy training for bypass replacement.
- DRL has completed local-goal and hard-gap training stages in Gazebo.
- Hard S-curve and winding-path training still need more work.
- Real robot DRL deployment is prepared conceptually, but not validated as direct control yet.

## Important DRL Files

```text
src/turtlebot3_drl_local_planner/config/gap_sac.yaml
src/turtlebot3_drl_local_planner/launch/train_gap_sac.launch.py
src/turtlebot3_drl_local_planner/launch/drl_controller.launch.py
src/turtlebot3_drl_local_planner/launch/drl_nav2_replacement.launch.py
src/turtlebot3_drl_local_planner/turtlebot3_drl_local_planner/train_gap_sac.py
src/turtlebot3_drl_local_planner/turtlebot3_drl_local_planner/envs/gazebo_nav_env.py
src/turtlebot3_drl_local_planner/turtlebot3_drl_local_planner/nodes/drl_controller_node.py
```

## Important Run Artifacts

```text
runs/gap_sac_20260502_145623/deployment_checkpoint.pt
runs/gap_sac_20260503_143312/deployment_checkpoint.pt
runs/gap_sac_20260503_164614/stage_summary.csv
```

The run `gap_sac_20260503_164614` failed at `hard_s_curve_sequences` by timeout, not collision. This means the next training work should continue staged winding-path training.

## Build

```bash
cd ~/tb3_sim_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Continue DRL Training

```bash
cd ~/tb3_sim_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch turtlebot3_drl_local_planner train_gap_sac.launch.py \
  episodes:=1000 \
  curriculum:=true \
  start_stage:=hard_two_turn_sequences \
  resume_checkpoint:=/home/hug/tb3_sim_ws/runs/gap_sac_20260503_164614/best_checkpoint.pt
```

Fallback checkpoint if the latest failed-stage checkpoint is unstable:

```text
/home/hug/tb3_sim_ws/runs/gap_sac_20260503_143312/deployment_checkpoint.pt
```

## Documentation

Use these files for thesis/report writing and future AI handoff:

```text
docs/DRL_LOCAL_PLANNER_REPORT_DRAFT.md
docs/PROJECT_CONTEXT_FOR_AI.md
docs/MAINTENANCE_LOG.md
```

These documentation files are kept local in this workspace and are excluded from Git pushes by `.gitignore`.

## Git Export Scope

The GitHub export intentionally excludes generated ROS workspace output, local run artifacts, `docs/`, and the experimental native Nav2 controller package at `src/turtlebot3_drl_nav2_controller/`.

## Safety And Claims

Do not claim the DRL policy fully replaces Nav2 on the real robot yet.

Accurate current claim:

```text
The project has a working Gazebo/Nav2 baseline, a Gazebo-trained DRL local planner candidate, successful local/hard-gap simulation results, and a prepared path for future real-robot shadow validation.
```

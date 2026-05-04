# AGENTS.md

## Project Rules

- This project uses TurtleBot3 Waffle Pi for warehouse AMR research.
- The existing Gazebo + SLAM + Nav2 baseline must remain working.
- Do not remove or break existing SLAM/Nav2 launch files.
- DRL work must be isolated in `turtlebot3_drl_local_planner`.
- The first DRL inference mode must publish to `/cmd_vel_drl`, not `/cmd_vel`.
- Direct control of `/cmd_vel` is disabled by default.
- Always keep Nav2 baseline available as fallback.
- README.md must be updated after every important implementation change.
- Mark paper-derived equations separately from ROS/Gazebo engineering adaptations.
- Do not claim real-robot readiness until Gazebo training, evaluation, and safety checks pass.
- Prefer small, testable changes.
- Run build/import checks after implementation whenever possible.

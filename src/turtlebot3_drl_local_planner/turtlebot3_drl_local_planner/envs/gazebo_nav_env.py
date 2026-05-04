import math
import time
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import numpy as np

from turtlebot3_drl_local_planner.utils.geometry import goal_relative_geometry, quaternion_to_yaw
from turtlebot3_drl_local_planner.utils.ros_gazebo import try_call_empty_service
from turtlebot3_drl_local_planner.utils.scan_features import (
    build_lidar_feature,
    clean_lidar_ranges,
    compute_collision_condition,
    compute_nearest_obstacle,
)


@dataclass
class EnvMetrics:
    path_length: float = 0.0
    min_obstacle_distance: float = math.inf
    initial_DG: float = math.inf
    min_DG: float = math.inf
    final_DG: float = math.inf
    steps: int = 0
    close_obstacle_steps: int = 0
    narrow_gap_steps: int = 0
    left_min_obstacle_distance: float = math.inf
    right_min_obstacle_distance: float = math.inf
    waypoints_reached: int = 0
    previous_position: Optional[Tuple[float, float]] = None
    previous_DG: Optional[float] = None
    previous_action: Optional[Tuple[float, float]] = None


class GazeboNavEnv:
    """Minimal ROS 2/Gazebo RL environment for GAP_SAC training.

    Implementation adaptation: training may publish to /cmd_vel only when explicitly configured.
    Shadow/evaluation can publish to /cmd_vel_drl to keep Nav2 baseline untouched.
    """

    def __init__(self, node, config: Dict):
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import LaserScan

        self.node = node
        self.config = config
        self.state_cfg = config.get("state", {})
        self.reward_cfg = config.get("reward", {})
        self.ros_cfg = config.get("ros", {})
        self.training_cfg = config.get("training", {})

        self.latest_scan: Optional[LaserScan] = None
        self.latest_odom: Optional[Odometry] = None
        self.goal = self.training_cfg.get("fixed_goal", {"x": 1.0, "y": 0.0, "yaw": 0.0})
        self.current_stage: Optional[Dict] = None
        self.current_start_pose = dict(self.training_cfg.get("start_pose", {}))
        self.last_goal_mode = "absolute"
        self.last_local_goal_offset = {"dx": 0.0, "dy": 0.0, "dyaw": 0.0}
        self._waypoint_goals = []
        self._waypoint_offsets = []
        self._active_waypoint_index = 0
        self.metrics = EnvMetrics()
        self.reset_cooldown_sec = float(self.training_cfg.get("reset_cooldown_sec", 1.5))
        self.reset_max_attempts = int(self.training_cfg.get("reset_max_attempts", 3))
        self.step_duration_sec = float(self.training_cfg.get("step_duration_sec", 0.10))
        self._warned_reset_unavailable = False
        self._reset_collision = False
        self._last_action: Optional[Tuple[float, float]] = None

        self.scan_sub = node.create_subscription(
            LaserScan,
            self.ros_cfg.get("input_scan_topic", "/scan"),
            self._scan_callback,
            10,
        )
        self.odom_sub = node.create_subscription(
            Odometry,
            self.ros_cfg.get("input_odom_topic", "/odom"),
            self._odom_callback,
            10,
        )
        cmd_topic = self.ros_cfg.get("training_cmd_vel_topic", "/cmd_vel")
        if not self.ros_cfg.get("publish_directly_to_cmd_vel", False):
            cmd_topic = self.ros_cfg.get("shadow_cmd_vel_topic", "/cmd_vel_drl")
        self.cmd_pub = node.create_publisher(Twist, cmd_topic, 10)

    def set_stage(self, stage: Optional[Dict]) -> None:
        self.current_stage = dict(stage) if stage else None

    def _scan_callback(self, msg):
        self.latest_scan = msg

    def _odom_callback(self, msg):
        self.latest_odom = msg

    def wait_for_sensors(self, timeout_sec: float = 5.0) -> bool:
        import rclpy

        end_time = time.time() + timeout_sec
        while time.time() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if self.latest_scan is not None and self.latest_odom is not None:
                return True
        return False

    def _publish_zero_velocity(self, repeat: int = 3) -> None:
        from geometry_msgs.msg import Twist
        import rclpy

        for _ in range(repeat):
            self.cmd_pub.publish(Twist())
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _reset_world_if_available(self) -> bool:
        for service_name in (
            self.training_cfg.get("reset_world_service", "/reset_world"),
            self.training_cfg.get("reset_simulation_service", "/reset_simulation"),
        ):
            if service_name and try_call_empty_service(self.node, service_name, timeout_sec=1.0):
                return True
        return False

    def _select_start_pose(self) -> Dict:
        stage = self.current_stage or {}
        if stage.get("start_pose"):
            return dict(stage["start_pose"])
        stage_starts = list(stage.get("start_poses") or [])
        if stage_starts:
            return dict(stage_starts[int(np.random.randint(0, len(stage_starts)))])
        if stage.get("random_start"):
            safe_starts = list(stage.get("safe_start_poses") or self.training_cfg.get("safe_start_poses", []))
            if safe_starts:
                return dict(safe_starts[int(np.random.randint(0, len(safe_starts)))])
        return dict(self.training_cfg.get("start_pose", {}))

    def _set_entity_start_pose_if_available(self) -> bool:
        import rclpy

        try:
            from gazebo_msgs.srv import SetEntityState
            from geometry_msgs.msg import Twist
        except ImportError:
            return False

        service_name = self.training_cfg.get("set_entity_state_service", "/set_entity_state")
        client = self.node.create_client(SetEntityState, service_name)
        if not client.wait_for_service(timeout_sec=1.0):
            return False

        start_pose = self.current_start_pose or self.training_cfg.get("start_pose", {})
        yaw = float(start_pose.get("yaw", 0.0))
        request = SetEntityState.Request()
        request.state.name = self.training_cfg.get("entity_name", "turtlebot3_waffle_pi")
        request.state.reference_frame = "world"
        request.state.pose.position.x = float(start_pose.get("x", -2.0))
        request.state.pose.position.y = float(start_pose.get("y", -0.5))
        request.state.pose.position.z = float(start_pose.get("z", 0.05))
        request.state.pose.orientation.z = math.sin(yaw * 0.5)
        request.state.pose.orientation.w = math.cos(yaw * 0.5)
        request.state.twist = Twist()

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=1.0)
        return future.done() and future.exception() is None and bool(future.result().success)

    def _robot_pose_xy_yaw(self) -> Tuple[float, float, float]:
        if self.latest_odom is None:
            start_pose = self.current_start_pose or self.training_cfg.get("start_pose", {})
            return (
                float(start_pose.get("x", -2.0)),
                float(start_pose.get("y", -0.5)),
                float(start_pose.get("yaw", 0.0)),
            )
        pose = self.latest_odom.pose.pose
        yaw = quaternion_to_yaw(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        return float(pose.position.x), float(pose.position.y), float(yaw)

    def _local_offset_to_goal(self, offset: Dict) -> Dict:
        return self._local_offset_to_goal_from_pose(self._robot_pose_xy_yaw(), offset)

    def _local_offset_to_goal_from_pose(self, pose_xy_yaw: Tuple[float, float, float], offset: Dict) -> Dict:
        x, y, yaw = pose_xy_yaw
        dx = float(offset.get("dx", 0.6))
        dy = float(offset.get("dy", 0.0))
        dyaw = float(offset.get("dyaw", 0.0))
        return {
            "x": x + math.cos(yaw) * dx - math.sin(yaw) * dy,
            "y": y + math.sin(yaw) * dx + math.cos(yaw) * dy,
            "yaw": yaw + dyaw,
        }

    def _select_goal_for_episode(self) -> None:
        stage = self.current_stage or {}
        self._waypoint_goals = []
        self._waypoint_offsets = []
        self._active_waypoint_index = 0
        if stage.get("local_goal_sequence") or stage.get("local_goal_sequences"):
            sequences = stage.get("local_goal_sequences")
            if sequences:
                sequence = list(sequences[int(np.random.randint(0, len(sequences)))])
            else:
                sequence = list(stage.get("local_goal_sequence") or [])
            if not sequence:
                sequence = [{"dx": 0.8, "dy": 0.0, "dyaw": 0.0}]
            start_pose = self._robot_pose_xy_yaw()
            self._waypoint_offsets = [dict(offset) for offset in sequence]
            self._waypoint_goals = [
                self._local_offset_to_goal_from_pose(start_pose, offset) for offset in self._waypoint_offsets
            ]
            self.goal = dict(self._waypoint_goals[0])
            self.last_goal_mode = "local_goal_sequence"
            self.last_local_goal_offset = dict(self._waypoint_offsets[0])
            return
        if stage.get("random_local_goal"):
            offsets = list(stage.get("local_goal_offsets") or self.training_cfg.get("safe_local_offsets", []))
            if not offsets:
                offsets = [{"dx": 0.6, "dy": 0.0, "dyaw": 0.0}]
            offset = dict(offsets[int(np.random.randint(0, len(offsets)))])
            self.goal = self._local_offset_to_goal(offset)
            self.last_goal_mode = "random_local_offset"
            self.last_local_goal_offset = offset
            return
        if stage.get("local_goal_offset"):
            offset = dict(stage["local_goal_offset"])
            self.goal = self._local_offset_to_goal(offset)
            self.last_goal_mode = "local_offset"
            self.last_local_goal_offset = offset
            return
        if stage.get("goal"):
            self.goal = dict(stage["goal"])
            self.last_goal_mode = "absolute"
            self.last_local_goal_offset = {"dx": 0.0, "dy": 0.0, "dyaw": 0.0}
            return
        self.last_goal_mode = "absolute"
        self.last_local_goal_offset = {"dx": 0.0, "dy": 0.0, "dyaw": 0.0}

    def _settle_after_reset(self) -> None:
        import rclpy

        end_time = time.time() + self.reset_cooldown_sec
        while time.time() < end_time:
            self._publish_zero_velocity(repeat=1)
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _reset_once(self) -> bool:
        self._publish_zero_velocity(repeat=5)
        reset_ok = self._set_entity_start_pose_if_available()
        if not reset_ok:
            reset_ok = self._reset_world_if_available()
        if not reset_ok and not self._warned_reset_unavailable:
            self.node.get_logger().warn(
                "Gazebo reset services are unavailable. Episodes will continue from the current robot state."
            )
            self._warned_reset_unavailable = True
        self._settle_after_reset()
        return reset_ok

    def reset(self) -> np.ndarray:
        self.metrics = EnvMetrics()
        self._reset_collision = False
        self._last_action = None
        self.current_start_pose = self._select_start_pose()
        obs = None
        info = None
        collision_distance = float(self.reward_cfg.get("collision_distance", 0.12))
        min_reset_clearance = float((self.current_stage or {}).get("min_reset_clearance", collision_distance))

        for attempt in range(max(self.reset_max_attempts, 1)):
            self._reset_once()
            self.wait_for_sensors()
            self._select_goal_for_episode()
            obs, info = self._build_observation()
            if float(info["DO"]) >= min_reset_clearance:
                break
            if attempt + 1 < max(self.reset_max_attempts, 1):
                self.node.get_logger().warn(
                    f"Reset attempt {attempt + 1} started in collision "
                    f"(DO={float(info['DO']):.3f}); retrying."
                )
        else:
            self._reset_collision = True
            self.node.get_logger().warn(
                f"Reset still starts in collision after {self.reset_max_attempts} attempts "
                f"(DO={float(info['DO']):.3f}). Episode will terminate immediately."
            )

        if obs is None or info is None:
            obs, info = self._build_observation()
        self.metrics.initial_DG = float(info["DG"])
        self.metrics.min_DG = float(info["DG"])
        self.metrics.final_DG = float(info["DG"])
        self.metrics.previous_DG = info["DG"]
        self.metrics.previous_position = (float(info.get("x", 0.0)), float(info.get("y", 0.0)))
        return obs

    def _advance_waypoint_if_ready(self, info: Dict) -> Tuple[bool, float]:
        if not self._waypoint_goals or self._active_waypoint_index >= len(self._waypoint_goals) - 1:
            return False, 0.0
        stage = self.current_stage or {}
        goal_tolerance = float(stage.get("waypoint_tolerance", stage.get("goal_tolerance", self.reward_cfg.get("goal_tolerance", 0.2))))
        if float(info["DG"]) > goal_tolerance:
            return False, 0.0
        self.metrics.waypoints_reached += 1
        self._active_waypoint_index += 1
        self.goal = dict(self._waypoint_goals[self._active_waypoint_index])
        self.last_local_goal_offset = dict(self._waypoint_offsets[self._active_waypoint_index])
        self.metrics.previous_DG = None
        return True, float(stage.get("waypoint_reward", self.reward_cfg.get("near_goal_bonus_weight", 0.0)))

    def step(self, action) -> Tuple[np.ndarray, float, bool, Dict]:
        from geometry_msgs.msg import Twist
        import rclpy

        twist = Twist()
        twist.linear.x = float(action[0])
        twist.angular.z = float(action[1])
        self._last_action = (twist.linear.x, twist.angular.z)
        self.cmd_pub.publish(twist)
        end_time = time.time() + self.step_duration_sec
        while time.time() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.02)

        obs, info = self._build_observation()
        waypoint_advanced, waypoint_reward = self._advance_waypoint_if_ready(info)
        if waypoint_advanced:
            obs, info = self._build_observation()
        if self._reset_collision:
            self._reset_collision = False
            self._update_metrics(info)
            info["terminal_reason"] = "reset_collision"
            info["path_length"] = self.metrics.path_length
            info["min_obstacle_distance"] = self.metrics.min_obstacle_distance
            info["initial_DG"] = self.metrics.initial_DG
            info["final_DG"] = self.metrics.final_DG
            info["min_DG"] = self.metrics.min_DG
            info["close_obstacle_steps"] = self.metrics.close_obstacle_steps
            info["narrow_gap_steps"] = self.metrics.narrow_gap_steps
            info["left_min_obstacle_distance"] = self.metrics.left_min_obstacle_distance
            info["right_min_obstacle_distance"] = self.metrics.right_min_obstacle_distance
            info["waypoints_reached"] = self.metrics.waypoints_reached
            info["waypoint_count"] = len(self._waypoint_goals)
            return obs, float(self.reward_cfg.get("collision_penalty_R4", -800.0)), True, info

        reward, done, terminal_reason = self._compute_reward(info)
        reward += waypoint_reward
        self._update_metrics(info)
        info["terminal_reason"] = terminal_reason
        info["path_length"] = self.metrics.path_length
        info["min_obstacle_distance"] = self.metrics.min_obstacle_distance
        info["initial_DG"] = self.metrics.initial_DG
        info["final_DG"] = self.metrics.final_DG
        info["min_DG"] = self.metrics.min_DG
        info["close_obstacle_steps"] = self.metrics.close_obstacle_steps
        info["narrow_gap_steps"] = self.metrics.narrow_gap_steps
        info["left_min_obstacle_distance"] = self.metrics.left_min_obstacle_distance
        info["right_min_obstacle_distance"] = self.metrics.right_min_obstacle_distance
        info["waypoints_reached"] = self.metrics.waypoints_reached
        info["waypoint_count"] = len(self._waypoint_goals)
        return obs, reward, done, info

    def _build_observation(self) -> Tuple[np.ndarray, Dict]:
        if self.latest_scan is None or self.latest_odom is None:
            size = int(self.state_cfg.get("downsampled_lidar_size", 36))
            return np.zeros(size + 4, dtype=np.float32), {"DG": math.inf, "DO": math.inf, "AO": 0.0, "MA": 0.0}

        min_range = float(self.state_cfg.get("lidar_min_range", 0.12))
        max_range = float(self.state_cfg.get("lidar_max_range", 3.5))
        lidar = build_lidar_feature(
            self.latest_scan.ranges,
            min_range,
            max_range,
            int(self.state_cfg.get("downsampled_lidar_size", 36)),
            bool(self.state_cfg.get("normalize_lidar", True)),
        )
        cleaned = clean_lidar_ranges(self.latest_scan.ranges, min_range, max_range)
        # Engineering adaptation: ignore the exact scanner minimum range because Gazebo's ray sensor
        # can report 0.12 m from the robot's own body at startup and during sharp turns.
        valid_indices = np.where(cleaned > (min_range + 1e-3))[0]
        if valid_indices.size:
            nearest_valid_index = int(valid_indices[int(np.argmin(cleaned[valid_indices]))])
            DO = float(cleaned[nearest_valid_index])
            AO = self._normalize_angle(self.latest_scan.angle_min + nearest_valid_index * self.latest_scan.angle_increment)
        else:
            DO, AO = compute_nearest_obstacle(
                cleaned,
                self.latest_scan.angle_min,
                self.latest_scan.angle_increment,
            )

        pose = self.latest_odom.pose.pose
        x = pose.position.x
        y = pose.position.y
        yaw = quaternion_to_yaw(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        DG, MA = goal_relative_geometry(x, y, yaw, float(self.goal["x"]), float(self.goal["y"]))
        features = np.asarray([DG, DO, AO, MA], dtype=np.float32)
        left_min = self._sector_min(cleaned, 0.35, 1.57)
        right_min = self._sector_min(cleaned, -1.57, -0.35)
        front_min = self._sector_min(cleaned, -0.35, 0.35)
        return np.concatenate([lidar, features]).astype(np.float32), {
            "DG": DG,
            "DO": DO,
            "AO": AO,
            "MA": MA,
            "x": x,
            "y": y,
            "left_min": left_min,
            "right_min": right_min,
            "front_min": front_min,
        }

    def _sector_min(self, cleaned: np.ndarray, min_angle: float, max_angle: float) -> float:
        if self.latest_scan is None:
            return math.inf
        min_range = float(self.state_cfg.get("lidar_min_range", 0.12))
        max_range = float(self.state_cfg.get("lidar_max_range", 3.5))
        angles = self._normalize_angles(self.latest_scan.angle_min + np.arange(len(cleaned)) * self.latest_scan.angle_increment)
        mask = (angles >= min_angle) & (angles <= max_angle) & (cleaned > (min_range + 1e-3))
        if not np.any(mask):
            return max_range
        return float(np.min(cleaned[mask]))

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def _normalize_angles(angles: np.ndarray) -> np.ndarray:
        return np.arctan2(np.sin(angles), np.cos(angles))

    def _compute_reward(self, info: Dict) -> Tuple[float, bool, str]:
        DG = float(info["DG"])
        DO = float(info["DO"])
        MA = float(info["MA"])
        goal_tolerance = float((self.current_stage or {}).get("goal_tolerance", self.reward_cfg.get("goal_tolerance", 0.2)))
        collision_distance = float(self.reward_cfg.get("collision_distance", 0.12))
        if DG <= goal_tolerance:
            return float(self.reward_cfg.get("goal_reward_R0", 1000.0)), True, "goal"
        if compute_collision_condition(DO, collision_distance):
            return float(self.reward_cfg.get("collision_penalty_R4", -800.0)), True, "collision"

        previous_DG = self.metrics.previous_DG if self.metrics.previous_DG is not None else DG
        eps = float(self.reward_cfg.get("eps", 1e-6))
        R1 = float(self.reward_cfg.get("lambda_heading", 4.0)) * (math.pi / 3.0 - abs(MA))
        R2 = float(self.reward_cfg.get("lambda_distance", math.e)) * (previous_DG / max(DG, eps) - 1.0)
        R3 = float(self.reward_cfg.get("lambda_obstacle", -5.0)) if DO < float(self.reward_cfg.get("min_obstacle_distance", 0.25)) else 0.0
        reward = R1 * R2 + R3

        # ROS/Gazebo curriculum adaptations. These are deliberately separate from the
        # paper reward terms above so we can disable or retune them stage by stage.
        if self.reward_cfg.get("use_goal_progress_reward", False):
            reward += float(self.reward_cfg.get("goal_progress_reward_weight", 0.0)) * (previous_DG - DG)

        if self.reward_cfg.get("use_near_goal_bonus", False):
            near_goal_distance = float(self.reward_cfg.get("near_goal_distance", 0.65))
            if near_goal_distance > 0.0 and DG < near_goal_distance:
                reward += float(self.reward_cfg.get("near_goal_bonus_weight", 0.0)) * (
                    (near_goal_distance - DG) / near_goal_distance
                )

        if self.reward_cfg.get("use_clearance_penalty", False):
            unsafe_clearance = float(self.reward_cfg.get("unsafe_clearance_distance", 0.22))
            clearance_weight = float(self.reward_cfg.get("clearance_penalty_weight", 4.0))
            if DO < unsafe_clearance:
                reward -= clearance_weight * ((unsafe_clearance - DO) / max(unsafe_clearance, 1e-6))

        if self.reward_cfg.get("use_narrow_gap_centering_reward", False):
            left_min = float(info.get("left_min", math.inf))
            right_min = float(info.get("right_min", math.inf))
            gap_clearance = float(self.reward_cfg.get("narrow_gap_clearance", 0.45))
            if left_min < gap_clearance and right_min < gap_clearance:
                balance = abs(left_min - right_min)
                reward += float(self.reward_cfg.get("narrow_gap_centering_weight", 1.0)) * max(0.0, 1.0 - balance / gap_clearance)
                if previous_DG > DG:
                    reward += float(self.reward_cfg.get("narrow_gap_progress_weight", 2.0)) * (previous_DG - DG)

        if self.reward_cfg.get("use_action_smoothness_penalty", False) and self.metrics.previous_action is not None:
            previous_linear, previous_angular = self.metrics.previous_action
            current_linear, current_angular = self._last_action or (0.0, 0.0)
            action_delta = abs(current_linear - previous_linear) + abs(current_angular - previous_angular)
            reward -= float(self.reward_cfg.get("smoothness_penalty_weight", 0.01)) * action_delta

        if self.reward_cfg.get("use_step_penalty", False):
            reward += float(self.reward_cfg.get("step_penalty", -0.01))
        return float(reward), False, ""

    def _update_metrics(self, info: Dict) -> None:
        self.metrics.steps += 1
        self.metrics.min_obstacle_distance = min(self.metrics.min_obstacle_distance, float(info["DO"]))
        self.metrics.left_min_obstacle_distance = min(
            self.metrics.left_min_obstacle_distance,
            float(info.get("left_min", math.inf)),
        )
        self.metrics.right_min_obstacle_distance = min(
            self.metrics.right_min_obstacle_distance,
            float(info.get("right_min", math.inf)),
        )
        close_threshold = float((self.current_stage or {}).get("exposure_distance", self.reward_cfg.get("min_obstacle_distance", 0.25)))
        if float(info["DO"]) <= close_threshold:
            self.metrics.close_obstacle_steps += 1
        gap_clearance = float((self.current_stage or {}).get("narrow_gap_clearance", self.reward_cfg.get("narrow_gap_clearance", 0.45)))
        if float(info.get("left_min", math.inf)) < gap_clearance and float(info.get("right_min", math.inf)) < gap_clearance:
            self.metrics.narrow_gap_steps += 1
        self.metrics.final_DG = float(info["DG"])
        self.metrics.min_DG = min(self.metrics.min_DG, float(info["DG"]))
        position = (float(info.get("x", 0.0)), float(info.get("y", 0.0)))
        if self.metrics.previous_position is not None:
            dx = position[0] - self.metrics.previous_position[0]
            dy = position[1] - self.metrics.previous_position[1]
            self.metrics.path_length += math.hypot(dx, dy)
        self.metrics.previous_position = position
        self.metrics.previous_DG = float(info["DG"])
        self.metrics.previous_action = self._last_action

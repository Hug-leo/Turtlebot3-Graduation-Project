#!/usr/bin/env python3
"""
OpenTCS Publisher
- Better theta estimation with path prediction
- Smoothing filter for noisy data
- Error handling and retry logic
- Performance monitoring
"""

import math
import rclpy
from rclpy.node import Node
import requests
import xml.etree.ElementTree as ET
import os
import json
from std_msgs.msg import String
from collections import deque
import time


class OpenTCSPublisher(Node):
    def __init__(self):
        super().__init__("opentcs_publisher_node")

        # Map loading
        map_file = "/home/hug/turtlebot3_ws/opentcs-6.3.0-bin/opentcs-modeleditor/data/Demo-01.xml"
        self.points_map, self.paths = self.load_map_with_paths(map_file)

        # ROS2 publisher
        self.cmd_pub = self.create_publisher(String, "/opentcs/vehicle_command", 10)

        # State tracking
        self.robot_state = {}
        self.last_vehicle_state = {}
        self.order_active = {}

        # Performance monitoring
        self.msg_count = 0
        self.error_count = 0
        self.last_stats_time = time.time()

        # Configurable parameters
        self.declare_parameter("poll_rate", 5.0)
        self.declare_parameter("history_size", 5)
        self.declare_parameter("theta_smoothing", 0.3)

        poll_rate = self.get_parameter("poll_rate").value
        self.history_size = self.get_parameter("history_size").value
        self.theta_alpha = self.get_parameter("theta_smoothing").value

        # Timer with configurable rate
        self.timer = self.create_timer(1.0 / poll_rate, self.timer_callback)

        # Statistics timer
        self.stats_timer = self.create_timer(10.0, self.print_statistics)

        self.get_logger().info("=" * 70)
        self.get_logger().info("OpenTCS Publisher Started")
        self.get_logger().info(f"   Poll rate: {poll_rate} Hz")
        self.get_logger().info(f"   Points loaded: {len(self.points_map)}")
        self.get_logger().info(f"   Paths loaded: {len(self.paths)}")
        self.get_logger().info("=" * 70)

    def load_map_with_paths(self, map_file):
        """Load points AND paths for better theta prediction"""
        points = {}
        paths = {}

        if not os.path.exists(map_file):
            self.get_logger().error(f"Map file not found: {map_file}")
            return points, paths

        try:
            tree = ET.parse(map_file)
            root = tree.getroot()

            # Load points
            for point in root.findall(".//point"):
                name = point.get("name")
                x = float(point.get("positionX", "0")) * 0.001
                y = float(point.get("positionY", "0")) * 0.001
                points[name] = (x, y)

            # Load paths for theta prediction
            for path in root.findall(".//path"):
                src = path.get("sourcePoint")
                dst = path.get("destinationPoint")

                if src in points and dst in points:
                    x1, y1 = points[src]
                    x2, y2 = points[dst]
                    theta = math.atan2(y2 - y1, x2 - x1)
                    paths[(src, dst)] = theta

            self.get_logger().info(f"Loaded {len(points)} points, {len(paths)} paths")

        except Exception as e:
            self.get_logger().error(f"Error parsing map: {e}")

        return points, paths

    def compute_theta_smart(self, vehicle, curr_point, prev_point=None):
        """
        Theta computation:
        1. Use path data if available
        2. Use position history with weighted average
        3. Apply smoothing filter
        """
        if vehicle not in self.robot_state:
            return None

        state = self.robot_state[vehicle]
        curr_pos = self.points_map.get(curr_point)

        if not curr_pos:
            return state.get("theta")

        x_curr, y_curr = curr_pos

        # Method 1: Use predefined path angle
        if prev_point and (prev_point, curr_point) in self.paths:
            path_theta = self.paths[(prev_point, curr_point)]
            return self.smooth_theta(state, path_theta)

        # Method 2: Use position history (weighted average)
        history = state.get("history", deque(maxlen=self.history_size))

        if len(history) >= 2:
            dx_total = 0.0
            dy_total = 0.0
            weight_total = 0.0

            for i in range(len(history) - 1):
                x1, y1 = history[i]
                x2, y2 = history[i + 1]
                dx = x2 - x1
                dy = y2 - y1

                # More recent movements have higher weight
                weight = (i + 1) / len(history)
                dx_total += dx * weight
                dy_total += dy * weight
                weight_total += weight

            if weight_total > 0 and (dx_total**2 + dy_total**2) > 0.001:
                theta = math.atan2(dy_total / weight_total, dx_total / weight_total)
                return self.smooth_theta(state, theta)

        # Method 3: Simple computation from last two positions
        if len(history) >= 1:
            x_prev, y_prev = history[-1]
            dx = x_curr - x_prev
            dy = y_curr - y_prev

            if dx**2 + dy**2 > 0.0001:
                theta = math.atan2(dy, dx)
                return self.smooth_theta(state, theta)

        return state.get("theta")

    def smooth_theta(self, state, new_theta):
        """Low-pass filter for theta to reduce noise"""
        prev_theta = state.get("theta")

        if prev_theta is None:
            return new_theta

        # Handle angle wrapping
        diff = self.normalize_angle(new_theta - prev_theta)

        # Exponential smoothing
        smoothed = prev_theta + self.theta_alpha * diff

        return self.normalize_angle(smoothed)

    @staticmethod
    def normalize_angle(angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def publish_command(self, vehicle, state, x, y, theta, point):
        """Publish command with timestamp"""
        msg = {
            "vehicle": vehicle,
            "state": state,
            "x": x,
            "y": y,
            "theta": theta,
            "point": point,
            "timestamp": time.time(),
        }

        ros_msg = String()
        ros_msg.data = json.dumps(msg)
        self.cmd_pub.publish(ros_msg)

        self.msg_count += 1

    def timer_callback(self):
        try:
            url = "http://localhost:55200/v1/vehicles"
            response = requests.get(url, timeout=1.0)

            if response.status_code != 200:
                self.error_count += 1
                return

            vehicles = response.json()

            # Cleanup inactive vehicles
            active_names = {v.get("name") for v in vehicles}
            for name in list(self.robot_state.keys()):
                if name not in active_names:
                    self.robot_state.pop(name, None)
                    self.order_active.pop(name, None)

            self.process_vehicles(vehicles)

        except requests.exceptions.Timeout:
            self.error_count += 1
        except requests.exceptions.RequestException as e:
            self.error_count += 1
            if self.error_count % 10 == 1:
                self.get_logger().warn(f"Connection error: {e}")
        except Exception as e:
            self.error_count += 1
            self.get_logger().error(f"Unexpected error: {e}")

    def process_vehicles(self, vehicles):
        """Process vehicle updates with improved logic"""
        for v in vehicles:
            name = v.get("name")
            opentcs_state = v.get("procState") or v.get("state")
            point_name = v.get("currentPosition")

            if point_name not in self.points_map:
                continue

            x, y = self.points_map[point_name]
            moving = opentcs_state == "PROCESSING_ORDER"
            was_moving = self.order_active.get(name, False)

            # Initialize state if new vehicle
            if name not in self.robot_state:
                self.robot_state[name] = {
                    "x": x,
                    "y": y,
                    "theta": None,
                    "history": deque([(x, y)], maxlen=self.history_size),
                    "prev_point": point_name,
                }

            state = self.robot_state[name]
            prev_point = state.get("prev_point")

            # START OF MOVEMENT
            if not was_moving and moving:
                theta = self.compute_theta_smart(name, point_name, prev_point)

                state["x"] = x
                state["y"] = y
                state["theta"] = theta
                state["history"].append((x, y))
                state["prev_point"] = point_name

                self.order_active[name] = True

                self.get_logger().info(
                    f"{name} | START | "
                    f"pos=({x:.2f}, {y:.2f}) | "
                    f"theta={math.degrees(theta):.1f} deg | "
                    f"point={point_name}"
                )

                self.publish_command(name, "START", x, y, theta, point_name)
                continue

            # PROCESSING
            if moving:
                prev_x = state["x"]
                prev_y = state["y"]

                # Only update if position actually changed
                if abs(x - prev_x) > 0.001 or abs(y - prev_y) > 0.001:
                    theta = self.compute_theta_smart(name, point_name, prev_point)

                    state["x"] = x
                    state["y"] = y
                    state["theta"] = theta
                    state["history"].append((x, y))
                    state["prev_point"] = point_name

                    distance = math.sqrt((x - prev_x) ** 2 + (y - prev_y) ** 2)

                    self.get_logger().info(
                        f"{name} | PROCESSING | "
                        f"pos=({x:.2f}, {y:.2f}) | "
                        f"theta={math.degrees(theta):.1f} deg | "
                        f"delta={distance:.3f}m | "
                        f"point={point_name}"
                    )

                    self.publish_command(name, "PROCESSING", x, y, theta, point_name)
                continue

            # END OF MOVEMENT
            if was_moving and not moving:
                theta = state.get("theta")

                state["x"] = x
                state["y"] = y
                state["prev_point"] = point_name

                self.order_active[name] = False

                self.get_logger().info(
                    f"{name} | END | "
                    f"pos=({x:.2f}, {y:.2f}) | "
                    f"theta={math.degrees(theta):.1f} deg | "
                    f"point={point_name}"
                )

                self.publish_command(name, "END", x, y, theta, point_name)

    def print_statistics(self):
        """Print performance statistics"""
        current_time = time.time()
        elapsed = current_time - self.last_stats_time

        if elapsed > 0:
            msg_rate = self.msg_count / elapsed
            error_rate = self.error_count / elapsed

            self.get_logger().info("-" * 70)
            self.get_logger().info(f"STATISTICS:")
            self.get_logger().info(
                f"   Messages sent: {self.msg_count} ({msg_rate:.1f} msg/s)"
            )
            self.get_logger().info(
                f"   Errors: {self.error_count} ({error_rate:.1f} err/s)"
            )
            self.get_logger().info(f"   Active vehicles: {len(self.order_active)}")
            self.get_logger().info("-" * 70)

        # Reset counters
        self.msg_count = 0
        self.error_count = 0
        self.last_stats_time = current_time


def main(args=None):
    rclpy.init(args=args)
    node = OpenTCSPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

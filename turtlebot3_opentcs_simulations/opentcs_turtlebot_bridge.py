#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import requests
import xml.etree.ElementTree as ET
import math
import time
import os
import threading


class OpenTCSBridge(Node):
    def __init__(self):
        super().__init__("opentcs_bridge")

        # ROS2 interface
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )

        # Robot state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.odom_received = False
        self.odom_lock = threading.Lock()

        # Control parameters
        self.linear_speed = 0.22
        self.angular_speed = 0.8  # Increased for faster rotation
        self.distance_tolerance = 0.35
        self.angle_tolerance = 0.25

        # OpenTCS config
        self.opentcs_url = "http://localhost:55200/v1/vehicles"
        self.vehicle_name = "Vehicle-01"  # CHANGE THIS IF NEEDED

        # CRITICAL: Adjust this scale factor!
        # If your map is 30000mm x 20000mm in OpenTCS but should be 3m x 2m in Gazebo:
        # Use scale = 0.0001 (divide by 10000)
        # If your map should be 30m x 20m, use scale = 0.001 (divide by 1000)
        self.coordinate_scale = 0.0001  # ⚠ ADJUST THIS!

        # Load map
        map_file = os.path.expanduser(
            "~/Desktop/opentcs-6.3.0-bin/opentcs-modeleditor/data/Demo-01.xml"
        )
        self.points_map = self.load_points_from_map(map_file)

        # State
        self.last_position = None
        self.is_moving = False

        # Start monitoring
        self.timer = self.create_timer(2.0, self.check_opentcs)

        self.get_logger().info("=" * 70)
        self.get_logger().info(" OpenTCS → TurtleBot3 Bridge STARTED")
        self.get_logger().info(f"   Vehicle: {self.vehicle_name}")
        self.get_logger().info(f"   Scale: {self.coordinate_scale}")
        self.get_logger().info(f"   Points loaded: {len(self.points_map)}")
        self.get_logger().info("=" * 70)

    def load_points_from_map(self, map_file):
        points = {}
        if not os.path.exists(map_file):
            self.get_logger().error(f" Map file not found: {map_file}")
            return points

        try:
            tree = ET.parse(map_file)
            root = tree.getroot()

            for point in root.findall(".//point"):
                name = point.get("name")
                x = float(point.get("positionX", "0")) * self.coordinate_scale
                y = float(point.get("positionY", "0")) * self.coordinate_scale
                points[name] = (x, y)

            # Show sample
            self.get_logger().info("Sample points (scaled to Gazebo):")
            for name, (x, y) in list(points.items())[:5]:
                self.get_logger().info(f"  {name}: ({x:.3f}, {y:.3f}) m")

        except Exception as e:
            self.get_logger().error(f"Error loading map: {e}")

        return points

    def odom_callback(self, msg):
        """CRITICAL: This updates robot position - must work!"""
        with self.odom_lock:
            self.current_x = msg.pose.pose.position.x
            self.current_y = msg.pose.pose.position.y

            q = msg.pose.pose.orientation
            _, _, self.current_theta = euler_from_quaternion([q.x, q.y, q.z, q.w])

            if not self.odom_received:
                self.odom_received = True
                self.get_logger().info(
                    f" Odometry OK: ({self.current_x:.2f}, {self.current_y:.2f})"
                )

    def check_opentcs(self):
        """Check OpenTCS for new position"""
        if self.is_moving:
            return

        try:
            resp = requests.get(self.opentcs_url, timeout=1.5)
            if resp.status_code != 200:
                return

            vehicles = resp.json()
            for v in vehicles:
                if v.get("name") == self.vehicle_name:
                    pos = v.get("currentPosition")

                    if pos and pos != self.last_position and pos in self.points_map:
                        tx, ty = self.points_map[pos]

                        with self.odom_lock:
                            cx, cy = self.current_x, self.current_y

                        dist = math.sqrt((tx - cx) ** 2 + (ty - cy) ** 2)

                        self.get_logger().info("─" * 70)
                        self.get_logger().info(f" OpenTCS → {pos}")
                        self.get_logger().info(
                            f"   From: ({cx:.2f}, {cy:.2f}) → To: ({tx:.2f}, {ty:.2f})"
                        )
                        self.get_logger().info(f"   Distance: {dist:.2f}m")

                        if dist > 50:
                            self.get_logger().error(
                                f" Target too far! Check coordinate_scale!"
                            )
                            return

                        if dist < self.distance_tolerance:
                            self.get_logger().info(" Already at target")
                            self.last_position = pos
                            return

                        # Start movement in separate thread
                        self.last_position = pos
                        threading.Thread(
                            target=self.move_to_goal, args=(tx, ty), daemon=True
                        ).start()

                    break

        except Exception as e:
            pass  # Silently ignore connection errors

    def move_to_goal(self, tx, ty):
        """Move robot to target"""
        self.is_moving = True
        self.get_logger().info(f" MOVING to ({tx:.2f}, {ty:.2f})")

        rate = self.create_rate(20)  # 20 Hz for smoother control
        timeout_iterations = 600  # 30 seconds max
        stuck_threshold = 50  # 2.5 seconds without progress

        iteration = 0
        stuck_count = 0
        last_dist = float("inf")

        while rclpy.ok() and iteration < timeout_iterations:
            with self.odom_lock:
                cx, cy, ctheta = self.current_x, self.current_y, self.current_theta

            dx = tx - cx
            dy = ty - cy
            dist = math.sqrt(dx * dx + dy * dy)

            # Stuck detection
            if abs(dist - last_dist) < 0.005:
                stuck_count += 1
                if stuck_count >= stuck_threshold:
                    self.get_logger().warn(" Robot stuck - stopping")
                    break
            else:
                stuck_count = 0
            last_dist = dist

            # Goal reached?
            if dist < self.distance_tolerance:
                self.get_logger().info(f" GOAL REACHED ({dist:.3f}m)")
                self.stop()
                self.is_moving = False
                return

            # Calculate control
            desired_angle = math.atan2(dy, dx)
            angle_diff = self.normalize_angle(desired_angle - ctheta)

            twist = Twist()

            if abs(angle_diff) > self.angle_tolerance:
                # Pure rotation
                twist.angular.z = (
                    self.angular_speed if angle_diff > 0 else -self.angular_speed
                )

                if iteration % 40 == 0:
                    self.get_logger().info(
                        f"  ↻ Angle: {math.degrees(angle_diff):.1f}°"
                    )
            else:
                # Move forward with correction
                speed = min(self.linear_speed, dist)
                twist.linear.x = speed
                twist.angular.z = 0.5 * angle_diff

                if iteration % 40 == 0:
                    self.get_logger().info(f"  → Dist: {dist:.2f}m")

            self.cmd_vel_pub.publish(twist)
            rate.sleep()
            iteration += 1

        self.stop()
        self.is_moving = False

    def stop(self):
        """Emergency stop"""
        twist = Twist()
        for _ in range(10):
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.02)

    def normalize_angle(self, a):
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a


def main():
    rclpy.init()
    bridge = OpenTCSBridge()

    # Wait for odometry
    timeout = 5
    start = time.time()
    while not bridge.odom_received and (time.time() - start) < timeout:
        rclpy.spin_once(bridge, timeout_sec=0.1)

    if not bridge.odom_received:
        bridge.get_logger().error(" No odometry! Is Gazebo running?")
        return

    bridge.get_logger().info(" Ready! Send vehicle to a point in OpenTCS...\n")

    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        bridge.stop()
    finally:
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

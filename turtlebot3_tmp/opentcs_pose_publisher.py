#!/usr/bin/env python3
"""
OpenTCS Pose Publisher - Reads position AND orientation from OpenTCS
Publishes to ROS2 topic /target_pose
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
import requests
import xml.etree.ElementTree as ET
import os
import math


class OpenTCSPosePublisher(Node):
    def __init__(self):
        super().__init__("opentcs_pose_publisher")

        # Publisher - Send pose to Raspberry Pi
        self.pose_pub = self.create_publisher(Pose2D, "/target_pose", 10)

        # OpenTCS config
        self.opentcs_url = "http://localhost:55200/v1/vehicles"
        self.vehicle_name = "Vehicle-01"

        # Coordinate scale
        self.coordinate_scale = 0.001  # mm to meters

        # Load map
        map_file = os.path.expanduser(
            "~/Desktop/opentcs-6.3.0-bin/opentcs-modeleditor/data/Demo-01.xml"
        )
        self.points_map = self.load_points_from_map(map_file)

        # State
        self.last_position = None

        # Timer - Check OpenTCS every 2 seconds
        self.timer = self.create_timer(2.0, self.check_opentcs)

        self.get_logger().info("=" * 70)
        self.get_logger().info("    OpenTCS Pose Publisher Started")
        self.get_logger().info(f"   Monitoring vehicle: {self.vehicle_name}")
        self.get_logger().info(f"   Points loaded: {len(self.points_map)}")
        self.get_logger().info(f"   Publishing to: /target_pose")
        self.get_logger().info("=" * 70)

    def load_points_from_map(self, map_file):
        """Load coordinates from XML file"""
        points = {}

        if not os.path.exists(map_file):
            self.get_logger().error(f"File not found: {map_file}")
            return points

        try:
            tree = ET.parse(map_file)
            root = tree.getroot()

            for point in root.findall(".//point"):
                name = point.get("name")
                # Convert from mm to meters
                x = float(point.get("positionX", "0")) * self.coordinate_scale
                y = float(point.get("positionY", "0")) * self.coordinate_scale
                points[name] = (x, y)

            self.get_logger().info("Sample points (first 5):")
            for name, (x, y) in list(points.items())[:5]:
                self.get_logger().info(f"  {name}: ({x:.3f}, {y:.3f}) m")

        except Exception as e:
            self.get_logger().error(f"Error loading map: {e}")

        return points

    def check_opentcs(self):
        """Check OpenTCS and publish new pose"""
        try:
            resp = requests.get(self.opentcs_url, timeout=1.5)
            if resp.status_code != 200:
                return

            vehicles = resp.json()
            for v in vehicles:
                if v.get("name") == self.vehicle_name:
                    pos = v.get("currentPosition")

                    # Get orientation angle from OpenTCS
                    orientation_angle_str = v.get("orientationAngle", "0.0")

                    # Handle "NaN" case
                    if orientation_angle_str == "NaN":
                        orientation_degrees = 0.0
                    else:
                        try:
                            orientation_degrees = float(orientation_angle_str)
                        except:
                            orientation_degrees = 0.0

                    # Convert degrees to radians
                    orientation_rad = math.radians(orientation_degrees)

                    # If position changed
                    if pos and pos != self.last_position and pos in self.points_map:
                        tx, ty = self.points_map[pos]

                        self.get_logger().info("─" * 70)
                        self.get_logger().info(f"   OpenTCS Update: Vehicle at {pos}")
                        self.get_logger().info(f"   Position: ({tx:.2f}, {ty:.2f}) m")
                        self.get_logger().info(
                            f"   Orientation: {orientation_degrees:.1f}° ({orientation_rad:.3f} rad)"
                        )

                        # Publish pose to ROS2
                        pose_msg = Pose2D()
                        pose_msg.x = tx
                        pose_msg.y = ty
                        pose_msg.theta = orientation_rad

                        self.pose_pub.publish(pose_msg)
                        self.get_logger().info(f"Sent pose to Raspberry Pi")

                        self.last_position = pos

                    break

        except Exception as e:
            pass  # Silent if OpenTCS not connected


def main(args=None):
    rclpy.init(args=args)
    node = OpenTCSPosePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

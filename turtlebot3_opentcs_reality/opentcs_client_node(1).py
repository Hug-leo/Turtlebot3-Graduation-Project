import math
import rclpy
from rclpy.node import Node
import requests
import xml.etree.ElementTree as ET
import os
import json
from std_msgs.msg import String


def compute_theta(p_prev, p_curr):
    dx = p_curr[0] - p_prev[0]
    dy = p_curr[1] - p_prev[1]
    return math.atan2(dy, dx)  # rad


class OpenTCSClient(Node):
    def __init__(self):
        super().__init__('opentcs_client_node')

        map_file = "/home/dinhsieu/opentcs-6.3.0-bin/opentcs-modeleditor/data/Demo-01.xml"
        self.points_map = self.load_points_from_map(map_file)

        self.timer = self.create_timer(2.0, self.timer_callback)

        self.robot_pose = {}          # {vehicle: {x, y, theta}}
        self.last_vehicle_state = {}  # {vehicle: opentcs_state}
        self.order_active = {}        # {vehicle: True/False}

        # ===== ROS2 publisher gửi command cho Raspberry Pi =====
        self.cmd_pub = self.create_publisher(
            String,
            '/opentcs/vehicle_command',
            10
        )

    def load_points_from_map(self, map_file):
        points = {}
        if not os.path.exists(map_file):
            self.get_logger().warn(f"Map file not found: {map_file}")
            return points

        try:
            tree = ET.parse(map_file)
            root = tree.getroot()

            for point in root.findall(".//point"):
                name = point.get("name")
                x = float(point.get("positionX", "0")) * 0.001
                y = float(point.get("positionY", "0")) * 0.001
                points[name] = (x, y)

            self.get_logger().info(
                f"Loaded {len(points)} points from map (converted to meters)."
            )
        except Exception as e:
            self.get_logger().error(f"Error parsing map: {e}")

        return points

    # ===== HÀM GỬI COMMAND QUA ROS2 =====
    def publish_command(self, vehicle, state, x, y, theta, point):
        msg = {
            "vehicle": vehicle,
            "state": state,      # START / PROCESSING / END
            "x": x,
            "y": y,
            "theta": theta,
            "point": point
        }

        ros_msg = String()
        ros_msg.data = json.dumps(msg)

        self.cmd_pub.publish(ros_msg)

    def timer_callback(self):
        try:
            url = "http://localhost:55200/v1/vehicles"
            response = requests.get(url, timeout=2)

            if response.status_code != 200:
                self.get_logger().warn(
                    f"Error {response.status_code}: {response.text}"
                )
                return

            vehicles = response.json()

            # Xoá xe không còn tồn tại
            active_names = {v.get("name") for v in vehicles}
            for name in list(self.order_active.keys()):
                if name not in active_names:
                    self.order_active.pop(name, None)
                    self.robot_pose.pop(name, None)

            self.display_vehicle_status(vehicles)

        except Exception as e:
            self.get_logger().error(f"Connection error: {e}")

    def display_vehicle_status(self, vehicles):
        for v in vehicles:
            name = v.get("name")
            opentcs_state = v.get("procState") or v.get("state")
            point_name = v.get("currentPosition")

            if point_name not in self.points_map:
                continue

            x, y = self.points_map[point_name]
            moving = (opentcs_state == "PROCESSING_ORDER")
            was_moving = self.order_active.get(name, False)

            # ===== START =====
            if not was_moving and moving:
                self.robot_pose[name] = {
                    "x": x,
                    "y": y,
                    "theta": None
                }
                self.order_active[name] = True

                self.get_logger().info(
                    f"{name} | START | x={x:.2f}, y={y:.2f}, θ=UNKNOWN | point={point_name}"
                )

                self.publish_command(
                    name, "START", x, y, None, point_name
                )
                continue

            # ===== PROCESSING =====
            if moving and name in self.robot_pose:
                prev = self.robot_pose[name]
                prev_xy = (prev["x"], prev["y"])
                curr_xy = (x, y)

                if curr_xy != prev_xy:
                    theta = compute_theta(prev_xy, curr_xy)

                    self.robot_pose[name] = {
                        "x": x,
                        "y": y,
                        "theta": theta
                    }

                    self.get_logger().info(
                        f"{name} | PROCESSING | x={x:.2f}, y={y:.2f}, θ={theta:.3f} | point={point_name}"
                    )

                    self.publish_command(
                        name, "PROCESSING", x, y, theta, point_name
                    )
                continue

            # ===== END =====
            if was_moving and not moving:
                prev = self.robot_pose.get(name, {})
                theta = prev.get("theta")

                self.order_active[name] = False

                self.get_logger().info(
                    f"{name} | END | x={x:.2f}, y={y:.2f}, θ={theta:.3f} | point={point_name}"
                )

                self.publish_command(
                    name, "END", x, y, theta, point_name
                )


def main(args=None):
    rclpy.init(args=args)
    node = OpenTCSClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

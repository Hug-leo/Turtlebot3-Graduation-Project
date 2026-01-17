#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math
import sys
import time


class MoveToPosition(Node):
    def __init__(self):
        super().__init__("move_to_position")

        # Publishers and Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )

        # Robot state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.odom_received = False

        # Control parameters
        self.linear_speed = 0.15  # Reduced speed for smoother movement
        self.angular_speed = 0.3  # Reduced rotation speed
        self.distance_tolerance = 0.15  # Slightly larger tolerance
        self.angle_tolerance = 0.15  # Slightly larger tolerance

        self.get_logger().info("MoveToPosition node initialized")

    def odom_callback(self, msg):
        # Update current position
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Extract yaw from quaternion
        orientation_q = msg.pose.pose.orientation
        orientation_list = [
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w,
        ]
        _, _, self.current_theta = euler_from_quaternion(orientation_list)

        if not self.odom_received:
            self.odom_received = True
            self.get_logger().info(
                f"Initial position: x={self.current_x:.2f}, y={self.current_y:.2f}, theta={self.current_theta:.2f}"
            )

    def wait_for_odometry(self, timeout=5.0):
        """Wait for first odometry message"""
        self.get_logger().info("Waiting for odometry data...")
        start_time = time.time()

        while not self.odom_received and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

        if not self.odom_received:
            self.get_logger().error("Timeout waiting for odometry!")
            return False

        return True

    def move_to_goal(self, target_x, target_y):
        self.get_logger().info(f"Target goal: ({target_x:.2f}, {target_y:.2f})")

        # Calculate initial distance
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        initial_distance = math.sqrt(dx**2 + dy**2)
        self.get_logger().info(f"Initial distance to goal: {initial_distance:.2f}m")

        # Create timer for control loop
        loop_rate = 10  # Hz
        dt = 1.0 / loop_rate

        max_iterations = 1000  # Prevent infinite loops
        iteration = 0

        while rclpy.ok() and iteration < max_iterations:
            # Spin once to update odometry
            rclpy.spin_once(self, timeout_sec=0.01)

            # Calculate distance and angle to goal
            dx = target_x - self.current_x
            dy = target_y - self.current_y
            distance = math.sqrt(dx**2 + dy**2)

            # Check if goal is reached
            if distance < self.distance_tolerance:
                self.get_logger().info(f"Goal reached! Final distance: {distance:.3f}m")
                self.stop_robot()
                time.sleep(0.5)  # Let the stop command take effect
                return True

            # Calculate desired angle
            desired_theta = math.atan2(dy, dx)
            angle_diff = self.normalize_angle(desired_theta - self.current_theta)

            # Create velocity command
            twist = Twist()

            # Decision logic: rotate first, then move
            if abs(angle_diff) > self.angle_tolerance:
                # Only rotate
                twist.linear.x = 0.0
                twist.angular.z = (
                    self.angular_speed if angle_diff > 0 else -self.angular_speed
                )

                if iteration % 10 == 0:  # Log every second
                    self.get_logger().info(
                        f"Rotating... angle_diff={angle_diff:.2f} rad, distance={distance:.2f}m"
                    )
            else:
                # Move forward with slight angular correction
                twist.linear.x = min(
                    self.linear_speed, distance * 0.5
                )  # Slow down as approaching
                twist.angular.z = 0.3 * angle_diff  # Proportional angular correction

                if iteration % 10 == 0:  # Log every second
                    self.get_logger().info(
                        f"Moving forward... distance={distance:.2f}m, speed={twist.linear.x:.2f}m/s"
                    )

            # Publish velocity command
            self.cmd_vel_pub.publish(twist)

            # Sleep to maintain loop rate
            time.sleep(dt)
            iteration += 1

        if iteration >= max_iterations:
            self.get_logger().warn("Maximum iterations reached!")
            self.stop_robot()
            return False

        return False

    def stop_robot(self):
        """Send zero velocity to stop the robot"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0

        # Send stop command multiple times to ensure it's received
        for _ in range(5):
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.05)

        self.get_logger().info("Robot stopped")

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


def main():
    # Parse command line arguments
    if len(sys.argv) < 3:
        print("Usage: python3 move_to_position.py <x> <y>")
        print("Example: python3 move_to_position.py 2.0 1.5")
        return

    try:
        target_x = float(sys.argv[1])
        target_y = float(sys.argv[2])
    except ValueError:
        print("Error: x and y must be numbers")
        return

    # Initialize ROS2
    rclpy.init()

    # Create node
    node = MoveToPosition()

    try:
        # Wait for odometry data
        if not node.wait_for_odometry():
            node.get_logger().error("Failed to receive odometry data")
            return

        # Move to goal
        success = node.move_to_goal(target_x, target_y)

        if success:
            node.get_logger().info("Mission completed successfully!")
        else:
            node.get_logger().warn("Mission failed or interrupted")

    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user")
        node.stop_robot()
    except Exception as e:
        node.get_logger().error(f"Error: {str(e)}")
        node.stop_robot()
    finally:
        # Cleanup
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import time


class SimpleAIBridge(Node):
    """
    A simple AI bridge that demonstrates the connection between
    an AI agent and ROS controllers.
    """

    def __init__(self):
        super().__init__('ai_bridge')

        # Publishers for ROS controllers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscribers for sensor data
        self.laser_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            10
        )

        # Timer for AI decision loop
        self.timer = self.create_timer(0.2, self.ai_decision_loop)  # 5 Hz

        # AI agent state
        self.sensor_data = None
        self.ai_state = 'idle'

        self.get_logger().info('AI-ROS bridge initialized')

    def laser_callback(self, msg):
        """Receive sensor data from the robot."""
        self.sensor_data = msg.ranges  # Store laser scan data
        self.get_logger().debug(f'Received laser scan with {len(msg.ranges)} points')

    def ai_decision_loop(self):
        """Main AI decision-making loop."""
        if self.sensor_data is None:
            return  # Wait for sensor data

        # Simple AI logic: avoid obstacles
        cmd_vel = self.simple_obstacle_avoidance(self.sensor_data)

        # Publish command to ROS controller
        self.cmd_vel_pub.publish(cmd_vel)

        self.get_logger().info(f'AI command: linear={cmd_vel.linear.x}, angular={cmd_vel.angular.z}')

    def simple_obstacle_avoidance(self, scan_data):
        """Simple AI algorithm for obstacle avoidance."""
        cmd = Twist()

        # Find minimum distance in front of robot
        front_scan = scan_data[len(scan_data)//2 - 30:len(scan_data)//2 + 30]  # Front 60 degrees
        min_distance = min(front_scan) if front_scan else float('inf')

        if min_distance < 0.5:  # Obstacle too close
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5  # Turn right
        else:
            cmd.linear.x = 0.2  # Move forward
            cmd.angular.z = 0.0

        return cmd


def main(args=None):
    rclpy.init(args=args)
    ai_bridge = SimpleAIBridge()

    try:
        rclpy.spin(ai_bridge)
    except KeyboardInterrupt:
        ai_bridge.get_logger().info('Shutting down AI bridge...')
    finally:
        ai_bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
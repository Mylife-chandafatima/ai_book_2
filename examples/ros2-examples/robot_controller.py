#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import math


class RobotController(Node):
    """
    A simple robot controller that demonstrates ROS 2 communication patterns.
    This controller receives velocity commands and publishes joint positions.
    """

    def __init__(self):
        super().__init__('robot_controller')

        # Publisher for joint positions
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        # Subscriber for velocity commands
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz

        # Robot state
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.joint_positions = [0.0, 0.0, 0.0]  # Example: 3 joints
        self.time = 0.0

        self.get_logger().info('Robot controller node initialized')

    def cmd_vel_callback(self, msg):
        """Receive velocity commands."""
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z
        self.get_logger().info(f'Received velocity commands: linear={self.linear_velocity}, angular={self.angular_velocity}')

    def control_loop(self):
        """Main control loop that updates joint positions based on velocity commands."""
        dt = 0.1  # Time step (matches timer period)

        # Simple kinematic model - integrate velocities to get positions
        self.time += dt

        # Update joint positions based on velocity commands
        # This is a simplified model - in reality, you'd have inverse kinematics
        self.joint_positions[0] += self.linear_velocity * dt * 0.1  # Scale factor
        self.joint_positions[1] += self.angular_velocity * dt * 0.2  # Scale factor
        self.joint_positions[2] = math.sin(self.time) * 0.5  # Example oscillating joint

        # Publish joint states
        joint_msg = JointState()
        joint_msg.name = ['joint_1', 'joint_2', 'joint_3']
        joint_msg.position = self.joint_positions
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.header.frame_id = 'base_link'

        self.joint_pub.publish(joint_msg)

        self.get_logger().debug(f'Published joint states: {self.joint_positions}')


def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Shutting down robot controller...')
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
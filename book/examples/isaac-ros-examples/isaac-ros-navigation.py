#!/usr/bin/env python3
"""
Isaac ROS Navigation Example for Humanoid Robot
Implements navigation stack integration with Isaac Sim and ROS 2
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan, Image, CameraInfo
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import MarkerArray
from tf2_ros import TransformBroadcaster
import numpy as np
from scipy.spatial.transform import Rotation as R
import tf2_geometry_msgs
import tf2_ros


class IsaacROSNavigationNode(Node):
    """
    Navigation node that integrates Isaac Sim perception with ROS 2 navigation stack
    """
    def __init__(self):
        super().__init__('isaac_ros_navigation_node')

        # Navigation parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_frame', 'base_link'),
                ('odom_frame', 'odom'),
                ('map_frame', 'map'),
                ('update_rate', 5.0),  # Hz
                ('linear_vel_limit', 0.5),  # m/s
                ('angular_vel_limit', 0.5),  # rad/s
                ('min_distance_to_goal', 0.1),  # meters
            ]
        )

        # Initialize navigation state
        self.current_pose = PoseStamped()
        self.current_twist = Twist()
        self.goal_pose = None
        self.path = Path()
        self.navigation_active = False

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribers
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/wheel/odometry',
            self.odometry_callback,
            10
        )

        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.goal_subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.path_publisher = self.create_publisher(
            Path,
            '/current_path',
            10
        )

        self.marker_publisher = self.create_publisher(
            MarkerArray,
            '/navigation_markers',
            10
        )

        # Navigation timer
        self.nav_timer = self.create_timer(
            1.0 / self.get_parameter('update_rate').value,
            self.navigation_control_loop
        )

        self.get_logger().info('Isaac ROS Navigation node initialized')

    def odometry_callback(self, msg):
        """
        Handle odometry updates from Isaac Sim
        """
        # Update current pose from odometry
        self.current_pose.header = msg.header
        self.current_pose.pose = msg.pose.pose

        # Update twist (velocity)
        self.current_twist = msg.twist.twist

        # Broadcast transform
        self.broadcast_transform()

    def scan_callback(self, msg):
        """
        Handle laser scan data from Isaac Sim sensors
        """
        # Process scan data for obstacle detection
        self.process_scan_data(msg)

    def goal_callback(self, msg):
        """
        Handle new navigation goal
        """
        self.goal_pose = msg
        self.navigation_active = True
        self.get_logger().info(f'New navigation goal received: [{msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}]')

    def process_scan_data(self, scan_msg):
        """
        Process laser scan data for obstacle detection and path planning
        """
        # Convert scan to obstacle points in robot frame
        angle_increment = scan_msg.angle_increment
        current_angle = scan_msg.angle_min

        obstacles = []
        for range_val in scan_msg.ranges:
            if scan_msg.range_min <= range_val <= scan_msg.range_max:
                # Convert polar to Cartesian coordinates
                x = range_val * np.cos(current_angle)
                y = range_val * np.sin(current_angle)
                obstacles.append((x, y))
            current_angle += angle_increment

        # Update obstacle markers for visualization
        self.publish_obstacle_markers(obstacles)

    def broadcast_transform(self):
        """
        Broadcast TF transforms for robot pose
        """
        from geometry_msgs.msg import TransformStamped

        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.get_parameter('odom_frame').value
        t.child_frame_id = self.get_parameter('robot_frame').value

        t.transform.translation.x = self.current_pose.pose.position.x
        t.transform.translation.y = self.current_pose.pose.position.y
        t.transform.translation.z = self.current_pose.pose.position.z

        t.transform.rotation = self.current_pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)

    def publish_obstacle_markers(self, obstacles):
        """
        Publish obstacle markers for visualization
        """
        from visualization_msgs.msg import Marker, MarkerArray
        from geometry_msgs.msg import Point

        marker_array = MarkerArray()

        for i, (x, y) in enumerate(obstacles):
            marker = Marker()
            marker.header.frame_id = self.get_parameter('robot_frame').value
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "obstacles"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1

            marker.color.a = 1.0  # Don't forget to set the alpha!
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0

            marker_array.markers.append(marker)

        self.marker_publisher.publish(marker_array)

    def navigation_control_loop(self):
        """
        Main navigation control loop
        """
        if not self.navigation_active or self.goal_pose is None:
            # Stop robot if no active navigation
            stop_cmd = Twist()
            self.cmd_vel_publisher.publish(stop_cmd)
            return

        # Calculate distance to goal
        dx = self.goal_pose.pose.position.x - self.current_pose.pose.position.x
        dy = self.goal_pose.pose.position.y - self.current_pose.pose.position.y
        distance_to_goal = np.sqrt(dx*dx + dy*dy)

        # Check if reached goal
        if distance_to_goal < self.get_parameter('min_distance_to_goal').value:
            self.get_logger().info('Reached navigation goal!')
            self.navigation_active = False
            stop_cmd = Twist()
            self.cmd_vel_publisher.publish(stop_cmd)
            return

        # Simple proportional controller for navigation
        linear_gain = 0.5
        angular_gain = 1.0

        # Calculate desired linear velocity (proportional to distance)
        desired_linear = min(linear_gain * distance_to_goal,
                            self.get_parameter('linear_vel_limit').value)

        # Calculate desired angular velocity (to face goal)
        current_yaw = self.get_yaw_from_quaternion(self.current_pose.pose.orientation)
        desired_yaw = np.arctan2(dy, dx)
        yaw_error = desired_yaw - current_yaw

        # Normalize angle to [-pi, pi]
        while yaw_error > np.pi:
            yaw_error -= 2 * np.pi
        while yaw_error < -np.pi:
            yaw_error += 2 * np.pi

        desired_angular = min(max(angular_gain * yaw_error,
                                 -self.get_parameter('angular_vel_limit').value),
                             self.get_parameter('angular_vel_limit').value)

        # Create velocity command
        cmd_vel = Twist()
        cmd_vel.linear.x = desired_linear
        cmd_vel.angular.z = desired_angular

        # Publish command
        self.cmd_vel_publisher.publish(cmd_vel)

        # Log navigation status
        self.get_logger().debug(f'Nav: dist={distance_to_goal:.2f}m, lin={desired_linear:.2f}, ang={desired_angular:.2f}')

    def get_yaw_from_quaternion(self, quaternion):
        """
        Extract yaw angle from quaternion
        """
        q = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        r = R.from_quat(q)
        euler = r.as_euler('xyz')
        return euler[2]  # Yaw is the third Euler angle

    def plan_path_to_goal(self):
        """
        Plan path to current goal using Isaac ROS path planning capabilities
        """
        if self.goal_pose is None:
            return None

        # In a real implementation, this would use Isaac ROS path planning
        # For this example, we'll create a simple straight-line path
        path = Path()
        path.header.frame_id = self.get_parameter('map_frame').value
        path.header.stamp = self.get_clock().now().to_msg()

        # Create straight-line path from current pose to goal
        steps = 10
        for i in range(steps + 1):
            ratio = i / steps

            pose_stamped = PoseStamped()
            pose_stamped.header = path.header

            pose_stamped.pose.position.x = self.current_pose.pose.position.x + \
                                          ratio * (self.goal_pose.pose.position.x - self.current_pose.pose.position.x)
            pose_stamped.pose.position.y = self.current_pose.pose.position.y + \
                                          ratio * (self.goal_pose.pose.position.y - self.current_pose.pose.position.y)
            pose_stamped.pose.position.z = self.current_pose.pose.position.z + \
                                          ratio * (self.goal_pose.pose.position.z - self.current_pose.pose.position.z)

            # For orientation, gradually rotate toward goal
            current_yaw = self.get_yaw_from_quaternion(self.current_pose.pose.orientation)
            goal_yaw = self.get_yaw_from_quaternion(self.goal_pose.pose.orientation)

            target_yaw = current_yaw + ratio * (goal_yaw - current_yaw)
            quat = self.yaw_to_quaternion(target_yaw)

            pose_stamped.pose.orientation.x = quat[0]
            pose_stamped.pose.orientation.y = quat[1]
            pose_stamped.pose.orientation.z = quat[2]
            pose_stamped.pose.orientation.w = quat[3]

            path.poses.append(pose_stamped)

        return path

    def yaw_to_quaternion(self, yaw):
        """
        Convert yaw angle to quaternion
        """
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        return [0.0, 0.0, sy, cy]


def main(args=None):
    """
    Main function to run the Isaac ROS Navigation node
    """
    rclpy.init(args=args)

    navigation_node = IsaacROSNavigationNode()

    try:
        rclpy.spin(navigation_node)
    except KeyboardInterrupt:
        navigation_node.get_logger().info('Navigation node stopped by user')
    finally:
        navigation_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
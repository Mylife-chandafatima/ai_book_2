#!/usr/bin/env python3
"""
Isaac ROS Perception Integration Example
Demonstrates integration of Isaac Sim perception with ROS 2 navigation stack
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, LaserScan
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
import numpy as np
import cv2
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation as R
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Point, Quaternion, Vector3
from sensor_msgs_py import point_cloud2
from sensor_msgs.msg import PointField


class IsaacROSPeceptionNode(Node):
    """
    Isaac ROS Perception Node
    Integrates Isaac Sim perception outputs with ROS 2 navigation stack
    """
    def __init__(self):
        super().__init__('isaac_ros_perception_node')

        # Initialize CV Bridge for image conversion
        self.cv_bridge = CvBridge()

        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Perception data storage
        self.rgb_image = None
        self.depth_image = None
        self.camera_info = None
        self.odom_data = None

        # Navigation state
        self.current_pose = PoseStamped()
        self.current_twist = Twist()
        self.navigation_goal = None
        self.path_to_goal = []

        # Perception parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_frame', 'base_link'),
                ('camera_frame', 'camera_rgb_optical_frame'),
                ('map_frame', 'map'),
                ('detection_confidence_threshold', 0.7),
                ('min_object_area', 100),
                ('tracking_iou_threshold', 0.3),
                ('depth_valid_range_min', 0.1),
                ('depth_valid_range_max', 10.0),
            ]
        )

        # Subscribers for Isaac Sim sensor data
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/rgb/image_rect_color',
            self.image_callback,
            10
        )

        self.depth_subscription = self.create_subscription(
            Image,
            '/camera/depth/image_rect_raw',
            self.depth_callback,
            10
        )

        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            '/camera/rgb/camera_info',
            self.camera_info_callback,
            10
        )

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/robot/odom',
            self.odom_callback,
            10
        )

        # Publishers for processed perception data
        self.object_detection_publisher = self.create_publisher(
            MarkerArray,
            '/perception/object_detections',
            10
        )

        self.point_cloud_publisher = self.create_publisher(
            PointCloud2,
            '/perception/processed_pointcloud',
            10
        )

        self.semantic_map_publisher = self.create_publisher(
            MarkerArray,
            '/perception/semantic_map',
            10
        )

        # Navigation command publisher
        self.nav_cmd_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Perception processing timer
        self.perception_timer = self.create_timer(
            0.1,  # 10 Hz processing
            self.process_perception_data
        )

        self.get_logger().info('Isaac ROS Perception node initialized')

    def image_callback(self, msg):
        """
        Handle RGB image data from Isaac Sim
        """
        try:
            # Convert ROS Image message to OpenCV image
            self.rgb_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.get_logger().debug(f'Received RGB image: {self.rgb_image.shape}')
        except Exception as e:
            self.get_logger().error(f'Error converting RGB image: {e}')

    def depth_callback(self, msg):
        """
        Handle depth image data from Isaac Sim
        """
        try:
            # Convert ROS Image message to OpenCV image
            self.depth_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.get_logger().debug(f'Received depth image: {self.depth_image.shape}')
        except Exception as e:
            self.get_logger().error(f'Error converting depth image: {e}')

    def camera_info_callback(self, msg):
        """
        Handle camera calibration information
        """
        self.camera_info = msg
        self.get_logger().debug('Received camera info')

    def odom_callback(self, msg):
        """
        Handle odometry data from Isaac Sim
        """
        self.current_pose.header = msg.header
        self.current_pose.pose = msg.pose.pose
        self.current_twist = msg.twist.twist
        self.get_logger().debug('Received odometry data')

    def process_perception_data(self):
        """
        Process perception data and integrate with navigation
        """
        if self.rgb_image is None or self.depth_image is None:
            return

        # Process RGB data for object detection
        detected_objects = self.detect_objects_in_image(self.rgb_image)

        # Process depth data for 3D information
        point_cloud = self.generate_point_cloud_from_depth(
            self.rgb_image, self.depth_image, self.camera_info
        )

        # Publish processed data
        self.publish_object_detections(detected_objects)
        self.publish_point_cloud(point_cloud)
        self.update_semantic_map(detected_objects)

        # Integrate perception with navigation decisions
        self.integrate_perception_navigation(detected_objects)

    def detect_objects_in_image(self, image):
        """
        Simple object detection using color-based segmentation
        In a real implementation, this would use neural networks
        """
        if image is None:
            return []

        # Convert to HSV for color-based segmentation
        hsv = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        hsv = cv2.cvtColor(hsv, cv2.COLOR_BGR2HSV)

        # Define color ranges for our known objects
        color_ranges = [
            ("red_box", np.array([0, 50, 50]), np.array([10, 255, 255])),
            ("green_sphere", np.array([50, 50, 50]), np.array([70, 255, 255])),
            ("blue_cube", np.array([110, 50, 50]), np.array([130, 255, 255]))
        ]

        detected_objects = []
        for obj_name, lower, upper in color_ranges:
            # Create mask for this color
            mask = cv2.inRange(hsv, lower, upper)

            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                area = cv2.contourArea(contour)
                if area > self.get_parameter('min_object_area').value:
                    # Get bounding box
                    x, y, w, h = cv2.boundingRect(contour)

                    # Calculate center
                    center_x = x + w / 2
                    center_y = y + h / 2

                    # Convert to normalized coordinates (0-1)
                    norm_x = center_x / image.shape[1]
                    norm_y = center_y / image.shape[0]

                    # Estimate depth at object center
                    depth_at_center = self.get_depth_at_pixel(int(center_x), int(center_y))

                    detected_objects.append({
                        'name': obj_name,
                        'bbox': [int(x), int(y), int(w), int(h)],
                        'center': [float(norm_x), float(norm_y)],
                        'area': float(area),
                        'depth': float(depth_at_center),
                        'confidence': 0.8  # Fixed confidence for this example
                    })

        return detected_objects

    def get_depth_at_pixel(self, u, v):
        """
        Get depth value at specific pixel coordinates
        """
        if self.depth_image is not None and 0 <= v < self.depth_image.shape[0] and 0 <= u < self.depth_image.shape[1]:
            return float(self.depth_image[v, u])
        return -1.0  # Invalid depth

    def generate_point_cloud_from_depth(self, rgb_image, depth_image, camera_info):
        """
        Generate 3D point cloud from RGB and depth images
        """
        if rgb_image is None or depth_image is None or camera_info is None:
            return None

        # Get camera intrinsic parameters
        fx = camera_info.k[0]  # Focal length x
        fy = camera_info.k[4]  # Focal length y
        cx = camera_info.k[2]  # Principal point x
        cy = camera_info.k[5]  # Principal point y

        height, width = depth_image.shape

        # Create coordinate grids
        u_coords, v_coords = np.meshgrid(np.arange(width), np.arange(height))

        # Convert depth image to metric distances
        z = depth_image.astype(np.float32)

        # Filter valid depth values
        valid_mask = (z > self.get_parameter('depth_valid_range_min').value) & \
                     (z < self.get_parameter('depth_valid_range_max').value)

        # Calculate 3D coordinates
        x = (u_coords[valid_mask] - cx) * z[valid_mask] / fx
        y = (v_coords[valid_mask] - cy) * z[valid_mask] / fy
        z_vals = z[valid_mask]

        # Get colors for valid points
        colors = rgb_image[valid_mask] if len(rgb_image.shape) == 3 else np.repeat(rgb_image[valid_mask][:, np.newaxis], 3, axis=1)

        # Create point cloud structure
        points = np.column_stack((x, y, z_vals))
        if len(colors) > 0:
            # Convert colors to RGB format for point cloud
            point_cloud_data = []
            for i in range(len(points)):
                point = [points[i][0], points[i][1], points[i][2]]
                if i < len(colors):
                    # Pack RGB into single float (for PointCloud2 format)
                    rgb_packed = (int(colors[i][0]) << 16) | (int(colors[i][1]) << 8) | int(colors[i][2])
                    point.append(float(rgb_packed))
                point_cloud_data.append(tuple(point))

            return point_cloud_data

        return []

    def publish_object_detections(self, detected_objects):
        """
        Publish detected objects as visualization markers
        """
        marker_array = MarkerArray()
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.get_parameter('camera_frame').value

        for i, obj in enumerate(detected_objects):
            # Create marker for bounding box
            bbox_marker = Marker()
            bbox_marker.header = header
            bbox_marker.ns = "object_detections"
            bbox_marker.id = i * 2
            bbox_marker.type = Marker.LINE_STRIP
            bbox_marker.action = Marker.ADD

            # Convert 2D bounding box to 3D points at estimated depth
            x, y, w, h = obj['bbox']
            depth = obj['depth']

            # Create corner points of bounding box in 3D
            corners_3d = self.project_2d_bbox_to_3d(x, y, w, h, depth)
            bbox_marker.points = corners_3d

            bbox_marker.scale.x = 0.02  # Line width
            bbox_marker.color.a = 1.0
            bbox_marker.color.r = 1.0  # Red for bounding box
            bbox_marker.color.g = 0.0
            bbox_marker.color.b = 0.0

            # Create text marker for object label
            text_marker = Marker()
            text_marker.header = header
            text_marker.ns = "object_labels"
            text_marker.id = i * 2 + 1
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD

            # Position text above bounding box
            text_marker.pose.position.x = corners_3d[0].x
            text_marker.pose.position.y = corners_3d[0].y
            text_marker.pose.position.z = corners_3d[0].z + 0.1  # Slightly above
            text_marker.pose.orientation.w = 1.0

            text_marker.text = f"{obj['name']}: {obj['confidence']:.2f}"
            text_marker.scale.z = 0.1  # Text size
            text_marker.color.a = 1.0
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0

            marker_array.markers.extend([bbox_marker, text_marker])

        self.object_detection_publisher.publish(marker_array)

    def project_2d_bbox_to_3d(self, x, y, w, h, depth):
        """
        Project 2D bounding box to 3D space using depth information
        """
        # This is a simplified projection - in a real implementation
        # you would use camera intrinsics to properly convert 2D to 3D
        points = []

        # Convert 2D pixel coordinates to normalized coordinates
        center_x = (x + w / 2) / 640.0  # Assuming 640px width
        center_y = (y + h / 2) / 480.0  # Assuming 480px height

        # Create approximate 3D points based on depth
        # This is a simplified approach - real implementation would use camera matrix
        approx_width = w * depth * 0.001  # Rough scaling
        approx_height = h * depth * 0.001

        # Create rectangle points at the depth
        corners = [
            (center_x - approx_width/2, center_y - approx_height/2, depth),
            (center_x + approx_width/2, center_y - approx_height/2, depth),
            (center_x + approx_width/2, center_y + approx_height/2, depth),
            (center_x - approx_width/2, center_y + approx_height/2, depth),
            (center_x - approx_width/2, center_y - approx_height/2, depth)  # Close the loop
        ]

        from geometry_msgs.msg import Point
        ros_points = []
        for px, py, pz in corners:
            pt = Point()
            pt.x = float(px)
            pt.y = float(py)
            pt.z = float(pz)
            ros_points.append(pt)

        return ros_points

    def publish_point_cloud(self, point_cloud_data):
        """
        Publish point cloud data
        """
        if point_cloud_data is None or len(point_cloud_data) == 0:
            return

        # Create PointCloud2 message
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.get_parameter('camera_frame').value

        # Define point fields (x, y, z, rgb)
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)
        ]

        # Create PointCloud2 message
        pc2_msg = point_cloud2.create_cloud(header, fields, point_cloud_data)
        self.point_cloud_publisher.publish(pc2_msg)

    def update_semantic_map(self, detected_objects):
        """
        Update semantic map based on detected objects
        """
        # In a real implementation, this would update a semantic map
        # For this example, we'll just publish markers representing detected objects
        marker_array = MarkerArray()
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.get_parameter('map_frame').value

        for i, obj in enumerate(detected_objects):
            # Transform object from camera frame to map frame
            try:
                # This would require proper transformation using TF
                # For this example, we'll just create a marker at a relative position
                marker = Marker()
                marker.header = header
                marker.ns = "semantic_map"
                marker.id = i
                marker.type = Marker.CUBE
                marker.action = Marker.ADD

                # In a real implementation, this would be transformed to map coordinates
                marker.pose.position.x = float(i * 0.5)  # Placeholder position
                marker.pose.position.y = 0.0
                marker.pose.position.z = 0.5
                marker.pose.orientation.w = 1.0

                marker.scale.x = 0.3
                marker.scale.y = 0.3
                marker.scale.z = 0.3

                marker.color.a = 0.7  # Alpha
                if "red" in obj['name']:
                    marker.color.r = 1.0
                elif "green" in obj['name']:
                    marker.color.g = 1.0
                elif "blue" in obj['name']:
                    marker.color.b = 1.0

                marker_array.markers.append(marker)
            except Exception as e:
                self.get_logger().warn(f'Could not transform object to map frame: {e}')

        self.semantic_map_publisher.publish(marker_array)

    def integrate_perception_navigation(self, detected_objects):
        """
        Integrate perception results with navigation decisions
        """
        # Analyze detected objects for navigation planning
        obstacles = [obj for obj in detected_objects if obj['confidence'] > 0.5]

        if obstacles:
            # Check if obstacles block the navigation path
            if self.navigation_goal:
                path_obstructed = self.is_path_obstructed(obstacles)
                if path_obstructed:
                    # Implement obstacle avoidance behavior
                    self.execute_obstacle_avoidance(obstacles)

    def is_path_obstructed(self, obstacles):
        """
        Check if the current navigation path is obstructed by detected obstacles
        """
        # In a real implementation, this would check the current path against obstacle positions
        # For this example, we'll just return True if there are obstacles nearby
        return len(obstacles) > 0

    def execute_obstacle_avoidance(self, obstacles):
        """
        Execute obstacle avoidance behavior
        """
        # For this example, we'll just stop the robot
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.nav_cmd_publisher.publish(cmd_vel)

        self.get_logger().info(f'Obstacle avoidance activated - {len(obstacles)} obstacles detected')


def main(args=None):
    """
    Main function to run the Isaac ROS Perception node
    """
    rclpy.init(args=args)

    perception_node = IsaacROSPeceptionNode()

    try:
        rclpy.spin(perception_node)
    except KeyboardInterrupt:
        perception_node.get_logger().info('Perception node stopped by user')
    finally:
        perception_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
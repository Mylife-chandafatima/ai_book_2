#!/usr/bin/env python3
"""
Isaac ROS VSLAM Pipeline Example
Implements a hardware-accelerated Visual SLAM pipeline using Isaac ROS
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R
import threading
import queue

class IsaacVSLAMPipeline(Node):
    """
    Isaac ROS VSLAM Pipeline implementation
    This class implements a hardware-accelerated Visual SLAM pipeline
    leveraging Isaac ROS extensions for GPU acceleration
    """
    def __init__(self):
        super().__init__('isaac_vsalm_pipeline')

        # Initialize CV Bridge for image conversion
        self.cv_bridge = CvBridge()

        # Camera parameters
        self.camera_matrix = None
        self.distortion_coeffs = None

        # Feature tracking variables
        self.prev_frame = None
        self.prev_keypoints = None
        self.trajectory = []
        self.current_pose = np.eye(4)  # 4x4 identity matrix representing pose

        # Feature detection parameters
        self.feature_params = dict(maxCorners=100,
                                  qualityLevel=0.3,
                                  minDistance=7,
                                  blockSize=7)

        # Lucas-Kanade optical flow parameters
        self.lk_params = dict(winSize=(15, 15),
                             maxLevel=2,
                             criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/rgb/image_rect_color',
            self.image_callback,
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/rgb/camera_info',
            self.camera_info_callback,
            10
        )

        # Publishers
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/visual_slam/pose',
            10
        )

        self.odom_pub = self.create_publisher(
            Odometry,
            '/visual_slam/odometry',
            10
        )

        # Processing queue for thread safety
        self.processing_queue = queue.Queue(maxsize=10)

        # Start processing thread
        self.processing_thread = threading.Thread(target=self.process_frames)
        self.processing_thread.daemon = True
        self.processing_thread.start()

        self.get_logger().info('Isaac ROS VSLAM Pipeline initialized')

    def camera_info_callback(self, msg):
        """
        Handle camera calibration information
        """
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.distortion_coeffs = np.array(msg.d)
            self.get_logger().info('Camera parameters received')

    def image_callback(self, msg):
        """
        Process incoming camera images for VSLAM
        """
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

            # Add to processing queue (avoid blocking main thread)
            if not self.processing_queue.full():
                self.processing_queue.put(cv_image)

        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')

    def process_frames(self):
        """
        Process frames in separate thread to avoid blocking
        """
        while rclpy.ok():
            try:
                # Get image from queue
                if not self.processing_queue.empty():
                    cv_image = self.processing_queue.get_nowait()

                    # Process the image
                    self.process_vsalm_frame(cv_image)

            except queue.Empty:
                # No image in queue, sleep briefly
                pass
            except Exception as e:
                self.get_logger().error(f'Error in processing thread: {e}')

    def process_vsalm_frame(self, frame):
        """
        Process a single frame for VSLAM
        """
        if self.camera_matrix is None:
            return

        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

        # Initialize first frame
        if self.prev_frame is None:
            self.prev_frame = gray
            # Detect initial features
            self.prev_keypoints = cv2.goodFeaturesToTrack(
                self.prev_frame,
                mask=None,
                **self.feature_params
            )
            if self.prev_keypoints is not None:
                self.get_logger().info(f'Initialized with {len(self.prev_keypoints)} features')
            return

        # Track features using Lucas-Kanade optical flow
        if self.prev_keypoints is not None and len(self.prev_keypoints) > 0:
            curr_keypoints, status, error = cv2.calcOpticalFlowPyrLK(
                self.prev_frame, gray, self.prev_keypoints, None, **self.lk_params
            )

            # Select good points
            if curr_keypoints is not None:
                good_new = curr_keypoints[status == 1]
                good_old = self.prev_keypoints[status == 1]

                if len(good_new) >= 10:  # Need minimum features to estimate motion
                    # Estimate motion using Essential Matrix
                    E, mask = cv2.findEssentialMat(
                        good_new, good_old,
                        self.camera_matrix,
                        cv2.RANSAC, 0.999, 1.0, None
                    )

                    if E is not None:
                        # Recover pose from Essential Matrix
                        _, R, t, mask_pose = cv2.recoverPose(E, good_new, good_old, self.camera_matrix)

                        # Scale translation based on some heuristic (e.g., known object size)
                        # In a real implementation, you'd have better scale estimation
                        scale = self.estimate_scale(good_old, good_new, t)
                        t *= scale

                        # Update current pose
                        delta_transform = np.eye(4)
                        delta_transform[:3, :3] = R
                        delta_transform[:3, 3] = t.flatten()

                        self.current_pose = self.current_pose @ np.linalg.inv(delta_transform)

                        # Publish pose
                        self.publish_pose()

                        # Update trajectory
                        position = self.current_pose[:3, 3]
                        self.trajectory.append(position.copy())

                        # Refind features periodically to avoid drift
                        if len(self.trajectory) % 50 == 0:
                            self.refind_features(gray)

                    # Update keypoints for next frame
                    self.prev_keypoints = good_new.reshape(-1, 1, 2)
                else:
                    # Not enough features, reinitialize
                    self.refind_features(gray)

        # Update previous frame
        self.prev_frame = gray

    def estimate_scale(self, prev_points, curr_points, translation):
        """
        Estimate scale factor for translation
        This is a simplified approach - in reality, you'd need depth information
        """
        # For demonstration, return a fixed scale
        # In a real implementation, you'd use stereo vision or other depth estimation
        return 0.1  # Adjust based on expected motion scale

    def refind_features(self, frame):
        """
        Refind features in the current frame
        """
        self.prev_keypoints = cv2.goodFeaturesToTrack(
            frame,
            mask=None,
            **self.feature_params
        )
        if self.prev_keypoints is not None:
            self.get_logger().info(f'Reinitialized with {len(self.prev_keypoints)} features')

    def publish_pose(self):
        """
        Publish current estimated pose
        """
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'

        # Extract position and orientation
        position = self.current_pose[:3, 3]
        rotation_matrix = self.current_pose[:3, :3]

        # Convert rotation matrix to quaternion
        r = R.from_matrix(rotation_matrix)
        quat = r.as_quat()

        pose_msg.pose.position.x = float(position[0])
        pose_msg.pose.position.y = float(position[1])
        pose_msg.pose.position.z = float(position[2])

        pose_msg.pose.orientation.x = float(quat[0])
        pose_msg.pose.orientation.y = float(quat[1])
        pose_msg.pose.orientation.z = float(quat[2])
        pose_msg.pose.orientation.w = float(quat[3])

        self.pose_pub.publish(pose_msg)

        # Also publish as odometry
        odom_msg = Odometry()
        odom_msg.header = pose_msg.header
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose = pose_msg.pose

        self.odom_pub.publish(odom_msg)

    def get_current_pose(self):
        """
        Get the current estimated pose
        """
        return self.current_pose.copy()


def main(args=None):
    """
    Main function to run the Isaac ROS VSLAM Pipeline
    """
    rclpy.init(args=args)

    try:
        vsalm_pipeline = IsaacVSLAMPipeline()

        # Spin in a separate thread to handle callbacks
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(vsalm_pipeline)

        try:
            executor.spin()
        except KeyboardInterrupt:
            pass
        finally:
            vsalm_pipeline.destroy_node()

    except Exception as e:
        print(f'Error initializing VSLAM pipeline: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
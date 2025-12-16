#!/usr/bin/env python3
"""
Humanoid Robot Perception Example for Isaac Sim
Implements perception pipeline for humanoid robot with multiple sensors
"""

import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core.sensors import Camera
from omni.isaac.range_sensor import LidarRtx
from omni.isaac.core.objects import DynamicCuboid, DynamicSphere
from omni.isaac.core.materials import OmniPBRMaterial
import numpy as np
import cv2
from PIL import Image
import carb
import argparse
import os
import json


class HumanoidPerception:
    """
    Humanoid Robot Perception System for Isaac Sim
    Implements multi-modal perception for humanoid robots using Isaac Sim sensors
    """
    def __init__(self):
        # Initialize Isaac Sim world
        self.world = World(stage_units_in_meters=1.0)
        self.scene = self.world.scene

        # Perception components
        self.cameras = []
        self.lidars = []
        self.rgb_data = None
        self.depth_data = None
        self.semantic_seg_data = None
        self.lidar_data = None

        # Processing parameters
        self.detection_threshold = 0.5
        self.tracking_iou_threshold = 0.3

        print("✅ Humanoid Perception System initialized")

    def setup_environment(self):
        """
        Set up the simulation environment with humanoid robot and objects
        """
        # Add default ground plane
        self.scene.add_default_ground_plane()

        # Get assets root path
        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets root path")
            return False

        # Add humanoid robot model (using a simple representation for this example)
        # In a real implementation, you would load a humanoid robot model
        self.robot = self.scene.add(
            DynamicCuboid(
                prim_path="/World/HumanoidRobot",
                name="humanoid_robot",
                position=np.array([0.0, 0.0, 0.5]),
                size=0.3,
                color=np.array([0.5, 0.5, 0.5])  # Gray for humanoid
            )
        )

        # Add objects for perception tasks
        self.objects = []
        object_configs = [
            {"name": "red_cube", "pos": [1.0, 0.5, 0.2], "color": [0.8, 0.1, 0.1]},
            {"name": "blue_cube", "pos": [-0.8, -0.5, 0.3], "color": [0.1, 0.1, 0.8]},
            {"name": "green_sphere", "pos": [0.5, -1.0, 0.4], "color": [0.1, 0.8, 0.1]},
            {"name": "yellow_cube", "pos": [-0.3, 0.8, 0.25], "color": [0.8, 0.8, 0.1]}
        ]

        for config in object_configs:
            if "sphere" in config["name"]:
                obj = self.scene.add(
                    DynamicSphere(
                        prim_path=f"/World/{config['name']}",
                        name=config['name'],
                        position=np.array(config['pos']),
                        radius=0.15,
                        color=np.array(config['color'])
                    )
                )
            else:
                obj = self.scene.add(
                    DynamicCuboid(
                        prim_path=f"/World/{config['name']}",
                        name=config['name'],
                        position=np.array(config['pos']),
                        size=0.3,
                        color=np.array(config['color'])
                    )
                )
            self.objects.append(obj)

        print(f"✅ Environment setup complete with {len(self.objects) + 1} objects")
        return True

    def setup_sensors(self):
        """
        Set up perception sensors for the humanoid robot
        """
        # Create RGB-D camera mounted on the robot
        camera = self.scene.add(
            Camera(
                prim_path="/World/HumanoidRobot/Camera",
                name="humanoid_camera",
                position=np.array([0.2, 0.0, 0.1]),  # Slightly in front and above
                frequency=30,  # 30 Hz
                resolution=(640, 480)
            )
        )

        # Enable RGB sensor
        camera.add_raw_image_to_frame()

        # Enable depth sensor
        camera.add_depth_to_frame()

        # Enable semantic segmentation
        camera.add_semantic_segmentation_to_frame()

        self.cameras.append(camera)
        print(f"✅ RGB-D camera added: {len(self.cameras)} cameras active")

        # Set up LiDAR sensor for 3D perception
        try:
            from omni.isaac.sensor import RotatingLidarPhysX
            lidar = self.scene.add(
                RotatingLidarPhysX(
                    prim_path="/World/HumanoidRobot/Lidar",
                    name="humanoid_lidar",
                    translation=np.array([0.0, 0.0, 0.3]),  # Above the robot
                    config="Example_Rotary_Lidar",
                    depth_range=10,
                    frequency=10,
                    horizontal_resolution=0.5,
                    vertical_resolution=0.5,
                    horizontal_lasers=320,
                    vertical_lasers=16
                )
            )

            self.lidars.append(lidar)
            print(f"✅ LiDAR sensor added: {len(self.lidars)} lidars active")
        except Exception as e:
            print(f"⚠️  LiDAR setup failed (optional sensor): {e}")

        return True

    def capture_sensor_data(self):
        """
        Capture data from all perception sensors
        """
        # Step the world to update sensors
        self.world.step(render=True)

        # Capture camera data
        if self.cameras:
            camera = self.cameras[0]  # Use first camera for this example

            # Get RGB image
            try:
                rgb_data = camera.get_raw_image_data()
                if rgb_data is not None:
                    self.rgb_data = rgb_data
            except Exception as e:
                carb.log_warn(f"Could not get RGB data: {e}")

            # Get depth data
            try:
                depth_data = camera.get_depth_data()
                if depth_data is not None:
                    self.depth_data = depth_data
            except Exception as e:
                carb.log_warn(f"Could not get depth data: {e}")

            # Get semantic segmentation data
            try:
                seg_data = camera.get_semantic_segmentation_data()
                if seg_data is not None:
                    self.semantic_seg_data = seg_data
            except Exception as e:
                carb.log_warn(f"Could not get semantic segmentation data: {e}")

        # Capture LiDAR data if available
        if self.lidars:
            lidar = self.lidars[0]
            try:
                lidar_data = lidar.get_measured_flatrange_data()
                if lidar_data is not None:
                    self.lidar_data = lidar_data
            except Exception as e:
                carb.log_warn(f"Could not get LiDAR data: {e}")

    def process_perception_data(self):
        """
        Process captured perception data to extract meaningful information
        """
        results = {}

        # Process RGB data for object detection
        if self.rgb_data is not None:
            objects = self.detect_objects_in_image(self.rgb_data)
            results['detected_objects'] = objects

        # Process depth data for 3D reconstruction
        if self.depth_data is not None:
            point_cloud = self.generate_point_cloud(self.depth_data)
            results['point_cloud'] = point_cloud

        # Process semantic segmentation for scene understanding
        if self.semantic_seg_data is not None:
            semantic_analysis = self.analyze_semantic_data(self.semantic_seg_data)
            results['semantic_analysis'] = semantic_analysis

        # Process LiDAR data for obstacle detection
        if self.lidar_data is not None:
            obstacles = self.detect_obstacles_lidar(self.lidar_data)
            results['lidar_obstacles'] = obstacles

        return results

    def detect_objects_in_image(self, image_data):
        """
        Simple object detection in RGB image using color-based segmentation
        In a real implementation, this would use neural networks
        """
        if isinstance(image_data, np.ndarray) and len(image_data.shape) == 3:
            # Convert to OpenCV format if needed
            if image_data.dtype == np.uint8:
                img_bgr = cv2.cvtColor(image_data, cv2.COLOR_RGB2BGR)
            else:
                # Normalize float image to 0-255 range
                img_normalized = np.clip(image_data * 255, 0, 255).astype(np.uint8)
                img_bgr = cv2.cvtColor(img_normalized, cv2.COLOR_RGB2BGR)

            # Convert to HSV for color-based segmentation
            hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)

            # Define color ranges for our objects
            color_ranges = [
                ("red_box", np.array([0, 50, 50]), np.array([10, 255, 255])),      # Red objects
                ("green_sphere", np.array([50, 50, 50]), np.array([70, 255, 255])), # Green objects
                ("blue_box", np.array([110, 50, 50]), np.array([130, 255, 255]))   # Blue objects
            ]

            detected_objects = []
            for obj_name, lower, upper in color_ranges:
                # Create mask for this color
                mask = cv2.inRange(hsv, lower, upper)

                # Find contours
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                for contour in contours:
                    area = cv2.contourArea(contour)
                    if area > 100:  # Minimum area threshold
                        # Get bounding box
                        x, y, w, h = cv2.boundingRect(contour)

                        # Calculate center
                        center_x = x + w / 2
                        center_y = y + h / 2

                        # Convert to normalized coordinates (0-1)
                        norm_x = center_x / image_data.shape[1]
                        norm_y = center_y / image_data.shape[0]

                        detected_objects.append({
                            'name': obj_name,
                            'bbox': [int(x), int(y), int(w), int(h)],
                            'center': [float(norm_x), float(norm_y)],
                            'area': float(area),
                            'confidence': 0.8  # Fixed confidence for this example
                        })

            return detected_objects

        return []

    def generate_point_cloud(self, depth_data):
        """
        Generate 3D point cloud from depth data
        """
        if depth_data is None:
            return None

        # For this example, we'll create a simple point cloud
        # In a real implementation, this would use camera intrinsics
        height, width = depth_data.shape if len(depth_data.shape) == 2 else (depth_data.shape[0], depth_data.shape[1])

        # Create meshgrid for pixel coordinates
        u_coords, v_coords = np.meshgrid(np.arange(width), np.arange(height))

        # This is a simplified example - real implementation would use camera intrinsics
        points = []
        for u in range(0, width, 10):  # Downsample for performance
            for v in range(0, height, 10):
                z = depth_data[v, u]
                if z > 0 and z < 10:  # Valid depth range
                    # Convert pixel coordinates to 3D (simplified)
                    x = (u - width/2) * z * 0.001  # Rough conversion
                    y = (v - height/2) * z * 0.001
                    points.append([x, y, z])

        return np.array(points)

    def analyze_semantic_data(self, semantic_data):
        """
        Analyze semantic segmentation data for scene understanding
        """
        if semantic_data is None:
            return {}

        # In a real implementation, this would analyze the semantic labels
        # For this example, we'll just return basic statistics
        unique_labels = np.unique(semantic_data)

        analysis = {
            'unique_labels': int(len(unique_labels)),
            'label_distribution': {},
            'dominant_label': int(np.bincount(semantic_data.flatten()).argmax()) if semantic_data.size > 0 else 0
        }

        # Count distribution of labels
        for label in unique_labels:
            count = np.sum(semantic_data == label)
            analysis['label_distribution'][int(label)] = int(count)

        return analysis

    def detect_obstacles_lidar(self, lidar_data):
        """
        Detect obstacles from LiDAR data
        """
        if lidar_data is None:
            return []

        # Process LiDAR data to detect obstacles
        # In this example, we'll look for returns closer than a threshold
        obstacle_threshold = 2.0  # meters
        obstacle_points = lidar_data[lidar_data < obstacle_threshold]

        # Calculate approximate obstacle positions/densities
        num_obstacles = len(obstacle_points)

        obstacles = []
        if num_obstacles > 0:
            avg_distance = np.mean(obstacle_points) if len(obstacle_points) > 0 else float('inf')
            obstacles.append({
                'type': 'obstacle',
                'avg_distance': float(avg_distance),
                'count': int(num_obstacles),
                'density': float(num_obstacles / len(lidar_data)) if len(lidar_data) > 0 else 0.0
            })

        return obstacles

    def run_perception_pipeline(self, num_iterations=100):
        """
        Run the complete perception pipeline for specified iterations
        """
        print(f"🚀 Starting humanoid perception pipeline for {num_iterations} iterations...")

        all_results = []
        for i in range(num_iterations):
            # Capture sensor data
            self.capture_sensor_data()

            # Process perception data
            results = self.process_perception_data()
            all_results.append(results)

            # Log results periodically
            if i % 20 == 0:
                print(f"  Iteration {i}/{num_iterations}")
                if 'detected_objects' in results:
                    print(f"    Detected {len(results['detected_objects'])} objects")
                if 'lidar_obstacles' in results:
                    print(f"    Detected {len(results['lidar_obstacles'])} obstacle regions")

        print("✅ Perception pipeline completed")
        return all_results

    def save_perception_data(self, results, output_dir="perception_output"):
        """
        Save perception results to disk
        """
        import json
        import os

        # Create output directory
        os.makedirs(output_dir, exist_ok=True)

        # Save results to JSON
        output_path = os.path.join(output_dir, "perception_results.json")
        with open(output_path, 'w') as f:
            json.dump(results, f, indent=2, default=str)

        print(f"💾 Perception results saved to {output_path}")


def main():
    """
    Main function to run the Humanoid Perception Pipeline
    """
    print("Initializing Humanoid Perception Pipeline for Isaac Sim...")

    # Initialize perception pipeline
    perception_pipeline = HumanoidPerception()

    # Setup environment
    success = perception_pipeline.setup_environment()
    if not success:
        print("❌ Failed to set up environment")
        return

    # Setup sensors
    perception_pipeline.setup_sensors()

    # Run perception pipeline
    results = perception_pipeline.run_perception_pipeline(num_iterations=50)

    # Save results
    perception_pipeline.save_perception_data(results)

    # Clean up
    perception_pipeline.world.clear()

    print("\n🎉 Humanoid Perception Pipeline completed successfully!")
    print("Results saved in 'perception_output/perception_results.json'")


if __name__ == "__main__":
    main()
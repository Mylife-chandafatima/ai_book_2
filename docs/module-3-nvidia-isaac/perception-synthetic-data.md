# Advanced Perception and Synthetic Data Generation

## Overview

This section covers advanced perception techniques and synthetic data generation using NVIDIA Isaac Sim. You'll learn how to create photorealistic scenes, simulate sensors, and generate synthetic datasets for training AI perception models.

## Key Concepts

### Perception Pipeline

A perception pipeline in Isaac Sim typically includes:

1. **Scene Setup**: Creating photorealistic environments with proper lighting
2. **Sensor Simulation**: Modeling real-world sensors (cameras, LiDAR, IMUs) in simulation
3. **Data Generation**: Producing labeled datasets for training perception models
4. **Annotation**: Providing ground truth data for supervised learning

### Synthetic Data Generation Benefits

- **Safety**: Train models without physical robot risk
- **Cost Efficiency**: Generate large datasets without expensive hardware
- **Scenario Diversity**: Create rare or dangerous scenarios safely
- **Ground Truth**: Perfect annotations available for all generated data

## Setting Up Photorealistic Scenes

### Environment Configuration

```python
# Example: Setting up a basic scene for perception training
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.carb import wait_on_queue

def setup_perception_scene():
    """
    Set up a scene optimized for perception tasks
    """
    world = World(stage_units_in_meters=1.0)

    # Add ground plane
    world.scene.add_default_ground_plane()

    # Configure lighting for photorealism
    from omni.isaac.core.utils.prims import create_prim
    create_prim("/World/Light", "SphereLight", position=[5, 5, 10], attributes={"radius": 5})

    # Add objects for perception tasks
    # This is where you'd add objects for your specific perception challenges

    return world
```

### Sensor Configuration

Isaac Sim supports various sensor types:

- **RGB Cameras**: For visual perception tasks
- **Depth Cameras**: For 3D scene understanding
- **LiDAR**: For spatial mapping and obstacle detection
- **IMU**: For motion sensing and orientation

## Synthetic Data Generation

### Basic Data Generation Pipeline

```python
# Example: Basic synthetic data generation
import numpy as np
from PIL import Image

def generate_synthetic_dataset(world, num_samples=1000):
    """
    Generate a basic synthetic dataset with RGB and depth images
    """
    rgb_images = []
    depth_images = []
    annotations = []

    for i in range(num_samples):
        # Step the simulation
        world.step(render=True)

        # Capture sensor data (this would be implemented with actual Isaac Sim sensors)
        # rgb_img = capture_rgb_image()
        # depth_img = capture_depth_image()
        # annotation = generate_annotation(rgb_img)

        # Store data
        # rgb_images.append(rgb_img)
        # depth_images.append(depth_img)
        # annotations.append(annotation)

        if i % 100 == 0:
            print(f"Generated {i}/{num_samples} samples")

    return {
        'rgb_images': rgb_images,
        'depth_images': depth_images,
        'annotations': annotations
    }
```

### Data Annotation Techniques

Synthetic data generation in Isaac Sim provides perfect ground truth:

- **Semantic Segmentation**: Pixel-perfect object classification
- **Instance Segmentation**: Individual object identification
- **Bounding Boxes**: Object localization
- **Keypoints**: Humanoid joint positions for pose estimation

## Best Practices

### Scene Diversity

To maximize the value of synthetic data:

- **Vary Lighting Conditions**: Different times of day, weather
- **Change Camera Angles**: Multiple viewpoints
- **Modify Environments**: Indoor, outdoor, urban, rural
- **Adjust Object Configurations**: Different arrangements and poses

### Quality Assurance

- **Validation**: Compare synthetic vs real data distributions
- **Fidelity Assessment**: Ensure synthetic data resembles real data
- **Model Performance**: Test trained models on real-world data

## Synthetic Data Pipeline Setup

### Basic Pipeline Architecture

The synthetic data generation pipeline in Isaac Sim consists of several key components:

1. **Scene Generator**: Creates varied environments with different lighting, textures, and object arrangements
2. **Sensor Simulator**: Emulates real-world sensors (cameras, LiDAR, IMU) in simulation
3. **Data Processor**: Formats and annotates the generated data
4. **Labeler**: Provides ground truth annotations for supervised learning

### Setting Up the Pipeline

```python
# Example: Complete synthetic data pipeline setup
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.sensor import Camera
import numpy as np
import carb
import cv2
from PIL import Image
import json
import os

class SyntheticDataPipeline:
    def __init__(self, output_dir="synthetic_dataset"):
        self.world = None
        self.cameras = []
        self.output_dir = output_dir
        self.sample_count = 0
        self.camera_matrix = None  # Initialize camera matrix

        # Create output directory
        os.makedirs(output_dir, exist_ok=True)
        os.makedirs(os.path.join(output_dir, "images"), exist_ok=True)
        os.makedirs(os.path.join(output_dir, "labels"), exist_ok=True)
        os.makedirs(os.path.join(output_dir, "metadata"), exist_ok=True)

    def setup_world(self):
        """Initialize Isaac Sim world for data generation"""
        self.world = World(stage_units_in_meters=1.0)

        # Add ground plane
        self.world.scene.add_default_ground_plane()

        # Add basic environment objects
        self.add_environment_objects()

        print("✅ World initialized for synthetic data generation")

    def add_environment_objects(self):
        """Add objects to the environment for data diversity"""
        # Add some basic objects for scene variety
        from omni.isaac.core.objects import DynamicCuboid, DynamicSphere

        # Add various colored objects
        objects = [
            ("cube_red", [1.0, 0.0, 0.5], [0.8, 0.1, 0.1]),
            ("cube_blue", [0.5, 1.0, 0.5], [0.1, 0.1, 0.8]),
            ("sphere_green", [0.0, 0.5, 1.0], [0.1, 0.8, 0.1])
        ]

        for name, pos, color in objects:
            if "cube" in name:
                self.world.scene.add(
                    DynamicCuboid(
                        prim_path=f"/World/{name}",
                        name=name,
                        position=np.array(pos),
                        size=0.3,
                        color=np.array(color)
                    )
                )
            else:
                self.world.scene.add(
                    DynamicSphere(
                        prim_path=f"/World/{name}",
                        name=name,
                        position=np.array(pos),
                        radius=0.2,
                        color=np.array(color)
                    )
                )

    def setup_cameras(self):
        """Setup cameras for synthetic data capture"""
        # Add an RGB camera
        from omni.isaac.core.utils.prims import create_prim
        from omni.isaac.core.utils.rotations import euler_angles_to_quat

        # Create camera prim
        camera_path = "/World/Camera"
        create_prim(
            prim_path=camera_path,
            prim_type="Camera",
            position=[2.0, 0.0, 1.5],
            orientation=euler_angles_to_quat(np.array([0, 0, 0]))
        )

        # Add camera to scene
        camera = self.world.scene.add(
            Camera(
                prim_path=camera_path,
                frequency=30,  # 30 Hz capture rate
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

        print(f"✅ Camera setup complete: {len(self.cameras)} cameras active")

    def capture_sample(self):
        """Capture a single sample with all sensors"""
        if not self.world:
            raise RuntimeError("World not initialized")

        # Step the simulation
        self.world.step(render=True)

        # Capture data from all cameras
        sample_data = {}
        for i, camera in enumerate(self.cameras):
            # Get RGB image
            rgb_data = camera.get_raw_image_data()
            rgb_image = Image.fromarray(rgb_data)

            # Get depth data
            depth_data = camera.get_depth_data()

            # Get semantic segmentation
            seg_data = camera.get_semantic_segmentation_data()

            # Store data
            sample_id = f"sample_{self.sample_count:06d}_cam_{i}"
            sample_data[sample_id] = {
                'rgb': rgb_image,
                'depth': depth_data,
                'segmentation': seg_data,
                'timestamp': self.world.current_time_step_index
            }

        self.sample_count += 1
        return sample_data

    def save_sample(self, sample_data):
        """Save captured sample to disk"""
        for sample_id, data in sample_data.items():
            # Save RGB image
            rgb_path = os.path.join(self.output_dir, "images", f"{sample_id}.png")
            data['rgb'].save(rgb_path)

            # Save depth data as numpy array
            depth_path = os.path.join(self.output_dir, "images", f"{sample_id}_depth.npy")
            np.save(depth_path, data['depth'])

            # Save metadata
            metadata = {
                'sample_id': sample_id,
                'timestamp': int(data['timestamp']),
                'camera_config': {
                    'resolution': data['rgb'].size,
                    'position': [2.0, 0.0, 1.5],  # From camera setup
                    'orientation': [0, 0, 0]
                }
            }

            meta_path = os.path.join(self.output_dir, "metadata", f"{sample_id}.json")
            with open(meta_path, 'w') as f:
                json.dump(metadata, f, indent=2)

    def generate_dataset(self, num_samples=100):
        """Generate a complete synthetic dataset"""
        print(f"Generating {num_samples} synthetic data samples...")

        for i in range(num_samples):
            # Move objects slightly to create variation
            self.perturb_scene()

            # Capture sample
            sample_data = self.capture_sample()

            # Save sample
            self.save_sample(sample_data)

            # Progress update
            if (i + 1) % 20 == 0:
                print(f"  Processed {i + 1}/{num_samples} samples")

        print(f"✅ Generated {self.sample_count} samples in {self.output_dir}")

    def perturb_scene(self):
        """Slightly perturb scene objects for data diversity"""
        # In a real implementation, this would randomly move objects
        # For this example, we'll just ensure the world is stepped
        pass

    def run_pipeline(self, num_samples=100):
        """Run the complete synthetic data pipeline"""
        try:
            # Setup world and cameras
            self.setup_world()
            self.setup_cameras()

            # Reset world
            self.world.reset()

            # Generate dataset
            self.generate_dataset(num_samples)

            # Clean up
            self.world.clear()

            print("✅ Synthetic data pipeline completed successfully!")
            return True

        except Exception as e:
            print(f"❌ Error in synthetic data pipeline: {e}")
            if self.world:
                self.world.clear()
            return False

def main():
    """Main function to run the synthetic data pipeline"""
    print("Initializing Synthetic Data Generation Pipeline...")

    pipeline = SyntheticDataPipeline(output_dir="synthetic_robot_dataset")

    success = pipeline.run_pipeline(num_samples=50)  # Generate 50 samples for example

    if success:
        print("\n🎉 Synthetic data generation completed!")
        print("Dataset saved in 'synthetic_robot_dataset/' directory")
        print("Directory structure:")
        print("  ├── images/           # RGB, depth images")
        print("  ├── labels/           # Annotation files")
        print("  └── metadata/         # Sample metadata")
    else:
        print("\n❌ Synthetic data generation failed.")

if __name__ == "__main__":
    main()
```

## Implementation Example

Here's a complete example of setting up a basic perception training scenario:

```python
# Complete perception training setup example
from perception_pipeline import setup_perception_scene, generate_synthetic_dataset

def main():
    # Set up the scene
    world = setup_perception_scene()

    # Generate synthetic dataset
    dataset = generate_synthetic_dataset(world, num_samples=1000)

    # Save dataset for training
    save_dataset(dataset, "synthetic_perception_dataset")

    # Clean up
    world.clear()

    print("✅ Perception training dataset generated successfully!")

if __name__ == "__main__":
    main()
```

## References

- NVIDIA Isaac Sim Documentation on Perception
- Synthetic Data Generation for Robotics
- Photorealistic Simulation for AI Training

---

Next: [VSLAM and Navigation with Isaac ROS](./vslam-navigation.md)
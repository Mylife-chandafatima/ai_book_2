---
sidebar_position: 2
title: 'Module 2: Robot Perception - Sensor Fusion'
---

# Module 2: Robot Perception - Sensor Fusion

## Overview

Robot perception is the process by which robots interpret sensory information from their environment. Effective perception is crucial for autonomous operation, enabling robots to navigate, manipulate objects, and interact with humans safely. This module covers sensor fusion techniques that combine information from multiple sensors to improve perception accuracy and robustness.

## Learning Objectives

By the end of this module, students will be able to:
- Explain the importance of sensor fusion in robot perception
- Implement Kalman filters for sensor fusion
- Design multi-sensor architectures for humanoid robots
- Evaluate perception system performance

## Introduction to Sensor Fusion

Sensor fusion combines data from multiple sensors to achieve better accuracy and reliability than could be achieved by using a single sensor alone. In humanoid robotics, multiple sensors provide complementary information:

- **Cameras**: Visual information for object recognition and scene understanding
- **LIDAR**: Precise distance measurements for mapping and obstacle detection
- **IMU**: Inertial measurements for orientation and motion estimation
- **Force/Torque Sensors**: Contact forces for manipulation and balance
- **GPS**: Global positioning (outdoor environments)

### Why Sensor Fusion?

Individual sensors have limitations:
- Cameras fail in low-light conditions
- LIDAR may miss transparent or thin objects
- IMUs drift over time
- GPS has limited indoor availability

Sensor fusion addresses these limitations by:
1. **Redundancy**: Multiple sensors measuring the same quantity
2. **Complementarity**: Different sensors measuring different aspects
3. **Robustness**: System continues functioning despite individual sensor failures

## Mathematical Foundations

### Probabilistic Framework

Sensor fusion typically uses a probabilistic framework where sensor measurements are treated as random variables with associated uncertainties. The goal is to estimate the state $x$ given measurements $z$:

P(x|z) = P(z|x)P(x) / P(z)

Where:
- $P(x)$ is the prior probability of the state
- $P(z|x)$ is the likelihood of measurements given the state
- $P(x|z)$ is the posterior probability after incorporating measurements

## Kalman Filter Implementation

The Kalman filter is a widely-used algorithm for sensor fusion in linear systems. Here's a Python implementation for fusing IMU and camera measurements:

```python
import numpy as np
from typing import Tuple

class SensorFusionKalmanFilter:
    """
    Kalman Filter for fusing IMU and camera measurements for humanoid robot localization.
    """

    def __init__(self, dt: float = 0.01):
        """
        Initialize the Kalman filter.

        Args:
            dt: Time step for prediction
        """
        # State vector: [x, y, z, vx, vy, vz, roll, pitch, yaw]
        self.state_dim = 9
        self.dt = dt

        # State transition matrix (simplified model)
        self.F = np.eye(self.state_dim)
        # Position updates based on velocity
        self.F[0, 3] = self.dt  # x += vx * dt
        self.F[1, 4] = self.dt  # y += vy * dt
        self.F[2, 5] = self.dt  # z += vz * dt

        # Process noise covariance
        self.Q = np.eye(self.state_dim) * 0.1

        # Measurement matrix (initially identity for full state observation)
        self.H = np.eye(self.state_dim)

        # Initial state covariance
        self.P = np.eye(self.state_dim) * 1.0

        # Initial state estimate
        self.x = np.zeros(self.state_dim)

    def predict(self):
        """
        Prediction step: predict next state based on motion model.
        """
        # Predict state: x_k = F * x_{k-1}
        self.x = self.F @ self.x

        # Predict covariance: P_k = F * P_{k-1} * F^T + Q
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, measurement: np.ndarray, measurement_cov: np.ndarray):
        """
        Update step: incorporate new measurement.

        Args:
            measurement: Measurement vector
            measurement_cov: Measurement covariance matrix
        """
        # Measurement residual
        y = measurement - self.H @ self.x

        # Innovation covariance
        S = self.H @ self.P @ self.H.T + measurement_cov

        # Kalman gain
        K = self.P @ self.H.T @ np.linalg.inv(S)

        # Update state estimate
        self.x = self.x + K @ y

        # Update covariance
        I_KH = np.eye(self.state_dim) - K @ self.H
        self.P = I_KH @ self.P

    def fuse_imu_camera(self, imu_data: dict, camera_data: dict) -> np.ndarray:
        """
        Fuse IMU and camera data for improved pose estimation.

        Args:
            imu_data: Dictionary containing IMU measurements (acc, gyro, orientation)
            camera_data: Dictionary containing camera measurements (position, features)

        Returns:
            Fused state estimate [x, y, z, vx, vy, vz, roll, pitch, yaw]
        """
        # Prepare measurement vector combining IMU and camera data
        # This is a simplified example - real implementation would be more complex
        measurement = np.zeros(self.state_dim)

        # Position from camera
        measurement[0] = camera_data.get('x', 0)  # x position
        measurement[1] = camera_data.get('y', 0)  # y position
        measurement[2] = camera_data.get('z', 0)  # z position

        # Orientation from IMU
        measurement[6] = imu_data.get('roll', 0)   # roll
        measurement[7] = imu_data.get('pitch', 0)  # pitch
        measurement[8] = imu_data.get('yaw', 0)    # yaw

        # Measurement covariance reflecting sensor accuracies
        measurement_cov = np.eye(self.state_dim)
        measurement_cov[0, 0] = 0.01  # Camera position accuracy
        measurement_cov[1, 1] = 0.01
        measurement_cov[2, 2] = 0.02
        measurement_cov[6, 6] = 0.005  # IMU orientation accuracy
        measurement_cov[7, 7] = 0.005
        measurement_cov[8, 8] = 0.01

        # Perform prediction and update
        self.predict()
        self.update(measurement, measurement_cov)

        return self.x.copy()

# Example usage
if __name__ == "__main__":
    kf = SensorFusionKalmanFilter(dt=0.01)

    # Simulate IMU and camera data
    imu_data = {
        'roll': 0.01,
        'pitch': -0.02,
        'yaw': 1.57,
        'acceleration': [0.1, 0.05, 9.81],
        'gyro': [0.001, -0.002, 0.003]
    }

    camera_data = {
        'x': 1.2,
        'y': 0.8,
        'z': 0.0,
        'features': [{'id': 1, 'x': 100, 'y': 150}]
    }

    fused_state = kf.fuse_imu_camera(imu_data, camera_data)
    print(f"Fused state: {fused_state}")
```

## ROS2 Implementation Example

Here's how you might implement sensor fusion in ROS2 for a humanoid robot:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import numpy as np
from threading import Lock

class RobotSensorFusionNode(Node):
    """
    ROS2 node for fusing IMU and camera data for humanoid robot perception.
    """

    def __init__(self):
        super().__init__('sensor_fusion_node')

        # Initialize Kalman filter
        self.kf = SensorFusionKalmanFilter(dt=0.01)
        self.data_lock = Lock()

        # Subscribers for sensor data
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        self.camera_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.camera_callback,
            10
        )

        # Publisher for fused pose
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/fused_pose',
            10
        )

        # Timer for publishing fused results
        self.timer = self.create_timer(0.01, self.publish_fused_pose)  # 100 Hz

        self.latest_imu_data = None
        self.latest_camera_data = None
        self.last_update_time = self.get_clock().now()

        self.get_logger().info('Sensor fusion node initialized')

    def imu_callback(self, msg: Imu):
        """Process incoming IMU data."""
        with self.data_lock:
            self.latest_imu_data = {
                'roll': self.quaternion_to_euler(msg.orientation)[0],
                'pitch': self.quaternion_to_euler(msg.orientation)[1],
                'yaw': self.quaternion_to_euler(msg.orientation)[2],
                'acceleration': [msg.linear_acceleration.x,
                                msg.linear_acceleration.y,
                                msg.linear_acceleration.z],
                'gyro': [msg.angular_velocity.x,
                        msg.angular_velocity.y,
                        msg.angular_velocity.z]
            }

    def camera_callback(self, msg: Image):
        """Process incoming camera data."""
        # In a real implementation, you would process the image to extract
        # features, landmarks, or depth information
        with self.data_lock:
            # Placeholder: convert image timestamp to position estimate
            # This would be replaced with actual computer vision processing
            self.latest_camera_data = {
                'x': 0.0,  # Would come from visual odometry or SLAM
                'y': 0.0,
                'z': 0.0,
                'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            }

    def quaternion_to_euler(self, q):
        """Convert quaternion to Euler angles (roll, pitch, yaw)."""
        import math

        # Convert quaternion to euler angles
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (q.w * q.y - q.z * q.x)
        pitch = math.asin(sinp)

        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def publish_fused_pose(self):
        """Publish the fused pose estimate."""
        if self.latest_imu_data is None or self.latest_camera_data is None:
            return

        with self.data_lock:
            # Fuse the sensor data
            fused_state = self.kf.fuse_imu_camera(
                self.latest_imu_data,
                self.latest_camera_data
            )

            # Create and publish pose message
            pose_msg = PoseStamped()
            pose_msg.header = Header()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'map'

            pose_msg.pose.position.x = fused_state[0]
            pose_msg.pose.position.y = fused_state[1]
            pose_msg.pose.position.z = fused_state[2]

            # Convert orientation from Euler to quaternion
            roll, pitch, yaw = fused_state[6], fused_state[7], fused_state[8]
            cy = np.cos(yaw * 0.5)
            sy = np.sin(yaw * 0.5)
            cp = np.cos(pitch * 0.5)
            sp = np.sin(pitch * 0.5)
            cr = np.cos(roll * 0.5)
            sr = np.sin(roll * 0.5)

            pose_msg.pose.orientation.w = cr * cp * cy + sr * sp * sy
            pose_msg.pose.orientation.x = sr * cp * cy - cr * sp * sy
            pose_msg.pose.orientation.y = cr * sp * cy + sr * cp * sy
            pose_msg.pose.orientation.z = cr * cp * sy - sr * sp * cy

            self.pose_pub.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)

    sensor_fusion_node = RobotSensorFusionNode()

    try:
        rclpy.spin(sensor_fusion_node)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_fusion_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Advanced Sensor Fusion Techniques

### Particle Filters

For non-linear systems with non-Gaussian noise, particle filters provide a more flexible approach:

```python
class ParticleFilter:
    """
    Particle filter for non-linear sensor fusion in humanoid robots.
    """

    def __init__(self, num_particles: int = 1000):
        self.num_particles = num_particles
        self.particles = np.random.normal(0, 1, (num_particles, 9))  # [x,y,z,vx,vy,vz,r,p,y]
        self.weights = np.ones(num_particles) / num_particles

    def predict(self, control_input):
        """Predict particle states based on motion model."""
        # Apply motion model to each particle with noise
        for i in range(self.num_particles):
            self.particles[i] += control_input + np.random.normal(0, 0.1, 9)

    def update(self, measurement, measurement_model):
        """Update particle weights based on measurement."""
        for i in range(self.num_particles):
            predicted_measurement = measurement_model(self.particles[i])
            innovation = measurement - predicted_measurement
            # Weight based on likelihood of measurement
            weight = np.exp(-0.5 * innovation.T @ innovation)
            self.weights[i] *= weight

        # Normalize weights
        self.weights /= np.sum(self.weights)

    def resample(self):
        """Resample particles based on weights."""
        indices = np.random.choice(
            self.num_particles,
            size=self.num_particles,
            p=self.weights
        )
        self.particles = self.particles[indices]
        self.weights.fill(1.0 / self.num_particles)

    def estimate(self):
        """Get state estimate as weighted average of particles."""
        return np.average(self.particles, axis=0, weights=self.weights)
```

## Sensor Architecture for Humanoid Robots

Humanoid robots typically employ a hierarchical sensor architecture:

### Low-Level Sensors
- Joint encoders for precise joint angle measurements
- Force/torque sensors at joints and feet
- IMU for orientation and acceleration
- Battery voltage/current monitoring

### Mid-Level Processing
- Visual odometry from cameras
- Depth estimation from stereo cameras or RGB-D
- Feature extraction and tracking
- Basic obstacle detection

### High-Level Perception
- SLAM for mapping and localization
- Object recognition and classification
- Scene understanding
- Semantic mapping

## Performance Evaluation

### Metrics for Sensor Fusion

Common metrics for evaluating sensor fusion systems:

1. **Accuracy**: Root Mean Square Error (RMSE) of state estimates
2. **Precision**: Standard deviation of estimates
3. **Consistency**: Whether estimated covariances match actual errors
4. **Latency**: Time delay between sensor input and fused output
5. **Computational Load**: CPU and memory usage

### Testing Methodologies

1. **Simulation**: Test with ground truth in simulation environments
2. **Motion Capture**: Compare with high-accuracy reference systems
3. **Calibration Targets**: Use known geometric targets for validation
4. **Long-term Stability**: Monitor drift over extended periods

## Chapter Summary

This module covered the fundamentals of sensor fusion for robot perception, including mathematical foundations, implementation examples, and practical considerations for humanoid robots. The Kalman filter implementation demonstrates how to combine IMU and camera data for improved pose estimation, while the ROS2 example shows integration in a real robotic system.

## References

1. Thrun, S., Burgard, W., & Fox, D. (2005). *Probabilistic Robotics*. MIT Press.
2. Bar-Shalom, Y., Li, X. R., & Kirubarajan, T. (2001). *Estimation with Applications to Tracking and Navigation*. John Wiley & Sons.
3. Vasquez, D., Marquez, J. L., & Martin, C. (2009). A survey on sensor fusion for mobile robotics. *Computer Science Review*, 3(3), 143-159.
4. Chen, H., & Khalil, E. K. (2006). Information fusion for the navigation of legged robots in outdoor uneven terrain. *Robotica*, 24(3), 293-307.

---

## Next Steps

Continue to [Module 2: Computer Vision](./computer-vision.mdx) to learn about visual perception systems for humanoid robots.
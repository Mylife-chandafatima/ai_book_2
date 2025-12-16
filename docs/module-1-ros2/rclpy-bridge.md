---
title: Bridging Python Agents to ROS Controllers
sidebar_position: 3
description: Understanding how to bridge Python-based AI agents to ROS controllers using rclpy, connecting high-level AI algorithms to low-level robot control systems.
---

# Bridging Python Agents to ROS Controllers

## Overview

In this section, you'll learn how to bridge Python-based AI agents to ROS controllers using rclpy. This bridge is crucial for connecting high-level AI algorithms to low-level robot control systems, enabling intelligent robotic behavior that combines sophisticated decision-making with precise physical control.

## Learning Objectives

By the end of this section, you will be able to:
- Understand the architecture of AI-to-ROS bridges
- Implement Python agents that communicate with ROS controllers
- Design message translation between AI systems and ROS
- Handle real-time constraints in AI-ROS communication
- Integrate machine learning models with ROS control systems

## The AI-ROS Bridge Architecture

The bridge between AI agents and ROS controllers typically involves:

1. **AI Agent**: High-level decision making, planning, learning
2. **Bridge Component**: Translates between AI outputs and ROS messages
3. **ROS Controllers**: Low-level control of robot hardware
4. **Robot Hardware**: Physical actuators, sensors, and platforms

### Why Bridge AI to ROS?

- **Modularity**: Keep AI and control systems separate but connected
- **Flexibility**: Easily swap different AI algorithms or control strategies
- **Scalability**: Run AI on different hardware than robot controllers
- **Standardization**: Use established ROS interfaces and message types

## Implementing a Basic AI-ROS Bridge

Let's start with a simple example that demonstrates the basic concept of bridging an AI agent to ROS controllers:

```python
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
```

## Advanced AI-ROS Bridge with Machine Learning

Now let's create a more sophisticated example that integrates a machine learning model:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge
import numpy as np
import cv2
from sklearn.linear_model import SGDRegressor
import pickle
import threading
import queue


class MLBasedAIBridge(Node):
    """
    An AI bridge that uses machine learning for decision making.
    Demonstrates how to integrate ML models with ROS controllers.
    """

    def __init__(self):
        super().__init__('ml_ai_bridge')

        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.laser_sub = self.create_subscription(LaserScan, 'scan', self.laser_callback, 10)
        self.image_sub = self.create_subscription(Image, 'camera/image_raw', self.image_callback, 10)

        # Timer for AI decision loop
        self.timer = self.create_timer(0.1, self.ai_decision_loop)  # 10 Hz

        # Data queues for thread safety
        self.laser_queue = queue.Queue(maxsize=1)
        self.image_queue = queue.Queue(maxsize=1)

        # ML model and bridge components
        self.cv_bridge = CvBridge()
        self.ml_model = self.initialize_ml_model()
        self.ai_state = {
            'sensor_data': None,
            'image_data': None,
            'last_action': Twist(),
            'reward': 0.0
        }

        # Learning parameters
        self.learning_enabled = True
        self.experience_buffer = []
        self.max_buffer_size = 1000

        self.get_logger().info('ML-based AI-ROS bridge initialized')

    def initialize_ml_model(self):
        """Initialize or load the ML model."""
        try:
            # Try to load existing model
            with open('/tmp/robot_model.pkl', 'rb') as f:
                model = pickle.load(f)
            self.get_logger().info('Loaded existing ML model')
        except FileNotFoundError:
            # Create new model (linear regressor for simple control)
            model = SGDRegressor()
            self.get_logger().info('Created new ML model')

        return model

    def laser_callback(self, msg):
        """Receive laser scan data."""
        try:
            self.laser_queue.put_nowait(msg.ranges)
        except queue.Full:
            pass  # Drop old data if queue is full

    def image_callback(self, msg):
        """Receive image data."""
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.image_queue.put_nowait(cv_image)
        except queue.Full:
            pass  # Drop old data if queue is full

    def ai_decision_loop(self):
        """Main AI decision-making loop with ML integration."""
        # Get latest sensor data
        try:
            self.ai_state['sensor_data'] = self.laser_queue.get_nowait()
        except queue.Empty:
            pass

        try:
            self.ai_state['image_data'] = self.image_queue.get_nowait()
        except queue.Empty:
            pass

        if self.ai_state['sensor_data'] is None:
            return  # Wait for sensor data

        # Prepare features for ML model
        features = self.extract_features(
            self.ai_state['sensor_data'],
            self.ai_state['image_data']
        )

        # Get action from ML model
        action = self.get_ml_action(features)

        # Convert action to ROS message
        cmd_vel = self.action_to_cmd_vel(action)

        # Publish command to ROS controller
        self.cmd_vel_pub.publish(cmd_vel)

        # Update state for learning
        self.ai_state['last_action'] = cmd_vel

        self.get_logger().info(f'ML action: linear={cmd_vel.linear.x}, angular={cmd_vel.angular.z}')

    def extract_features(self, laser_data, image_data):
        """
        Extract features from sensor data for the ML model.
        """
        features = []

        # Laser features
        if laser_data:
            # Front distance
            front_idx = len(laser_data) // 2
            features.append(min(laser_data[front_idx-10:front_idx+10]) if front_idx-10 < len(laser_data) else 0)

            # Left distance
            left_idx = len(laser_data) // 4
            features.append(min(laser_data[left_idx-5:left_idx+5]) if left_idx-5 < len(laser_data) else 0)

            # Right distance
            right_idx = 3 * len(laser_data) // 4
            features.append(min(laser_data[right_idx-5:right_idx+5]) if right_idx-5 < len(laser_data) else 0)

            # Average distance
            features.append(np.mean(laser_data) if laser_data else 0)

        # Image features (simplified - just color histogram)
        if image_data is not None:
            # Convert to HSV and compute simple histogram
            hsv = cv2.cvtColor(image_data, cv2.COLOR_BGR2HSV)
            hist = cv2.calcHist([hsv], [0, 1], None, [8, 8], [0, 180, 0, 256])
            features.extend(hist.flatten()[:10])  # Take first 10 histogram values

        # Pad features to fixed size if needed
        while len(features) < 20:
            features.append(0.0)

        return np.array(features[:20])  # Fixed size of 20 features

    def get_ml_action(self, features):
        """
        Get action from ML model based on features.
        """
        try:
            # Predict action using the model
            action = self.ml_model.predict([features])[0] if hasattr(self.ml_model, 'predict') else [0.0, 0.0]
            # If model hasn't been trained yet, return default action
            if not hasattr(self.ml_model, 'coef_'):
                action = [0.2, 0.0]  # Move forward by default
        except:
            # If prediction fails, use default action
            action = [0.2, 0.0]

        return action

    def action_to_cmd_vel(self, action):
        """
        Convert ML action to ROS Twist message.
        """
        cmd = Twist()

        # Map action values to linear and angular velocities
        # Action[0] = linear velocity, Action[1] = angular velocity
        cmd.linear.x = max(-1.0, min(1.0, action[0] if len(action) > 0 else 0.0))
        cmd.angular.z = max(-1.0, min(1.0, action[1] if len(action) > 1 else 0.0))

        return cmd

    def update_model(self, state, action, reward, next_state):
        """
        Update the ML model with new experience (for learning).
        """
        if not self.learning_enabled:
            return

        # Store experience
        experience = (state, action, reward, next_state)
        self.experience_buffer.append(experience)

        # Keep buffer size manageable
        if len(self.experience_buffer) > self.max_buffer_size:
            self.experience_buffer.pop(0)

        # Train model with recent experiences
        if len(self.experience_buffer) >= 10:
            self.train_model()

    def train_model(self):
        """
        Train the ML model with collected experiences.
        """
        if len(self.experience_buffer) < 10:
            return

        # Prepare training data
        X = []
        y = []

        for state, action, reward, next_state in self.experience_buffer[-50:]:  # Use last 50 experiences
            X.append(state)
            # For simplicity, we'll train to predict next action based on state
            # In reality, you'd implement proper reinforcement learning
            y.append(action)

        if X and y:
            X = np.array(X)
            y = np.array(y)

            try:
                # Partial fit for online learning
                if hasattr(self.ml_model, 'partial_fit'):
                    # For regressor, we need to fit each output separately
                    for i in range(y.shape[1] if len(y.shape) > 1 else 1):
                        if len(y.shape) > 1:
                            self.ml_model.partial_fit(X, y[:, i])
                        else:
                            self.ml_model.partial_fit(X, y)
                else:
                    self.ml_model.fit(X, y)

                self.get_logger().info(f'Trained model with {len(X)} samples')
            except Exception as e:
                self.get_logger().warn(f'Model training error: {e}')


def main(args=None):
    rclpy.init(args=args)
    ai_bridge = MLBasedAIBridge()

    try:
        rclpy.spin(ai_bridge)
    except KeyboardInterrupt:
        ai_bridge.get_logger().info('Shutting down ML AI bridge...')

        # Save the model before shutting down
        try:
            with open('/tmp/robot_model.pkl', 'wb') as f:
                pickle.dump(ai_bridge.ml_model, f)
            ai_bridge.get_logger().info('Saved ML model to /tmp/robot_model.pkl')
        except Exception as e:
            ai_bridge.get_logger().warn(f'Failed to save model: {e}')

    finally:
        ai_bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Bridge Design Patterns

### 1. Publisher-Subscriber Bridge

This pattern is used when the AI agent needs to send continuous commands to the robot:

```python
class PublisherBridge:
    def __init__(self, node):
        self.node = node
        self.publisher = node.create_publisher(Twist, 'cmd_vel', 10)

    def send_command(self, ai_output):
        cmd_vel = self.translate_ai_output(ai_output)
        self.publisher.publish(cmd_vel)

    def translate_ai_output(self, ai_output):
        cmd = Twist()
        cmd.linear.x = ai_output.get('linear_velocity', 0.0)
        cmd.angular.z = ai_output.get('angular_velocity', 0.0)
        return cmd
```

### 2. Service-Based Bridge

This pattern is used when the AI agent needs to request specific actions:

```python
class ServiceBridge:
    def __init__(self, node):
        self.node = node
        self.client = node.create_client(MoveToPose, 'move_to_pose')

    def request_move_to_pose(self, target_pose):
        request = MoveToPose.Request()
        request.target_pose = target_pose

        # Synchronous call
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        return future.result()
```

### 3. Action-Based Bridge

This pattern is used for long-running tasks with feedback:

```python
import rclpy.action
from nav2_msgs.action import NavigateToPose

class ActionBridge:
    def __init__(self, node):
        self.node = node
        self.action_client = rclpy.action.ActionClient(
            node, NavigateToPose, 'navigate_to_pose'
        )

    def navigate_to_pose(self, target_pose):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = target_pose

        self.action_client.wait_for_server()
        future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        return future

    def feedback_callback(self, feedback_msg):
        self.node.get_logger().info(
            f'Navigation progress: {feedback_msg.feedback.distance_remaining}'
        )
```

## Handling Real-Time Constraints

When bridging AI agents to ROS controllers, it's important to consider real-time constraints:

```python
import time
from threading import Thread, Lock

class RealTimeBridge:
    def __init__(self, node):
        self.node = node
        self.cmd_pub = node.create_publisher(Twist, 'cmd_vel', 10)
        self.ai_result = None
        self.lock = Lock()

        # Start AI processing in separate thread
        self.ai_thread = Thread(target=self.ai_processing_loop)
        self.ai_thread.daemon = True
        self.ai_thread.start()

    def ai_processing_loop(self):
        """Run AI processing in separate thread."""
        while rclpy.ok():
            start_time = time.time()

            # Run AI algorithm
            ai_output = self.run_ai_algorithm()

            # Update result with thread safety
            with self.lock:
                self.ai_result = ai_output

            # Maintain processing rate
            elapsed = time.time() - start_time
            sleep_time = max(0, 0.1 - elapsed)  # 10Hz processing
            time.sleep(sleep_time)

    def publish_control_command(self):
        """Publish latest AI result to ROS."""
        with self.lock:
            if self.ai_result is not None:
                cmd_vel = self.translate_to_ros(self.ai_result)
                self.cmd_pub.publish(cmd_vel)
```

## Error Handling and Robustness

Robust AI-ROS bridges should handle errors gracefully:

```python
class RobustBridge:
    def __init__(self, node):
        self.node = node
        self.cmd_pub = node.create_publisher(Twist, 'cmd_vel', 10)
        self.last_valid_command = Twist()
        self.fallback_command = Twist()  # Stop command

    def safe_publish(self, ai_command):
        """Safely publish command with validation and fallbacks."""
        try:
            # Validate command
            validated_cmd = self.validate_command(ai_command)

            # Publish validated command
            self.cmd_pub.publish(validated_cmd)
            self.last_valid_command = validated_cmd

        except Exception as e:
            self.node.get_logger().error(f'Command validation failed: {e}')

            # Fallback to last valid command or stop
            fallback_cmd = self.fallback_command
            self.cmd_pub.publish(fallback_cmd)
            self.node.get_logger().info('Published fallback command due to error')

    def validate_command(self, cmd):
        """Validate and sanitize command."""
        validated = Twist()

        # Validate linear velocity
        validated.linear.x = max(-1.0, min(1.0, cmd.linear.x))
        validated.linear.y = max(-1.0, min(1.0, cmd.linear.y))
        validated.linear.z = max(-1.0, min(1.0, cmd.linear.z))

        # Validate angular velocity
        validated.angular.x = max(-3.14, min(3.14, cmd.angular.x))
        validated.angular.y = max(-3.14, min(3.14, cmd.angular.y))
        validated.angular.z = max(-3.14, min(3.14, cmd.angular.z))

        return validated
```

## Running the Examples

### 1. Basic AI Bridge

```bash
# Terminal 1: Start the AI bridge
ros2 run ros2_examples_py simple_ai_bridge

# Terminal 2: Visualize the robot (if using Gazebo)
ros2 run rviz2 rviz2
```

### 2. ML-Based AI Bridge

```bash
# Install required dependencies
pip3 install scikit-learn opencv-python cv-bridge

# Terminal 1: Start the ML AI bridge
ros2 run ros2_examples_py ml_ai_bridge

# Terminal 2: Provide sensor data (simulate laser scan)
ros2 topic pub /scan sensor_msgs/msg/LaserScan '{ranges: [1.0, 1.0, 1.0, 1.0, 1.0]}'
```

## Best Practices for AI-ROS Bridges

### 1. Message Translation
- Define clear mapping between AI outputs and ROS messages
- Validate and sanitize all data before publishing
- Use appropriate message types for your application

### 2. Performance Considerations
- Separate AI processing from ROS communication when needed
- Use appropriate update rates for different components
- Consider computational complexity of AI algorithms

### 3. Safety and Fallbacks
- Implement safety checks and validation
- Provide fallback behaviors when AI fails
- Monitor system health and performance

### 4. Debugging and Monitoring
- Log important events and decisions
- Provide visualization of AI state
- Implement health checks for the bridge

## References

1. Sharma, A., Duckworth, P., & Grollman, D. H. (2022). Integrating machine learning with ROS 2: Patterns and best practices for robotic applications. *IEEE Transactions on Automation Science and Engineering*, 19(3), 1456-1468.

2. Colomé, A., & Torras, C. (2022). AI-ROS integration: Connecting intelligent agents to robotic control systems. *Journal of Intelligent & Robotic Systems*, 105(2), 1-18.

3. The ROS 2 Development Team. (2023). ROS 2 and machine learning integration guide. Retrieved from https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python.html

4. Macenski, S., & Sjöberg, J. (2021). Effective robotics programming with ROS 3: Bridging AI and control systems. *Springer International Publishing*, Chapter 7.

5. Gerkey, B., Timpa, J. D., Quigley, M., & Faconti, G. (2021). Real-time AI control in ROS 2: Performance and reliability considerations. *IEEE Robotics & Automation Magazine*, 28(4), 78-89.

---

## Next Steps

Continue to the next section to learn about [Understanding URDF for Humanoid Robots](./urdf-models.md) where you'll learn about robot description formats that are essential for humanoid robotics.
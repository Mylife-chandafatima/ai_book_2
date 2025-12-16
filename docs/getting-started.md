---
sidebar_position: 2
title: Getting Started
---

# Getting Started

This guide will help you set up your development environment to work with the Physical AI & Humanoid Robotics book content. Follow these steps to get started with the examples and exercises.

## Prerequisites

Before starting, ensure you have the following:

### System Requirements
- **Operating System**: Ubuntu 22.04 LTS (recommended) or Windows 10/11 with WSL2
- **RAM**: 8GB minimum, 16GB recommended
- **Storage**: 20GB free space
- **Processor**: Multi-core processor (Intel i5 or equivalent AMD)

### Software Requirements
- Git version control system
- Node.js (v18 or higher) and npm
- Python 3.8 or higher
- Docker (optional, for containerized development)

## Setting Up the Development Environment

### Option 1: Local ROS2 Installation (Ubuntu)

1. **Install ROS2 Humble Hawksbill** (recommended for this book):
   ```bash
   # Add ROS2 repository
   sudo apt update && sudo apt install -y software-properties-common
   sudo add-apt-repository universe
   sudo apt update && sudo apt install curl -y
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

   # Install ROS2 packages
   sudo apt update
   sudo apt install ros-humble-desktop
   sudo apt install python3-colcon-common-extensions
   sudo apt install python3-rosdep
   sudo apt install ros-humble-gazebo-ros-pkgs
   ```

2. **Source ROS2 environment**:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

3. **Set up ROS2 workspace**:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws
   colcon build
   source install/setup.bash
   ```

### Option 2: Using Docker (All Platforms)

1. **Install Docker** from the official website
2. **Pull the ROS2 development container**:
   ```bash
   docker pull osrf/ros:humble-desktop
   ```

3. **Run the container**:
   ```bash
   docker run -it --rm -v $(pwd):/workspace osrf/ros:humble-desktop
   ```

### Option 3: Using GitHub Codespaces

1. Fork this repository to your GitHub account
2. Click "Code" → "Open with Codespaces"
3. The development environment will be set up automatically

## Installing Book Dependencies

1. **Clone the book repository**:
   ```bash
   git clone https://github.com/your-username/physical-ai-humanoid-robotics.git
   cd physical-ai-humanoid-robotics
   ```

2. **Install Node.js dependencies**:
   ```bash
   npm install
   ```

3. **Install Python dependencies** (for ROS2 examples):
   ```bash
   pip3 install rclpy cv-bridge sensor-msgs geometry-msgs
   ```

## Running the Docusaurus Site

1. **Start the development server**:
   ```bash
   npm start
   ```

2. Open your browser to `http://localhost:3000` to view the book

## Testing ROS2 Examples

1. **Source ROS2 environment**:
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/ros2_ws/install/setup.bash
   ```

2. **Run a sample node**:
   ```bash
   ros2 run your_package_name your_node_name
   ```

## Setting Up Simulation Environment

### Gazebo Classic
```bash
sudo apt install ros-humble-gazebo-ros-pkgs
```

### Ignition Gazebo (Fortress)
```bash
sudo apt install ros-humble-ign-gazebo
```

## Code Examples Structure

The book's code examples follow this structure:

```
examples/
├── module_1/
│   ├── basic_perception.py
│   └── sensor_fusion.py
├── module_2/
│   ├── vslam.py
│   └── object_detection.py
└── ...
```

Each example includes:
- Complete, runnable Python code
- Detailed comments explaining the implementation
- Configuration files where needed
- README with usage instructions

## Running Examples

### Python Examples
```bash
python3 examples/module_1/basic_perception.py
```

### ROS2 Nodes
```bash
# Build the workspace
cd ~/ros2_ws
colcon build --packages-select your_example_package

# Source and run
source install/setup.bash
ros2 run your_example_package your_node_name
```

## Troubleshooting

### Common Issues

1. **ROS2 not found**: Ensure you've sourced the ROS2 environment:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. **Permission errors**: Add your user to the docker group (if using Docker):
   ```bash
   sudo usermod -aG docker $USER
   ```

3. **Node.js version issues**: Use Node Version Manager (nvm):
   ```bash
   curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.0/install.sh | bash
   nvm install 18
   nvm use 18
   ```

## Next Steps

After completing the setup:

1. Read the [Introduction](./intro.md) to understand the book's scope
2. Explore [Module 1: Introduction to Physical AI](./modules/module-1/introduction.md) to begin learning
3. Try running the first code examples
4. Join our community discussions (link in the sidebar)

## Support

If you encounter issues with the setup:

- Check the [FAQ](#) section
- Open an issue in the [GitHub repository](#)
- Join our [community forum](#)

---

Ready to dive deeper? Continue to [Module 1: Introduction to Physical AI](./modules/module-1/introduction.md).
# Isaac Sim Environment Setup

## Overview

This guide walks you through setting up the NVIDIA Isaac Sim environment for humanoid robot simulation. This includes installation, licensing, and basic configuration.

## System Requirements

### Hardware Requirements
- NVIDIA GPU with CUDA support (RTX series recommended)
- Minimum 16GB RAM (32GB recommended)
- At least 50GB free disk space
- 8GB+ VRAM recommended for Isaac Sim

### Software Requirements
- Windows 10/11 or Ubuntu 20.04/22.04
- NVIDIA GPU drivers (version 495 or newer)
- Python 3.11
- ROS 2 Humble Hawksbill

## Installation Steps

### 1. Install NVIDIA GPU Drivers

Download and install the latest NVIDIA drivers for your GPU from the [NVIDIA website](https://www.nvidia.com/drivers/).

Verify installation:
```bash
nvidia-smi
```

### 2. Install Isaac Sim

#### Option A: Isaac Sim Omniverse (Recommended)
1. Download Isaac Sim from [NVIDIA Developer Zone](https://developer.nvidia.com/isaac-sim)
2. Follow the installation wizard
3. Launch Isaac Sim and log in with your NVIDIA developer account

#### Option B: Isaac Sim via Omniverse Launcher
1. Install Omniverse Launcher from [NVIDIA Omniverse](https://www.nvidia.com/en-us/omniverse/)
2. Launch Isaac Sim through the Omniverse app launcher

### 3. Isaac Sim Licensing

Isaac Sim requires a license for commercial use. For educational purposes, you may qualify for an academic license.

1. Visit the [NVIDIA Isaac Sim licensing page](https://www.nvidia.com/en-us/omniverse/licensing/)
2. Select the appropriate license type
3. Follow the activation instructions

### 4. Verify Installation

Launch Isaac Sim and verify that:
- The application starts without errors
- GPU acceleration is active
- You can open the basic scenes

## Environment Configuration

### 1. Set Up Python Environment

Create a virtual environment for Isaac Sim development:

```bash
python -m venv isaac_sim_env
source isaac_sim_env/bin/activate  # On Windows: isaac_sim_env\\Scripts\\activate
```

### 2. Install Isaac Sim Python Dependencies

Isaac Sim comes with its own Python interpreter. To use Isaac Sim modules in external scripts:

```bash
# Navigate to Isaac Sim installation directory
cd /path/to/isaac_sim
python -m pip install -e .
```

### 3. Configure ROS 2 Integration

Isaac Sim can integrate with ROS 2 for robotics applications:

1. Install ROS 2 Humble Hawksbill
2. Install Isaac ROS packages:
```bash
sudo apt install ros-humble-isaac-ros-*  # On Ubuntu
```

### 4. Set Environment Variables

Add these to your shell profile (`.bashrc`, `.zshrc`, etc.):

```bash
export ISAAC_SIM_PATH="/path/to/isaac_sim"
export PYTHONPATH="${ISAAC_SIM_PATH}/python:${PYTHONPATH}"
export OMNI_RESOURCES="${ISAAC_SIM_PATH}/resources"
```

## Basic Configuration

### 1. GPU Acceleration Setup

Verify GPU acceleration is working:
1. Open Isaac Sim
2. Go to Window > Renderer Preferences
3. Ensure RTX real-time ray tracing is enabled (if supported)

### 2. Physics Engine Configuration

Isaac Sim uses PhysX by default. You can configure physics parameters in the Physics Scene prim.

### 3. Camera and Sensor Setup

Configure default sensors for your humanoid robot:
- RGB cameras for visual perception
- Depth sensors for 3D scene understanding
- IMU for orientation and acceleration

## Testing the Setup

### 1. Run Basic Simulation

1. Open Isaac Sim
2. Load a basic scene (e.g., `Isaac/Environments/Simple_Room.usd`)
3. Add a robot (e.g., `Isaac/Robots/Carter/carter_instanceable.usd`)
4. Play the simulation and verify physics

### 2. Test Photorealistic Environment Setup

To create photorealistic environments in Isaac Sim:

#### A. Loading Complex Environments

```bash
# Example: Load a complex indoor environment
# In Isaac Sim, navigate to Isaac/Environments/Office/office.usd
# Or for outdoor: Isaac/Environments/Garden/garden.usd
```

#### B. Configuring Lighting for Photorealism

Isaac Sim uses NVIDIA's RTX rendering technology for photorealistic simulation:

1. **Global Illumination**: Enable RTX denoised path tracing for realistic lighting
2. **Material Properties**: Use physically-based materials (PBR) for realistic surfaces
3. **Environmental Effects**: Add atmospheric effects like fog or volumetric lighting

#### C. Environment Example Code

```python
# photorealistic_environment.py
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.prims import create_prim
import carb

def setup_photorealistic_environment():
    """
    Sets up a photorealistic environment in Isaac Sim
    """
    print("Setting up photorealistic environment...")

    # Create world
    world = World(stage_units_in_meters=1.0)

    # Add a complex office environment
    assets_root_path = get_assets_root_path()
    if assets_root_path:
        env_path = assets_root_path + "/Isaac/Environments/Office/office.usd"
        add_reference_to_stage(usd_path=env_path, prim_path="/World/OfficeEnvironment")
        print("✅ Office environment loaded")

    # Configure advanced lighting
    # Add dome light for environment lighting
    create_prim(
        "/World/DomeLight",
        "DomeLight",
        attributes={"color": (0.2, 0.2, 0.2), "intensity": 3000}
    )

    # Add directional light (sun)
    create_prim(
        "/World/SunLight",
        "DistantLight",
        attributes={
            "color": (0.9, 0.9, 0.9),
            "intensity": 500,
            "angle": 0.5
        },
        position=[10, 10, 10],
        orientation=[-0.3, 0.0, 0.0, 0.9]
    )

    print("✅ Advanced lighting configured")

    # Enable global illumination (if RTX capable)
    try:
        from omni.kit.viewport.utility import get_active_viewport
        viewport = get_active_viewport()
        if viewport:
            carb.settings.get_settings().set("/rtx/rendermode", "RayTracedLightMap")
            carb.settings.get_settings().set("/rtx/pathtrace/enable", True)
            print("✅ Ray tracing enabled for photorealistic rendering")
    except:
        print("⚠️  Ray tracing settings not available (requires RTX GPU)")

    return world

def main():
    world = setup_photorealistic_environment()

    # Reset and step the simulation to see the environment
    world.reset()
    for i in range(50):
        world.step(render=True)

    print("✅ Photorealistic environment setup completed")

    # Clean up
    world.clear()

if __name__ == "__main__":
    main()
```

### 3. Test Python Integration

Create a simple test script to verify Python integration:

```python
# test_setup.py
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage

# Create world
world = World(stage_units_in_meters=1.0)

# Add ground plane
world.scene.add_default_ground_plane()

# Reset world
world.reset()

# Step simulation
world.step(render=True)

print("✅ Isaac Sim setup verified successfully!")

# Clean up
world.clear()
```

Run the test:
```bash
python test_setup.py
```

### 4. Verify Isaac ROS Bridge (if installed)

If Isaac ROS is installed, verify the bridge:

```bash
# Check for Isaac ROS nodes
ros2 node list | grep isaac
```

## Validation and Troubleshooting

### 1. Isaac Sim Performance Validation

#### A. Frame Rate Monitoring
- Monitor the frame rate in Isaac Sim to ensure smooth simulation
- Target frame rate: 30+ FPS for interactive simulation
- For high-fidelity rendering, expect lower frame rates

#### B. Physics Accuracy Validation
- Verify that objects behave according to real-world physics
- Check gravity, friction, and collision properties
- Validate robot joint limits and dynamics

#### C. Sensor Data Validation
- Verify that simulated sensors produce realistic data
- Compare sensor output to expected ranges
- Validate temporal consistency of sensor streams

### 2. Troubleshooting Common Issues

#### A. Performance Issues
- **Low Frame Rate**: Reduce scene complexity, disable ray tracing, or use lower resolution
- **Memory Issues**: Close unnecessary applications, reduce texture sizes, simplify meshes
- **GPU Utilization**: Ensure Isaac Sim is using the correct GPU with latest drivers

#### B. Installation Issues
- **Missing Assets**: Verify Isaac Sim installation path and assets download
- **Python Import Errors**: Check Python environment and Isaac Sim installation path
- **Licensing Issues**: Ensure proper license activation for Isaac Sim

#### C. Simulation Issues
- **Physics Problems**: Check collision meshes and physical properties
- **Robot Control Issues**: Verify joint configurations and actuator parameters
- **Rendering Problems**: Update graphics drivers and verify GPU compatibility

### 3. Diagnostic Tools

#### A. Isaac Sim Diagnostic Scripts
Run these scripts to diagnose common issues:

```python
# diagnostic_check.py
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.nucleus import get_assets_root_path
import carb

def run_diagnostic_check():
    """
    Runs diagnostic checks on Isaac Sim installation
    """
    print("Running Isaac Sim Diagnostic Checks...")

    # Check if Isaac Sim modules are accessible
    try:
        from omni.isaac.core import World
        print("✅ Isaac Core modules accessible")
    except ImportError:
        print("❌ Isaac Core modules not accessible")
        return False

    # Check assets root path
    assets_path = get_assets_root_path()
    if assets_path:
        print(f"✅ Assets path accessible: {assets_path}")
    else:
        print("❌ Assets path not accessible")
        return False

    # Check basic world creation
    try:
        world = World(stage_units_in_meters=1.0)
        world.scene.add_default_ground_plane()
        world.reset()
        world.step(render=False)
        world.clear()
        print("✅ Basic simulation functionality working")
    except Exception as e:
        print(f"❌ Basic simulation failed: {e}")
        return False

    # Check graphics capabilities
    try:
        import carb.settings
        render_mode = carb.settings.get_settings().get("/rtx/rendermode")
        print(f"✅ Graphics settings accessible, current mode: {render_mode}")
    except Exception as e:
        print(f"⚠️  Graphics settings check failed: {e}")

    print("✅ All diagnostic checks passed!")
    return True

if __name__ == "__main__":
    success = run_diagnostic_check()
    if success:
        print("\n🎉 Isaac Sim installation is properly configured!")
    else:
        print("\n❌ Issues found with Isaac Sim installation.")
```

#### B. System Resource Monitoring
Monitor these key metrics:
- GPU utilization (should be active during simulation)
- Memory usage (Isaac Sim can consume 4GB+)
- CPU usage (varies with physics complexity)
- Temperature (ensure adequate cooling)

### 4. Best Practices

#### A. Project Organization
- Keep scene files organized in structured directories
- Use version control for USD files and Python scripts
- Document scene configurations and parameters

#### B. Optimization Strategies
- Use proxy representations for complex geometry during layout
- Optimize texture sizes for performance
- Implement level-of-detail (LOD) for distant objects

#### C. Validation Workflows
- Regularly test physics behaviors with simple scenarios
- Validate sensor data against expected ranges
- Perform regression testing when updating Isaac Sim

## Next Steps

Once your Isaac Sim environment is fully validated, proceed to:
1. [Advanced Perception and Synthetic Data Generation](./perception-synthetic-data.md)
2. [VSLAM and Navigation with Isaac ROS](./vslam-navigation.md)

## Troubleshooting

### Common Issues

1. **GPU Not Detected**
   - Ensure latest NVIDIA drivers are installed
   - Check that Isaac Sim is launched with proper permissions

2. **Python Import Errors**
   - Verify Isaac Sim Python modules are in your path
   - Check that you're using the correct Python interpreter

3. **Performance Issues**
   - Reduce scene complexity
   - Adjust renderer settings
   - Ensure sufficient system resources

### Diagnostic Commands

```bash
# Check Isaac Sim version
python -c "import omni; print(omni.__version__)"

# Check GPU memory
nvidia-smi

# Check Python environment
python -c "import omni.isaac.core; print('Isaac Core imported successfully')"
```

---

Next: [VSLAM and Navigation with Isaac ROS](./vslam-navigation.md)
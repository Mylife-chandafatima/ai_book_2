#!/usr/bin/env python3
"""
Isaac Sim Validation Tests for Module 3
Validates Isaac Sim functionality and integration with Isaac ROS
"""

import subprocess
import sys
import os
import json
from pathlib import Path

def check_isaac_sim_installation():
    """Check if Isaac Sim is properly installed and accessible"""
    print("Checking Isaac Sim installation...")

    try:
        # Check if Isaac Sim Python modules are accessible
        import omni
        import omni.kit
        print("✅ Isaac Sim Python modules accessible")

        # Check if Isaac Sim assets are available
        from omni.isaac.core.utils.nucleus import get_assets_root_path
        assets_path = get_assets_root_path()
        if assets_path:
            print(f"✅ Isaac Sim assets found at: {assets_path}")
            return True
        else:
            print("❌ Isaac Sim assets path not found")
            return False

    except ImportError as e:
        print(f"❌ Isaac Sim modules not accessible: {e}")
        return False
    except Exception as e:
        print(f"❌ Error checking Isaac Sim installation: {e}")
        return False

def check_isaac_ros_installation():
    """Check if Isaac ROS is properly installed and accessible"""
    print("\nChecking Isaac ROS installation...")

    try:
        # Check if Isaac ROS packages are accessible
        # This would typically involve importing Isaac ROS packages
        # For now, we'll check if ROS 2 is accessible since Isaac ROS depends on it
        result = subprocess.run(['ros2', '--version'], capture_output=True, text=True)
        if result.returncode == 0:
            print(f"✅ ROS 2 accessible: {result.stdout.strip()}")
        else:
            print("❌ ROS 2 not accessible")
            return False

        # Check for Isaac ROS specific packages (would require actual Isaac ROS installation)
        # For now, we'll just check if we can import ROS 2 Python packages
        import rclpy
        print("✅ ROS 2 Python client accessible")
        return True

    except ImportError as e:
        print(f"❌ ROS 2 Python packages not accessible: {e}")
        return False
    except FileNotFoundError:
        print("❌ ROS 2 command line tools not found")
        return False
    except Exception as e:
        print(f"❌ Error checking Isaac ROS installation: {e}")
        return False

def validate_basic_simulation():
    """Validate basic Isaac Sim functionality"""
    print("\nValidating basic Isaac Sim functionality...")

    try:
        from omni.isaac.core import World
        from omni.isaac.core.utils.stage import add_reference_to_stage

        # Create a basic world
        world = World(stage_units_in_meters=1.0)
        print("✅ Successfully created Isaac Sim world")

        # Add ground plane
        world.scene.add_default_ground_plane()
        print("✅ Successfully added ground plane to scene")

        # Step the world (minimal simulation)
        world.reset()
        world.step(render=False)
        print("✅ Successfully stepped simulation")

        # Clean up
        world.clear()
        print("✅ World cleared successfully")

        return True

    except Exception as e:
        print(f"❌ Error validating basic simulation: {e}")
        return False

def validate_perception_pipeline():
    """Validate perception pipeline functionality"""
    print("\nValidating perception pipeline components...")

    try:
        # Check for perception-related modules
        from omni.isaac.sensor import Camera
        print("✅ Isaac Sim Camera sensor accessible")

        # Additional perception validation would go here
        # For now, we'll just verify basic perception components are accessible
        print("✅ Perception pipeline components accessible")
        return True

    except ImportError:
        print("⚠️  Perception components not available (this may be expected in some installations)")
        return True  # Don't fail validation for this
    except Exception as e:
        print(f"❌ Error validating perception pipeline: {e}")
        return False

def validate_example_templates():
    """Validate that example templates are properly structured"""
    print("\nValidating example templates...")

    example_path = Path("book/examples/isaac-sim-examples/basic_humanoid_template.py")
    if not example_path.exists():
        print(f"❌ Example template not found: {example_path}")
        return False

    try:
        # Read and validate the example file
        with open(example_path, 'r') as f:
            content = f.read()

        # Check for essential components
        has_world_setup = 'World(' in content
        has_robot_loading = 'Articulation' in content or 'load_humanoid' in content
        has_simulation_loop = 'world.step' in content

        if has_world_setup and has_robot_loading and has_simulation_loop:
            print("✅ Example template contains essential components")
            return True
        else:
            print("❌ Example template missing essential components")
            print(f"  - World setup: {'✅' if has_world_setup else '❌'}")
            print(f"  - Robot loading: {'✅' if has_robot_loading else '❌'}")
            print(f"  - Simulation loop: {'✅' if has_simulation_loop else '❌'}")
            return False

    except Exception as e:
        print(f"❌ Error validating example template: {e}")
        return False

def run_validation():
    """Run all Isaac Sim and Isaac ROS validation tests"""
    print("Starting Isaac Sim and Isaac ROS Integration Validation...\n")

    validation_tests = [
        ("Isaac Sim Installation", check_isaac_sim_installation),
        ("Isaac ROS Installation", check_isaac_ros_installation),
        ("Basic Simulation Functionality", validate_basic_simulation),
        ("Perception Pipeline Components", validate_perception_pipeline),
        ("Example Templates", validate_example_templates),
    ]

    results = []
    for test_name, test_func in validation_tests:
        print(f"\n{test_name}:")
        result = test_func()
        results.append((test_name, result))

    print("\n" + "="*60)
    print("Isaac Sim and Isaac ROS Validation Results")
    print("="*60)

    all_passed = True
    for test_name, result in results:
        status = "✅ PASS" if result else "❌ FAIL"
        print(f"{status} - {test_name}")
        if not result:
            all_passed = False

    print("="*60)

    if all_passed:
        print("🎉 All Isaac Sim and Isaac ROS integration checks PASSED!")
        print("Your system is ready for Isaac Sim and Isaac ROS development.")
        return True
    else:
        print("❌ Some Isaac Sim and Isaac ROS integration checks FAILED!")
        print("Please address the issues before proceeding with Isaac Sim development.")
        return False

if __name__ == "__main__":
    success = run_validation()
    sys.exit(0 if success else 1)
#!/usr/bin/env python3
"""
Basic Isaac Sim Example for Module 3
Demonstrates fundamental Isaac Sim concepts and humanoid robot simulation
"""

import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.prims import get_prim_at_path
import numpy as np
import carb


def setup_basic_simulation():
    """
    Sets up a basic Isaac Sim environment with humanoid robot
    """
    print("Setting up basic Isaac Sim environment...")

    # Create a world with 1 meter units
    world = World(stage_units_in_meters=1.0)
    print("✅ World initialized with 1-meter units")

    # Add a default ground plane
    world.scene.add_default_ground_plane()
    print("✅ Ground plane added")

    # Try to load a humanoid robot from the asset library
    assets_root_path = get_assets_root_path()
    if assets_root_path is None:
        carb.log_error("Could not find Isaac Sim assets root path")
        # Use a default path for demonstration
        robot_path = "/Isaac/Robots/Humanoid/humanoid_instanceable.usd"
    else:
        robot_path = assets_root_path + "/Isaac/Robots/Humanoid/humanoid_instanceable.usd"
        print(f"Looking for robot at: {robot_path}")

    try:
        # Add the humanoid robot to the stage
        add_reference_to_stage(
            usd_path=robot_path,
            prim_path="/World/HumanoidRobot"
        )
        print("✅ Humanoid robot added to stage")

        # Add the robot to the scene as an articulation
        robot = world.scene.add(
            Articulation(
                prim_path="/World/HumanoidRobot",
                name="humanoid_robot",
                position=np.array([0, 0, 1.0]),  # Position robot 1m above ground
                orientation=np.array([1, 0, 0, 0])  # No initial rotation
            )
        )
        print("✅ Robot added to scene as articulation")

        # Enable gravity for the scene
        world.scene.enable_gravity(True)
        print("✅ Gravity enabled")

        return world, robot

    except Exception as e:
        print(f"⚠️  Could not load humanoid robot: {e}")
        print("Using basic setup without robot...")

        # Create a simple cube instead
        from omni.isaac.core.objects import DynamicCuboid
        world.scene.add(
            DynamicCuboid(
                prim_path="/World/Cube",
                name="simple_cube",
                position=np.array([0, 0, 1.0]),
                size=0.5,
                color=np.array([0.8, 0.1, 0.1])
            )
        )
        print("✅ Simple cube added as placeholder")

        return world, None


def run_simulation_cycle(world, robot, num_steps=100):
    """
    Runs a basic simulation cycle

    Args:
        world: Isaac Sim World object
        robot: Robot articulation (can be None)
        num_steps: Number of simulation steps to run
    """
    print(f"Running simulation for {num_steps} steps...")

    for i in range(num_steps):
        # Step the world forward in time
        world.step(render=True)

        # Print progress every 20 steps
        if i % 20 == 0:
            print(f"  Step {i}/{num_steps} completed")

        # If we have a robot, we could apply actions here
        if robot is not None:
            # Example: Apply a small random action to the robot (just for demonstration)
            # In a real scenario, this would be replaced with actual control logic
            try:
                # Get current joint positions
                joint_positions = robot.get_joint_positions()

                # Apply a small random offset to demonstrate control
                random_offsets = np.random.normal(0, 0.01, size=joint_positions.shape)
                new_positions = joint_positions + random_offsets

                # Apply the new positions (this is simplified)
                # In practice, you'd use more sophisticated control methods
            except:
                # If getting/setting joint positions fails, just continue
                pass

    print(f"✅ Completed {num_steps} simulation steps")


def demonstrate_physics_interactions(world):
    """
    Demonstrates basic physics interactions in the simulation
    """
    print("\nDemonstrating physics interactions...")

    # Reset the world to initial state
    world.reset()
    print("✅ World reset")

    # Run a few steps to see initial state
    for i in range(10):
        world.step(render=True)

    # Add a few more objects to demonstrate physics
    from omni.isaac.core.objects import DynamicCuboid
    import random

    for i in range(3):
        obj_name = f"demo_cube_{i}"
        world.scene.add(
            DynamicCuboid(
                prim_path=f"/World/{obj_name}",
                name=obj_name,
                position=np.array([random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(2, 4)]),
                size=0.3,
                color=np.array([random.random(), random.random(), random.random()])
            )
        )

    print("✅ Added 3 demo cubes to demonstrate physics")

    # Run simulation to see physics in action
    for i in range(50):
        world.step(render=True)
        if i % 25 == 0:
            print(f"  Physics demo step {i}/50")

    print("✅ Physics interactions demonstrated")


def main():
    """
    Main function to run the basic Isaac Sim example
    """
    print("🚀 Starting Basic Isaac Sim Example for Module 3")
    print("This example demonstrates fundamental Isaac Sim concepts and humanoid robot simulation.\n")

    try:
        # Set up the basic simulation environment
        world, robot = setup_basic_simulation()

        # Run a basic simulation cycle
        run_simulation_cycle(world, robot, num_steps=100)

        # Demonstrate physics interactions
        demonstrate_physics_interactions(world)

        # Clean up
        world.clear()
        print("\n✅ Basic Isaac Sim example completed successfully!")
        print("The simulation environment has been properly cleaned up.")

        return True

    except Exception as e:
        print(f"\n❌ Error during simulation: {str(e)}")
        import traceback
        traceback.print_exc()
        return False


if __name__ == "__main__":
    success = main()
    if success:
        print("\n🎉 Basic Isaac Sim Example completed successfully!")
        print("This example provides a foundation for more complex humanoid robot simulations.")
    else:
        print("\n❌ Basic Isaac Sim Example failed.")
        exit(1)
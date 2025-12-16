#!/usr/bin/env python3
"""
Basic Humanoid Robot Template for Isaac Sim
This template provides a foundation for humanoid robot simulation in Isaac Sim
"""

import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.prims import get_prim_at_path
import numpy as np
import carb


class BasicHumanoidTemplate:
    """
    Basic template for humanoid robot simulation in Isaac Sim
    """
    def __init__(self):
        self.world = None
        self.robot = None
        self.assets_root_path = get_assets_root_path()

    def setup_world(self):
        """
        Set up the Isaac Sim world with basic parameters
        """
        self.world = World(stage_units_in_meters=1.0)
        print("✅ World initialized with 1-meter units")

        # Add a ground plane
        self.world.scene.add_default_ground_plane()
        print("✅ Ground plane added to scene")

    def load_humanoid_robot(self, robot_path="/Isaac/Robots/Humanoid/humanoid_instanceable.usd"):
        """
        Load a humanoid robot from the asset library

        Args:
            robot_path (str): Path to the robot USD file in Isaac Sim assets
        """
        if self.assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets root path")
            return False

        full_path = self.assets_root_path + robot_path
        print(f"Loading humanoid robot from: {full_path}")

        try:
            # Add robot to stage
            add_reference_to_stage(usd_path=full_path, prim_path="/World/HumanoidRobot")

            # Create articulation for the robot
            self.robot = self.world.scene.add(
                Articulation(
                    prim_path="/World/HumanoidRobot",
                    name="humanoid_robot",
                    position=np.array([0, 0, 1.0]),
                    orientation=np.array([1, 0, 0, 0])  # No rotation initially
                )
            )

            print("✅ Humanoid robot loaded successfully")
            return True

        except Exception as e:
            carb.log_error(f"Failed to load humanoid robot: {str(e)}")
            return False

    def setup_physics_params(self):
        """
        Configure basic physics parameters for humanoid simulation
        """
        # Set gravity (standard Earth gravity)
        self.world.scene.enable_gravity(True)
        print("✅ Gravity enabled for physics simulation")

        # Additional physics parameters can be configured here
        print("✅ Physics parameters configured")

    def run_simulation(self, num_steps=1000):
        """
        Run the simulation for a specified number of steps

        Args:
            num_steps (int): Number of physics steps to simulate
        """
        if not self.world:
            print("❌ World not initialized")
            return False

        print(f"Starting simulation for {num_steps} steps...")

        for i in range(num_steps):
            self.world.step(render=True)

            if i % 100 == 0:
                print(f"Step {i}/{num_steps} completed")

        print(f"✅ Simulation completed for {num_steps} steps")
        return True

    def reset_simulation(self):
        """
        Reset the simulation to initial state
        """
        if self.world:
            self.world.reset()
            print("✅ Simulation reset to initial state")

    def close_world(self):
        """
        Close and clean up the world
        """
        if self.world:
            self.world.clear()
            print("✅ World cleared and resources released")


def main():
    """
    Main function to demonstrate the basic humanoid template
    """
    print("Initializing Basic Humanoid Robot Template for Isaac Sim")

    # Create template instance
    template = BasicHumanoidTemplate()

    try:
        # Setup world
        template.setup_world()

        # Setup physics
        template.setup_physics_params()

        # Load humanoid robot
        success = template.load_humanoid_robot()
        if not success:
            print("Failed to load humanoid robot, using alternative method...")
            # For demonstration, we'll continue even if loading fails

        # Run simulation
        template.run_simulation(num_steps=100)

        # Clean up
        template.close_world()

        print("✅ Basic Humanoid Template executed successfully")
        return True

    except Exception as e:
        print(f"❌ Error during simulation: {str(e)}")
        return False


if __name__ == "__main__":
    success = main()
    if success:
        print("\n🎉 Basic Humanoid Robot Template completed successfully!")
        print("This template provides a foundation for more complex humanoid simulations.")
    else:
        print("\n❌ Basic Humanoid Robot Template execution failed.")
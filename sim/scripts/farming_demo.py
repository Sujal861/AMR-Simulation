#!/usr/bin/env python3
"""
Farming Field Demo
Demonstrates the agricultural robot working in a realistic paddy field environment
"""

import pybullet as p
import pybullet_data
import numpy as np
import time
import os
import sys

# Add parent directory to path for imports
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(__file__))))

from sim.scripts.create_farming_field import FarmingFieldGenerator
from sim.scripts.launch_sim import RobotSimulation
from configs.poses import AGRICULTURAL_POSES, get_pose

class FarmingDemo:
    def __init__(self):
        """Initialize farming demo"""
        print("Initializing Farming Field Demo...")
        
        # Connect to PyBullet
        self.physics_client = p.connect(p.GUI)
        p.setGravity(0, 0, -9.81)
        
        # Set camera for farming view
        p.resetDebugVisualizerCamera(
            cameraDistance=20,
            cameraYaw=45,
            cameraPitch=-30,
            cameraTargetPosition=[0, 0, 0]
        )
        
        # Create farming field
        self.field_generator = FarmingFieldGenerator(self.physics_client)
        self.field_data = self.field_generator.create_complete_farming_field()
        
        # Create robot simulation without creating another field
        self.robot_sim = RobotSimulation(gui=False, skip_field_creation=True)  # We handle GUI ourselves
        
        print("[SUCCESS] Farming demo initialized")
    
    def demo_planting_sequence(self):
        """Demonstrate planting sequence"""
        print("\n=== Planting Sequence Demo ===")
        
        # Move to planting position
        print("Moving to planting position...")
        planting_pose = get_pose("approach_plant")
        self.robot_sim.set_joint_positions(planting_pose)
        time.sleep(2)
        
        # Simulate planting motion
        print("Planting seeds...")
        for i in range(3):
            # Lower arm
            current_pose = self.robot_sim.get_joint_positions()
            planting_pose[2] += 0.1  # Lower joint 3
            self.robot_sim.set_joint_positions(planting_pose)
            time.sleep(1)
            
            # Raise arm
            planting_pose[2] -= 0.1
            self.robot_sim.set_joint_positions(planting_pose)
            time.sleep(1)
            
            print(f"  Planted seed {i+1}")
        
        print("[OK] Planting sequence completed")
    
    def demo_watering_sequence(self):
        """Demonstrate watering sequence"""
        print("\n=== Watering Sequence Demo ===")
        
        # Move to watering position
        print("Moving to watering position...")
        watering_pose = get_pose("watering_position")
        self.robot_sim.set_joint_positions(watering_pose)
        time.sleep(2)
        
        # Simulate watering motion
        print("Watering plants...")
        for i in range(5):
            # Spray motion
            current_pose = self.robot_sim.get_joint_positions()
            watering_pose[5] += 0.2  # Rotate end effector
            self.robot_sim.set_joint_positions(watering_pose)
            time.sleep(0.5)
            
            watering_pose[5] -= 0.2
            self.robot_sim.set_joint_positions(watering_pose)
            time.sleep(0.5)
            
            print(f"  Watered area {i+1}")
        
        print("[OK] Watering sequence completed")
    
    def demo_harvesting_sequence(self):
        """Demonstrate harvesting sequence"""
        print("\n=== Harvesting Sequence Demo ===")
        
        # Move to harvesting position
        print("Moving to harvesting position...")
        harvest_pose = get_pose("harvest_position")
        self.robot_sim.set_joint_positions(harvest_pose)
        time.sleep(2)
        
        # Simulate harvesting motion
        print("Harvesting rice...")
        for i in range(4):
            # Cutting motion
            current_pose = self.robot_sim.get_joint_positions()
            harvest_pose[4] += 0.3  # Bend wrist
            self.robot_sim.set_joint_positions(harvest_pose)
            time.sleep(0.8)
            
            harvest_pose[4] -= 0.3
            self.robot_sim.set_joint_positions(harvest_pose)
            time.sleep(0.8)
            
            print(f"  Harvested plant {i+1}")
        
        print("[OK] Harvesting sequence completed")
    
    def demo_field_navigation(self):
        """Demonstrate navigation between fields"""
        print("\n=== Field Navigation Demo ===")
        
        # Navigate to different field positions
        field_positions = [
            ([-5, 0, 0], "Field 1 - Left Paddy"),
            ([0, 0, 0], "Irrigation Channel"),
            ([5, 0, 0], "Field 2 - Right Paddy"),
            ([0, 8, 0], "Terrain Feature - Hill"),
            ([0, 0, 0], "Return to Center")
        ]
        
        for position, description in field_positions:
            print(f"Navigating to {description}...")
            
            # Calculate robot base position
            robot_base_pos = [position[0], position[1], 0.1]
            p.resetBasePositionAndOrientation(
                self.robot_sim.robot_id, 
                robot_base_pos, 
                [0, 0, 0, 1]
            )
            
            # Move arm to survey position
            survey_pose = get_pose("survey_position")
            self.robot_sim.set_joint_positions(survey_pose)
            
            time.sleep(2)
            print(f"  [OK] Arrived at {description}")
        
        print("[OK] Field navigation completed")
    
    def demo_irrigation_control(self):
        """Demonstrate irrigation system control"""
        print("\n=== Irrigation Control Demo ===")
        
        # Move to irrigation control points
        irrigation_points = [
            ([-5, 0, 0.1], "Field 1 Inlet"),
            ([0, 0, 0.1], "Main Channel"),
            ([5, 0, 0.1], "Field 2 Inlet")
        ]
        
        for position, description in irrigation_points:
            print(f"Controlling {description}...")
            
            # Move robot to control point
            robot_base_pos = [position[0], position[1], 0.1]
            p.resetBasePositionAndOrientation(
                self.robot_sim.robot_id, 
                robot_base_pos, 
                [0, 0, 0, 1]
            )
            
            # Simulate valve control
            maintenance_pose = get_pose("maintenance_position")
            self.robot_sim.set_joint_positions(maintenance_pose)
            time.sleep(1)
            
            # Simulate valve turning
            for i in range(3):
                maintenance_pose[5] += 0.5
                self.robot_sim.set_joint_positions(maintenance_pose)
                time.sleep(0.3)
                
                maintenance_pose[5] -= 0.5
                self.robot_sim.set_joint_positions(maintenance_pose)
                time.sleep(0.3)
            
            print(f"  [OK] {description} controlled")
        
        print("[OK] Irrigation control completed")
    
    def run_complete_demo(self):
        """Run the complete farming demo"""
        print("🌾 Starting Complete Farming Field Demo 🌾")
        print("=" * 50)
        
        try:
            # Run all demo sequences
            self.demo_field_navigation()
            self.demo_planting_sequence()
            self.demo_watering_sequence()
            self.demo_irrigation_control()
            self.demo_harvesting_sequence()
            
            # Return to home
            print("\n=== Returning to Home Position ===")
            home_pose = get_pose("home")
            self.robot_sim.set_joint_positions(home_pose)
            time.sleep(2)
            
            print("\n🎉 Complete Farming Demo Finished! 🎉")
            print("\nDemo Summary:")
            print("- Field navigation between paddy fields")
            print("- Planting sequence with seed placement")
            print("- Watering sequence with spray motion")
            print("- Irrigation system control")
            print("- Harvesting sequence with cutting motion")
            print("- Return to home position")
            
        except KeyboardInterrupt:
            print("\nDemo stopped by user")
        
        # Keep simulation running
        print("\nSimulation running. Press Ctrl+C to exit.")
        try:
            while True:
                p.stepSimulation()
                time.sleep(1.0/240.0)
        except KeyboardInterrupt:
            print("\nSimulation stopped")
        finally:
            p.disconnect()

def main():
    """Main function"""
    print("Agricultural Robot Farming Field Demo")
    print("=" * 50)
    
    # Create and run demo
    demo = FarmingDemo()
    demo.run_complete_demo()

if __name__ == "__main__":
    main()

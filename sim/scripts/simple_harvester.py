#!/usr/bin/env python3
"""
Simple Autonomous Paddy Harvester for AMR
Automatically picks paddy plants one by one with simple controls
"""
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..'))

import pybullet as p
import time
import numpy as np
import math
from sim.scripts.launch_sim import RobotSimulation
from configs.poses import get_pose, get_pose_description

class SimpleHarvester:
    def __init__(self):
        """Initialize simple harvester"""
        print("Initializing Simple Autonomous Paddy Harvester...")
        self.sim = RobotSimulation(gui=True, skip_field_creation=False)
        self.running = True
        
        # Harvesting parameters
        self.movement_speed = 1.0  # m/s
        self.rotation_speed = 0.5  # rad/s
        
        # Field parameters (simplified - just one field)
        self.field_center = [-5, 0, 0]  # Left field
        self.plants_per_row = 3
        self.rows_per_field = 3
        self.plant_spacing = 1.5  # meters
        
        # Current position
        self.current_row = 0
        self.current_plant = 0
        
        # Storage area
        self.storage_area = [0, 2, 0]  # Behind the robot
        
        # Statistics
        self.harvested_count = 0
        self.total_plants = self.plants_per_row * self.rows_per_field
        
        print(f"Simple Harvester Ready!")
        print(f"Total plants to harvest: {self.total_plants}")
        print(f"Field layout: {self.rows_per_field} rows x {self.plants_per_row} plants")
    
    def get_plant_position(self, row, plant):
        """Get position of a specific plant"""
        field_x, field_y, field_z = self.field_center
        
        # Calculate plant position within the field
        plant_x = field_x + (plant - 1) * self.plant_spacing  # Start from left
        plant_y = field_y + (row - 1) * self.plant_spacing    # Start from front
        plant_z = field_z + 0.1  # Slightly above ground
        
        return [plant_x, plant_y, plant_z]
    
    def get_robot_position(self):
        """Get current robot position"""
        base_pos, _ = self.sim.get_base_position_and_orientation()
        return base_pos
    
    def move_to_position(self, target_position, tolerance=0.2):
        """Move robot to target position"""
        print(f"Moving to position: {[round(x, 2) for x in target_position]}")
        
        max_time = 10.0  # Maximum time to reach position
        start_time = time.time()
        
        while time.time() - start_time < max_time:
            current_pos = self.get_robot_position()
            distance = np.linalg.norm(np.array(target_position[:2]) - np.array(current_pos[:2]))
            
            if distance < tolerance:
                break
            
            # Calculate direction
            direction = np.array(target_position[:2]) - np.array(current_pos[:2])
            if np.linalg.norm(direction) > 0:
                direction = direction / np.linalg.norm(direction)
                
                # Set wheel velocities for movement
                wheel_velocities = [
                    self.movement_speed, self.movement_speed,
                    self.movement_speed, self.movement_speed
                ]
                
                self.sim.set_wheel_velocities(wheel_velocities)
                time.sleep(0.01)
        
        # Stop wheels
        self.sim.set_wheel_velocities([0, 0, 0, 0])
        print("Position reached")
    
    def rotate_to_face_plant(self, plant_position):
        """Rotate robot to face the plant"""
        current_pos = self.get_robot_position()
        
        # Calculate angle to plant
        direction = np.array(plant_position[:2]) - np.array(current_pos[:2])
        target_angle = math.atan2(direction[1], direction[0])
        
        # Get current orientation (simplified)
        _, current_orn = self.sim.get_base_position_and_orientation()
        current_angle = 0  # Assume robot starts facing forward
        
        # Calculate rotation needed
        angle_diff = target_angle - current_angle
        
        # Normalize angle
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        if abs(angle_diff) > 0.1:
            print(f"Rotating {angle_diff:.2f} radians to face plant")
            
            # Set wheel velocities for rotation
            direction = 1 if angle_diff > 0 else -1
            wheel_velocities = [
                direction * self.rotation_speed, -direction * self.rotation_speed,
                -direction * self.rotation_speed, direction * self.rotation_speed
            ]
            
            rotation_time = abs(angle_diff) / self.rotation_speed
            start_time = time.time()
            
            while time.time() - start_time < rotation_time:
                self.sim.set_wheel_velocities(wheel_velocities)
                time.sleep(0.01)
            
            # Stop wheels
            self.sim.set_wheel_velocities([0, 0, 0, 0])
            print("Rotation complete")
    
    def move_arm_to_pose(self, pose_name, duration=2.0):
        """Move arm to specified pose"""
        pose = get_pose(pose_name)
        description = get_pose_description(pose_name)
        print(f"Moving arm to {pose_name}: {description}")
        
        # Get current joint positions
        current_angles = self.sim.get_joint_positions()
        
        # Interpolate to target pose
        steps = int(duration * 50)  # 50 Hz control
        for i in range(steps + 1):
            t = i / steps
            interpolated_angles = []
            for j in range(6):
                angle = current_angles[j] + t * (pose[j] - current_angles[j])
                interpolated_angles.append(angle)
            
            self.sim.set_joint_positions(interpolated_angles)
            time.sleep(0.02)
    
    def control_gripper(self, action, duration=1.0):
        """Control gripper"""
        position = 0.04 if action == 'open' else 0.0
        print(f"Gripper {action}")
        
        self.sim.set_gripper_position(position)
        time.sleep(duration)
    
    def harvest_single_plant(self):
        """Harvest a single paddy plant"""
        # Get plant position
        plant_pos = self.get_plant_position(self.current_row, self.current_plant)
        
        print(f"\n=== Harvesting Plant {self.current_plant + 1} in Row {self.current_row + 1} ===")
        print(f"Plant position: {[round(x, 2) for x in plant_pos]}")
        
        # Step 1: Move to plant position (0.5m away)
        robot_target_pos = [plant_pos[0] - 0.5, plant_pos[1], plant_pos[2]]
        self.move_to_position(robot_target_pos)
        
        # Step 2: Rotate to face plant
        self.rotate_to_face_plant(plant_pos)
        
        # Step 3: Execute harvesting sequence
        print("Executing harvesting sequence...")
        
        # Prepare arm
        self.move_arm_to_pose('navigation_ready', 1.5)
        time.sleep(0.5)
        
        # Approach plant
        self.move_arm_to_pose('approach_plant', 2.0)
        time.sleep(0.5)
        
        # Open gripper
        self.control_gripper('open', 1.0)
        
        # Move to harvest position
        self.move_arm_to_pose('harvest_position', 2.0)
        time.sleep(0.5)
        
        # Close gripper to grasp
        self.control_gripper('close', 1.0)
        
        # Lift plant
        self.move_arm_to_pose('home', 2.0)
        time.sleep(0.5)
        
        # Move to storage area
        print("Moving to storage area...")
        self.move_to_position(self.storage_area)
        
        # Drop plant
        self.move_arm_to_pose('watering_position', 2.0)
        time.sleep(0.5)
        
        # Open gripper to release
        self.control_gripper('open', 1.0)
        
        # Return to home position
        self.move_arm_to_pose('home', 2.0)
        
        # Update statistics
        self.harvested_count += 1
        print(f"✓ Plant harvested! ({self.harvested_count}/{self.total_plants})")
        
        return True
    
    def move_to_next_plant(self):
        """Move to next plant position"""
        self.current_plant += 1
        
        # Check if we need to move to next row
        if self.current_plant >= self.plants_per_row:
            self.current_plant = 0
            self.current_row += 1
            
            # Check if all rows are done
            if self.current_row >= self.rows_per_field:
                return False  # All plants harvested
        
        return True
    
    def reset_position(self):
        """Reset robot to starting position"""
        print("Resetting robot position...")
        self.move_to_position([0, 0, 0.1])
        self.move_arm_to_pose('home')
        self.control_gripper('close')
        
        # Reset counters
        self.current_row = 0
        self.current_plant = 0
        self.harvested_count = 0
        
        print("Position reset complete")
    
    def run_autonomous_harvesting(self):
        """Run the complete autonomous harvesting process"""
        print("\n=== AUTONOMOUS PADDY HARVESTING STARTED ===")
        print("The AMR will now automatically harvest all paddy plants")
        print("Press Ctrl+C to stop at any time")
        
        try:
            while self.harvested_count < self.total_plants:
                # Harvest current plant
                success = self.harvest_single_plant()
                
                if not success:
                    break
                
                # Move to next plant
                if not self.move_to_next_plant():
                    break
                
                # Small pause between plants
                time.sleep(1.0)
            
            if self.harvested_count >= self.total_plants:
                print("\n=== HARVESTING COMPLETE ===")
                print(f"Successfully harvested {self.harvested_count} paddy plants!")
                print("All fields have been cleared")
            else:
                print("\nHarvesting stopped by user")
                
        except KeyboardInterrupt:
            print("\nHarvesting interrupted by user")
    
    def show_menu(self):
        """Show control menu"""
        print("\n" + "="*50)
        print("AUTONOMOUS PADDY HARVESTER")
        print("="*50)
        print(f"Progress: {self.harvested_count}/{self.total_plants} plants harvested")
        print(f"Current position: Row {self.current_row + 1}, Plant {self.current_plant + 1}")
        print("\nOptions:")
        print("1. Start autonomous harvesting")
        print("2. Harvest single plant")
        print("3. Reset position")
        print("4. Show plant positions")
        print("5. Quit")
        print("="*50)
    
    def show_plant_positions(self):
        """Show all plant positions"""
        print("\nPlant Positions:")
        for row in range(self.rows_per_field):
            for plant in range(self.plants_per_row):
                pos = self.get_plant_position(row, plant)
                status = "✓" if (row < self.current_row or (row == self.current_row and plant < self.current_plant)) else "○"
                print(f"  {status} Row {row + 1}, Plant {plant + 1}: [{pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f}]")
    
    def run(self):
        """Main run loop"""
        print("\n=== SIMPLE AUTONOMOUS PADDY HARVESTER ===")
        print("This robot will automatically harvest paddy plants from the field")
        
        while self.running:
            try:
                self.show_menu()
                choice = input("Enter your choice (1-5): ").strip()
                
                if choice == '1':
                    self.run_autonomous_harvesting()
                elif choice == '2':
                    if self.harvested_count < self.total_plants:
                        self.harvest_single_plant()
                        self.move_to_next_plant()
                    else:
                        print("All plants have been harvested!")
                elif choice == '3':
                    self.reset_position()
                elif choice == '4':
                    self.show_plant_positions()
                elif choice == '5':
                    print("Quitting...")
                    self.running = False
                else:
                    print("Invalid choice. Please enter 1-5.")
                    
            except KeyboardInterrupt:
                print("\nProgram interrupted by user")
                break
            except Exception as e:
                print(f"Error: {e}")
        
        self.cleanup()
    
    def cleanup(self):
        """Cleanup resources"""
        self.running = False
        p.disconnect()
        print("Simple Harvester closed")

def main():
    """Main function"""
    harvester = SimpleHarvester()
    harvester.run()

if __name__ == "__main__":
    main()

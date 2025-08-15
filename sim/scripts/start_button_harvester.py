#!/usr/bin/env python3
"""
Start Button Autonomous Paddy Harvester for AMR
Press 'START' button to begin automatic paddy harvesting
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

class StartButtonHarvester:
    def __init__(self):
        """Initialize start button harvester"""
        print("Initializing Start Button Autonomous Paddy Harvester...")
        self.sim = RobotSimulation(gui=True, skip_field_creation=False)
        self.running = True
        self.harvesting = False
        
        # Harvesting parameters
        self.movement_speed = 0.8  # m/s
        self.rotation_speed = 0.4  # rad/s
        
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
        
        print(f"Start Button Harvester Ready!")
        print(f"Total plants to harvest: {self.total_plants}")
        print(f"Field layout: {self.rows_per_field} rows x {self.plants_per_row} plants")
        print(f"Press 'START' to begin autonomous harvesting!")
    
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
        try:
            base_pos, _ = self.sim.get_base_position_and_orientation()
            return base_pos
        except:
            return [0, 0, 0.1]  # Fallback position
    
    def move_to_position(self, target_position, tolerance=0.3):
        """Move robot to target position"""
        print(f"Moving to position: {[round(x, 2) for x in target_position]}")
        
        max_time = 15.0  # Maximum time to reach position
        start_time = time.time()
        
        while time.time() - start_time < max_time:
            try:
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
                    time.sleep(0.02)
            except:
                print("Movement error, continuing...")
                break
        
        # Stop wheels
        try:
            self.sim.set_wheel_velocities([0, 0, 0, 0])
        except:
            pass
        print("Position reached")
    
    def rotate_to_face_plant(self, plant_position):
        """Rotate robot to face the plant"""
        try:
            current_pos = self.get_robot_position()
            
            # Calculate angle to plant
            direction = np.array(plant_position[:2]) - np.array(current_pos[:2])
            target_angle = math.atan2(direction[1], direction[0])
            
            # Simplified rotation (assume robot starts facing forward)
            current_angle = 0
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
                    try:
                        self.sim.set_wheel_velocities(wheel_velocities)
                        time.sleep(0.02)
                    except:
                        break
                
                # Stop wheels
                try:
                    self.sim.set_wheel_velocities([0, 0, 0, 0])
                except:
                    pass
                print("Rotation complete")
        except Exception as e:
            print(f"Rotation error: {e}")
    
    def move_arm_to_pose(self, pose_name, duration=2.0):
        """Move arm to specified pose"""
        try:
            pose = get_pose(pose_name)
            description = get_pose_description(pose_name)
            print(f"Moving arm to {pose_name}: {description}")
            
            # Get current joint positions
            current_angles = self.sim.get_joint_positions()
            
            # Interpolate to target pose
            steps = int(duration * 30)  # 30 Hz control
            for i in range(steps + 1):
                t = i / steps
                interpolated_angles = []
                for j in range(6):
                    angle = current_angles[j] + t * (pose[j] - current_angles[j])
                    interpolated_angles.append(angle)
                
                try:
                    self.sim.set_joint_positions(interpolated_angles)
                    time.sleep(0.03)
                except:
                    break
        except Exception as e:
            print(f"Arm movement error: {e}")
    
    def control_gripper(self, action, duration=1.0):
        """Control gripper"""
        try:
            position = 0.04 if action == 'open' else 0.0
            print(f"Gripper {action}")
            
            self.sim.set_gripper_position(position)
            time.sleep(duration)
        except Exception as e:
            print(f"Gripper error: {e}")
    
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
        print("\n" + "="*60)
        print("🚀 AUTONOMOUS PADDY HARVESTING STARTED 🚀")
        print("="*60)
        print("The AMR will now automatically harvest all paddy plants")
        print("Press Ctrl+C to stop at any time")
        print("="*60)
        
        self.harvesting = True
        
        try:
            while self.harvesting and self.harvested_count < self.total_plants:
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
                print("\n" + "="*60)
                print("🎉 HARVESTING COMPLETE 🎉")
                print("="*60)
                print(f"Successfully harvested {self.harvested_count} paddy plants!")
                print("All fields have been cleared")
                print("="*60)
            else:
                print("\nHarvesting stopped by user")
                
        except KeyboardInterrupt:
            print("\nHarvesting interrupted by user")
        finally:
            self.harvesting = False
    
    def show_start_screen(self):
        """Show the start screen with big START button"""
        print("\n" + "="*60)
        print("🤖 AUTONOMOUS PADDY HARVESTER 🤖")
        print("="*60)
        print(f"Progress: {self.harvested_count}/{self.total_plants} plants harvested")
        print(f"Current position: Row {self.current_row + 1}, Plant {self.current_plant + 1}")
        print("\n" + "="*60)
        print("           🟢 START BUTTON 🟢")
        print("="*60)
        print("Press 'START' to begin autonomous harvesting")
        print("Press 'RESET' to reset robot position")
        print("Press 'QUIT' to exit")
        print("="*60)
    
    def run(self):
        """Main run loop"""
        print("\n=== START BUTTON AUTONOMOUS PADDY HARVESTER ===")
        print("This robot will automatically harvest paddy plants from the field")
        print("Just press START to begin!")
        
        while self.running:
            try:
                self.show_start_screen()
                choice = input("Enter command (START/RESET/QUIT): ").strip().upper()
                
                if choice == 'START':
                    if self.harvested_count >= self.total_plants:
                        print("All plants have been harvested! Press RESET to start over.")
                    else:
                        self.run_autonomous_harvesting()
                elif choice == 'RESET':
                    self.reset_position()
                elif choice == 'QUIT':
                    print("Quitting...")
                    self.running = False
                else:
                    print("Invalid command. Please enter START, RESET, or QUIT.")
                    
            except KeyboardInterrupt:
                print("\nProgram interrupted by user")
                break
            except Exception as e:
                print(f"Error: {e}")
                time.sleep(1.0)
        
        self.cleanup()
    
    def cleanup(self):
        """Cleanup resources"""
        self.running = False
        self.harvesting = False
        try:
            p.disconnect()
        except:
            pass
        print("Start Button Harvester closed")

def main():
    """Main function"""
    harvester = StartButtonHarvester()
    harvester.run()

if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
Autonomous Paddy Harvester for AMR
Automatically picks paddy plants one by one when start button is pressed
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

class AutonomousHarvester:
    def __init__(self):
        """Initialize autonomous harvester"""
        print("Initializing Autonomous Paddy Harvester...")
        self.sim = RobotSimulation(gui=True, skip_field_creation=False)
        self.running = False
        self.harvesting = False
        
        # Harvesting parameters
        self.movement_speed = 1.5  # m/s
        self.rotation_speed = 0.8  # rad/s
        self.arm_speed = 0.3       # rad/s
        self.gripper_speed = 0.5   # m/s
        
        # Field parameters
        self.field_positions = [
            (-5, 0, 0),  # Left field
            (5, 0, 0)    # Right field
        ]
        self.current_field = 0
        self.current_row = 0
        self.current_plant = 0
        
        # Plant grid parameters (5x5 grid per field)
        self.plants_per_row = 5
        self.rows_per_field = 5
        self.plant_spacing = 1.0  # meters
        
        # Harvesting sequence
        self.harvest_sequence = [
            'navigation_ready',
            'approach_plant',
            'harvest_position',
            'gripper_open',
            'grasp_plant',
            'gripper_close',
            'lift_plant',
            'transport_position',
            'drop_position',
            'gripper_open',
            'home'
        ]
        
        # Storage area for harvested paddy
        self.storage_area = [0, 3, 0]  # Behind the robot
        
        # Statistics
        self.harvested_count = 0
        self.total_plants = self.plants_per_row * self.rows_per_field * len(self.field_positions)
        
        print(f"Autonomous Harvester Ready!")
        print(f"Total plants to harvest: {self.total_plants}")
        print(f"Press 's' to start/stop harvesting")
        print(f"Press 'r' to reset position")
        print(f"Press 'q' to quit")
    
    def get_plant_position(self, field_idx, row, plant):
        """Get position of a specific plant"""
        field_x, field_y, field_z = self.field_positions[field_idx]
        
        # Calculate plant position within the field
        plant_x = field_x + (plant - 2) * self.plant_spacing  # Center the grid
        plant_y = field_y + (row - 2) * self.plant_spacing
        plant_z = field_z + 0.1  # Slightly above ground
        
        return [plant_x, plant_y, plant_z]
    
    def get_robot_position(self):
        """Get current robot position"""
        base_pos, _ = self.sim.get_base_position_and_orientation()
        return base_pos
    
    def move_to_position(self, target_position, tolerance=0.1):
        """Move robot to target position"""
        print(f"Moving to position: {[round(x, 2) for x in target_position]}")
        
        while True:
            current_pos = self.get_robot_position()
            distance = np.linalg.norm(np.array(target_position[:2]) - np.array(current_pos[:2]))
            
            if distance < tolerance:
                break
            
            # Calculate direction
            direction = np.array(target_position[:2]) - np.array(current_pos[:2])
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
        
        # Get current orientation
        _, current_orn = self.sim.get_base_position_and_orientation()
        current_angle = math.atan2(2 * (current_orn[3] * current_orn[2] + current_orn[0] * current_orn[1]),
                                  1 - 2 * (current_orn[1] * current_orn[1] + current_orn[2] * current_orn[2]))
        
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
            
            start_time = time.time()
            while abs(angle_diff) > 0.1 and time.time() - start_time < 5.0:
                self.sim.set_wheel_velocities(wheel_velocities)
                time.sleep(0.01)
                
                # Recalculate angle difference
                _, current_orn = self.sim.get_base_position_and_orientation()
                current_angle = math.atan2(2 * (current_orn[3] * current_orn[2] + current_orn[0] * current_orn[1]),
                                          1 - 2 * (current_orn[1] * current_orn[1] + current_orn[2] * current_orn[2]))
                angle_diff = target_angle - current_angle
                
                while angle_diff > math.pi:
                    angle_diff -= 2 * math.pi
                while angle_diff < -math.pi:
                    angle_diff += 2 * math.pi
            
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
            if not self.running:
                break
            
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
        if not self.harvesting:
            return False
        
        # Get plant position
        plant_pos = self.get_plant_position(self.current_field, self.current_row, self.current_plant)
        
        print(f"\n=== Harvesting Plant {self.current_plant + 1} in Row {self.current_row + 1} ===")
        
        # Step 1: Move to plant position
        robot_target_pos = [plant_pos[0] - 0.5, plant_pos[1], plant_pos[2]]  # 0.5m away from plant
        self.move_to_position(robot_target_pos)
        
        # Step 2: Rotate to face plant
        self.rotate_to_face_plant(plant_pos)
        
        # Step 3: Execute harvesting sequence
        for step in self.harvest_sequence:
            if not self.harvesting:
                break
            
            if step == 'navigation_ready':
                self.move_arm_to_pose('navigation_ready', 1.5)
            elif step == 'approach_plant':
                self.move_arm_to_pose('approach_plant', 2.0)
            elif step == 'harvest_position':
                self.move_arm_to_pose('harvest_position', 2.0)
            elif step == 'gripper_open':
                self.control_gripper('open', 1.0)
            elif step == 'grasp_plant':
                # Move closer to plant
                self.move_arm_to_pose('approach_plant', 1.0)
            elif step == 'gripper_close':
                self.control_gripper('close', 1.0)
            elif step == 'lift_plant':
                self.move_arm_to_pose('home', 2.0)
            elif step == 'transport_position':
                # Move to storage area
                self.move_to_position(self.storage_area)
            elif step == 'drop_position':
                self.move_arm_to_pose('watering_position', 2.0)
            elif step == 'home':
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
            
            # Check if we need to move to next field
            if self.current_row >= self.rows_per_field:
                self.current_row = 0
                self.current_field += 1
                
                # Check if all fields are done
                if self.current_field >= len(self.field_positions):
                    return False  # All plants harvested
        
        return True
    
    def reset_position(self):
        """Reset robot to starting position"""
        print("Resetting robot position...")
        self.move_to_position([0, 0, 0.1])
        self.move_arm_to_pose('home')
        self.control_gripper('close')
        
        # Reset counters
        self.current_field = 0
        self.current_row = 0
        self.current_plant = 0
        self.harvested_count = 0
        
        print("Position reset complete")
    
    def run_autonomous_harvesting(self):
        """Run the complete autonomous harvesting process"""
        print("\n=== AUTONOMOUS PADDY HARVESTING STARTED ===")
        print("The AMR will now automatically harvest all paddy plants")
        print("Press 's' to pause/resume, 'r' to reset, 'q' to quit")
        
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
                time.sleep(0.5)
            
            if self.harvested_count >= self.total_plants:
                print("\n=== HARVESTING COMPLETE ===")
                print(f"Successfully harvested {self.harvested_count} paddy plants!")
                print("All fields have been cleared")
            else:
                print("\nHarvesting stopped by user")
                
        except KeyboardInterrupt:
            print("\nHarvesting interrupted by user")
        finally:
            self.harvesting = False
    
    def handle_keyboard_input(self):
        """Handle keyboard input for control"""
        import msvcrt  # Windows-specific
        
        if msvcrt.kbhit():
            key = msvcrt.getch().decode('utf-8').lower()
            
            if key == 's':
                if not self.harvesting:
                    print("\nStarting autonomous harvesting...")
                    self.run_autonomous_harvesting()
                else:
                    print("\nPausing harvesting...")
                    self.harvesting = False
            elif key == 'r':
                print("\nResetting position...")
                self.reset_position()
            elif key == 'q':
                print("\nQuitting...")
                self.running = False
                return False
        
        return True
    
    def run(self):
        """Main run loop"""
        self.running = True
        
        print("\n=== AUTONOMOUS PADDY HARVESTER ===")
        print("Controls:")
        print("  's' - Start/Stop harvesting")
        print("  'r' - Reset robot position")
        print("  'q' - Quit")
        print("\nPress 's' to start autonomous harvesting...")
        
        try:
            while self.running:
                if not self.handle_keyboard_input():
                    break
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            print("\nProgram interrupted by user")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Cleanup resources"""
        self.running = False
        self.harvesting = False
        p.disconnect()
        print("Autonomous Harvester closed")

def main():
    """Main function"""
    harvester = AutonomousHarvester()
    harvester.run()

if __name__ == "__main__":
    main()

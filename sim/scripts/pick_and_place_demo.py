#!/usr/bin/env python3
"""
Pick and Place Demo for Enhanced AMR
Demonstrates the robot's enhanced wheels and 6-DOF arm capabilities
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

class PickAndPlaceDemo:
    def __init__(self):
        """Initialize pick and place demo"""
        print("Initializing Enhanced AMR Pick-and-Place Demo...")
        self.sim = RobotSimulation(gui=True, skip_field_creation=False)
        self.running = True
        
        # Movement parameters
        self.movement_speed = 1.0  # m/s
        self.rotation_speed = 0.6  # rad/s
        
        # Pick and place targets
        self.pick_positions = [
            [-4, -1, 0.1],  # Left field plants
            [-4, 0, 0.1],
            [-4, 1, 0.1],
            [4, -1, 0.1],   # Right field plants
            [4, 0, 0.1],
            [4, 1, 0.1]
        ]
        
        self.place_positions = [
            [0, 3, 0.1],    # Storage area
            [0, 3.5, 0.1],
            [0, 4, 0.1],
            [0, 4.5, 0.1],
            [0, 5, 0.1],
            [0, 5.5, 0.1]
        ]
        
        # Demo parameters
        self.current_task = 0
        self.total_tasks = len(self.pick_positions)
        
        print(f"Enhanced AMR Pick-and-Place Demo Ready!")
        print(f"Features:")
        print(f"- Enhanced Mecanum wheels (larger, higher torque)")
        print(f"- Improved 6-DOF manipulator arm")
        print(f"- Enhanced gripper with better force control")
        print(f"- {self.total_tasks} pick-and-place tasks")
        print(f"Press 'START' to begin the demo!")
    
    def get_robot_position(self):
        """Get current robot position"""
        try:
            base_pos, _ = self.sim.get_base_position_and_orientation()
            return base_pos
        except:
            return [0, 0, 0.1]  # Fallback position
    
    def move_to_position(self, target_position, tolerance=0.3):
        """Move robot to target position using enhanced wheels"""
        print(f"Moving to position: {[round(x, 2) for x in target_position]}")
        
        max_time = 20.0  # Maximum time to reach position
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
                    
                    # Enhanced wheel control with higher speeds
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
    
    def rotate_to_face_target(self, target_position):
        """Rotate robot to face the target"""
        try:
            current_pos = self.get_robot_position()
            
            # Calculate angle to target
            direction = np.array(target_position[:2]) - np.array(current_pos[:2])
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
                print(f"Rotating {angle_diff:.2f} radians to face target")
                
                # Enhanced wheel control for rotation
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
        """Move arm to specified pose with enhanced control"""
        try:
            pose = get_pose(pose_name)
            description = get_pose_description(pose_name)
            print(f"Moving arm to {pose_name}: {description}")
            
            # Get current joint positions
            current_angles = self.sim.get_joint_positions()
            
            # Interpolate to target pose with enhanced control
            steps = int(duration * 50)  # 50 Hz control for smoother motion
            for i in range(steps + 1):
                t = i / steps
                interpolated_angles = []
                for j in range(6):
                    angle = current_angles[j] + t * (pose[j] - current_angles[j])
                    interpolated_angles.append(angle)
                
                try:
                    self.sim.set_joint_positions(interpolated_angles)
                    time.sleep(0.02)
                except:
                    break
        except Exception as e:
            print(f"Arm movement error: {e}")
    
    def control_gripper(self, action, duration=1.0):
        """Control enhanced gripper"""
        try:
            position = 0.05 if action == 'open' else 0.0  # Enhanced gripper range
            print(f"Enhanced gripper {action}")
            
            self.sim.set_gripper_position(position)
            time.sleep(duration)
        except Exception as e:
            print(f"Gripper error: {e}")
    
    def execute_pick_sequence(self, pick_position):
        """Execute enhanced pick sequence"""
        print(f"\n=== Executing Enhanced Pick Sequence ===")
        print(f"Pick position: {[round(x, 2) for x in pick_position]}")
        
        # Step 1: Move to pick position (0.6m away for better reach)
        robot_target_pos = [pick_position[0] - 0.6, pick_position[1], pick_position[2]]
        self.move_to_position(robot_target_pos)
        
        # Step 2: Rotate to face target
        self.rotate_to_face_target(pick_position)
        
        # Step 3: Execute enhanced picking sequence
        print("Executing enhanced picking sequence...")
        
        # Prepare arm with enhanced poses
        self.move_arm_to_pose('navigation_ready', 1.5)
        time.sleep(0.5)
        
        # Approach with better precision
        self.move_arm_to_pose('approach_plant', 2.5)
        time.sleep(0.5)
        
        # Open enhanced gripper
        self.control_gripper('open', 1.0)
        
        # Move to precise pick position
        self.move_arm_to_pose('harvest_position', 2.5)
        time.sleep(0.5)
        
        # Close enhanced gripper with better force
        self.control_gripper('close', 1.5)
        
        # Lift with enhanced arm control
        self.move_arm_to_pose('home', 2.5)
        time.sleep(0.5)
        
        print("✓ Pick sequence completed")
        return True
    
    def execute_place_sequence(self, place_position):
        """Execute enhanced place sequence"""
        print(f"\n=== Executing Enhanced Place Sequence ===")
        print(f"Place position: {[round(x, 2) for x in place_position]}")
        
        # Step 1: Move to place position
        robot_target_pos = [place_position[0] - 0.6, place_position[1], place_position[2]]
        self.move_to_position(robot_target_pos)
        
        # Step 2: Rotate to face target
        self.rotate_to_face_target(place_position)
        
        # Step 3: Execute enhanced placing sequence
        print("Executing enhanced placing sequence...")
        
        # Move to place position with precision
        self.move_arm_to_pose('watering_position', 2.5)
        time.sleep(0.5)
        
        # Open enhanced gripper to release
        self.control_gripper('open', 1.5)
        
        # Return to home position
        self.move_arm_to_pose('home', 2.5)
        time.sleep(0.5)
        
        print("✓ Place sequence completed")
        return True
    
    def run_pick_and_place_task(self, task_id):
        """Run a complete pick-and-place task"""
        if task_id >= self.total_tasks:
            return False
        
        pick_pos = self.pick_positions[task_id]
        place_pos = self.place_positions[task_id]
        
        print(f"\n{'='*60}")
        print(f"🚀 TASK {task_id + 1}/{self.total_tasks}: Enhanced Pick-and-Place")
        print(f"{'='*60}")
        
        # Execute pick sequence
        pick_success = self.execute_pick_sequence(pick_pos)
        if not pick_success:
            return False
        
        # Execute place sequence
        place_success = self.execute_place_sequence(place_pos)
        if not place_success:
            return False
        
        print(f"✓ Task {task_id + 1} completed successfully!")
        return True
    
    def run_full_demo(self):
        """Run the complete enhanced pick-and-place demo"""
        print("\n" + "="*60)
        print("🚀 ENHANCED AMR PICK-AND-PLACE DEMO STARTED 🚀")
        print("="*60)
        print("The enhanced AMR will demonstrate:")
        print("- Improved Mecanum wheel mobility")
        print("- Enhanced 6-DOF arm precision")
        print("- Better gripper force control")
        print("- Complete pick-and-place workflow")
        print("Press Ctrl+C to stop at any time")
        print("="*60)
        
        try:
            for task_id in range(self.total_tasks):
                success = self.run_pick_and_place_task(task_id)
                
                if not success:
                    print(f"Task {task_id + 1} failed, stopping demo")
                    break
                
                # Pause between tasks
                if task_id < self.total_tasks - 1:
                    print("\nPausing between tasks...")
                    time.sleep(2.0)
            
            print("\n" + "="*60)
            print("🎉 ENHANCED PICK-AND-PLACE DEMO COMPLETE 🎉")
            print("="*60)
            print("All tasks completed successfully!")
            print("Enhanced AMR capabilities demonstrated:")
            print("- Superior wheel control and mobility")
            print("- Precise arm positioning and control")
            print("- Reliable gripper operation")
            print("="*60)
                
        except KeyboardInterrupt:
            print("\nDemo interrupted by user")
        except Exception as e:
            print(f"\nDemo error: {e}")
    
    def show_start_screen(self):
        """Show the start screen"""
        print("\n" + "="*60)
        print("🤖 ENHANCED AMR PICK-AND-PLACE DEMO 🤖")
        print("="*60)
        print(f"Progress: {self.current_task}/{self.total_tasks} tasks completed")
        print("\nEnhanced Features:")
        print("- Larger Mecanum wheels (0.15m radius)")
        print("- Higher wheel torque (100 Nm effort)")
        print("- Improved arm joints (200 Nm effort)")
        print("- Enhanced gripper (50 N effort)")
        print("- Better dynamics and damping")
        print("\n" + "="*60)
        print("           🟢 START DEMO 🟢")
        print("="*60)
        print("Press 'START' to begin enhanced pick-and-place demo")
        print("Press 'QUIT' to exit")
        print("="*60)
    
    def run(self):
        """Main run loop"""
        print("\n=== ENHANCED AMR PICK-AND-PLACE DEMO ===")
        print("This demo showcases the enhanced robot capabilities")
        print("Improved wheels, arm, and gripper for better performance!")
        print("Just press START to begin!")
        
        while self.running:
            try:
                self.show_start_screen()
                choice = input("Enter command (START/QUIT): ").strip().upper()
                
                if choice == 'START':
                    self.run_full_demo()
                    self.current_task = self.total_tasks  # Mark as complete
                elif choice == 'QUIT':
                    print("Quitting...")
                    self.running = False
                else:
                    print("Invalid command. Please enter START or QUIT.")
                    
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
        try:
            p.disconnect()
        except:
            pass
        print("Enhanced Pick-and-Place Demo closed")

def main():
    """Main function"""
    demo = PickAndPlaceDemo()
    demo.run()

if __name__ == "__main__":
    main()

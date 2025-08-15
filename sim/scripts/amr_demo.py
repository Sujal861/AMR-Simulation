#!/usr/bin/env python3
"""
AMR (Autonomous Mobile Manipulator) Demo
Demonstrates the capabilities of the enhanced AMR robot
"""
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..'))

import pybullet as p
import time
import numpy as np
from sim.scripts.launch_sim import RobotSimulation
from configs.poses import get_pose, get_pose_description

class AMRDemo:
    def __init__(self):
        """Initialize AMR demo"""
        print("Initializing AMR Demo...")
        self.sim = RobotSimulation(gui=True, skip_field_creation=False)
        self.running = True
        
        # Demo parameters
        self.movement_speed = 2.0  # m/s
        self.rotation_speed = 1.0  # rad/s
        self.arm_speed = 0.5       # rad/s
        
        # Demo sequences
        self.setup_demo_sequences()
    
    def setup_demo_sequences(self):
        """Setup demo sequences"""
        self.demo_sequences = {
            'navigation': [
                {'type': 'move_forward', 'duration': 3.0},
                {'type': 'rotate', 'angle': np.pi/2, 'duration': 2.0},
                {'type': 'move_forward', 'duration': 2.0},
                {'type': 'rotate', 'angle': -np.pi/2, 'duration': 2.0},
                {'type': 'move_forward', 'duration': 3.0}
            ],
            'manipulation': [
                {'type': 'pose', 'pose_name': 'approach_plant', 'duration': 2.0},
                {'type': 'pose', 'pose_name': 'harvest_position', 'duration': 2.0},
                {'type': 'gripper', 'action': 'open', 'duration': 1.0},
                {'type': 'pose', 'pose_name': 'home', 'duration': 2.0},
                {'type': 'gripper', 'action': 'close', 'duration': 1.0}
            ],
            'inspection': [
                {'type': 'pose', 'pose_name': 'survey_position', 'duration': 2.0},
                {'type': 'pose', 'pose_name': 'inspection_position', 'duration': 2.0},
                {'type': 'pose', 'pose_name': 'sensor_clearance', 'duration': 2.0},
                {'type': 'pose', 'pose_name': 'home', 'duration': 2.0}
            ],
            'agricultural_workflow': [
                {'type': 'pose', 'pose_name': 'navigation_ready', 'duration': 1.0},
                {'type': 'move_forward', 'duration': 2.0},
                {'type': 'pose', 'pose_name': 'approach_plant', 'duration': 2.0},
                {'type': 'pose', 'pose_name': 'drill_position', 'duration': 2.0},
                {'type': 'pose', 'pose_name': 'place_seed', 'duration': 2.0},
                {'type': 'pose', 'pose_name': 'watering_position', 'duration': 2.0},
                {'type': 'pose', 'pose_name': 'home', 'duration': 2.0}
            ]
        }
    
    def move_forward(self, duration):
        """Move robot forward"""
        print(f"Moving forward for {duration} seconds")
        
        # Mecanum wheel velocities for forward motion
        wheel_velocities = [self.movement_speed, self.movement_speed, 
                           self.movement_speed, self.movement_speed]
        
        start_time = time.time()
        while time.time() - start_time < duration and self.running:
            self.sim.set_wheel_velocities(wheel_velocities)
            time.sleep(0.01)
        
        # Stop wheels
        self.sim.set_wheel_velocities([0, 0, 0, 0])
    
    def rotate(self, angle, duration):
        """Rotate robot"""
        direction = 1 if angle > 0 else -1
        print(f"Rotating {abs(angle):.2f} radians for {duration} seconds")
        
        # Mecanum wheel velocities for rotation
        wheel_velocities = [direction * self.rotation_speed, -direction * self.rotation_speed,
                           -direction * self.rotation_speed, direction * self.rotation_speed]
        
        start_time = time.time()
        while time.time() - start_time < duration and self.running:
            self.sim.set_wheel_velocities(wheel_velocities)
            time.sleep(0.01)
        
        # Stop wheels
        self.sim.set_wheel_velocities([0, 0, 0, 0])
    
    def move_to_pose(self, pose_name, duration):
        """Move arm to specified pose"""
        pose = get_pose(pose_name)
        description = get_pose_description(pose_name)
        print(f"Moving to {pose_name}: {description}")
        
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
    
    def control_gripper(self, action, duration):
        """Control gripper"""
        position = 0.04 if action == 'open' else 0.0
        print(f"Gripper {action}")
        
        self.sim.set_gripper_position(position)
        time.sleep(duration)
    
    def run_sequence(self, sequence_name):
        """Run a demo sequence"""
        if sequence_name not in self.demo_sequences:
            print(f"Unknown sequence: {sequence_name}")
            return
        
        print(f"\n=== Running {sequence_name.upper()} Sequence ===")
        sequence = self.demo_sequences[sequence_name]
        
        for step in sequence:
            if not self.running:
                break
            
            step_type = step['type']
            duration = step['duration']
            
            if step_type == 'move_forward':
                self.move_forward(duration)
            elif step_type == 'rotate':
                self.rotate(step['angle'], duration)
            elif step_type == 'pose':
                self.move_to_pose(step['pose_name'], duration)
            elif step_type == 'gripper':
                self.control_gripper(step['action'], duration)
            
            time.sleep(0.5)  # Pause between steps
        
        print(f"=== {sequence_name.upper()} Sequence Complete ===\n")
    
    def run_full_demo(self):
        """Run the complete AMR demo"""
        print("=== AMR AUTONOMOUS MOBILE MANIPULATOR DEMO ===")
        print("This demo showcases the enhanced AMR capabilities:")
        print("1. Mobile navigation with Mecanum wheels")
        print("2. 6-DOF manipulator arm control")
        print("3. Gripper control and object handling")
        print("4. Sensor-based inspection and mapping")
        print("5. Agricultural task automation")
        print("6. Realistic farming environment")
        
        try:
            # Run different demo sequences
            sequences = ['navigation', 'manipulation', 'inspection', 'agricultural_workflow']
            
            for sequence in sequences:
                if not self.running:
                    break
                
                self.run_sequence(sequence)
                time.sleep(2.0)  # Pause between sequences
            
            print("=== DEMO COMPLETE ===")
            print("AMR successfully demonstrated:")
            print("✓ Autonomous navigation")
            print("✓ Precise manipulation")
            print("✓ Sensor-based inspection")
            print("✓ Agricultural task execution")
            
        except KeyboardInterrupt:
            print("\nDemo stopped by user")
            self.running = False
    
    def interactive_demo(self):
        """Interactive demo with user control"""
        print("\n=== INTERACTIVE AMR DEMO ===")
        print("Available commands:")
        print("  'nav' - Navigation demo")
        print("  'manip' - Manipulation demo")
        print("  'inspect' - Inspection demo")
        print("  'agri' - Agricultural workflow")
        print("  'full' - Full demo")
        print("  'quit' - Exit")
        
        while self.running:
            try:
                command = input("\nEnter command: ").strip().lower()
                
                if command == 'quit':
                    break
                elif command == 'nav':
                    self.run_sequence('navigation')
                elif command == 'manip':
                    self.run_sequence('manipulation')
                elif command == 'inspect':
                    self.run_sequence('inspection')
                elif command == 'agri':
                    self.run_sequence('agricultural_workflow')
                elif command == 'full':
                    self.run_full_demo()
                else:
                    print("Unknown command. Type 'quit' to exit.")
                    
            except KeyboardInterrupt:
                break
        
        print("Interactive demo ended")
    
    def cleanup(self):
        """Cleanup resources"""
        self.running = False
        p.disconnect()

def main():
    """Main function"""
    demo = AMRDemo()
    
    try:
        # Check if user wants interactive mode
        print("\nChoose demo mode:")
        print("1. Full automated demo")
        print("2. Interactive demo")
        
        choice = input("Enter choice (1 or 2): ").strip()
        
        if choice == '2':
            demo.interactive_demo()
        else:
            demo.run_full_demo()
            
    except KeyboardInterrupt:
        print("\nDemo interrupted by user")
    finally:
        demo.cleanup()
        print("AMR Demo completed")

if __name__ == "__main__":
    main()

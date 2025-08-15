#!/usr/bin/env python3
"""
Demo Poses Script for Agricultural Robot
Cycles through different agricultural poses for demonstration
"""

import time
import os
import sys
from typing import List, Dict

# Add parent directory to path for imports
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(__file__))))

from sim.scripts.launch_sim import RobotSimulation
from configs.poses import AGRICULTURAL_POSES, HOME_POSE, get_pose

class PoseDemo:
    def __init__(self):
        """Initialize pose demo"""
        self.sim = RobotSimulation(gui=True)
        
        # Demo parameters
        self.pose_duration = 3.0  # seconds per pose
        self.transition_duration = 1.0  # seconds for transitions
        
        # Demo sequences
        self.demo_sequences = {
            'basic': [
                'home',
                'approach_plant',
                'drill_position',
                'place_seed',
                'home'
            ],
            'full_cycle': [
                'home',
                'approach_plant',
                'drill_position',
                'place_seed',
                'watering_position',
                'harvest_position',
                'survey_position',
                'home'
            ],
            'maintenance': [
                'home',
                'maintenance_position',
                'storage_position',
                'home'
            ],
            'inspection': [
                'home',
                'survey_position',
                'approach_plant',
                'survey_position',
                'home'
            ]
        }
        
        print("Pose Demo initialized!")
        print(f"Available demo sequences: {list(self.demo_sequences.keys())}")
        print(f"Available poses: {list(AGRICULTURAL_POSES.keys())}")
    
    def move_to_pose(self, pose_name: str, duration: float = None):
        """
        Move robot to a specific pose
        Args:
            pose_name: Name of the pose
            duration: Duration for the movement
        """
        if duration is None:
            duration = self.pose_duration
        
        pose = get_pose(pose_name)
        print(f"Moving to {pose_name} pose: {pose}")
        
        # Set joint positions
        self.sim.set_joint_positions(pose)
        
        # Wait for specified duration
        time.sleep(duration)
    
    def run_sequence(self, sequence_name: str, loop: bool = False):
        """
        Run a predefined sequence of poses
        Args:
            sequence_name: Name of the sequence
            loop: Whether to loop the sequence
        """
        if sequence_name not in self.demo_sequences:
            print(f"Sequence '{sequence_name}' not found")
            return
        
        sequence = self.demo_sequences[sequence_name]
        print(f"Running sequence: {sequence_name}")
        print(f"Poses: {sequence}")
        
        try:
            while True:
                for i, pose_name in enumerate(sequence):
                    print(f"\n[{i+1}/{len(sequence)}] Moving to {pose_name}")
                    self.move_to_pose(pose_name)
                    
                    # Step simulation
                    for _ in range(int(self.pose_duration * 240)):  # 240 Hz simulation
                        self.sim.step_simulation()
                
                if not loop:
                    break
                print("\nSequence completed. Starting again...")
                
        except KeyboardInterrupt:
            print("\nSequence interrupted by user")
            self.move_to_pose('home', 1.0)
    
    def run_interactive_demo(self):
        """Run interactive demo mode"""
        print("Interactive Pose Demo Mode")
        print("Commands:")
        print("  'pose <name>' - Move to specific pose")
        print("  'sequence <name>' - Run predefined sequence")
        print("  'loop <name>' - Run sequence in loop")
        print("  'list' - List available poses and sequences")
        print("  'home' - Go to home pose")
        print("  'quit' - Exit")
        
        try:
            while True:
                command = input("\nEnter command: ").strip().lower().split()
                
                if not command:
                    continue
                
                if command[0] == 'quit':
                    break
                elif command[0] == 'home':
                    self.move_to_pose('home', 1.0)
                elif command[0] == 'pose' and len(command) > 1:
                    pose_name = command[1]
                    if pose_name in AGRICULTURAL_POSES:
                        self.move_to_pose(pose_name)
                    else:
                        print(f"Pose '{pose_name}' not found")
                elif command[0] == 'sequence' and len(command) > 1:
                    sequence_name = command[1]
                    self.run_sequence(sequence_name, loop=False)
                elif command[0] == 'loop' and len(command) > 1:
                    sequence_name = command[1]
                    self.run_sequence(sequence_name, loop=True)
                elif command[0] == 'list':
                    print("\nAvailable poses:")
                    for pose in AGRICULTURAL_POSES.keys():
                        print(f"  - {pose}")
                    print("\nAvailable sequences:")
                    for seq in self.demo_sequences.keys():
                        print(f"  - {seq}")
                else:
                    print("Unknown command")
                    
        except KeyboardInterrupt:
            print("\nInteractive demo stopped")
    
    def run_agricultural_workflow(self):
        """Run a complete agricultural workflow demonstration"""
        print("Running Agricultural Workflow Demo")
        print("This demonstrates a complete planting cycle...")
        
        workflow_steps = [
            ("Initial Survey", "survey_position", 2.0),
            ("Approach Plant", "approach_plant", 2.0),
            ("Drill Soil", "drill_position", 3.0),
            ("Place Seed", "place_seed", 2.0),
            ("Water Plant", "watering_position", 2.0),
            ("Final Survey", "survey_position", 2.0),
            ("Return Home", "home", 2.0)
        ]
        
        try:
            for step_name, pose_name, duration in workflow_steps:
                print(f"\n--- {step_name} ---")
                print(f"Moving to {pose_name} pose...")
                
                self.move_to_pose(pose_name, duration)
                
                # Step simulation during movement
                for _ in range(int(duration * 240)):
                    self.sim.step_simulation()
            
            print("\nAgricultural workflow completed successfully!")
            
        except KeyboardInterrupt:
            print("\nWorkflow interrupted by user")
            self.move_to_pose('home', 1.0)
    
    def run_speed_demo(self):
        """Run a speed demonstration with different pose transitions"""
        print("Running Speed Demo")
        print("This demonstrates fast pose transitions...")
        
        fast_poses = ['home', 'approach_plant', 'drill_position', 'place_seed', 'home']
        
        try:
            for pose_name in fast_poses:
                print(f"Quick move to {pose_name}")
                self.move_to_pose(pose_name, 0.5)  # Fast transitions
                
                # Step simulation
                for _ in range(int(0.5 * 240)):
                    self.sim.step_simulation()
            
            print("Speed demo completed!")
            
        except KeyboardInterrupt:
            print("\nSpeed demo interrupted")
            self.move_to_pose('home', 1.0)
    
    def run_safety_demo(self):
        """Run a safety demonstration"""
        print("Running Safety Demo")
        print("This demonstrates safety features...")
        
        try:
            # Start from home
            print("Starting from home position")
            self.move_to_pose('home', 1.0)
            
            # Move to extreme position
            print("Moving to extreme position (should be limited by joint limits)")
            extreme_pose = [3.14, 3.14, 3.14, 3.14, 3.14, 3.14]  # Max angles
            self.sim.set_joint_positions(extreme_pose)
            time.sleep(2.0)
            
            # Return to safe position
            print("Returning to safe position")
            self.move_to_pose('emergency_stop', 1.0)
            
            # Return to home
            print("Returning to home")
            self.move_to_pose('home', 1.0)
            
            print("Safety demo completed!")
            
        except KeyboardInterrupt:
            print("\nSafety demo interrupted")
            self.move_to_pose('home', 1.0)

def main():
    """Main function"""
    print("Starting Agricultural Robot Pose Demo...")
    
    # Create pose demo
    demo = PoseDemo()
    
    # Run interactive demo
    demo.run_interactive_demo()

if __name__ == "__main__":
    main()

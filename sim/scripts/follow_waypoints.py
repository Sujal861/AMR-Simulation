#!/usr/bin/env python3
"""
Waypoint Following Script for Agricultural Robot
Executes planned trajectories in simulation
"""

import pybullet as p
import time
import os
import sys
import numpy as np
from typing import List, Tuple

# Add parent directory to path for imports
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(__file__))))

from sim.scripts.launch_sim import RobotSimulation
from kinematics.trajectory import TrajectoryPlanner
from configs.poses import HOME_POSE, AGRICULTURAL_POSES

class WaypointFollower:
    def __init__(self):
        """Initialize waypoint follower"""
        self.sim = RobotSimulation(gui=True)
        self.planner = TrajectoryPlanner()
        
        # Control parameters
        self.execution_rate = 50  # Hz
        self.dt = 1.0 / self.execution_rate
        
        # Safety parameters
        self.max_joint_velocity = 1.0  # rad/s
        self.max_position_error = 0.05  # meters
        self.max_orientation_error = 0.1  # radians
        
        # Current state
        self.current_trajectory = None
        self.current_time = 0.0
        self.trajectory_start_time = 0.0
        self.is_executing = False
        
        print("Waypoint Follower initialized!")
        print("Available poses:", list(AGRICULTURAL_POSES.keys()))
    
    def load_trajectory(self, waypoints: List[List[float]], durations: List[float] = None,
                       interpolation_type: str = "cubic") -> bool:
        """
        Load trajectory for execution
        Args:
            waypoints: List of joint position waypoints
            durations: List of durations for each segment
            interpolation_type: "cubic" or "quintic"
        Returns:
            True if trajectory loaded successfully
        """
        try:
            # Generate trajectory
            positions, times = self.planner.waypoint_trajectory(
                waypoints, durations, self.dt, interpolation_type
            )
            
            # Validate trajectory
            joint_limits = self.sim.joint_limits
            if not self.planner.validate_trajectory(positions, joint_limits):
                print("Warning: Trajectory violates joint limits")
                return False
            
            # Store trajectory
            self.current_trajectory = {
                'positions': positions,
                'times': times,
                'current_index': 0
            }
            
            print(f"Trajectory loaded: {len(positions)} points, {times[-1]:.2f}s duration")
            return True
            
        except Exception as e:
            print(f"Error loading trajectory: {e}")
            return False
    
    def load_pose_sequence(self, pose_names: List[str], durations: List[float] = None) -> bool:
        """
        Load sequence of poses for execution
        Args:
            pose_names: List of pose names from configs/poses.py
            durations: List of durations for each segment
        Returns:
            True if sequence loaded successfully
        """
        waypoints = []
        
        for pose_name in pose_names:
            if pose_name in AGRICULTURAL_POSES:
                waypoints.append(AGRICULTURAL_POSES[pose_name])
            else:
                print(f"Warning: Pose '{pose_name}' not found, using home pose")
                waypoints.append(HOME_POSE)
        
        return self.load_trajectory(waypoints, durations)
    
    def start_execution(self):
        """Start trajectory execution"""
        if self.current_trajectory is None:
            print("No trajectory loaded")
            return False
        
        self.is_executing = True
        self.trajectory_start_time = time.time()
        self.current_time = 0.0
        self.current_trajectory['current_index'] = 0
        
        print("Starting trajectory execution...")
        return True
    
    def stop_execution(self):
        """Stop trajectory execution"""
        self.is_executing = False
        print("Trajectory execution stopped")
    
    def execute_step(self) -> bool:
        """
        Execute one step of the trajectory
        Returns:
            True if execution should continue
        """
        if not self.is_executing or self.current_trajectory is None:
            return False
        
        trajectory = self.current_trajectory
        current_index = trajectory['current_index']
        positions = trajectory['positions']
        times = trajectory['times']
        
        # Check if trajectory is complete
        if current_index >= len(positions):
            self.is_executing = False
            print("Trajectory execution completed")
            return False
        
        # Get current target position
        target_position = positions[current_index]
        
        # Set joint positions
        self.sim.set_joint_positions(target_position)
        
        # Check execution time
        elapsed_time = time.time() - self.trajectory_start_time
        expected_time = times[current_index]
        
        # Move to next point if time has elapsed
        if elapsed_time >= expected_time:
            trajectory['current_index'] += 1
        
        return True
    
    def get_execution_progress(self) -> Tuple[float, float]:
        """
        Get execution progress
        Returns:
            Tuple of (progress_percentage, elapsed_time)
        """
        if self.current_trajectory is None:
            return 0.0, 0.0
        
        current_index = self.current_trajectory['current_index']
        total_points = len(self.current_trajectory['positions'])
        elapsed_time = time.time() - self.trajectory_start_time
        
        progress = (current_index / total_points) * 100.0
        return progress, elapsed_time
    
    def run_demo_sequence(self):
        """Run a demo sequence of agricultural poses"""
        print("Running demo sequence...")
        
        # Define demo sequence
        demo_poses = [
            "home",
            "approach_plant",
            "drill_position",
            "place_seed",
            "home"
        ]
        
        demo_durations = [3.0, 2.0, 2.0, 2.0, 3.0]
        
        # Load and execute sequence
        if self.load_pose_sequence(demo_poses, demo_durations):
            self.start_execution()
            
            try:
                while self.is_executing:
                    self.execute_step()
                    self.sim.step_simulation()
                    
                    # Display progress
                    progress, elapsed = self.get_execution_progress()
                    print(f"\rProgress: {progress:.1f}% | Time: {elapsed:.1f}s", end="")
                    
                    time.sleep(self.dt)
                    
            except KeyboardInterrupt:
                print("\nDemo interrupted by user")
                self.stop_execution()
    
    def run_circular_motion(self, radius: float = 0.2, height: float = 0.1, duration: float = 10.0):
        """Run circular motion demo"""
        print(f"Running circular motion (radius={radius}, height={height}, duration={duration})...")
        
        # Generate circular trajectory
        center_pos = HOME_POSE.copy()
        positions, times = self.planner.circular_trajectory(center_pos, radius, height, duration, self.dt)
        
        # Load trajectory
        if self.load_trajectory([center_pos] + positions + [center_pos], [2.0, duration, 2.0]):
            self.start_execution()
            
            try:
                while self.is_executing:
                    self.execute_step()
                    self.sim.step_simulation()
                    
                    # Display progress
                    progress, elapsed = self.get_execution_progress()
                    print(f"\rProgress: {progress:.1f}% | Time: {elapsed:.1f}s", end="")
                    
                    time.sleep(self.dt)
                    
            except KeyboardInterrupt:
                print("\nCircular motion interrupted by user")
                self.stop_execution()
    
    def interactive_waypoint_following(self):
        """Interactive waypoint following mode"""
        print("Interactive Waypoint Following Mode")
        print("Commands:")
        print("  'home' - Go to home pose")
        print("  'pose <name>' - Go to specific pose")
        print("  'sequence <pose1> <pose2> ...' - Execute pose sequence")
        print("  'circle' - Run circular motion")
        print("  'demo' - Run demo sequence")
        print("  'quit' - Exit")
        
        try:
            while True:
                command = input("\nEnter command: ").strip().lower().split()
                
                if not command:
                    continue
                
                if command[0] == 'quit':
                    break
                elif command[0] == 'home':
                    self.sim.set_joint_positions(HOME_POSE)
                    print("Moved to home pose")
                elif command[0] == 'pose' and len(command) > 1:
                    pose_name = command[1]
                    if pose_name in AGRICULTURAL_POSES:
                        self.sim.set_joint_positions(AGRICULTURAL_POSES[pose_name])
                        print(f"Moved to {pose_name} pose")
                    else:
                        print(f"Pose '{pose_name}' not found")
                elif command[0] == 'sequence' and len(command) > 1:
                    pose_names = command[1:]
                    if self.load_pose_sequence(pose_names):
                        self.start_execution()
                        while self.is_executing:
                            self.execute_step()
                            self.sim.step_simulation()
                            time.sleep(self.dt)
                elif command[0] == 'circle':
                    self.run_circular_motion()
                elif command[0] == 'demo':
                    self.run_demo_sequence()
                else:
                    print("Unknown command")
                    
        except KeyboardInterrupt:
            print("\nInteractive mode stopped")

def main():
    """Main function"""
    print("Starting Agricultural Robot Waypoint Follower...")
    
    # Create waypoint follower
    follower = WaypointFollower()
    
    # Run interactive mode
    follower.interactive_waypoint_following()

if __name__ == "__main__":
    main()

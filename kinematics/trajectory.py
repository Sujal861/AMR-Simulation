#!/usr/bin/env python3
"""
Trajectory Planning for Agricultural Robot Arm
Implements cubic and quintic joint interpolation
"""

import numpy as np
from typing import List, Tuple, Optional
from scipy.interpolate import CubicSpline
import time

class TrajectoryPlanner:
    def __init__(self):
        """Initialize trajectory planner"""
        self.default_duration = 2.0  # seconds
        self.default_dt = 0.01  # seconds
        
    def cubic_interpolation(self, start_pos: List[float], end_pos: List[float],
                           duration: float = None, dt: float = None) -> Tuple[List[List[float]], List[float]]:
        """
        Generate cubic trajectory between two joint positions
        Args:
            start_pos: Starting joint positions
            end_pos: Ending joint positions
            duration: Trajectory duration in seconds
            dt: Time step in seconds
        Returns:
            Tuple of (joint_positions, time_points)
        """
        if duration is None:
            duration = self.default_duration
        if dt is None:
            dt = self.default_dt
        
        # Ensure same number of joints
        if len(start_pos) != len(end_pos):
            raise ValueError("Start and end positions must have same number of joints")
        
        num_joints = len(start_pos)
        num_points = int(duration / dt) + 1
        
        # Time array
        time_points = np.linspace(0, duration, num_points)
        
        # Generate trajectories for each joint
        joint_trajectories = []
        
        for joint_idx in range(num_joints):
            # Cubic polynomial: p(t) = a*t^3 + b*t^2 + c*t + d
            # Boundary conditions:
            # p(0) = start_pos, p(duration) = end_pos
            # p'(0) = 0, p'(duration) = 0 (zero velocity at endpoints)
            
            p0 = start_pos[joint_idx]
            pf = end_pos[joint_idx]
            T = duration
            
            # Solve for coefficients
            a = -2 * (pf - p0) / (T**3)
            b = 3 * (pf - p0) / (T**2)
            c = 0
            d = p0
            
            # Generate trajectory
            trajectory = []
            for t in time_points:
                pos = a * t**3 + b * t**2 + c * t + d
                trajectory.append(pos)
            
            joint_trajectories.append(trajectory)
        
        # Transpose to get positions at each time step
        positions = list(map(list, zip(*joint_trajectories)))
        
        return positions, time_points.tolist()
    
    def quintic_interpolation(self, start_pos: List[float], end_pos: List[float],
                             start_vel: List[float] = None, end_vel: List[float] = None,
                             duration: float = None, dt: float = None) -> Tuple[List[List[float]], List[float]]:
        """
        Generate quintic trajectory between two joint positions
        Args:
            start_pos: Starting joint positions
            end_pos: Ending joint positions
            start_vel: Starting joint velocities (optional)
            end_vel: Ending joint velocities (optional)
            duration: Trajectory duration in seconds
            dt: Time step in seconds
        Returns:
            Tuple of (joint_positions, time_points)
        """
        if duration is None:
            duration = self.default_duration
        if dt is None:
            dt = self.default_dt
        
        # Ensure same number of joints
        if len(start_pos) != len(end_pos):
            raise ValueError("Start and end positions must have same number of joints")
        
        num_joints = len(start_pos)
        num_points = int(duration / dt) + 1
        
        # Default velocities
        if start_vel is None:
            start_vel = [0.0] * num_joints
        if end_vel is None:
            end_vel = [0.0] * num_joints
        
        # Time array
        time_points = np.linspace(0, duration, num_points)
        
        # Generate trajectories for each joint
        joint_trajectories = []
        
        for joint_idx in range(num_joints):
            # Quintic polynomial: p(t) = a*t^5 + b*t^4 + c*t^3 + d*t^2 + e*t + f
            # Boundary conditions:
            # p(0) = start_pos, p(duration) = end_pos
            # p'(0) = start_vel, p'(duration) = end_vel
            # p''(0) = 0, p''(duration) = 0 (zero acceleration at endpoints)
            
            p0 = start_pos[joint_idx]
            pf = end_pos[joint_idx]
            v0 = start_vel[joint_idx]
            vf = end_vel[joint_idx]
            T = duration
            
            # Solve for coefficients
            a = 6 * (pf - p0) / (T**5) - 3 * (vf + v0) / (T**4)
            b = -15 * (pf - p0) / (T**4) + 7 * (vf + v0) / (T**3)
            c = 10 * (pf - p0) / (T**3) - 4 * (vf + v0) / (T**2)
            d = 0
            e = v0
            f = p0
            
            # Generate trajectory
            trajectory = []
            for t in time_points:
                pos = a * t**5 + b * t**4 + c * t**3 + d * t**2 + e * t + f
                trajectory.append(pos)
            
            joint_trajectories.append(trajectory)
        
        # Transpose to get positions at each time step
        positions = list(map(list, zip(*joint_trajectories)))
        
        return positions, time_points.tolist()
    
    def waypoint_trajectory(self, waypoints: List[List[float]], durations: List[float] = None,
                           dt: float = None, interpolation_type: str = "cubic") -> Tuple[List[List[float]], List[float]]:
        """
        Generate trajectory through multiple waypoints
        Args:
            waypoints: List of joint position waypoints
            durations: List of durations for each segment
            dt: Time step in seconds
            interpolation_type: "cubic" or "quintic"
        Returns:
            Tuple of (joint_positions, time_points)
        """
        if len(waypoints) < 2:
            raise ValueError("Need at least 2 waypoints")
        
        if dt is None:
            dt = self.default_dt
        
        if durations is None:
            durations = [self.default_duration] * (len(waypoints) - 1)
        
        if len(durations) != len(waypoints) - 1:
            raise ValueError("Number of durations must be one less than number of waypoints")
        
        # Generate trajectory for each segment
        all_positions = []
        all_times = []
        current_time = 0.0
        
        for i in range(len(waypoints) - 1):
            start_pos = waypoints[i]
            end_pos = waypoints[i + 1]
            duration = durations[i]
            
            if interpolation_type == "cubic":
                positions, times = self.cubic_interpolation(start_pos, end_pos, duration, dt)
            elif interpolation_type == "quintic":
                positions, times = self.quintic_interpolation(start_pos, end_pos, duration=duration, dt=dt)
            else:
                raise ValueError("Interpolation type must be 'cubic' or 'quintic'")
            
            # Adjust time points
            adjusted_times = [t + current_time for t in times]
            
            # Add positions and times (skip first point to avoid duplication)
            if i == 0:
                all_positions.extend(positions)
                all_times.extend(adjusted_times)
            else:
                all_positions.extend(positions[1:])
                all_times.extend(adjusted_times[1:])
            
            current_time += duration
        
        return all_positions, all_times
    
    def circular_trajectory(self, center_pos: List[float], radius: float, height: float,
                           duration: float = None, dt: float = None) -> Tuple[List[List[float]], List[float]]:
        """
        Generate circular trajectory in joint space
        Args:
            center_pos: Center joint position
            radius: Radius of circle in joint space
            height: Height offset
            duration: Trajectory duration
            dt: Time step
        Returns:
            Tuple of (joint_positions, time_points)
        """
        if duration is None:
            duration = self.default_duration
        if dt is None:
            dt = self.default_dt
        
        num_points = int(duration / dt) + 1
        time_points = np.linspace(0, duration, num_points)
        
        # Generate circular motion
        positions = []
        for t in time_points:
            # Circular motion in first two joints
            angle = 2 * np.pi * t / duration
            pos = center_pos.copy()
            pos[0] += radius * np.cos(angle)
            pos[1] += radius * np.sin(angle)
            pos[2] += height * np.sin(2 * angle)  # Add some vertical motion
            
            positions.append(pos)
        
        return positions, time_points.tolist()
    
    def smooth_trajectory(self, waypoints: List[List[float]], durations: List[float] = None,
                         dt: float = None) -> Tuple[List[List[float]], List[float]]:
        """
        Generate smooth trajectory using cubic spline interpolation
        Args:
            waypoints: List of joint position waypoints
            durations: List of durations for each segment
            dt: Time step in seconds
        Returns:
            Tuple of (joint_positions, time_points)
        """
        if len(waypoints) < 2:
            raise ValueError("Need at least 2 waypoints")
        
        if dt is None:
            dt = self.default_dt
        
        if durations is None:
            durations = [self.default_duration] * (len(waypoints) - 1)
        
        # Calculate cumulative time
        cumulative_time = [0.0]
        for duration in durations:
            cumulative_time.append(cumulative_time[-1] + duration)
        
        # Generate time points
        total_duration = cumulative_time[-1]
        num_points = int(total_duration / dt) + 1
        time_points = np.linspace(0, total_duration, num_points)
        
        # Generate spline for each joint
        num_joints = len(waypoints[0])
        joint_trajectories = []
        
        for joint_idx in range(num_joints):
            joint_waypoints = [waypoint[joint_idx] for waypoint in waypoints]
            
            # Create cubic spline
            spline = CubicSpline(cumulative_time, joint_waypoints, bc_type='natural')
            
            # Evaluate spline
            trajectory = spline(time_points)
            joint_trajectories.append(trajectory)
        
        # Transpose to get positions at each time step
        positions = list(map(list, zip(*joint_trajectories)))
        
        return positions, time_points.tolist()
    
    def validate_trajectory(self, trajectory: List[List[float]], joint_limits: List[Tuple[float, float]]) -> bool:
        """
        Validate trajectory against joint limits
        Args:
            trajectory: List of joint positions
            joint_limits: List of (min, max) joint limits
        Returns:
            True if trajectory is valid
        """
        for positions in trajectory:
            for i, pos in enumerate(positions):
                if i < len(joint_limits):
                    min_limit, max_limit = joint_limits[i]
                    if pos < min_limit or pos > max_limit:
                        return False
        return True
    
    def get_trajectory_velocity(self, trajectory: List[List[float]], time_points: List[float]) -> List[List[float]]:
        """
        Compute velocities from trajectory
        Args:
            trajectory: List of joint positions
            time_points: List of time points
        Returns:
            List of joint velocities
        """
        if len(trajectory) != len(time_points):
            raise ValueError("Trajectory and time points must have same length")
        
        velocities = []
        num_joints = len(trajectory[0])
        
        for i in range(len(trajectory) - 1):
            dt = time_points[i + 1] - time_points[i]
            if dt <= 0:
                velocities.append([0.0] * num_joints)
                continue
            
            vel = []
            for j in range(num_joints):
                dpos = trajectory[i + 1][j] - trajectory[i][j]
                vel.append(dpos / dt)
            velocities.append(vel)
        
        # Add zero velocity for last point
        velocities.append([0.0] * num_joints)
        
        return velocities

def main():
    """Test trajectory planning"""
    planner = TrajectoryPlanner()
    
    # Test cubic interpolation
    start_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    end_pos = [1.0, 0.5, 0.3, 0.2, 0.1, 0.0]
    
    print("Cubic Interpolation Test:")
    positions, times = planner.cubic_interpolation(start_pos, end_pos, duration=3.0)
    print(f"Generated {len(positions)} trajectory points")
    print(f"Start: {positions[0]}")
    print(f"End: {positions[-1]}")
    
    # Test quintic interpolation
    print("\nQuintic Interpolation Test:")
    positions, times = planner.quintic_interpolation(start_pos, end_pos, duration=3.0)
    print(f"Generated {len(positions)} trajectory points")
    
    # Test waypoint trajectory
    print("\nWaypoint Trajectory Test:")
    waypoints = [
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.5, 0.2, 0.1, 0.1, 0.05, 0.0],
        [1.0, 0.5, 0.3, 0.2, 0.1, 0.0]
    ]
    positions, times = planner.waypoint_trajectory(waypoints, durations=[2.0, 2.0])
    print(f"Generated {len(positions)} trajectory points through {len(waypoints)} waypoints")
    
    # Test circular trajectory
    print("\nCircular Trajectory Test:")
    center_pos = [0.5, 0.0, 0.0, 0.0, 0.0, 0.0]
    positions, times = planner.circular_trajectory(center_pos, radius=0.2, height=0.1, duration=5.0)
    print(f"Generated {len(positions)} trajectory points for circular motion")

if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
Pose configuration management for AMR manipulator
"""
import yaml
import os

def load_poses():
    """Load poses from YAML configuration file"""
    config_path = os.path.join(os.path.dirname(__file__), "poses.yaml")
    
    with open(config_path, 'r') as file:
        config = yaml.safe_load(file)
    
    return config.get('poses', {})

# Load poses
AGRICULTURAL_POSES = load_poses()

# Home pose (default position)
HOME_POSE = AGRICULTURAL_POSES.get('home', {}).get('joints', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

def get_pose(pose_name):
    """Get joint angles for a specific pose"""
    pose_data = AGRICULTURAL_POSES.get(pose_name, {})
    return pose_data.get('joints', HOME_POSE)

def get_pose_description(pose_name):
    """Get description for a specific pose"""
    pose_data = AGRICULTURAL_POSES.get(pose_name, {})
    return pose_data.get('description', 'No description available')

def get_available_poses():
    """Get list of available pose names"""
    return list(AGRICULTURAL_POSES.keys())

def validate_pose(joint_angles):
    """Validate joint angles against limits"""
    if len(joint_angles) != 6:
        return False
    
    # Basic validation - check if angles are within reasonable range
    for angle in joint_angles:
        if not isinstance(angle, (int, float)) or abs(angle) > 3.15:
            return False
    
    return True

def interpolate_poses(pose1_name, pose2_name, steps=10):
    """Interpolate between two poses"""
    pose1 = get_pose(pose1_name)
    pose2 = get_pose(pose2_name)
    
    if not validate_pose(pose1) or not validate_pose(pose2):
        return []
    
    interpolated = []
    for i in range(steps + 1):
        t = i / steps
        interpolated_pose = []
        for j in range(6):
            angle = pose1[j] + t * (pose2[j] - pose1[j])
            interpolated_pose.append(angle)
        interpolated.append(interpolated_pose)
    
    return interpolated

def create_pose_sequence(pose_names, durations=None):
    """Create a sequence of poses with optional durations"""
    if durations is None:
        durations = [2.0] * len(pose_names)
    
    if len(pose_names) != len(durations):
        raise ValueError("Number of poses and durations must match")
    
    sequence = []
    for pose_name, duration in zip(pose_names, durations):
        pose_data = {
            'name': pose_name,
            'joints': get_pose(pose_name),
            'description': get_pose_description(pose_name),
            'duration': duration
        }
        sequence.append(pose_data)
    
    return sequence

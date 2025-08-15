#!/usr/bin/env python3
"""
Forward Kinematics for AMR Manipulator
"""
import numpy as np
import yaml
import os

class ForwardKinematics:
    def __init__(self, dh_params_file=None):
        """Initialize forward kinematics with DH parameters"""
        if dh_params_file is None:
            dh_params_file = os.path.join(os.path.dirname(__file__), "..", "configs", "dh_params.yaml")
        
        self.dh_params = self.load_dh_params(dh_params_file)
        self.joint_names = list(self.dh_params['joints'].keys())
        
    def load_dh_params(self, file_path):
        """Load DH parameters from YAML file"""
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)
    
    def dh_transform_matrix(self, a, alpha, d, theta):
        """Compute DH transformation matrix"""
        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)
        
        T = np.array([
            [ct, -st*ca, st*sa, a*ct],
            [st, ct*ca, -ct*sa, a*st],
            [0, sa, ca, d],
            [0, 0, 0, 1]
        ])
        
        return T
    
    def forward_kinematics(self, joint_angles):
        """
        Compute forward kinematics
        Args:
            joint_angles: List of joint angles [shoulder_pan, shoulder_lift, upper_arm, forearm, wrist_flex, wrist_rotate]
        Returns:
            position: [x, y, z] end effector position
            orientation: [roll, pitch, yaw] end effector orientation
        """
        if len(joint_angles) != 6:
            raise ValueError("Expected 6 joint angles")
        
        # Start with identity matrix
        T = np.eye(4)
        
        # Apply base frame offset
        base_offset = self.dh_params['base_frame']
        base_T = self.create_transform_matrix(
            base_offset['x'], base_offset['y'], base_offset['z'],
            base_offset['roll'], base_offset['pitch'], base_offset['yaw']
        )
        T = base_T @ T
        
        # Apply DH transformations for each joint
        for i, joint_name in enumerate(self.joint_names):
            joint_params = self.dh_params['joints'][joint_name]
            
            # Use provided joint angle
            theta = joint_angles[i]
            
            # Get DH parameters
            a = joint_params['a']
            alpha = joint_params['alpha']
            d = joint_params['d']
            
            # Compute transformation matrix
            joint_T = self.dh_transform_matrix(a, alpha, d, theta)
            T = T @ joint_T
        
        # Apply tool frame offset
        tool_offset = self.dh_params['tool_frame']
        tool_T = self.create_transform_matrix(
            tool_offset['x'], tool_offset['y'], tool_offset['z'],
            tool_offset['roll'], tool_offset['pitch'], tool_offset['yaw']
        )
        T = T @ tool_T
        
        # Extract position and orientation
        position = T[:3, 3]
        orientation = self.rotation_matrix_to_euler(T[:3, :3])
        
        return position, orientation
    
    def create_transform_matrix(self, x, y, z, roll, pitch, yaw):
        """Create transformation matrix from position and orientation"""
        # Rotation matrix from Euler angles (ZYX convention)
        cr = np.cos(roll)
        sr = np.sin(roll)
        cp = np.cos(pitch)
        sp = np.sin(pitch)
        cy = np.cos(yaw)
        sy = np.sin(yaw)
        
        R = np.array([
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp, cp*sr, cp*cr]
        ])
        
        # Create transformation matrix
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [x, y, z]
        
        return T
    
    def rotation_matrix_to_euler(self, R):
        """Convert rotation matrix to Euler angles (ZYX convention)"""
        # Extract Euler angles from rotation matrix
        sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
        singular = sy < 1e-6
        
        if not singular:
            x = np.arctan2(R[2, 1], R[2, 2])
            y = np.arctan2(-R[2, 0], sy)
            z = np.arctan2(R[1, 0], R[0, 0])
        else:
            x = np.arctan2(-R[1, 2], R[1, 1])
            y = np.arctan2(-R[2, 0], sy)
            z = 0
        
        return np.array([x, y, z])
    
    def get_joint_positions(self, joint_angles):
        """Get positions of all joints"""
        positions = []
        T = np.eye(4)
        
        # Apply base frame offset
        base_offset = self.dh_params['base_frame']
        base_T = self.create_transform_matrix(
            base_offset['x'], base_offset['y'], base_offset['z'],
            base_offset['roll'], base_offset['pitch'], base_offset['yaw']
        )
        T = base_T @ T
        positions.append(T[:3, 3].copy())
        
        # Apply DH transformations for each joint
        for i, joint_name in enumerate(self.joint_names):
            joint_params = self.dh_params['joints'][joint_name]
            
            theta = joint_angles[i]
            a = joint_params['a']
            alpha = joint_params['alpha']
            d = joint_params['d']
            
            joint_T = self.dh_transform_matrix(a, alpha, d, theta)
            T = T @ joint_T
            positions.append(T[:3, 3].copy())
        
        return positions
    
    def check_joint_limits(self, joint_angles):
        """Check if joint angles are within limits"""
        for i, joint_name in enumerate(self.joint_names):
            joint_params = self.dh_params['joints'][joint_name]
            limits = joint_params['limits']
            
            if joint_angles[i] < limits[0] or joint_angles[i] > limits[1]:
                return False, f"Joint {joint_name} angle {joint_angles[i]} outside limits {limits}"
        
        return True, "All joints within limits"
    
    def clamp_joint_angles(self, joint_angles):
        """Clamp joint angles to limits"""
        clamped = []
        for i, joint_name in enumerate(self.joint_names):
            joint_params = self.dh_params['joints'][joint_name]
            limits = joint_params['limits']
            
            angle = np.clip(joint_angles[i], limits[0], limits[1])
            clamped.append(angle)
        
        return clamped

def main():
    """Test forward kinematics"""
    fk = ForwardKinematics()
    
    # Test home position
    home_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    position, orientation = fk.forward_kinematics(home_angles)
    
    print("Forward Kinematics Test:")
    print(f"Home position: {position}")
    print(f"Home orientation: {orientation}")
    
    # Test joint positions
    joint_positions = fk.get_joint_positions(home_angles)
    print(f"Joint positions: {len(joint_positions)} points")
    
    # Test joint limits
    valid, message = fk.check_joint_limits(home_angles)
    print(f"Joint limits check: {message}")
    
    # Test with different pose
    test_angles = [0.0, -0.5, 0.8, -0.3, 0.0, 0.0]
    position, orientation = fk.forward_kinematics(test_angles)
    print(f"Test pose position: {position}")
    print(f"Test pose orientation: {orientation}")

if __name__ == "__main__":
    main()

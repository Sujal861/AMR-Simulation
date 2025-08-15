#!/usr/bin/env python3
"""
Numerical Inverse Kinematics for AMR Manipulator
"""
import numpy as np
import yaml
import os
from scipy.optimize import minimize
from kinematics.fk import ForwardKinematics

class NumericalInverseKinematics:
    def __init__(self, dh_params_file=None):
        """Initialize inverse kinematics with DH parameters"""
        if dh_params_file is None:
            dh_params_file = os.path.join(os.path.dirname(__file__), "..", "configs", "dh_params.yaml")
        
        self.fk = ForwardKinematics(dh_params_file)
        self.dh_params = self.fk.dh_params
        self.joint_names = self.fk.joint_names
        
    def compute_jacobian(self, joint_angles, epsilon=1e-6):
        """
        Compute Jacobian matrix numerically
        Args:
            joint_angles: Current joint angles
            epsilon: Small perturbation for numerical differentiation
        Returns:
            Jacobian matrix (6x6)
        """
        J = np.zeros((6, 6))
        
        # Get current end effector pose
        current_pos, current_orn = self.fk.forward_kinematics(joint_angles)
        current_pose = np.concatenate([current_pos, current_orn])
        
        # Compute Jacobian by perturbing each joint
        for i in range(6):
            perturbed_angles = joint_angles.copy()
            perturbed_angles[i] += epsilon
            
            perturbed_pos, perturbed_orn = self.fk.forward_kinematics(perturbed_angles)
            perturbed_pose = np.concatenate([perturbed_pos, perturbed_orn])
            
            # Finite difference
            J[:, i] = (perturbed_pose - current_pose) / epsilon
        
        return J
    
    def rotation_matrix_to_euler(self, R):
        """Convert rotation matrix to Euler angles (ZYX convention)"""
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
    
    def euler_to_rotation_matrix(self, euler):
        """Convert Euler angles to rotation matrix (ZYX convention)"""
        roll, pitch, yaw = euler
        
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
        
        return R
    
    def inverse_kinematics(self, target_position, target_orientation, 
                          initial_guess=None, max_iterations=100, tolerance=1e-3):
        """
        Solve inverse kinematics using damped least squares
        Args:
            target_position: [x, y, z] target position
            target_orientation: [roll, pitch, yaw] target orientation
            initial_guess: Initial joint angles guess
            max_iterations: Maximum iterations
            tolerance: Convergence tolerance
        Returns:
            joint_angles: Solution joint angles
            success: Whether solution was found
        """
        if initial_guess is None:
            initial_guess = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        joint_angles = np.array(initial_guess)
        target_pose = np.concatenate([target_position, target_orientation])
        
        for iteration in range(max_iterations):
            # Get current pose
            current_pos, current_orn = self.fk.forward_kinematics(joint_angles)
            current_pose = np.concatenate([current_pos, current_orn])
            
            # Compute error
            error = target_pose - current_pose
            
            # Check convergence
            if np.linalg.norm(error) < tolerance:
                return joint_angles, True
            
            # Compute Jacobian
            J = self.compute_jacobian(joint_angles)
            
            # Damped least squares
            lambda_val = 0.1
            J_damped = J.T @ J + lambda_val * np.eye(6)
            
            # Solve for joint angle changes
            delta_theta = np.linalg.solve(J_damped, J.T @ error)
            
            # Update joint angles
            joint_angles += delta_theta
            
            # Clamp to joint limits
            joint_angles = self.fk.clamp_joint_angles(joint_angles)
        
        return joint_angles, False
    
    def inverse_kinematics_optimization(self, target_position, target_orientation, 
                                      initial_guess=None):
        """
        Solve inverse kinematics using optimization
        Args:
            target_position: [x, y, z] target position
            target_orientation: [roll, pitch, yaw] target orientation
            initial_guess: Initial joint angles guess
        Returns:
            joint_angles: Solution joint angles
            success: Whether solution was found
        """
        if initial_guess is None:
            initial_guess = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        def objective_function(joint_angles):
            """Objective function to minimize"""
            current_pos, current_orn = self.fk.forward_kinematics(joint_angles)
            
            # Position error
            pos_error = np.linalg.norm(current_pos - target_position)
            
            # Orientation error (simplified)
            orn_error = np.linalg.norm(current_orn - target_orientation)
            
            return pos_error + 0.1 * orn_error
        
        def constraint_function(joint_angles):
            """Joint limit constraints"""
            constraints = []
            for i, joint_name in enumerate(self.joint_names):
                joint_params = self.dh_params['joints'][joint_name]
                limits = joint_params['limits']
                
                # Lower bound constraint
                constraints.append(joint_angles[i] - limits[0])
                # Upper bound constraint
                constraints.append(limits[1] - joint_angles[i])
            
            return constraints
        
        # Define bounds for each joint
        bounds = []
        for joint_name in self.joint_names:
            joint_params = self.dh_params['joints'][joint_name]
            bounds.append(joint_params['limits'])
        
        # Run optimization
        result = minimize(
            objective_function,
            initial_guess,
            method='SLSQP',
            bounds=bounds,
            constraints={'type': 'ineq', 'fun': constraint_function},
            options={'maxiter': 1000}
        )
        
        return result.x, result.success
    
    def multiple_solutions(self, target_position, target_orientation, 
                          num_solutions=4, initial_guesses=None):
        """
        Find multiple IK solutions
        Args:
            target_position: [x, y, z] target position
            target_orientation: [roll, pitch, yaw] target orientation
            num_solutions: Number of solutions to find
            initial_guesses: List of initial guesses
        Returns:
            List of (joint_angles, success) tuples
        """
        if initial_guesses is None:
            # Generate different initial guesses
            initial_guesses = []
            for i in range(num_solutions):
                guess = np.random.uniform(-np.pi, np.pi, 6)
                initial_guesses.append(guess)
        
        solutions = []
        for guess in initial_guesses:
            solution, success = self.inverse_kinematics_optimization(
                target_position, target_orientation, guess
            )
            solutions.append((solution, success))
        
        return solutions
    
    def validate_solution(self, joint_angles, target_position, target_orientation, 
                         position_tolerance=0.01, orientation_tolerance=0.1):
        """
        Validate IK solution
        Args:
            joint_angles: Solution joint angles
            target_position: Target position
            target_orientation: Target orientation
            position_tolerance: Position error tolerance
            orientation_tolerance: Orientation error tolerance
        Returns:
            is_valid: Whether solution is valid
            position_error: Position error
            orientation_error: Orientation error
        """
        current_pos, current_orn = self.fk.forward_kinematics(joint_angles)
        
        position_error = np.linalg.norm(current_pos - target_position)
        orientation_error = np.linalg.norm(current_orn - target_orientation)
        
        is_valid = (position_error < position_tolerance and 
                   orientation_error < orientation_tolerance)
        
        return is_valid, position_error, orientation_error

def main():
    """Test inverse kinematics"""
    ik = NumericalInverseKinematics()
    
    # Test target pose within reachable workspace
    target_position = np.array([0.6, 0.1, 0.4])
    target_orientation = np.array([0.0, 0.0, 0.0])
    
    print("Inverse Kinematics Test:")
    print(f"Target position: {target_position}")
    print(f"Target orientation: {target_orientation}")
    
    # Test iterative IK
    solution, success = ik.inverse_kinematics(target_position, target_orientation)
    print(f"Iterative IK success: {success}")
    print(f"Solution: {solution}")
    
    # Test optimization IK
    solution_opt, success_opt = ik.inverse_kinematics_optimization(
        target_position, target_orientation
    )
    print(f"Optimization IK success: {success_opt}")
    print(f"Solution: {solution_opt}")
    
    # Validate solution
    is_valid, pos_error, orn_error = ik.validate_solution(
        solution_opt, target_position, target_orientation
    )
    print(f"Solution valid: {is_valid}")
    print(f"Position error: {pos_error:.4f}")
    print(f"Orientation error: {orn_error:.4f}")
    
    # Test multiple solutions
    solutions = ik.multiple_solutions(target_position, target_orientation, num_solutions=3)
    print(f"Multiple solutions found: {len(solutions)}")
    for i, (sol, success) in enumerate(solutions):
        print(f"Solution {i+1}: success={success}")

if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
Launch PyBullet simulation for AMR (Autonomous Mobile Manipulator)
"""
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..'))

import pybullet as p
import pybullet_data
import time
from configs.poses import HOME_POSE

class RobotSimulation:
    def __init__(self, gui=True, skip_field_creation=False):
        """Initialize PyBullet simulation"""
        self.gui = gui
        self.skip_field_creation = skip_field_creation
        
        # Connect to PyBullet
        if gui:
            self.physics_client = p.connect(p.GUI)
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
            p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
            p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
            p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
        else:
            self.physics_client = p.connect(p.DIRECT)
        
        # Set physics parameters with proper gravity
        p.setGravity(0, 0, -9.81)  # Earth gravity
        p.setTimeStep(1.0/240.0)   # 240 Hz physics simulation
        p.setRealTimeSimulation(1) # Enable real-time simulation
        
        # Configure physics engine for better stability
        p.setPhysicsEngineParameter(
            fixedTimeStep=1.0/240.0,
            numSolverIterations=50,
            numSubSteps=1,
            solverResidualThreshold=1e-10
        )
        
        # Load plane with proper physics properties
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.plane_id = p.loadURDF("plane.urdf")
        
        # Set plane physics properties
        p.changeDynamics(self.plane_id, -1, 
                        lateralFriction=0.8, 
                        rollingFriction=0.1, 
                        restitution=0.0)
        
        # Create farming field (if not skipped)
        if not self.skip_field_creation:
            from sim.scripts.create_farming_field import FarmingFieldGenerator
            field_generator = FarmingFieldGenerator(self.physics_client)
            self.field_data = field_generator.create_complete_farming_field()
            # Store plant bodies for access by other scripts
            self.plant_bodies = self.field_data.get('plant_bodies', [])
        else:
            self.field_data = {}
            self.plant_bodies = []
        
        # Load robot with proper initial height
        robot_path = os.path.join(os.path.dirname(__file__), "..", "models", "robot.urdf")
        self.robot_id = p.loadURDF(robot_path, [0, 0, 0.2])  # Start higher to avoid sinking
        
        # Set robot physics properties
        p.changeDynamics(self.robot_id, -1, 
                        lateralFriction=0.5, 
                        rollingFriction=0.1, 
                        restitution=0.0,
                        linearDamping=0.1,
                        angularDamping=0.1)
        
        # Get joint information
        self.num_joints = p.getNumJoints(self.robot_id)
        self.joint_names = []
        self.joint_indices = {}
        
        for i in range(self.num_joints):
            joint_info = p.getJointInfo(self.robot_id, i)
            joint_name = joint_info[1].decode('utf-8')
            self.joint_names.append(joint_name)
            self.joint_indices[joint_name] = i
        
        print(f"Loaded AMR with {self.num_joints} joints:")
        for name in self.joint_names:
            print(f"  - {name}")
        
        # Set initial pose
        self.set_joint_positions(HOME_POSE)
        
        # Enable joint torque control with proper limits
        for i in range(self.num_joints):
            joint_info = p.getJointInfo(self.robot_id, i)
            joint_type = joint_info[2]
            
            if joint_type == 4:  # JOINT_CONTINUOUS
                # For wheels - velocity control
                p.setJointMotorControl2(
                    self.robot_id, i, p.VELOCITY_CONTROL, force=0
                )
            elif joint_type == 0:  # JOINT_REVOLUTE
                # For arm joints - position control with limits
                p.setJointMotorControl2(
                    self.robot_id, i, p.POSITION_CONTROL, force=100
                )
            elif joint_type == 1:  # JOINT_PRISMATIC
                # For gripper - position control
                p.setJointMotorControl2(
                    self.robot_id, i, p.POSITION_CONTROL, force=50
                )
        
        # Let the robot settle on the ground
        print("Letting robot settle on ground...")
        for _ in range(100):  # 100 physics steps to settle
            p.stepSimulation()
            if self.gui:
                time.sleep(1.0/240.0)
        
        print("Physics simulation ready with proper gravity!")
    
    def set_joint_positions(self, joint_angles):
        """Set joint positions for manipulator arm"""
        # Map joint angles to manipulator joints
        manipulator_joints = [
            'shoulder_pan_joint',
            'shoulder_lift_joint', 
            'upper_arm_joint',
            'forearm_joint',
            'wrist_flex_joint',
            'wrist_rotate_joint'
        ]
        
        for i, joint_name in enumerate(manipulator_joints):
            if joint_name in self.joint_indices and i < len(joint_angles):
                joint_idx = self.joint_indices[joint_name]
                p.resetJointState(self.robot_id, joint_idx, joint_angles[i])
    
    def get_joint_positions(self):
        """Get current joint positions for manipulator arm"""
        manipulator_joints = [
            'shoulder_pan_joint',
            'shoulder_lift_joint', 
            'upper_arm_joint',
            'forearm_joint',
            'wrist_flex_joint',
            'wrist_rotate_joint'
        ]
        
        joint_angles = []
        for joint_name in manipulator_joints:
            if joint_name in self.joint_indices:
                joint_idx = self.joint_indices[joint_name]
                joint_state = p.getJointState(self.robot_id, joint_idx)
                joint_angles.append(joint_state[0])
        
        return joint_angles
    
    def get_end_effector_pose(self):
        """Get end effector pose"""
        # Get end effector link state
        end_effector_link = 'end_effector'
        if end_effector_link in self.joint_indices:
            link_idx = self.joint_indices[end_effector_link]
        else:
            # Fallback to last manipulator joint
            link_idx = self.joint_indices['wrist_rotate_joint']
        
        link_state = p.getLinkState(self.robot_id, link_idx)
        position = link_state[0]
        orientation = link_state[1]
        
        return position, orientation
    
    def get_base_position_and_orientation(self):
        """Get robot base position and orientation"""
        base_state = p.getBasePositionAndOrientation(self.robot_id)
        return base_state[0], base_state[1]
    
    def set_wheel_velocities(self, wheel_velocities):
        """Set wheel velocities for mobile base"""
        wheel_joints = [
            'wheel_fl_joint',
            'wheel_fr_joint', 
            'wheel_rl_joint',
            'wheel_rr_joint'
        ]
        
        for i, joint_name in enumerate(wheel_joints):
            if joint_name in self.joint_indices and i < len(wheel_velocities):
                joint_idx = self.joint_indices[joint_name]
                p.setJointMotorControl2(
                    self.robot_id, joint_idx, p.VELOCITY_CONTROL, 
                    targetVelocity=wheel_velocities[i],
                    force=100  # Increased force for better wheel control
                )
    
    def set_gripper_position(self, position):
        """Set gripper position (0 = closed, 1 = open)"""
        gripper_joints = ['gripper_left_joint', 'gripper_right_joint']
        
        for joint_name in gripper_joints:
            if joint_name in self.joint_indices:
                joint_idx = self.joint_indices[joint_name]
                p.setJointMotorControl2(
                    self.robot_id, joint_idx, p.POSITION_CONTROL, 
                    targetPosition=position,
                    force=50  # Increased force for gripper
                )
    
    def run_simulation(self, duration=10.0):
        """Run simulation for specified duration"""
        start_time = time.time()
        
        while time.time() - start_time < duration:
            if self.gui:
                p.stepSimulation()
                time.sleep(1.0/240.0)
        
        print("Simulation completed")

def main():
    """Main function"""
    print("Launching AMR Simulation...")
    
    # Create simulation
    sim = RobotSimulation(gui=True)
    
    print("\nAMR Simulation Ready!")
    print("Features:")
    print("- 6-DOF manipulator arm with gripper")
    print("- Mecanum wheel mobile base")
    print("- LiDAR and camera sensors")
    print("- Realistic farming environment")
    print("- Proper gravity and physics simulation")
    
    print("\nControls:")
    print("- Use teleop.py for manual control")
    print("- Use follow_waypoints.py for automated sequences")
    print("- Use demo_poses.py for pose demonstrations")
    print("- Use start_button_harvester.py for autonomous harvesting")
    
    # Run simulation
    try:
        sim.run_simulation(duration=60.0)  # Run for 60 seconds
    except KeyboardInterrupt:
        print("\nSimulation stopped by user")
    
    # Disconnect
    p.disconnect()
    print("Simulation closed")

if __name__ == "__main__":
    main()

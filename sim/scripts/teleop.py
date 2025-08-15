#!/usr/bin/env python3
"""
Teleoperation Script for Agricultural Robot
Controls base movement (Vx, Vy, Wz) and arm joints via keyboard
"""

import pybullet as p
import time
import os
import sys
import threading
from collections import deque

# Add parent directory to path for imports
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(__file__))))

from sim.scripts.launch_sim import RobotSimulation
from configs.poses import HOME_POSE

class TeleopController:
    def __init__(self):
        """Initialize teleoperation controller"""
        self.sim = RobotSimulation(gui=True)
        
        # Control parameters
        self.joint_step = 0.1  # radians
        self.base_step = 0.1   # m/s
        self.angular_step = 0.1  # rad/s
        
        # Current velocities
        self.base_vx = 0.0
        self.base_vy = 0.0
        self.base_wz = 0.0
        
        # Current joint positions
        self.joint_positions = HOME_POSE.copy()
        
        # Safety
        self.estop_active = False
        self.last_command_time = time.time()
        self.watchdog_timeout = 0.5  # 500ms
        
        # Command queue for smooth execution
        self.command_queue = deque(maxlen=10)
        
        print("Teleoperation initialized!")
        print("Controls:")
        print("  Base movement: WASD (forward/left/back/right)")
        print("  Rotation: QE (left/right)")
        print("  Arm joints: 1-6 keys + arrow keys")
        print("  Emergency stop: SPACE")
        print("  Reset to home: R")
        print("  Quit: Ctrl+C")
    
    def clamp_joint_limits(self, joint_idx, position):
        """Clamp joint position to limits"""
        lower, upper = self.sim.joint_limits[joint_idx]
        return max(lower, min(upper, position))
    
    def handle_keyboard_input(self):
        """Handle keyboard input for teleoperation"""
        keys = p.getKeyboardEvents()
        
        for key, state in keys.items():
            if state & p.KEY_WAS_TRIGGERED:
                self.handle_key_press(key)
            elif state & p.KEY_WAS_RELEASED:
                self.handle_key_release(key)
    
    def handle_key_press(self, key):
        """Handle key press events"""
        if key == ord(' '):  # SPACE - Emergency stop
            self.emergency_stop()
            return
        
        if self.estop_active:
            return
        
        # Base movement
        if key == ord('w'):  # Forward
            self.base_vx = self.base_step
        elif key == ord('s'):  # Backward
            self.base_vx = -self.base_step
        elif key == ord('a'):  # Left
            self.base_vy = self.base_step
        elif key == ord('d'):  # Right
            self.base_vy = -self.base_step
        elif key == ord('q'):  # Rotate left
            self.base_wz = self.angular_step
        elif key == ord('e'):  # Rotate right
            self.base_wz = -self.angular_step
        
        # Arm joint control
        elif key in [ord('1'), ord('2'), ord('3'), ord('4'), ord('5'), ord('6')]:
            joint_idx = key - ord('1')
            if joint_idx < len(self.joint_positions):
                # Get arrow key for direction
                arrow_keys = p.getKeyboardEvents()
                for arrow_key, arrow_state in arrow_keys.items():
                    if arrow_state & p.KEY_WAS_TRIGGERED:
                        if arrow_key == p.B3G_UP_ARROW:
                            self.joint_positions[joint_idx] += self.joint_step
                        elif arrow_key == p.B3G_DOWN_ARROW:
                            self.joint_positions[joint_idx] -= self.joint_step
                        
                        # Clamp to limits
                        self.joint_positions[joint_idx] = self.clamp_joint_limits(
                            joint_idx, self.joint_positions[joint_idx]
                        )
                        break
        
        # Reset to home
        elif key == ord('r'):
            self.joint_positions = HOME_POSE.copy()
        
        self.last_command_time = time.time()
    
    def handle_key_release(self, key):
        """Handle key release events"""
        # Stop base movement when keys are released
        if key == ord('w') or key == ord('s'):
            self.base_vx = 0.0
        elif key == ord('a') or key == ord('d'):
            self.base_vy = 0.0
        elif key == ord('q') or key == ord('e'):
            self.base_wz = 0.0
    
    def emergency_stop(self):
        """Emergency stop function"""
        self.estop_active = not self.estop_active
        if self.estop_active:
            print("EMERGENCY STOP ACTIVATED!")
            self.base_vx = 0.0
            self.base_vy = 0.0
            self.base_wz = 0.0
        else:
            print("Emergency stop deactivated.")
    
    def watchdog_check(self):
        """Check if commands are still active"""
        if time.time() - self.last_command_time > self.watchdog_timeout:
            # Stop all movement if no commands received
            self.base_vx = 0.0
            self.base_vy = 0.0
            self.base_wz = 0.0
    
    def update_base_pose(self):
        """Update base pose based on velocities"""
        if self.estop_active:
            return
        
        # Simple integration for base movement
        # In a real implementation, this would use Mecanum kinematics
        current_pos, current_orn = p.getBasePositionAndOrientation(self.sim.robot_id)
        
        # Update position based on velocities
        dt = 1.0/240.0  # Simulation timestep
        new_x = current_pos[0] + self.base_vx * dt
        new_y = current_pos[1] + self.base_vy * dt
        
        # Update orientation based on angular velocity
        current_yaw = p.getEulerFromQuaternion(current_orn)[2]
        new_yaw = current_yaw + self.base_wz * dt
        
        new_orn = p.getQuaternionFromEuler([0, 0, new_yaw])
        
        # Set new base pose
        p.resetBasePositionAndOrientation(
            self.sim.robot_id, 
            [new_x, new_y, current_pos[2]], 
            new_orn
        )
    
    def update_arm_joints(self):
        """Update arm joint positions"""
        if self.estop_active:
            return
        
        self.sim.set_joint_positions(self.joint_positions)
    
    def display_status(self):
        """Display current status"""
        if self.estop_active:
            print("\r[ESTOP] Emergency stop active", end="")
        else:
            print(f"\rBase: Vx={self.base_vx:.2f}, Vy={self.base_vy:.2f}, Wz={self.base_wz:.2f} | "
                  f"Joints: {[f'{j:.2f}' for j in self.joint_positions]}", end="")
    
    def run(self):
        """Main teleoperation loop"""
        print("Starting teleoperation...")
        
        try:
            while True:
                # Handle input
                self.handle_keyboard_input()
                
                # Safety checks
                self.watchdog_check()
                
                # Update robot
                self.update_base_pose()
                self.update_arm_joints()
                
                # Step simulation
                self.sim.step_simulation()
                
                # Display status (less frequently)
                if int(time.time() * 10) % 10 == 0:
                    self.display_status()
                
        except KeyboardInterrupt:
            print("\nTeleoperation stopped.")
        finally:
            p.disconnect()

def main():
    """Main function"""
    print("Starting Agricultural Robot Teleoperation...")
    
    # Create teleop controller
    controller = TeleopController()
    
    # Run teleoperation
    controller.run()

if __name__ == "__main__":
    main()

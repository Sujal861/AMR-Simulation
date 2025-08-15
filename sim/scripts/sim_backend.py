#!/usr/bin/env python3
"""
Simulation Backend for Agricultural Robot
Subscribes to commands and publishes telemetry via TCP/WebSocket
"""

import asyncio
import websockets
import json
import time
import os
import sys
import threading
from typing import Dict, Any, Optional
from dataclasses import dataclass, asdict

# Add parent directory to path for imports
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(__file__))))

from sim.scripts.launch_sim import RobotSimulation
from kinematics.fk import ForwardKinematics
from kinematics.ik_numeric import NumericalInverseKinematics
from kinematics.trajectory import TrajectoryPlanner
from configs.poses import HOME_POSE, get_pose

@dataclass
class RobotState:
    """Robot state data structure"""
    joint_positions: list
    joint_velocities: list
    end_effector_position: list
    end_effector_orientation: list
    base_position: list
    base_orientation: list
    timestamp: float
    is_moving: bool
    error_code: int
    error_message: str

@dataclass
class Command:
    """Command data structure"""
    command_type: str
    target_joints: Optional[list] = None
    target_pose: Optional[list] = None
    target_position: Optional[list] = None
    target_orientation: Optional[list] = None
    pose_name: Optional[str] = None
    trajectory_waypoints: Optional[list] = None
    trajectory_durations: Optional[list] = None
    velocity_scale: float = 1.0
    acceleration_scale: float = 1.0

class SimulationBackend:
    def __init__(self, host: str = "localhost", port: int = 8765):
        """Initialize simulation backend"""
        self.host = host
        self.port = port
        
        # Initialize simulation
        self.sim = RobotSimulation(gui=False)  # Headless for backend
        
        # Initialize kinematics
        self.fk = ForwardKinematics()
        self.ik = NumericalInverseKinematics()
        self.planner = TrajectoryPlanner()
        
        # Control state
        self.is_running = False
        self.is_moving = False
        self.current_command = None
        self.command_queue = asyncio.Queue()
        
        # Safety parameters
        self.max_joint_velocity = 1.0  # rad/s
        self.max_position_error = 0.05  # meters
        self.watchdog_timeout = 0.5  # seconds
        self.last_command_time = time.time()
        
        # Telemetry
        self.telemetry_rate = 50  # Hz
        self.telemetry_interval = 1.0 / self.telemetry_rate
        
        # Connected clients
        self.clients = set()
        
        print(f"Simulation Backend initialized on {host}:{port}")
    
    def get_robot_state(self) -> RobotState:
        """Get current robot state"""
        # Get joint information
        joint_positions = self.sim.get_joint_positions()
        joint_velocities = [0.0] * len(joint_positions)  # Placeholder
        
        # Get end effector pose
        end_pos, end_orn = self.sim.get_end_effector_pose()
        
        # Get base pose
        base_pos, base_orn = self.sim.get_base_position_and_orientation()
        
        # Check if moving
        is_moving = self.is_moving
        
        # Error handling
        error_code = 0
        error_message = ""
        
        # Check joint limits
        within_limits, _ = self.fk.check_joint_limits(joint_positions)
        if not within_limits:
            error_code = 1
            error_message = "Joint limits exceeded"
        
        # Check watchdog
        if time.time() - self.last_command_time > self.watchdog_timeout:
            if self.is_moving:
                error_code = 2
                error_message = "Watchdog timeout"
                self.is_moving = False
        
        return RobotState(
            joint_positions=joint_positions,
            joint_velocities=joint_velocities,
            end_effector_position=list(end_pos),
            end_effector_orientation=list(end_orn),
            base_position=list(base_pos),
            base_orientation=list(base_orn),
            timestamp=time.time(),
            is_moving=is_moving,
            error_code=error_code,
            error_message=error_message
        )
    
    def execute_joint_command(self, target_joints: list) -> bool:
        """Execute joint position command"""
        try:
            # Validate joint limits
            clamped_joints = self.fk.clamp_joint_angles(target_joints)
            
            # Set joint positions
            self.sim.set_joint_positions(clamped_joints)
            
            self.is_moving = True
            self.last_command_time = time.time()
            
            return True
            
        except Exception as e:
            print(f"Error executing joint command: {e}")
            return False
    
    def execute_pose_command(self, pose_name: str) -> bool:
        """Execute pose command"""
        try:
            pose = get_pose(pose_name)
            return self.execute_joint_command(pose)
            
        except Exception as e:
            print(f"Error executing pose command: {e}")
            return False
    
    def execute_ik_command(self, target_position: list, target_orientation: list) -> bool:
        """Execute inverse kinematics command"""
        try:
            # Convert to numpy arrays
            import numpy as np
            target_pos = np.array(target_position)
            target_orn = np.array(target_orientation)
            
            # Get current joint positions as initial guess
            current_joints = self.sim.get_joint_positions()
            
            # Solve IK
            solution, success = self.ik.inverse_kinematics(
                target_pos, target_orn, current_joints
            )
            
            if success:
                return self.execute_joint_command(solution)
            else:
                print("IK solution not found")
                return False
                
        except Exception as e:
            print(f"Error executing IK command: {e}")
            return False
    
    def execute_trajectory_command(self, waypoints: list, durations: list) -> bool:
        """Execute trajectory command"""
        try:
            # Generate trajectory
            positions, times = self.planner.waypoint_trajectory(
                waypoints, durations, dt=0.01
            )
            
            # Store trajectory for execution
            self.current_trajectory = {
                'positions': positions,
                'times': times,
                'current_index': 0,
                'start_time': time.time()
            }
            
            self.is_moving = True
            self.last_command_time = time.time()
            
            return True
            
        except Exception as e:
            print(f"Error executing trajectory command: {e}")
            return False
    
    def execute_command(self, command: Command) -> Dict[str, Any]:
        """Execute a command and return result"""
        try:
            success = False
            
            if command.command_type == "joint":
                success = self.execute_joint_command(command.target_joints)
            elif command.command_type == "pose":
                success = self.execute_pose_command(command.pose_name)
            elif command.command_type == "ik":
                success = self.execute_ik_command(command.target_position, command.target_orientation)
            elif command.command_type == "trajectory":
                success = self.execute_trajectory_command(command.trajectory_waypoints, command.trajectory_durations)
            elif command.command_type == "stop":
                self.is_moving = False
                success = True
            elif command.command_type == "home":
                success = self.execute_pose_command("home")
            else:
                return {
                    "success": False,
                    "error": f"Unknown command type: {command.command_type}"
                }
            
            return {
                "success": success,
                "command_type": command.command_type,
                "timestamp": time.time()
            }
            
        except Exception as e:
            return {
                "success": False,
                "error": str(e),
                "command_type": command.command_type,
                "timestamp": time.time()
            }
    
    def update_trajectory(self):
        """Update trajectory execution"""
        if self.current_trajectory is None or not self.is_moving:
            return
        
        trajectory = self.current_trajectory
        current_index = trajectory['current_index']
        positions = trajectory['positions']
        times = trajectory['times']
        
        # Check if trajectory is complete
        if current_index >= len(positions):
            self.is_moving = False
            self.current_trajectory = None
            return
        
        # Get current target position
        target_position = positions[current_index]
        
        # Set joint positions
        self.sim.set_joint_positions(target_position)
        
        # Check execution time
        elapsed_time = time.time() - trajectory['start_time']
        expected_time = times[current_index]
        
        # Move to next point if time has elapsed
        if elapsed_time >= expected_time:
            trajectory['current_index'] += 1
    
    async def handle_client(self, websocket, path):
        """Handle WebSocket client connection"""
        client_id = id(websocket)
        self.clients.add(websocket)
        
        print(f"Client {client_id} connected")
        
        try:
            async for message in websocket:
                try:
                    # Parse command
                    data = json.loads(message)
                    command = Command(**data)
                    
                    # Execute command
                    result = self.execute_command(command)
                    
                    # Send response
                    await websocket.send(json.dumps(result))
                    
                except json.JSONDecodeError:
                    await websocket.send(json.dumps({
                        "success": False,
                        "error": "Invalid JSON"
                    }))
                except Exception as e:
                    await websocket.send(json.dumps({
                        "success": False,
                        "error": str(e)
                    }))
                    
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            self.clients.remove(websocket)
            print(f"Client {client_id} disconnected")
    
    async def telemetry_loop(self):
        """Telemetry publishing loop"""
        while self.is_running:
            try:
                # Update trajectory if needed
                self.update_trajectory()
                
                # Get robot state
                state = self.get_robot_state()
                
                # Send telemetry to all clients
                if self.clients:
                    telemetry_data = {
                        "type": "telemetry",
                        "data": asdict(state)
                    }
                    
                    # Send to all connected clients
                    disconnected_clients = set()
                    for client in self.clients:
                        try:
                            await client.send(json.dumps(telemetry_data))
                        except websockets.exceptions.ConnectionClosed:
                            disconnected_clients.add(client)
                    
                    # Remove disconnected clients
                    self.clients -= disconnected_clients
                
                # Step simulation
                self.sim.step_simulation()
                
                # Wait for next telemetry cycle
                await asyncio.sleep(self.telemetry_interval)
                
            except Exception as e:
                print(f"Error in telemetry loop: {e}")
                await asyncio.sleep(0.1)
    
    async def start_server(self):
        """Start the WebSocket server"""
        self.is_running = True
        
        # Start telemetry loop
        telemetry_task = asyncio.create_task(self.telemetry_loop())
        
        # Start WebSocket server
        server = await websockets.serve(
            self.handle_client,
            self.host,
            self.port
        )
        
        print(f"WebSocket server started on ws://{self.host}:{self.port}")
        
        try:
            # Keep server running
            await asyncio.Future()  # Run forever
        except KeyboardInterrupt:
            print("Shutting down server...")
        finally:
            self.is_running = False
            server.close()
            await server.wait_closed()
            telemetry_task.cancel()
    
    def run(self):
        """Run the simulation backend"""
        try:
            asyncio.run(self.start_server())
        except KeyboardInterrupt:
            print("Simulation backend stopped")

def main():
    """Main function"""
    print("Starting Agricultural Robot Simulation Backend...")
    
    # Create backend
    backend = SimulationBackend()
    
    # Run backend
    backend.run()

if __name__ == "__main__":
    main()

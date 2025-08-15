#!/usr/bin/env python3
"""
Control Server for Agricultural Robot
Sends commands to simulation backend via WebSocket
"""

import asyncio
import websockets
import json
import time
import argparse
from typing import Dict, Any, Optional

class ControlServer:
    def __init__(self, backend_host: str = "localhost", backend_port: int = 8765):
        """Initialize control server"""
        self.backend_url = f"ws://{backend_host}:{backend_port}"
        self.websocket = None
        self.is_connected = False
        
        # Command templates
        self.command_templates = {
            "home": {
                "command_type": "home"
            },
            "stop": {
                "command_type": "stop"
            },
            "joint": {
                "command_type": "joint",
                "target_joints": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            },
            "pose": {
                "command_type": "pose",
                "pose_name": "home"
            },
            "ik": {
                "command_type": "ik",
                "target_position": [0.2, 0.05, 0.3],
                "target_orientation": [0.0, 0.0, 0.0]
            },
            "trajectory": {
                "command_type": "trajectory",
                "trajectory_waypoints": [
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.5, 0.2, 0.1, 0.1, 0.05, 0.0],
                    [1.0, 0.5, 0.3, 0.2, 0.1, 0.0]
                ],
                "trajectory_durations": [2.0, 2.0]
            }
        }
        
        print(f"Control Server initialized, connecting to {self.backend_url}")
    
    async def connect(self):
        """Connect to simulation backend"""
        try:
            self.websocket = await websockets.connect(self.backend_url)
            self.is_connected = True
            print("Connected to simulation backend")
            return True
        except Exception as e:
            print(f"Failed to connect: {e}")
            return False
    
    async def disconnect(self):
        """Disconnect from simulation backend"""
        if self.websocket:
            await self.websocket.close()
            self.is_connected = False
            print("Disconnected from simulation backend")
    
    async def send_command(self, command: Dict[str, Any]) -> Dict[str, Any]:
        """Send command to backend and get response"""
        if not self.is_connected:
            return {"success": False, "error": "Not connected"}
        
        try:
            # Send command
            await self.websocket.send(json.dumps(command))
            
            # Wait for response
            response = await self.websocket.recv()
            return json.loads(response)
            
        except Exception as e:
            return {"success": False, "error": str(e)}
    
    async def listen_telemetry(self):
        """Listen for telemetry data from backend"""
        if not self.is_connected:
            return
        
        try:
            async for message in self.websocket:
                try:
                    data = json.loads(message)
                    if data.get("type") == "telemetry":
                        self.display_telemetry(data["data"])
                except json.JSONDecodeError:
                    pass
        except websockets.exceptions.ConnectionClosed:
            self.is_connected = False
            print("Connection to backend closed")
    
    def display_telemetry(self, telemetry: Dict[str, Any]):
        """Display telemetry data"""
        print(f"\rRobot State: Joints={telemetry['joint_positions'][:3]}... "
              f"EE Pos={telemetry['end_effector_position']} "
              f"Moving={telemetry['is_moving']} "
              f"Error={telemetry['error_code']}", end="")
    
    async def interactive_control(self):
        """Interactive control mode"""
        print("Interactive Control Mode")
        print("Commands:")
        print("  'home' - Go to home pose")
        print("  'stop' - Stop movement")
        print("  'pose <name>' - Go to specific pose")
        print("  'joint <j1> <j2> <j3> <j4> <j5> <j6>' - Set joint positions")
        print("  'ik <x> <y> <z> <roll> <pitch> <yaw>' - Inverse kinematics")
        print("  'trajectory' - Execute demo trajectory")
        print("  'status' - Show robot status")
        print("  'quit' - Exit")
        
        # Start telemetry listener
        telemetry_task = asyncio.create_task(self.listen_telemetry())
        
        try:
            while True:
                command = input("\nEnter command: ").strip().lower().split()
                
                if not command:
                    continue
                
                if command[0] == 'quit':
                    break
                elif command[0] == 'home':
                    result = await self.send_command(self.command_templates["home"])
                    print(f"Result: {result}")
                elif command[0] == 'stop':
                    result = await self.send_command(self.command_templates["stop"])
                    print(f"Result: {result}")
                elif command[0] == 'pose' and len(command) > 1:
                    pose_name = command[1]
                    cmd = self.command_templates["pose"].copy()
                    cmd["pose_name"] = pose_name
                    result = await self.send_command(cmd)
                    print(f"Result: {result}")
                elif command[0] == 'joint' and len(command) > 6:
                    try:
                        joints = [float(x) for x in command[1:7]]
                        cmd = self.command_templates["joint"].copy()
                        cmd["target_joints"] = joints
                        result = await self.send_command(cmd)
                        print(f"Result: {result}")
                    except ValueError:
                        print("Invalid joint values")
                elif command[0] == 'ik' and len(command) > 5:
                    try:
                        pos = [float(x) for x in command[1:4]]
                        orn = [float(x) for x in command[4:7]]
                        cmd = self.command_templates["ik"].copy()
                        cmd["target_position"] = pos
                        cmd["target_orientation"] = orn
                        result = await self.send_command(cmd)
                        print(f"Result: {result}")
                    except ValueError:
                        print("Invalid IK values")
                elif command[0] == 'trajectory':
                    result = await self.send_command(self.command_templates["trajectory"])
                    print(f"Result: {result}")
                elif command[0] == 'status':
                    print(f"Connected: {self.is_connected}")
                else:
                    print("Unknown command")
                    
        except KeyboardInterrupt:
            print("\nInteractive control stopped")
        finally:
            telemetry_task.cancel()
    
    async def run_demo_sequence(self):
        """Run a demo sequence of commands"""
        print("Running demo sequence...")
        
        demo_commands = [
            ("Go to home", self.command_templates["home"]),
            ("Go to approach pose", {"command_type": "pose", "pose_name": "approach_plant"}),
            ("Go to drill pose", {"command_type": "pose", "pose_name": "drill_position"}),
            ("Go to place pose", {"command_type": "pose", "pose_name": "place_seed"}),
            ("Return home", self.command_templates["home"])
        ]
        
        for name, command in demo_commands:
            print(f"\nExecuting: {name}")
            result = await self.send_command(command)
            print(f"Result: {result}")
            
            if not result.get("success", False):
                print(f"Command failed: {result.get('error', 'Unknown error')}")
                break
            
            # Wait between commands
            await asyncio.sleep(2.0)
        
        print("Demo sequence completed")
    
    async def run_ik_demo(self):
        """Run inverse kinematics demo"""
        print("Running IK demo...")
        
        # Define target positions
        targets = [
            ([0.2, 0.05, 0.3], [0.0, 0.0, 0.0]),  # Forward position
            ([0.3, 0.2, 0.4], [0.0, 0.0, 0.0]),  # Right position
            ([0.3, -0.2, 0.4], [0.0, 0.0, 0.0]), # Left position
            ([0.2, 0.0, 0.6], [0.0, 0.0, 0.0]),  # Higher position
        ]
        
        for i, (pos, orn) in enumerate(targets):
            print(f"\nIK target {i+1}: position={pos}, orientation={orn}")
            
            command = {
                "command_type": "ik",
                "target_position": pos,
                "target_orientation": orn
            }
            
            result = await self.send_command(command)
            print(f"Result: {result}")
            
            if not result.get("success", False):
                print(f"IK failed: {result.get('error', 'Unknown error')}")
                break
            
            # Wait between targets
            await asyncio.sleep(3.0)
        
        print("IK demo completed")

async def main():
    """Main function"""
    parser = argparse.ArgumentParser(description="Agricultural Robot Control Server")
    parser.add_argument("--host", default="localhost", help="Backend host")
    parser.add_argument("--port", type=int, default=8765, help="Backend port")
    parser.add_argument("--demo", action="store_true", help="Run demo sequence")
    parser.add_argument("--ik-demo", action="store_true", help="Run IK demo")
    
    args = parser.parse_args()
    
    print("Starting Agricultural Robot Control Server...")
    
    # Create control server
    server = ControlServer(args.host, args.port)
    
    # Connect to backend
    if not await server.connect():
        print("Failed to connect to backend. Make sure sim_backend.py is running.")
        return
    
    try:
        if args.demo:
            await server.run_demo_sequence()
        elif args.ik_demo:
            await server.run_ik_demo()
        else:
            await server.interactive_control()
    finally:
        await server.disconnect()

if __name__ == "__main__":
    asyncio.run(main())

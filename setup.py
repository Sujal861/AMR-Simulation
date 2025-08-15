#!/usr/bin/env python3
"""
Setup script for Agricultural Robot Simulation
"""

import subprocess
import sys
import os

def run_command(command, description):
    """Run a command and handle errors"""
    print(f"Running: {description}")
    try:
        result = subprocess.run(command, shell=True, check=True, capture_output=True, text=True)
        print(f"[OK] {description} completed successfully")
        return True
    except subprocess.CalledProcessError as e:
        print(f"[FAIL] {description} failed: {e}")
        print(f"Error output: {e.stderr}")
        return False

def check_python_version():
    """Check Python version"""
    version = sys.version_info
    if version.major < 3 or (version.major == 3 and version.minor < 8):
        print("[FAIL] Python 3.8 or higher is required")
        return False
    print(f"[OK] Python {version.major}.{version.minor}.{version.micro} detected")
    return True

def install_dependencies():
    """Install required dependencies"""
    print("\nInstalling dependencies...")
    
    # Upgrade pip
    if not run_command("python -m pip install --upgrade pip", "Upgrading pip"):
        return False
    
    # Install requirements
    if not run_command("pip install -r requirements.txt", "Installing requirements"):
        return False
    
    return True

def create_virtual_environment():
    """Create virtual environment if it doesn't exist"""
    venv_path = "venv"
    
    if os.path.exists(venv_path):
        print(f"[OK] Virtual environment already exists at {venv_path}")
        return True
    
    print("Creating virtual environment...")
    if not run_command("python -m venv venv", "Creating virtual environment"):
        return False
    
    print("[OK] Virtual environment created")
    print("\nTo activate the virtual environment:")
    if os.name == 'nt':  # Windows
        print("  venv\\Scripts\\activate")
    else:  # Linux/macOS
        print("  source venv/bin/activate")
    
    return True

def run_tests():
    """Run the test script"""
    print("\nRunning tests...")
    if not run_command("python test_setup.py", "Running setup tests"):
        return False
    return True

def main():
    """Main setup function"""
    print("Agricultural Robot Simulation Setup")
    print("=" * 50)
    
    # Check Python version
    if not check_python_version():
        return 1
    
    # Create virtual environment
    if not create_virtual_environment():
        return 1
    
    # Install dependencies
    if not install_dependencies():
        return 1
    
    # Run tests
    if not run_tests():
        return 1
    
    print("\n" + "=" * 50)
    print("[SUCCESS] Setup completed successfully!")
    print("\nNext steps:")
    print("1. Activate virtual environment:")
    if os.name == 'nt':  # Windows
        print("   venv\\Scripts\\activate")
    else:  # Linux/macOS
        print("   source venv/bin/activate")
    print("2. Run simulation: python sim/scripts/launch_sim.py")
    print("3. Run teleoperation: python sim/scripts/teleop.py")
    print("4. Run waypoint following: python sim/scripts/follow_waypoints.py")
    print("5. Run pose demo: python sim/scripts/demo_poses.py")
    print("6. Run tests: pytest tests/")
    print("\nFor more information, see README.md")
    
    return 0

if __name__ == "__main__":
    sys.exit(main())

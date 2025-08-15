#!/usr/bin/env python3
"""
Test script to verify agricultural robot simulation setup
"""

import sys
import os
import numpy as np

def test_imports():
    """Test that all modules can be imported"""
    print("Testing imports...")
    
    try:
        # Test kinematics imports
        from kinematics.fk import ForwardKinematics
        from kinematics.ik_numeric import NumericalInverseKinematics
        from kinematics.trajectory import TrajectoryPlanner
        print("[OK] Kinematics modules imported successfully")
        
        # Test config imports
        from configs.poses import HOME_POSE, AGRICULTURAL_POSES
        print("[OK] Config modules imported successfully")
        
        # Test simulation imports (without GUI)
        import pybullet as p
        print("[OK] PyBullet imported successfully")
        
        return True
        
    except ImportError as e:
        print(f"[FAIL] Import error: {e}")
        return False
    except Exception as e:
        print(f"[FAIL] Unexpected error: {e}")
        return False

def test_kinematics():
    """Test kinematics functionality"""
    print("\nTesting kinematics...")
    
    try:
        from kinematics.fk import ForwardKinematics
        from kinematics.ik_numeric import NumericalInverseKinematics
        
        # Test forward kinematics
        fk = ForwardKinematics()
        home_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        position, orientation = fk.forward_kinematics(home_pose)
        print(f"[OK] Forward kinematics: position={position}")
        
        # Test inverse kinematics
        ik = NumericalInverseKinematics()
        target_pos = np.array([0.2, 0.05, 0.3])
        target_orn = np.array([0.0, 0.0, 0.0])
        solution, success = ik.inverse_kinematics(target_pos, target_orn)
        print(f"[OK] Inverse kinematics: success={success}")
        
        return True
        
    except Exception as e:
        print(f"[FAIL] Kinematics error: {e}")
        return False

def test_trajectory():
    """Test trajectory planning"""
    print("\nTesting trajectory planning...")
    
    try:
        from kinematics.trajectory import TrajectoryPlanner
        
        planner = TrajectoryPlanner()
        start_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        end_pos = [1.0, 0.5, 0.3, 0.2, 0.1, 0.0]
        
        positions, times = planner.cubic_interpolation(start_pos, end_pos, duration=2.0)
        print(f"[OK] Trajectory planning: {len(positions)} points generated")
        
        return True
        
    except Exception as e:
        print(f"[FAIL] Trajectory error: {e}")
        return False

def test_poses():
    """Test pose configuration"""
    print("\nTesting pose configuration...")
    
    try:
        from configs.poses import HOME_POSE, AGRICULTURAL_POSES, get_pose
        
        print(f"[OK] Home pose: {HOME_POSE}")
        print(f"[OK] Available poses: {list(AGRICULTURAL_POSES.keys())}")
        
        # Test pose retrieval
        drill_pose = get_pose("drill_position")
        print(f"[OK] Drill pose: {drill_pose}")
        
        return True
        
    except Exception as e:
        print(f"[FAIL] Pose error: {e}")
        return False

def test_simulation():
    """Test simulation setup (headless)"""
    print("\nTesting simulation setup...")
    
    try:
        import pybullet as p
        
        # Connect to PyBullet in headless mode
        physics_client = p.connect(p.DIRECT)
        
        # Set gravity
        p.setGravity(0, 0, -9.81)
        
        # Load plane
        import pybullet_data
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        plane_id = p.loadURDF("plane.urdf")
        
        # Load robot
        robot_path = os.path.join("sim", "models", "robot.urdf")
        robot_id = p.loadURDF(robot_path, [0, 0, 0.1])
        
        num_joints = p.getNumJoints(robot_id)
        print(f"[OK] Robot loaded with {num_joints} joints")
        
        # Disconnect
        p.disconnect()
        
        return True
        
    except Exception as e:
        print(f"[FAIL] Simulation error: {e}")
        return False

def main():
    """Run all tests"""
    print("Agricultural Robot Simulation Setup Test")
    print("=" * 50)
    
    tests = [
        test_imports,
        test_kinematics,
        test_trajectory,
        test_poses,
        test_simulation
    ]
    
    passed = 0
    total = len(tests)
    
    for test in tests:
        if test():
            passed += 1
    
    print("\n" + "=" * 50)
    print(f"Test Results: {passed}/{total} tests passed")
    
    if passed == total:
        print("[SUCCESS] All tests passed! Setup is working correctly.")
        print("\nNext steps:")
        print("1. Run simulation: python sim/scripts/launch_sim.py")
        print("2. Run teleoperation: python sim/scripts/teleop.py")
        print("3. Run waypoint following: python sim/scripts/follow_waypoints.py")
        print("4. Run pose demo: python sim/scripts/demo_poses.py")
        print("5. Run tests: pytest tests/")
    else:
        print("[FAIL] Some tests failed. Please check the errors above.")
        return 1
    
    return 0

if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""
VNC Test Script for Phase 1

Simple test script to verify Phase 1 components work in VNC environment.
Can be run without ROS2 for basic functionality testing.
"""

import sys
import time
import numpy as np
from pathlib import Path

# Add scripts path for canvas coordinate system
# Support both local and VNC environments
import os
if os.path.exists('/workspace/robot/scripts'):
    sys.path.append('/workspace/robot/scripts')  # VNC environment
else:
    sys.path.append('/Users/smallspring/programs/wanderline/robot/scripts')  # Local environment

try:
    from system_state import create_default_system_state, ContactPoint
    from coordinate_calculator import create_coordinate_calculator
    from canvas_coordinate_system import CanvasCoordinateSystem
    print("✅ All Phase 1 imports successful")
except ImportError as e:
    print(f"❌ Import error: {e}")
    sys.exit(1)


def test_basic_functionality():
    """Test basic Phase 1 functionality without ROS2."""
    print("\n🧪 Testing Phase 1 Basic Functionality...")
    
    # Test 1: System State
    print("1. Testing SystemState...")
    state = create_default_system_state()
    print(f"   Initial state: {state.get_system_status()}")
    
    # Test 2: Coordinate Calculator
    print("2. Testing CoordinateCalculator...")
    config = {
        'segments': 24,
        'max_stroke_length': 0.005,
        'closure_threshold': 0.95,
        'perimeter_tolerance': 5.0
    }
    calculator = create_coordinate_calculator(config)
    
    current_pos = (400, 300)
    next_coord, complete = calculator.calculate_next_coordinate(current_pos, state)
    print(f"   Next coordinate: {next_coord}, Complete: {complete}")
    
    # Test 3: Canvas Coordinate System
    print("3. Testing CanvasCoordinateSystem...")
    canvas_system = CanvasCoordinateSystem()  # Using default config
    
    # Test pixel to robot conversion
    robot_coords = canvas_system.pixel_to_robot_coords(400, 300, pen_down=True)
    print(f"   Canvas center (400,300) -> Robot coords: {robot_coords}")
    
    # Test robot to pixel conversion
    pixel_coords = canvas_system.robot_to_pixel_coords(robot_coords[0], robot_coords[1])
    print(f"   Robot coords -> Pixel coords: {pixel_coords}")
    
    # Test 4: Simulate drawing progress
    print("4. Testing drawing progress simulation...")
    
    # Add some contact points around the circle
    circle_center = (400, 300)
    circle_radius = 80
    
    for i in range(0, 360, 45):  # 8 points around circle
        angle = np.radians(i)
        x = circle_center[0] + circle_radius * np.cos(angle)
        y = circle_center[1] + circle_radius * np.sin(angle)
        
        # Convert to robot coordinates
        robot_x, robot_y, robot_z = canvas_system.pixel_to_robot_coords(x, y, pen_down=True)
        
        # Create contact point
        contact = ContactPoint(
            position_3d=(robot_x, robot_y, robot_z),
            position_2d=(int(x), int(y)),
            timestamp=time.time(),
            stroke_id=i // 45
        )
        
        state.add_contact_point(contact)
        
        # Calculate progress
        progress = calculator._calculate_circle_progress(state)
        print(f"   Point {i//45 + 1}: ({x:.1f}, {y:.1f}) -> Progress: {progress:.2f}")
    
    print("✅ Basic functionality test completed!")


def test_joint_calculation():
    """Test joint position calculation."""
    print("\n🤖 Testing Joint Position Calculation...")
    
    canvas_system = CanvasCoordinateSystem()  # Using default config
    
    # Base joint configuration (from existing system)
    base_joints = [0.0, -1.2, -1.0, -1.5, 1.57, 0.0]
    
    # Test positions around circle
    test_positions = [
        (400, 300),  # Center
        (480, 300),  # Right
        (400, 220),  # Top
        (320, 300),  # Left
        (400, 380),  # Bottom
    ]
    
    for i, (pixel_x, pixel_y) in enumerate(test_positions):
        # Convert to robot coordinates
        robot_x, robot_y, robot_z = canvas_system.pixel_to_robot_coords(pixel_x, pixel_y, pen_down=True)
        
        # Calculate joint positions (simplified)
        canvas_center_x, canvas_center_y = 0.5, 0.0
        pan_amplitude = 0.6   # ±34 degrees
        lift_amplitude = 0.3  # ±17 degrees
        
        pan_offset = (robot_x - canvas_center_x) / 0.2 * pan_amplitude
        lift_offset = (robot_y - canvas_center_y) / 0.2 * lift_amplitude
        
        joints = base_joints.copy()
        joints[0] = base_joints[0] + pan_offset   # shoulder_pan_joint
        joints[1] = base_joints[1] + lift_offset  # shoulder_lift_joint
        
        print(f"   Position {i+1}: ({pixel_x}, {pixel_y}) -> Joints: [{joints[0]:.3f}, {joints[1]:.3f}, ...]")
    
    print("✅ Joint calculation test completed!")


def test_config_loading():
    """Test configuration loading."""
    print("\n⚙️ Testing Configuration Loading...")
    
    config_path = Path(__file__).parent / 'config.yaml'
    
    if config_path.exists():
        import yaml
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        print(f"   Config loaded: {config['phase1']['robot']['base_joints']}")
        print(f"   Drawing segments: {config['phase1']['drawing']['segments']}")
    else:
        print("   ⚠️ Config file not found, using defaults")
    
    print("✅ Configuration test completed!")


def main():
    """Run all VNC tests."""
    print("🚀 Phase 1 VNC Test Suite")
    print("=" * 50)
    
    try:
        test_basic_functionality()
        test_joint_calculation()
        test_config_loading()
        
        print("\n" + "=" * 50)
        print("✅ All VNC tests passed!")
        print("🎯 Phase 1 is ready for VNC environment testing")
        
    except Exception as e:
        print(f"\n❌ Test failed: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
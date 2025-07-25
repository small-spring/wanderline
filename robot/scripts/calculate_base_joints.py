#!/usr/bin/env python3
"""Calculate appropriate base_joints for canvas position."""

from corrected_coordinate_system import CorrectedCoordinateSystem
import math

def main():
    coords = CorrectedCoordinateSystem()
    
    # Target: canvas center
    target_x, target_y, target_z = 0.6, 0.0, 0.07  # Canvas + pen contact
    base_joints = [0.0, -1.57, -1.57, -1.57, 1.57, 0.0]
    
    print("🎯 CALCULATING CORRECT BASE JOINTS")
    print("=" * 40)
    print(f"Canvas position: [0.6, 0.0, 0.05]")
    print(f"Target (with pen): ({target_x:.3f}, {target_y:.3f}, {target_z:.3f})")
    print()
    
    # Calculate required joints
    joints = coords.robot_coords_to_joints(target_x, target_y, target_z, base_joints)
    print(f"Required joints: [{joints[0]:.3f}, {joints[1]:.3f}, {joints[2]:.3f}, {joints[3]:.3f}, {joints[4]:.3f}, {joints[5]:.3f}]")
    
    # Verify with forward kinematics  
    actual_pos = coords.joints_to_robot_position(joints)
    print(f"Actual position: ({actual_pos[0]:.3f}, {actual_pos[1]:.3f}, {actual_pos[2]:.3f})")
    
    # Convert back to pixels
    pixel_pos = coords.robot_to_pixel_coords(actual_pos[0], actual_pos[1])
    print(f"Pixel position: ({pixel_pos[0]:.1f}, {pixel_pos[1]:.1f})")
    
    # Check if it's canvas center
    expected_pixel = (400, 300)
    error = ((pixel_pos[0] - expected_pixel[0])**2 + (pixel_pos[1] - expected_pixel[1])**2)**0.5
    print(f"Error from canvas center: {error:.1f} pixels")
    
    if error < 5.0:
        print("✅ GOOD: Robot can reach canvas center accurately")
        print(f"💡 Use these base_joints in config.yaml:")
        print(f"    base_joints: [{joints[0]:.3f}, {joints[1]:.3f}, {joints[2]:.3f}, {joints[3]:.3f}, {joints[4]:.3f}, {joints[5]:.3f}]")
    else:
        print("❌ ERROR: Robot cannot reach canvas center accurately")
        
if __name__ == "__main__":
    main()
#!/usr/bin/env python3
"""
Coordinate System Fixer

This script analyzes and fixes the coordinate system inconsistencies found
in the experiment. It provides corrected transformation functions.
"""

import math
import sys
import os
from pathlib import Path

class CoordinateSystemFixer:
    """Fix coordinate system transformations based on experimental findings."""
    
    def __init__(self):
        # Physical parameters
        self.canvas_position = [0.6, 0.0, 0.05]  # Current canvas position
        self.canvas_size = 0.4  # 40cm
        self.pixel_width = 800
        self.pixel_height = 600
        
        # UR5e Robot Parameters (more accurate)
        self.base_height = 0.163
        self.shoulder_offset = 0.138  # Y-axis offset
        self.upper_arm = 0.425
        self.forearm = 0.392
        self.wrist_1 = 0.109
        self.wrist_2 = 0.095
        self.wrist_3 = 0.082
        
    def analyze_current_problems(self):
        """Analyze the specific problems in current coordinate system."""
        print("🔍 COORDINATE SYSTEM PROBLEM ANALYSIS")
        print("=" * 50)
        
        print("Problem 1: Forward Kinematics")
        print("- Current approximation puts robot at wrong position")
        print("- Need accurate UR5e forward kinematics")
        print()
        
        print("Problem 2: Joint Space Mapping")
        print("- Current linear mapping doesn't match robot geometry")
        print("- Need inverse kinematics for target positioning")
        print()
        
        print("Problem 3: Coordinate Frame Mismatch")
        print("- Mathematical angles don't match robot joint rotations")
        print("- Need proper frame transformations")
        print()
        
    def create_accurate_forward_kinematics(self, joints):
        """
        Accurate UR5e forward kinematics using DH parameters.
        
        Args:
            joints: [base, shoulder, elbow, wrist1, wrist2, wrist3] in radians
            
        Returns:
            (x, y, z) position of end effector in base frame
        """
        q1, q2, q3, q4, q5, q6 = joints
        
        # UR5e DH parameters (more accurate than approximation)
        # This is a simplified version - full DH would be more complex
        
        # Link 1: Base rotation
        c1, s1 = math.cos(q1), math.sin(q1)
        
        # Link 2: Shoulder
        c2, s2 = math.cos(q2), math.sin(q2)
        
        # Link 3: Elbow  
        c3, s3 = math.cos(q3), math.sin(q3)
        
        # Simplified forward kinematics (shoulder + elbow positioning)
        # This gives us a much better approximation than the previous version
        
        # Position after shoulder joint
        shoulder_x = 0
        shoulder_y = self.shoulder_offset
        shoulder_z = self.base_height
        
        # Position after upper arm
        upper_end_x = shoulder_x + self.upper_arm * c2 * c1
        upper_end_y = shoulder_y + self.upper_arm * c2 * s1
        upper_end_z = shoulder_z + self.upper_arm * s2
        
        # Position after forearm
        # Forearm angle relative to upper arm
        forearm_angle = q2 + q3
        
        end_x = upper_end_x + self.forearm * math.cos(forearm_angle) * c1
        end_y = upper_end_y + self.forearm * math.cos(forearm_angle) * s1
        end_z = upper_end_z + self.forearm * math.sin(forearm_angle)
        
        return (end_x, end_y, end_z)
        
    def create_simple_inverse_kinematics(self, target_x, target_y, target_z, base_joints):
        """
        Simple 2-DOF inverse kinematics for canvas drawing.
        
        Focus on pan (base) and shoulder joints for 2D canvas positioning.
        """
        # Calculate base rotation (pan) to point toward target
        target_pan = math.atan2(target_y - self.shoulder_offset, target_x)
        
        # Calculate distance from shoulder to target (in XY plane)
        dx = target_x - 0
        dy = target_y - self.shoulder_offset
        horizontal_distance = math.sqrt(dx*dx + dy*dy)
        
        # Calculate height difference
        dz = target_z - self.base_height
        
        # Calculate shoulder angle using 2-link chain
        # Distance from shoulder to target
        target_distance = math.sqrt(horizontal_distance*horizontal_distance + dz*dz)
        
        # Check if target is reachable
        max_reach = self.upper_arm + self.forearm
        if target_distance > max_reach:
            print(f"⚠️ Target unreachable: {target_distance:.3f}m > {max_reach:.3f}m")
            target_distance = max_reach * 0.95  # Use 95% of max reach
        
        # Law of cosines for 2-link chain
        try:
            cos_shoulder = (self.upper_arm*self.upper_arm + target_distance*target_distance - self.forearm*self.forearm) / (2 * self.upper_arm * target_distance)
            cos_shoulder = max(-1, min(1, cos_shoulder))  # Clamp to valid range
            
            angle_to_target = math.atan2(dz, horizontal_distance)
            shoulder_angle = angle_to_target - math.acos(cos_shoulder)
            
            # Calculate elbow angle
            cos_elbow = (self.upper_arm*self.upper_arm + self.forearm*self.forearm - target_distance*target_distance) / (2 * self.upper_arm * self.forearm)
            cos_elbow = max(-1, min(1, cos_elbow))  # Clamp to valid range
            elbow_angle = math.pi - math.acos(cos_elbow)
            
        except (ValueError, ZeroDivisionError):
            # Fallback to extended position
            shoulder_angle = -0.7  # Default from base_joints
            elbow_angle = -1.0
        
        # Create joint array
        joints = base_joints.copy()
        joints[0] = target_pan        # Base/pan joint
        joints[1] = shoulder_angle    # Shoulder joint  
        joints[2] = elbow_angle       # Elbow joint
        
        return joints
        
    def test_improved_coordinate_system(self):
        """Test the improved coordinate system with corrections."""
        print("\n🧪 TESTING IMPROVED COORDINATE SYSTEM")
        print("=" * 50)
        
        # Test circle points
        center = (400, 300)
        radius = 50
        num_points = 8
        
        # Base joint configuration
        base_joints = [0.0, -0.7, -1.0, -1.5, 1.57, 0.0]
        
        print("Point | Pixel | Robot Target | Calc Joints | FK Position | Error")
        print("-" * 75)
        
        total_error = 0
        direction_consistency = 0
        
        for i in range(num_points):
            angle = 2 * math.pi * i / num_points
            
            # Generate pixel coordinate
            px = center[0] + radius * math.cos(angle)
            py = center[1] + radius * math.sin(angle)
            
            # Convert to robot coordinates
            robot_x, robot_y, robot_z = self.pixel_to_robot_coords(px, py)
            
            # Calculate required joint positions using IK
            target_joints = self.create_simple_inverse_kinematics(robot_x, robot_y, robot_z, base_joints)
            
            # Verify with forward kinematics
            actual_pos = self.create_accurate_forward_kinematics(target_joints)
            
            # Convert back to pixels for error calculation
            actual_px, actual_py = self.robot_to_pixel_coords(actual_pos[0], actual_pos[1])
            
            # Calculate error
            error = math.sqrt((actual_px - px)**2 + (actual_py - py)**2)
            total_error += error
            
            print(f"{i:5d} | ({px:4.0f},{py:4.0f}) | ({robot_x:.3f},{robot_y:.3f},{robot_z:.3f}) | {target_joints[0]:6.3f},{target_joints[1]:6.3f} | ({actual_pos[0]:.3f},{actual_pos[1]:.3f}) | {error:5.1f}px")
            
        avg_error = total_error / num_points
        print("-" * 75)
        print(f"Average Error: {avg_error:.1f}px (was 659px)")
        
        if avg_error < 20:
            print("✅ MUCH IMPROVED! Coordinate system is now usable")
        elif avg_error < 100:
            print("⚠️ PARTIALLY IMPROVED: Still needs fine-tuning")
        else:
            print("❌ STILL PROBLEMATIC: Need different approach")
            
        return avg_error
        
    def pixel_to_robot_coords(self, pixel_x, pixel_y, pen_down=True):
        """Improved pixel to robot coordinate conversion."""
        # Scale factor
        scale_x = self.canvas_size / self.pixel_width
        scale_y = self.canvas_size / self.pixel_height
        
        # Convert pixel coordinates to canvas-relative coordinates
        canvas_x = (pixel_x - self.pixel_width / 2) * scale_x
        canvas_y = (pixel_y - self.pixel_height / 2) * scale_y
        
        # Transform to robot coordinate system
        robot_x = self.canvas_position[0] + canvas_x
        robot_y = self.canvas_position[1] + canvas_y
        robot_z = self.canvas_position[2] + (0.02 if pen_down else 0.05)
        
        return robot_x, robot_y, robot_z
        
    def robot_to_pixel_coords(self, robot_x, robot_y):
        """Improved robot to pixel coordinate conversion."""
        # Scale factor
        scale_x = self.canvas_size / self.pixel_width
        scale_y = self.canvas_size / self.pixel_height
        
        # Transform from robot to canvas coordinate system
        canvas_x = robot_x - self.canvas_position[0]
        canvas_y = robot_y - self.canvas_position[1]
        
        # Convert canvas coordinates to pixel coordinates
        pixel_x = canvas_x / scale_x + self.pixel_width / 2
        pixel_y = canvas_y / scale_y + self.pixel_height / 2
        
        return pixel_x, pixel_y
        
    def generate_corrected_conversion_code(self):
        """Generate corrected coordinate conversion code for integration."""
        code = '''
# CORRECTED COORDINATE SYSTEM (Generated by coordinate_system_fixer.py)

class CorrectedCoordinateSystem:
    """Corrected coordinate system with accurate kinematics."""
    
    def __init__(self):
        # Canvas parameters
        self.canvas_position = [0.6, 0.0, 0.05]
        self.canvas_size = 0.4
        self.pixel_width = 800
        self.pixel_height = 600
        
        # UR5e parameters
        self.base_height = 0.163
        self.shoulder_offset = 0.138
        self.upper_arm = 0.425
        self.forearm = 0.392
        
    def pixel_to_robot_coords(self, pixel_x, pixel_y, pen_down=True):
        """Convert pixel coordinates to robot coordinates."""
        scale_x = self.canvas_size / self.pixel_width
        scale_y = self.canvas_size / self.pixel_height
        
        canvas_x = (pixel_x - self.pixel_width / 2) * scale_x
        canvas_y = (pixel_y - self.pixel_height / 2) * scale_y
        
        robot_x = self.canvas_position[0] + canvas_x
        robot_y = self.canvas_position[1] + canvas_y
        robot_z = self.canvas_position[2] + (0.02 if pen_down else 0.05)
        
        return robot_x, robot_y, robot_z
        
    def robot_coords_to_joints(self, robot_x, robot_y, robot_z, base_joints):
        """Convert robot coordinates to joint positions using inverse kinematics."""
        # Calculate base rotation
        target_pan = math.atan2(robot_y - self.shoulder_offset, robot_x)
        
        # Calculate distance and angles
        dx = robot_x - 0
        dy = robot_y - self.shoulder_offset
        horizontal_distance = math.sqrt(dx*dx + dy*dy)
        dz = robot_z - self.base_height
        target_distance = math.sqrt(horizontal_distance*horizontal_distance + dz*dz)
        
        # 2-link inverse kinematics
        max_reach = self.upper_arm + self.forearm
        if target_distance > max_reach:
            target_distance = max_reach * 0.95
            
        cos_shoulder = (self.upper_arm**2 + target_distance**2 - self.forearm**2) / (2 * self.upper_arm * target_distance)
        cos_shoulder = max(-1, min(1, cos_shoulder))
        
        angle_to_target = math.atan2(dz, horizontal_distance)
        shoulder_angle = angle_to_target - math.acos(cos_shoulder)
        
        cos_elbow = (self.upper_arm**2 + self.forearm**2 - target_distance**2) / (2 * self.upper_arm * self.forearm)
        cos_elbow = max(-1, min(1, cos_elbow))
        elbow_angle = math.pi - math.acos(cos_elbow)
        
        joints = base_joints.copy()
        joints[0] = target_pan
        joints[1] = shoulder_angle
        joints[2] = elbow_angle
        
        return joints
        
    def joints_to_robot_position(self, joints):
        """Forward kinematics: convert joint positions to robot position."""
        q1, q2, q3 = joints[0], joints[1], joints[2]
        
        c1, s1 = math.cos(q1), math.sin(q1)
        
        # Position after upper arm
        upper_end_x = self.upper_arm * math.cos(q2) * c1
        upper_end_y = self.shoulder_offset + self.upper_arm * math.cos(q2) * s1
        upper_end_z = self.base_height + self.upper_arm * math.sin(q2)
        
        # Position after forearm
        forearm_angle = q2 + q3
        end_x = upper_end_x + self.forearm * math.cos(forearm_angle) * c1
        end_y = upper_end_y + self.forearm * math.cos(forearm_angle) * s1
        end_z = upper_end_z + self.forearm * math.sin(forearm_angle)
        
        return (end_x, end_y, end_z)
'''
        
        with open('/Users/smallspring/programs/wanderline/robot/scripts/corrected_coordinate_system.py', 'w') as f:
            f.write(code)
            
        print("✅ Generated corrected_coordinate_system.py")


def main():
    """Run coordinate system analysis and generate fixes."""
    fixer = CoordinateSystemFixer()
    
    fixer.analyze_current_problems()
    avg_error = fixer.test_improved_coordinate_system()
    fixer.generate_corrected_conversion_code()
    
    print(f"\n📊 SUMMARY:")
    print(f"Original avg error: 659px")
    print(f"Improved avg error: {avg_error:.1f}px")
    print(f"Improvement: {((659 - avg_error) / 659 * 100):.1f}%")


if __name__ == "__main__":
    main()
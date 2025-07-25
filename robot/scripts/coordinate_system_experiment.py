#!/usr/bin/env python3
"""
Coordinate System Experiment Script

This script performs systematic tests to understand the relationship between:
1. Mathematical coordinate system (atan2, cos, sin)
2. Robot joint space 
3. Robot Cartesian space
4. Canvas pixel coordinates

Goal: Establish consistent coordinate transformations for synchronized drawing.
"""

import math
import sys
import os
from pathlib import Path

# Add parent directory to path for imports
sys.path.append(str(Path(__file__).parent))
from canvas_coordinate_system import CanvasCoordinateSystem

class CoordinateSystemExperiment:
    """Systematic coordinate system testing and analysis."""
    
    def __init__(self):
        self.canvas_system = CanvasCoordinateSystem()
        self.results = []
        
        # UR5e Robot Parameters (approximate)
        self.shoulder_height = 0.163  # m
        self.upper_arm_length = 0.425  # m  
        self.forearm_length = 0.392    # m
        
    def test_mathematical_circle(self, center_pixel=(400, 300), radius_pixel=50, num_points=8):
        """Test mathematical circle generation in pixel coordinates."""
        print("=" * 60)
        print("TEST 1: Mathematical Circle in Pixel Coordinates")
        print("=" * 60)
        
        points = []
        for i in range(num_points + 1):
            angle = 2 * math.pi * i / num_points
            x = center_pixel[0] + radius_pixel * math.cos(angle)
            y = center_pixel[1] + radius_pixel * math.sin(angle)
            points.append((x, y, angle))
            
        print(f"Circle center: {center_pixel}, radius: {radius_pixel}px")
        print("Point | Pixel Coords | Angle (rad) | Angle (deg) | Direction")
        print("-" * 70)
        
        for i, (x, y, angle) in enumerate(points):
            angle_deg = math.degrees(angle)
            direction = self._angle_to_direction(angle_deg)
            print(f"{i:5d} | ({x:6.1f}, {y:6.1f}) | {angle:9.3f} | {angle_deg:9.1f}° | {direction}")
            
        return points
    
    def test_robot_coordinate_conversion(self, pixel_points):
        """Test conversion from pixel coordinates to robot coordinates."""
        print("\n" + "=" * 60)
        print("TEST 2: Pixel to Robot Coordinate Conversion")
        print("=" * 60)
        
        robot_points = []
        print("Point | Pixel Coords | Robot Coords (m) | Distance from Origin")
        print("-" * 75)
        
        for i, (px, py, angle) in enumerate(pixel_points):
            robot_x, robot_y, robot_z = self.canvas_system.pixel_to_robot_coords(px, py, pen_down=True)
            distance = math.sqrt(robot_x**2 + robot_y**2)
            robot_points.append((robot_x, robot_y, robot_z, angle))
            
            print(f"{i:5d} | ({px:6.1f}, {py:6.1f}) | ({robot_x:6.3f}, {robot_y:6.3f}, {robot_z:6.3f}) | {distance:6.3f}m")
            
        return robot_points
    
    def test_robot_joint_calculation(self, robot_points, base_joints=[0.0, -0.7, -1.0, -1.5, 1.57, 0.0]):
        """Test conversion from robot coordinates to joint angles."""
        print("\n" + "=" * 60)
        print("TEST 3: Robot Coordinate to Joint Angle Conversion")
        print("=" * 60)
        
        joint_points = []
        print("Point | Robot Coords | Pan Joint | Lift Joint | Estimated End Effector")
        print("-" * 85)
        
        # Canvas center for reference
        canvas_center_x, canvas_center_y = self.canvas_system.canvas_position[0], self.canvas_system.canvas_position[1]
        pan_amplitude = 0.6   # From existing system
        lift_amplitude = 0.3  # From existing system
        
        for i, (robot_x, robot_y, robot_z, orig_angle) in enumerate(robot_points):
            # Calculate joint offsets (from existing main.py logic)
            pan_offset = (robot_x - canvas_center_x) / 0.2 * pan_amplitude
            lift_offset = (robot_y - canvas_center_y) / 0.2 * lift_amplitude
            
            # Apply to base configuration
            joints = base_joints.copy()
            joints[0] = base_joints[0] + pan_offset   # shoulder_pan_joint
            joints[1] = base_joints[1] + lift_offset  # shoulder_lift_joint
            
            # Estimate actual end effector position using forward kinematics
            estimated_pos = self._forward_kinematics_approximation(joints)
            
            joint_points.append((joints, estimated_pos, orig_angle))
            
            print(f"{i:5d} | ({robot_x:6.3f}, {robot_y:6.3f}, {robot_z:6.3f}) | {joints[0]:9.3f} | {joints[1]:10.3f} | ({estimated_pos[0]:6.3f}, {estimated_pos[1]:6.3f}, {estimated_pos[2]:6.3f})")
            
        return joint_points
        
    def test_roundtrip_consistency(self, original_pixels, joint_points):
        """Test roundtrip consistency: pixel -> robot -> joints -> estimated position -> pixel."""
        print("\n" + "=" * 60)
        print("TEST 4: Roundtrip Consistency Analysis")
        print("=" * 60)
        
        print("Point | Original Pixel | Estimated Pixel | Error (px) | Error (%)")
        print("-" * 70)
        
        total_error = 0.0
        max_error = 0.0
        
        for i, ((orig_px, orig_py, _), (joints, est_pos, _)) in enumerate(zip(original_pixels, joint_points)):
            # Convert estimated robot position back to pixel coordinates
            est_pixel_x, est_pixel_y = self.canvas_system.robot_to_pixel_coords(est_pos[0], est_pos[1])
            
            # Calculate error
            error_px = math.sqrt((est_pixel_x - orig_px)**2 + (est_pixel_y - orig_py)**2)
            error_percent = (error_px / 50.0) * 100  # Assuming 50px radius
            
            total_error += error_px
            max_error = max(max_error, error_px)
            
            print(f"{i:5d} | ({orig_px:6.1f}, {orig_py:6.1f}) | ({est_pixel_x:6.1f}, {est_pixel_y:6.1f}) | {error_px:8.2f} | {error_percent:7.2f}%")
            
        avg_error = total_error / len(original_pixels)
        print("-" * 70)
        print(f"Average Error: {avg_error:.2f}px, Max Error: {max_error:.2f}px")
        
        return avg_error, max_error
        
    def test_rotation_direction_consistency(self, joint_points):
        """Test if joint space rotation matches mathematical rotation."""
        print("\n" + "=" * 60)
        print("TEST 5: Rotation Direction Consistency")
        print("=" * 60)
        
        print("Point | Math Angle | Pan Joint | Joint Delta | Direction Match")
        print("-" * 65)
        
        direction_matches = 0
        total_points = len(joint_points) - 1  # Compare adjacent points
        
        for i in range(len(joint_points) - 1):
            curr_joints, _, curr_angle = joint_points[i]
            next_joints, _, next_angle = joint_points[i + 1]
            
            # Mathematical angle change
            math_delta = next_angle - curr_angle
            if math_delta > math.pi:
                math_delta -= 2 * math.pi
            elif math_delta < -math.pi:
                math_delta += 2 * math.pi
                
            # Joint angle change
            joint_delta = next_joints[0] - curr_joints[0]
            
            # Check if directions match (same sign)
            direction_match = (math_delta * joint_delta >= 0)
            if direction_match:
                direction_matches += 1
                
            print(f"{i:5d} | {math.degrees(curr_angle):9.1f}° | {curr_joints[0]:9.3f} | {joint_delta:11.3f} | {'✓' if direction_match else '✗'}")
            
        consistency_percent = (direction_matches / total_points) * 100
        print("-" * 65)
        print(f"Direction Consistency: {direction_matches}/{total_points} ({consistency_percent:.1f}%)")
        
        return consistency_percent
        
    def _forward_kinematics_approximation(self, joints):
        """Simplified forward kinematics for UR5e."""
        pan_angle = joints[0]
        lift_angle = joints[1] 
        elbow_angle = joints[2]
        
        # Calculate end effector position
        reach = self.upper_arm_length * math.cos(lift_angle) + self.forearm_length * math.cos(lift_angle + elbow_angle)
        x = reach * math.cos(pan_angle)
        y = reach * math.sin(pan_angle)  
        z = self.shoulder_height + self.upper_arm_length * math.sin(lift_angle) + self.forearm_length * math.sin(lift_angle + elbow_angle)
        
        return (x, y, z)
        
    def _angle_to_direction(self, angle_deg):
        """Convert angle to compass direction for readability."""
        angle_deg = angle_deg % 360
        if angle_deg < 22.5 or angle_deg >= 337.5:
            return "East"
        elif angle_deg < 67.5:
            return "NE"
        elif angle_deg < 112.5:
            return "North"
        elif angle_deg < 157.5:
            return "NW"
        elif angle_deg < 202.5:
            return "West"
        elif angle_deg < 247.5:
            return "SW"
        elif angle_deg < 292.5:
            return "South"
        else:
            return "SE"
            
    def run_full_experiment(self):
        """Run complete coordinate system experiment."""
        print("🧪 COORDINATE SYSTEM EXPERIMENT")
        print("Analyzing transformations: Pixel → Robot → Joints → Position → Pixel")
        print()
        
        # Test 1: Generate mathematical circle
        pixel_points = self.test_mathematical_circle()
        
        # Test 2: Convert to robot coordinates
        robot_points = self.test_robot_coordinate_conversion(pixel_points)
        
        # Test 3: Convert to joint angles
        joint_points = self.test_robot_joint_calculation(robot_points)
        
        # Test 4: Check roundtrip consistency
        avg_error, max_error = self.test_roundtrip_consistency(pixel_points, joint_points)
        
        # Test 5: Verify rotation direction
        direction_consistency = self.test_rotation_direction_consistency(joint_points)
        
        # Final analysis
        print("\n" + "=" * 60)
        print("EXPERIMENT SUMMARY")
        print("=" * 60)
        print(f"Average Position Error: {avg_error:.2f} pixels")
        print(f"Maximum Position Error: {max_error:.2f} pixels")
        print(f"Rotation Direction Consistency: {direction_consistency:.1f}%")
        
        if avg_error < 5.0 and direction_consistency > 80.0:
            print("✅ COORDINATE SYSTEM: Good consistency")
        elif avg_error < 10.0 and direction_consistency > 60.0:
            print("⚠️  COORDINATE SYSTEM: Acceptable, needs minor fixes")
        else:
            print("❌ COORDINATE SYSTEM: Major inconsistencies detected")
            
        print("\nRecommendations:")
        if direction_consistency < 80.0:
            print("- Fix rotation direction: Consider reversing angle calculation")
        if avg_error > 5.0:
            print("- Improve coordinate conversion accuracy")
        if max_error > 20.0:
            print("- Review joint space mapping parameters")
            
        return {
            'avg_error': avg_error,
            'max_error': max_error, 
            'direction_consistency': direction_consistency,
            'pixel_points': pixel_points,
            'robot_points': robot_points,
            'joint_points': joint_points
        }


def main():
    """Run coordinate system experiment."""
    experiment = CoordinateSystemExperiment()
    results = experiment.run_full_experiment()
    
    # Save results for further analysis
    print(f"\n📊 Results available in experiment object for further analysis")
    return results


if __name__ == "__main__":
    main()
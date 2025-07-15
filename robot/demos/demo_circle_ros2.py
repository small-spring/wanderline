#!/usr/bin/env python3
"""
🤖 ROS2 Circle Drawing Demo 
Simple circle calculation with ROS2 integration
"""

import math
import sys

def check_ros2():
    """Check if ROS2 is available"""
    try:
        import rclpy
        from geometry_msgs.msg import Point
        print("✅ ROS2 packages found!")
        return True
    except ImportError as e:
        print(f"❌ ROS2 not available: {e}")
        print("💡 This will run in basic mode (just math)")
        return False

def draw_circle_ros2():
    """Draw circle with ROS2 integration"""
    
    print("🤖 ROS2 Circle Drawing Demo Starting...")
    
    # Check ROS2 availability
    has_ros2 = check_ros2()
    
    # Circle parameters
    center_x, center_y, center_z = 0.3, 0.0, 0.2
    radius = 0.1
    points = 12
    
    print(f"📍 Circle center: ({center_x}, {center_y}, {center_z})")
    print(f"📏 Circle radius: {radius}m")
    print(f"🎯 Number of points: {points}")
    print()
    
    # Generate waypoints
    waypoints = []
    
    if has_ros2:
        # Use ROS2 Point messages
        from geometry_msgs.msg import Point
        print("🔧 Using ROS2 Point messages...")
        
        for i in range(points + 1):
            angle = 2 * math.pi * i / points
            
            # Create ROS2 Point message
            point = Point()
            point.x = center_x + radius * math.cos(angle)
            point.y = center_y + radius * math.sin(angle) 
            point.z = center_z
            
            waypoints.append(point)
            print(f"Point {i:2d}: x={point.x:.3f}, y={point.y:.3f}, z={point.z:.3f}")
    else:
        # Fallback to simple tuples
        print("🔧 Using simple coordinates...")
        
        for i in range(points + 1):
            angle = 2 * math.pi * i / points
            
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            z = center_z
            
            waypoints.append((x, y, z))
            print(f"Point {i:2d}: x={x:.3f}, y={y:.3f}, z={z:.3f}")
    
    print()
    print(f"✅ Generated {len(waypoints)} waypoints!")
    
    if has_ros2:
        print("🎉 ROS2 integration ready!")
        print("📝 Next: Create ROS2 node for robot control")
    else:
        print("🎉 Basic circle calculation complete!")
        print("📝 Next: Install ROS2 packages for robot integration")
    
    return waypoints

if __name__ == '__main__':
    draw_circle_ros2()
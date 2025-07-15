#!/usr/bin/env python3
"""
🤖 SIMPLE Circle Drawing Demo 
Just calculates circle points - no complex robot stuff yet!
"""

import math

def draw_circle_simple():
    """Calculate points for drawing a circle - super simple!"""
    
    print("🎯 Circle Drawing Demo Starting...")
    
    # Simple circle settings
    center_x = 0.3  # 30cm from robot base
    center_y = 0.0  # centered 
    center_z = 0.2  # 20cm height
    radius = 0.1    # 10cm circle
    points = 12     # 12 points (30° each)
    
    print(f"📍 Circle center: ({center_x}, {center_y}, {center_z})")
    print(f"📏 Circle radius: {radius}m")
    print(f"🎯 Number of points: {points}")
    print()
    
    # Calculate each point on the circle
    waypoints = []
    for i in range(points + 1):  # +1 to close the circle
        angle = 2 * math.pi * i / points
        
        # Math: x = center_x + radius * cos(angle)
        x = center_x + radius * math.cos(angle)
        y = center_y + radius * math.sin(angle)
        z = center_z  # same height
        
        waypoints.append((x, y, z))
        print(f"Point {i:2d}: x={x:.3f}, y={y:.3f}, z={z:.3f}")
    
    print()
    print(f"✅ Generated {len(waypoints)} waypoints!")
    print("🎉 Circle calculation complete!")
    print()
    print("Next steps:")
    print("1. Test this simple version first")
    print("2. Add ROS2 robot control")
    print("3. Add Gazebo simulation")
    
    return waypoints

if __name__ == '__main__':
    draw_circle_simple()
#!/usr/bin/env python3
"""
🧪 Simple tests for circle drawing demo
Test the math before we add robot complexity!
"""

import math
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'demos'))
from demo_circle import draw_circle_simple

def test_circle_math():
    """Test that our circle math is correct"""
    print("🧪 Testing circle math...")
    
    # Run the demo and get waypoints
    import io
    import sys
    from contextlib import redirect_stdout
    
    # Capture output so tests don't clutter terminal
    f = io.StringIO()
    with redirect_stdout(f):
        waypoints = draw_circle_simple()
    
    # Test 1: Check we have the right number of points
    assert len(waypoints) == 13, f"Expected 13 points, got {len(waypoints)}"
    print("✅ Correct number of waypoints")
    
    # Test 2: First and last point should be the same (closed circle)
    first_point = waypoints[0]
    last_point = waypoints[-1]
    distance = math.sqrt(
        (first_point[0] - last_point[0])**2 + 
        (first_point[1] - last_point[1])**2 + 
        (first_point[2] - last_point[2])**2
    )
    assert distance < 0.001, f"Circle not closed! Distance: {distance}"
    print("✅ Circle is properly closed")
    
    # Test 3: All points should be same distance from center
    center = (0.3, 0.0, 0.2)
    radius = 0.1
    
    for i, point in enumerate(waypoints[:-1]):  # Skip last (duplicate) point
        distance_from_center = math.sqrt(
            (point[0] - center[0])**2 + 
            (point[1] - center[1])**2
        )
        assert abs(distance_from_center - radius) < 0.001, \
            f"Point {i} wrong distance from center: {distance_from_center}"
    
    print("✅ All points are correct distance from center")
    
    # Test 4: All z coordinates should be the same
    z_values = [point[2] for point in waypoints]
    assert all(z == 0.2 for z in z_values), "Z coordinates not consistent"
    print("✅ All points at same height")
    
    print("🎉 All tests passed!")

def test_circle_visualization():
    """Visual test - print circle as ASCII art"""
    print("\n🎨 ASCII Circle Visualization:")
    
    # Get waypoints
    import io
    import sys
    from contextlib import redirect_stdout
    
    f = io.StringIO()
    with redirect_stdout(f):
        waypoints = draw_circle_simple()
    
    # Simple ASCII plot
    print("   Y")
    print("   ^")
    print("   |")
    
    # Scale points for ASCII display
    for i, (x, y, z) in enumerate(waypoints[:-1]):
        # Convert to display coordinates
        display_x = int((x - 0.2) * 50)  # Scale and center
        display_y = int(y * 50)
        
        # Simple representation
        print(f"Point {i:2d}: {'*' if i == 0 else 'o'} x={x:.2f}, y={y:.2f}")
    
    print("   |")
    print("   +----> X")
    print("✅ Visual check complete")

if __name__ == '__main__':
    print("🚀 Running circle drawing tests...")
    print("=" * 40)
    
    try:
        test_circle_math()
        test_circle_visualization()
        print("\n" + "=" * 40)
        print("🎉 ALL TESTS PASSED! 🎉")
        print("✅ Ready for robot integration!")
        
    except AssertionError as e:
        print(f"\n❌ TEST FAILED: {e}")
        exit(1)
    except Exception as e:
        print(f"\n💥 UNEXPECTED ERROR: {e}")
        exit(1)
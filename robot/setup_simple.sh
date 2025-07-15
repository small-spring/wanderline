#!/bin/bash
# Simple setup script for robot simulation - Basic version

echo "🤖 Setting up Robot Simulation Environment (Basic Mode)..."

# Check if we're in the robot directory
if [ ! -f "demo_circle.py" ]; then
    echo "❌ Please run this script from the robot directory"
    exit 1
fi

echo "✅ Robot directory confirmed"

# Run basic demo
echo "🔨 Running basic circle demo..."
python3 demo_circle.py

echo ""
echo "🎯 Basic demo completed! Next steps:"
echo "1. Review the circle waypoints above"
echo "2. Try: python3 demo_circle_ros2.py"
echo "3. For full ROS2 environment, use existing pai_ros2 container:"
echo "   docker exec -it pai_ros2 bash"
echo ""
echo "📝 Note: Full ROS2 container build needs package fixes"
echo "   Current focus: Learn circle generation first!"

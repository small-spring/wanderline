#!/bin/bash
# 🎨 Automated Circle Drawing Demo 
# Uses standard ROS2 patterns for clean launch

set -e

echo "🎨 Automated Circle Drawing Demo"
echo "==================================="
echo ""

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Complete cleanup
echo "🧹 Complete system cleanup..."
pkill -f robot_state_publisher || true
pkill -f joint_state_publisher || true  
pkill -f rviz2 || true
pkill -f ros2 || true
sleep 3

# Reset ROS2 daemon if needed
echo "🔄 Checking ROS2 daemon..."
ros2 daemon stop 2>/dev/null || true
sleep 2
ros2 daemon start

echo "🚀 Starting robot with standard ROS2 pattern..."
# Use our standard launch file with proper arguments
ros2 launch /workspace/robot/launch/ur5e_standard.launch.py \
    ur_type:=ur5e \
    jsp_gui:=false \
    use_rviz:=true &
LAUNCH_PID=$!

# Wait for robot to start
echo "⏳ Waiting for robot to initialize..."
sleep 8

# Verify clean state
echo "🔍 Verifying system state..."
NODE_COUNT=$(ros2 node list 2>/dev/null | wc -l || echo "0")
echo "📊 Active ROS2 nodes: $NODE_COUNT"

if [ "$NODE_COUNT" -lt "2" ]; then
    echo "⚠️  Warning: Insufficient nodes detected. Retrying..."
    sleep 3
fi

# Start circle drawing
echo "🎨 Starting circle drawing..."
python3 /workspace/robot/demos/robot_draw_circle.py &
CIRCLE_PID=$!

# Wait for circle drawing to start
sleep 2

echo "✅ Demo is running!"
echo ""
echo "📋 What you should see:"
echo "• RViz with UR5e robot model"
echo "• Robot drawing smooth circles"
echo "• No GUI sliders (clean control)"
echo ""
echo "🛑 To stop: Ctrl+C"

# Cleanup function
cleanup() {
    echo ""
    echo "🛑 Stopping demo..."
    kill $CIRCLE_PID 2>/dev/null || true
    kill $LAUNCH_PID 2>/dev/null || true
    pkill -f robot_draw_circle || true
    sleep 2
    echo "✅ Demo stopped cleanly"
    exit 0
}

trap cleanup SIGINT SIGTERM

# Wait for user interrupt
wait
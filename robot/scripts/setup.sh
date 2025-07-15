#!/bin/bash
# 🤖 Wanderline Robot - Unified Setup Script
# One script to rule them all!

set -e

print_header() {
    echo ""
    echo "🤖 Wanderline Robot Setup"
    echo "========================="
    echo "$1"
    echo ""
}

print_step() {
    echo "▶️  $1"
}

print_success() {
    echo "✅ $1"
}

# Check if running inside Docker container
if [ -f /.dockerenv ]; then
    # Inside container - install packages
    print_header "Installing packages inside container..."
    
    print_step "Updating package list..."
    apt-get update -qq
    
    print_step "Installing Gazebo and ROS2 packages..."
    apt-get install -y \
        gazebo \
        ros-humble-gazebo-ros-pkgs \
        ros-humble-gazebo-ros2-control \
        > /dev/null 2>&1
    
    print_step "Setting up ROS2 environment..."
    source /opt/ros/humble/setup.bash
    
    print_step "Testing basic functionality..."
    cd /workspace/robot/demos
    python3 demo_circle.py > /dev/null
    
    print_success "Container setup complete!"
    echo ""
    echo "🌐 Access VNC GUI: http://localhost:6081"
    echo "📋 In VNC terminal, run:"
    echo "   source /opt/ros/humble/setup.bash"
    echo "   ros2 launch ur_description view_ur.launch.py ur_type:=ur5e use_fake_hardware:=true launch_rviz:=true"
    
else
    # On host - manage Docker container
    print_header "Setting up Docker environment..."
    
    # Go to robot directory
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    cd "$SCRIPT_DIR/.."
    
    print_step "Stopping existing containers..."
    docker-compose -f docker/docker-compose.robot.yml down > /dev/null 2>&1 || true
    
    print_step "Building and starting container..."
    docker-compose -f docker/docker-compose.robot.yml up -d --build
    
    print_step "Waiting for container to start..."
    sleep 8
    
    print_step "Installing packages inside container..."
    docker exec docker-wanderline-robot-1 bash /workspace/robot/scripts/setup.sh
    
    print_success "Setup complete!"
    echo ""
    echo "🎉 Ready to use!"
    echo ""
    echo "📖 Quick Start:"
    echo "1. Open http://localhost:6081 in browser"
    echo "2. Click 'Connect'"
    echo "3. Right-click desktop → 'Open Terminal'"
    echo "4. Copy-paste:"
    echo "   source /opt/ros/humble/setup.bash"
    echo "   ros2 launch ur_description view_ur.launch.py ur_type:=ur5e use_fake_hardware:=true launch_rviz:=true"
    echo ""
    echo "🧪 Test demos:"
    echo "   python3 /workspace/robot/demos/demo_circle.py"
    echo "   python3 /workspace/robot/tests/test_circle.py"
fi
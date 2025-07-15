#!/bin/bash
# 🤖 Wanderline Robot - Unified Setup Script
#
# Usage: Run this script on your host machine.
# It will automatically start a Docker container.
# Then it executes itself inside the container.
# This installs all necessary ROS2 and Gazebo packages.
# Finally, you get a complete robot development environment.
# Access the GUI via VNC at http://localhost:6081


# Stop on errors
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
    # Try to install gazebo packages, but continue if they fail
    if apt-get install -y gazebo ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control > /dev/null 2>&1; then
        echo "✅ Gazebo packages installed successfully"
    else
        echo "⚠️  Gazebo packages not available - continuing without them"
    fi
    
    print_step "Setting up ROS2 environment..."
    source /opt/ros/humble/setup.bash
    
    print_success "Container setup complete!"
    echo ""
    echo "🌐 Access VNC GUI: http://localhost:6081"
    echo "📋 In VNC terminal, run:"
    echo "   # Option 1: Quick demo launcher"
    echo "   bash /workspace/robot/scripts/demo_launcher.sh"
    echo "   # Option 2: Manual ROS2 setup"
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
    
    print_success "Setup complete!"robot/scripts/demo_launcher.sh
    echo ""
    echo "🎉 Ready to use!"
    echo ""
    echo "📖 Quick Start:"
    echo "1. Open http://localhost:6081 in browser"
    echo "2. Click 'Connect'"
    echo "3. Right-click desktop → 'Open Terminal'"
    echo "4. Copy-paste:"
    echo "   # Option 1: Quick demo launcher"
    echo "   bash /workspace/robot/scripts/demo_launcher.sh"
    echo "   # Option 2: Manual ROS2 setup"
    echo "   source /opt/ros/humble/setup.bash"
    echo "   ros2 launch ur_description view_ur.launch.py ur_type:=ur5e use_fake_hardware:=true launch_rviz:=true"
    echo ""

fi
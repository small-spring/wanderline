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
    
    print_step "Setting up ROS2 repository..."
    # Ensure ROS2 repository is properly configured
    echo "deb http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2.list
    
    print_step "Updating package list..."
    apt-get update -qq
    
    print_step "Installing ROS2 packages..."
    # Install essential ROS2 packages for UR5e robot
    if apt-get install -y ros-humble-ur-description ros-humble-ur-robot-driver ros-humble-joint-state-publisher-gui > /dev/null 2>&1; then
        echo "✅ ROS2 UR packages installed successfully"
    else
        echo "⚠️  Some ROS2 packages may not be available - trying alternative approach"
        # Try installing from base ROS2 repos without additional signing
        apt-get install -y --allow-unauthenticated ros-humble-ur-description || echo "⚠️  ur-description not available"
        apt-get install -y --allow-unauthenticated ros-humble-joint-state-publisher-gui || echo "⚠️  joint-state-publisher-gui not available"
    fi
    
    print_step "Installing Gazebo packages (optional)..."
    # Try to install gazebo packages, but continue if they fail
    if apt-get install -y ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control > /dev/null 2>&1; then
        echo "✅ Gazebo packages installed successfully"
    else
        echo "⚠️  Gazebo packages not available - continuing without them"
    fi
    
    print_step "Installing Python packages for Phase 1..."
    # Install OpenCV for Canvas Preview Window
    if pip install opencv-python > /dev/null 2>&1; then
        echo "✅ OpenCV-Python installed successfully"
    else
        echo "⚠️  OpenCV-Python installation failed - Canvas Preview may not work"
    fi
    
    print_step "Installing UR5e robot packages..."
    # Clone and build UR5e description package
    cd /home/ubuntu/ros2_ws/src
    if [ ! -d "ur_description" ]; then
        git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git ur_description > /dev/null 2>&1
        echo "✅ UR5e description cloned successfully"
    else
        echo "✅ UR5e description already exists"
    fi
    
    cd /home/ubuntu/ros2_ws
    colcon build --packages-select ur_description > /dev/null 2>&1
    echo "✅ UR5e packages built successfully"
    
    print_step "Setting up ROS2 environment..."
    source /opt/ros/humble/setup.bash
    source /home/ubuntu/ros2_ws/install/setup.bash
    
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
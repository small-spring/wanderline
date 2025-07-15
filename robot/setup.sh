#!/bin/bash
# Simple setup script for robot simulation

echo "🤖 Setting up Robot Simulation Environment..."

# Check if Docker is installed
if ! command -v docker &> /dev/null; then
    echo "❌ Docker is not installed. Please install Docker first."
    exit 1
fi

echo "✅ Docker found"

# Build the robot container
echo "🔨 Building robot simulation container..."
# Use correct working directory
docker-compose -f docker-compose.robot.yml build

echo "🚀 Starting robot container..."
docker-compose -f docker-compose.robot.yml up -d

echo "✅ Robot simulation environment ready!"
echo ""
echo "Next steps:"
echo "1. docker-compose -f docker-compose.robot.yml exec wanderline-robot bash"
echo "2. cd robot && python3 demo_circle.py"
echo "3. Check the circle waypoints in the output"
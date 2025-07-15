#!/bin/bash
# VNC Environment Test Script for Phase 1

echo "🚀 Phase 1 VNC Environment Test"
echo "==============================="

# Check if we're in VNC environment
if [ -n "$DISPLAY" ]; then
    echo "✅ Display found: $DISPLAY"
else
    echo "⚠️  No display detected - running in headless mode"
fi

# Check if ROS2 is available
if command -v ros2 &> /dev/null; then
    echo "✅ ROS2 found"
    ROS_AVAILABLE=true
else
    echo "⚠️  ROS2 not found - running basic tests only"
    ROS_AVAILABLE=false
fi

# Test 1: Basic functionality test
echo ""
echo "🧪 Test 1: Basic Phase 1 functionality"
echo "-------------------------------------"
# Support both local and VNC environments
if [ -d "/workspace/robot/demos/phase1" ]; then
    cd /workspace/robot/demos/phase1  # VNC environment
else
    cd /Users/smallspring/programs/wanderline/robot/demos/phase1  # Local environment
fi
uv run python test_vnc.py

if [ $? -eq 0 ]; then
    echo "✅ Basic functionality test passed"
else
    echo "❌ Basic functionality test failed"
    exit 1
fi

# Test 2: ROS2 integration test (if available)
if [ "$ROS_AVAILABLE" = true ]; then
    echo ""
    echo "🤖 Test 2: ROS2 integration test"
    echo "------------------------------"
    
    # Check if robot simulation is running
    if ros2 node list | grep -q "robot_state_publisher"; then
        echo "✅ Robot simulation detected"
        
        # Test Phase 1 main.py import
        echo "Testing Phase 1 main.py imports..."
        # Support both local and VNC environments
        if [ -d "/workspace/robot/demos/phase1" ]; then
            cd /workspace/robot/demos/phase1  # VNC environment
        else
            cd /Users/smallspring/programs/wanderline/robot/demos/phase1  # Local environment
        fi
        python3 -c "
import sys
import os
# Support both local and VNC environments
if os.path.exists('/workspace/robot/scripts'):
    sys.path.append('/workspace/robot/scripts')  # VNC environment
else:
    sys.path.append('/Users/smallspring/programs/wanderline/robot/scripts')  # Local environment
try:
    from main import Phase1RobotDrawer
    print('✅ Phase1RobotDrawer import successful')
except ImportError as e:
    print(f'❌ Import failed: {e}')
    sys.exit(1)
"
        
        if [ $? -eq 0 ]; then
            echo "✅ ROS2 integration test passed"
        else
            echo "❌ ROS2 integration test failed"
            exit 1
        fi
    else
        echo "⚠️  Robot simulation not running - skipping ROS2 test"
        echo "💡 To run full test:"
        echo "   1. Start robot simulation: ros2 launch robot ur5e_standard.launch.py"
        echo "   2. Run Phase 1 demo: ros2 run robot main.py"
    fi
else
    echo "⚠️  ROS2 not available - skipping ROS2 integration test"
fi

# Test 3: Launch file validation
echo ""
echo "🚀 Test 3: Launch file validation"
echo "-------------------------------"

# Support both local and VNC environments
if [ -f "/workspace/robot/launch/phase1_demo.launch.py" ]; then
    launch_file="/workspace/robot/launch/phase1_demo.launch.py"  # VNC environment
else
    launch_file="/Users/smallspring/programs/wanderline/robot/launch/phase1_demo.launch.py"  # Local environment
fi
if [ -f "$launch_file" ]; then
    echo "✅ Launch file exists: $launch_file"
    
    # Basic syntax check
    python3 -m py_compile "$launch_file"
    if [ $? -eq 0 ]; then
        echo "✅ Launch file syntax is valid"
    else
        echo "❌ Launch file has syntax errors"
        exit 1
    fi
else
    echo "❌ Launch file not found: $launch_file"
    exit 1
fi

# Test 4: Config file validation
echo ""
echo "⚙️ Test 4: Configuration validation"
echo "--------------------------------"

# Support both local and VNC environments
if [ -f "/workspace/robot/demos/phase1/config.yaml" ]; then
    config_file="/workspace/robot/demos/phase1/config.yaml"  # VNC environment
else
    config_file="/Users/smallspring/programs/wanderline/robot/demos/phase1/config.yaml"  # Local environment
fi
if [ -f "$config_file" ]; then
    echo "✅ Config file exists: $config_file"
    
    # Test YAML parsing
    python3 -c "
import yaml
with open('$config_file', 'r') as f:
    config = yaml.safe_load(f)
print('✅ Config file parsing successful')
print(f'📊 Robot update rate: {config[\"phase1\"][\"robot\"][\"update_rate\"]}Hz')
print(f'🎯 Drawing segments: {config[\"phase1\"][\"drawing\"][\"segments\"]}')
"
    
    if [ $? -eq 0 ]; then
        echo "✅ Configuration validation passed"
    else
        echo "❌ Configuration validation failed"
        exit 1
    fi
else
    echo "❌ Config file not found: $config_file"
    exit 1
fi

# Summary
echo ""
echo "==============================="
echo "✅ All VNC tests completed successfully!"
echo ""
echo "🎯 Phase 1 is ready for VNC environment"
echo ""
echo "💡 To run Phase 1 demo:"
if [ "$ROS_AVAILABLE" = true ]; then
    echo "   ROS2 method:"
    echo "   1. ros2 launch robot ur5e_standard.launch.py"
    echo "   2. ros2 launch robot phase1_demo.launch.py"
    echo ""
fi
echo "   Direct method:"
echo "   1. cd /workspace/robot/demos/phase1  # VNC environment"
echo "   2. uv run python main.py"
echo ""
echo "🔍 For basic testing without ROS2:"
echo "   uv run python test_vnc.py"
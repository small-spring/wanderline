# 🤖 Robot Simulation for Wanderline

Transform Wanderline's optimized drawing strokes into physical robot movements using ROS2 and Gazebo simulation.

## 🎯 Project Overview

This module extends Wanderline's single-stroke drawing optimization to control a simulated UR5e robot arm, bridging the gap between algorithmic drawing and physical robotics.

**Pipeline**: `Wanderline Angles → 3D Waypoints → Robot Trajectories → Gazebo Simulation`

## 🚀 Quick Start

### 1. セットアップ実行
```bash
cd robot
./scripts/setup.sh
```

### 2. VNC GUI開く  
ブラウザで http://localhost:6081 にアクセス → 「Connect」をクリック

### 3. 円描画デモ実行（推奨）
VNCデスクトップでターミナルを開いて：
```bash
cd /workspace/robot
./scripts/auto_circle_demo.sh
```

## 🎯 Available Demos

### 🎨 Demo 1: Automatic Circle Drawing（推奨）

**Perfect for**: Programmatic control, smooth motion, drawing applications

**Quick Start**:
```bash
# Automated version (recommended)
./scripts/auto_circle_demo.sh

# Manual control version
ros2 launch /workspace/robot/launch/ur5e_standard.launch.py jsp_gui:=false use_rviz:=true
python3 /workspace/robot/demos/robot_draw_circle.py
```

**Features**:
- ✅ Automatic smooth circle drawing
- ✅ Coordinated multi-joint motion  
- ✅ 50Hz smooth interpolation
- ✅ No GUI control conflicts

### 🎮 Demo 2: Interactive GUI Control

**Perfect for**: Learning robot structure, manual exploration, joint testing

**Quick Start**:
```bash
# Standard UR5e with GUI sliders
source /opt/ros/humble/setup.bash
ros2 launch ur_description view_ur.launch.py ur_type:=ur5e use_fake_hardware:=true launch_rviz:=true

# Or use the demo launcher script
./scripts/demo_launcher.sh
```

**Features**:
- ✅ UR5e robot model in RViz
- ✅ Interactive sliders for each joint
- ✅ Manual control of robot pose
- ✅ Real-time visual feedback

## 📁 Project Structure

```
robot/
├── 🎮 demos/                   # Demo applications
│   ├── robot_draw_circle.py        # Circle drawing logic
│   └── simple_joint_move.py        # Basic movement test
├── 🚀 scripts/                 # Automation scripts
│   ├── auto_circle_demo.sh      # Main automated demo
│   ├── demo_launcher.sh            # Interactive demo chooser
│   └── setup.sh                    # Environment setup
├── 🔧 launch/                  # ROS2 launch files
│   └── ur5e_standard.launch.py     # Standard ROS2 pattern
├── 🐳 docker/                  # Container definitions
│   ├── Dockerfile.robot            # Main environment
│   └── docker-compose.robot.yml    # Service orchestration
├── 🧪 tests/                   # Validation tests
├── 📚 docs/                    # Documentation
└── 📦 models/                  # 3D models and SDF files
```

## 🔧 Technical Features

- **Dockerized Environment**: Complete ROS2 + Gazebo + Python stack
- **Modular Design**: Clear separation of concerns
- **Test-Driven**: Comprehensive validation suite
- **GUI Support**: Web VNC and Jupyter interfaces
- **Integration Ready**: Designed for Wanderline compatibility

## 🤝 Integration with Wanderline

```
Wanderline Core                Robot Module
┌─────────────────┐          ┌─────────────────┐
│ Angle Optimizer │ ──────→  │ Waypoint Gen    │
│ Stroke Planner  │          │ Trajectory Plan │
│ Canvas Manager  │          │ Robot Control   │
└─────────────────┘          └─────────────────┘
        ↓                            ↓
  Drawing Video              Gazebo Simulation
```

## 📖 Documentation

- **[Quick Start](QUICKSTART.md)** - 3-step setup guide
- **[Setup Guide](docs/SETUP.md)** - Detailed installation & VNC GUI guide
- **[Development Notes](docs/memo.md)** - Technical implementation details
- **[Project Root](../README.md)** - Main Wanderline documentation

## 🛠️ Troubleshooting

### Common Issues

**Model Error on Startup**:
- Normal behavior for 10 seconds while robot_description loads
- Resolves automatically when circle drawing starts

**Multiple RViz Windows**:
- Use `auto_circle_demo.sh` which prevents this issue
- Based on ROS2 standard patterns

**VNC Connection Issues**:
```bash
docker-compose -f docker/docker-compose.robot.yml restart
```

**Complete Reset**:
```bash
docker-compose -f docker/docker-compose.robot.yml down
docker system prune -f
./scripts/setup.sh
```

Perfect for bridging algorithmic optimization with physical robotics! 🚀
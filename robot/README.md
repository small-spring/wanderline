# 🤖 Robot Drawing System for Wanderline

Transform Wanderline's optimized drawing strokes into physical robot movements using a simulated UR5e robot arm.

## 🎯 Project Overview

This system bridges algorithmic drawing and physical robotics, featuring:
- **Circle drawing** with contact-based progress tracking
- **Real-time visualization** in RViz + Canvas Preview
- **Tool flange integration** with realistic pen simulation  
- **0.0px coordinate accuracy** (verified through testing)

**Pipeline**: `Circle Algorithm → Robot Coordinates → Joint Movements → RViz Visualization`

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

## 📚 Documentation

- **[QUICKSTART.md](QUICKSTART.md)** - Complete setup guide (Docker + VNC)
- **[docs/system_overview.md](docs/system_overview.md)** - System architecture & specifications
- **[docs/simulation_design.md](docs/simulation_design.md)** - Core simulation principles
- **[docs/UR5e_specifications.md](docs/UR5e_specifications.md)** - Robot hardware specifications
- **[docs/coordinate_system_reference.md](docs/coordinate_system_reference.md)** - Coordinate transformation math
- **[docs/development_log.md](docs/development_log.md)** - Technical development history

## 🎯 Available Demos

### 🎨 Demo 1: Canvas Drawing Visualization（推奨）

**Perfect for**: Complete drawing experience, real-time visualization, API access

**Quick Start**:
```bash
# Complete canvas drawing system
./scripts/canvas_demo.sh
```

**Features**:
- ✅ RViz canvas visualization with 3D surface
- ✅ Real-time pen position tracking (red=down, blue=up)
- ✅ Drawing trail visualization
- ✅ Multiple drawing patterns (circle, square, spiral, figure-8)
- ✅ Canvas state monitoring and API access
- ✅ Coordinate conversion (pixel ↔ robot coordinates)

### 🎨 Demo 2: Automatic Circle Drawing

**Perfect for**: Basic robot movement, learning joint control

**Quick Start**:
```bash
# Automated version
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

### 🎮 Demo 3: Interactive GUI Control

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
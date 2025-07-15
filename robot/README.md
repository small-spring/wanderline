# 🤖 Robot Simulation for Wanderline

Transform Wanderline's optimized drawing strokes into physical robot movements using ROS2 and Gazebo simulation.

## 🎯 Project Overview

This module extends Wanderline's single-stroke drawing optimization to control a simulated UR5e robot arm, bridging the gap between algorithmic drawing and physical robotics.

**Pipeline**: `Wanderline Angles → 3D Waypoints → Robot Trajectories → Gazebo Simulation`

## 🚀 Quick Start

**たった3ステップで動く！**

### 1. セットアップ実行
```bash
cd robot
./scripts/setup.sh
```

### 2. VNC GUI開く  
ブラウザで http://localhost:6081 にアクセス → 「Connect」をクリック

### 3. ロボット表示
VNCデスクトップでターミナルを開いて：
```bash
source /opt/ros/humble/setup.bash
ros2 launch ur_description view_ur.launch.py ur_type:=ur5e use_fake_hardware:=true launch_rviz:=true
```

**📖 詳細手順**: [QUICKSTART.md](QUICKSTART.md)

## 📁 Organized Structure

```
robot/
├── 📦 docker/              # Container definitions
│   ├── Dockerfile.robot    # Main environment
│   └── docker-compose.robot.yml
├── 🔧 scripts/             # Setup utilities
│   └── setup.sh           # Unified setup script
├── 🎯 demos/               # Example applications
│   ├── demo_circle.py     # Basic circle math
│   ├── demo_circle_ros2.py # ROS2 integration
│   └── move_robot_circle.py # Robot movement
├── 🧪 tests/               # Validation tests
│   ├── test_circle.py     # Circle math tests
│   ├── simple_joint_test.py # Joint control
│   └── auto_stop_joint_test.py # Advanced tests
├── 📚 docs/                # Documentation
│   ├── SETUP.md          # Detailed setup guide
│   └── memo.md           # Development notes
├── QUICKSTART.md         # 3-step quick start
└── README.md             # This file
```

## 🎓 Learning Path

### Phase 1: Foundation ✅
- [x] Docker environment with ROS2 + Gazebo
- [x] Basic circle waypoint calculation
- [x] Test framework validation

### Phase 2: Robot Integration ✅
- [x] UR5e robot model loading
- [ ] MoveIt2 trajectory planning
- [ ] Gazebo simulation testing

### Phase 3: Wanderline Integration 📋
- [ ] Angle-to-waypoint conversion
- [ ] Stroke sequence processing
- [ ] End-to-end drawing demo

## 🔧 Key Features

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

## 🎯 Example Output

```
🎯 Circle Drawing Demo Starting...
📍 Circle center: (0.3, 0.0, 0.2)
📏 Circle radius: 0.1m
🎯 Number of points: 12

Point  0: x=0.400, y=0.000, z=0.200
Point  1: x=0.387, y=0.050, z=0.200
...
✅ Generated 13 waypoints!
🎉 Circle calculation complete!
```

Perfect for bridging algorithmic optimization with physical robotics! 🚀
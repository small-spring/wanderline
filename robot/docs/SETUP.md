# 🤖 Robot Simulation Environment Setup

Complete guide for setting up the Wanderline robot simulation environment with Docker, ROS2, and Gazebo.

## 📋 Prerequisites

- **Docker** and **Docker Compose** installed
- **Git** for version control
- **X11 forwarding** (Linux/macOS) or **VNC** support for GUI applications

## 🚀 Quick Start

### 1. Build Environment
```bash
# From the robot directory
cd robot
./scripts/setup.sh
```

### 2. Run Basic Demo
```bash
# Test circle calculation
docker exec robot-wanderline-robot-1 python3 /workspace/robot/demos/demo_circle.py

# Run tests
docker exec robot-wanderline-robot-1 python3 /workspace/robot/tests/test_circle.py
```

### 3. Access GUI Applications

#### VNC Web Interface (推奨)
1. ブラウザで http://localhost:6081 を開く
2. 「Connect」ボタンをクリック
3. Ubuntu デスクトップが表示される

#### その他のアクセス方法
- **VNC Client**: localhost:5902 (VNCクライアントソフト用)
- **X11 Forward**: `export DISPLAY=:0` (Linux/macOS)

## 🏗️ Environment Architecture

### Docker Stack
```
Base Image: tiryoh/ros2-desktop-vnc:humble
├── ROS2 Humble (LTS)
├── Gazebo Classic
├── Python 3.12 + uv
├── Jupyter Notebook
└── Universal Robots packages
```

### Container Configuration
- **Name**: `docker-wanderline-robot-1`
- **Ports**: 
  - 6081 (VNC Web Interface)
  - 5902 (VNC Client Port)
  - 8888 (Jupyter)
  - 8000 (Development)
- **Volumes**:
  - Project: `/workspace`
  - Python cache: `/root/.cache/uv`
  - ROS workspace: `/workspace/robot_ws`

## 📁 Directory Structure

```
robot/
├── docker/
│   ├── Dockerfile.robot       # Main container definition
│   └── docker-compose.robot.yml  # Service orchestration
├── scripts/
│   ├── setup.sh              # Automated setup script
│   └── setup_simple.sh       # Minimal setup
├── demos/
│   ├── demo_circle.py         # Basic circle calculation
│   ├── demo_circle_ros2.py    # ROS2 integration demo
│   └── move_robot_circle.py   # Robot movement demo
├── tests/
│   ├── test_circle.py         # Circle math validation
│   ├── simple_joint_test.py   # Joint control tests
│   └── auto_stop_joint_test.py # Advanced joint tests
├── docs/
│   ├── SETUP.md              # This file
│   └── memo.md               # Development notes
└── README.md                 # Project overview
```

## 🔧 Manual Setup

### 1. Build Container
```bash
# From robot directory
docker build -f docker/Dockerfile.robot -t wanderline-robot .
```

### 2. Start Services
```bash
docker-compose -f docker/docker-compose.robot.yml up -d
```

### 3. Verify Installation
```bash
# Check container status
docker ps

# Test basic functionality
docker exec docker-wanderline-robot-1 python3 /workspace/robot/demos/demo_circle.py
```

## 🎯 Available Commands

### Container Management
```bash
# Start environment
docker-compose -f docker/docker-compose.robot.yml up -d

# Stop environment
docker-compose -f docker/docker-compose.robot.yml down

# View logs
docker-compose -f docker/docker-compose.robot.yml logs -f

# Enter container
docker exec -it docker-wanderline-robot-1 bash
```

### Development Commands
```bash
# Run demos
docker exec docker-wanderline-robot-1 python3 /workspace/robot/demos/demo_circle.py

# Run tests
docker exec docker-wanderline-robot-1 python3 /workspace/robot/tests/test_circle.py

# Start Jupyter
docker exec docker-wanderline-robot-1 jupyter notebook --ip=0.0.0.0 --allow-root

# Access VNC GUI
# Open http://localhost:6081 in browser, click "Connect"

# ROS2 environment
docker exec docker-wanderline-robot-1 bash -c "source /opt/ros/humble/setup.bash && ros2 --help"
```

## 🖥️ VNC GUI 詳細ガイド

### VNC設定詳細
- **Web VNC**: 6081 (ブラウザアクセス用)
- **VNC Client**: 5902 (VNCクライアント用)
- **解像度**: 1920x1080
- **色深度**: 24bit

### サービス構成
```
Host Browser → Port 6081 → Container Port 80 → websockify → VNC Server (5901)
```

### ROS2環境でのGUI使用
```bash
# VNCデスクトップ内のターミナルで実行
source /opt/ros/humble/setup.bash

# RViz2の起動
DISPLAY=:1 rviz2

# Gazeboの起動
DISPLAY=:1 gazebo

# UR5eロボット可視化（VNC GUI内で実行）
DISPLAY=:1 ros2 launch ur_description view_ur.launch.py ur_type:=ur5e use_fake_hardware:=true launch_rviz:=true
```

### デスクトップ環境の使用
- **ファイルマネージャー**: デスクトップ上のアイコンをクリック
- **ターミナル**: 右クリック → "Open Terminal"
- **プロジェクトディレクトリ**: `/workspace`

## 🔍 Troubleshooting

### VNC接続問題

**1. 接続できない**
```bash
# VNCサービス状態確認
docker exec docker-wanderline-robot-1 ps aux | grep vnc

# VNCサービス再起動（通常は不要）
docker-compose -f docker/docker-compose.robot.yml restart

# 接続テスト
curl -I http://localhost:6081
```

**2. 画面が表示されない**
```bash
# コンテナ再起動
docker-compose -f docker/docker-compose.robot.yml restart

# ブラウザキャッシュクリア (Ctrl+F5 または Cmd+R)
```

**3. 日本語入力設定**
VNCデスクトップ内で：
```bash
sudo apt-get update
sudo apt-get install ibus-mozc
ibus-setup
```

**4. パフォーマンス最適化**
```bash
# 色深度を下げる（高速化）
echo "vncserver -depth 16" > ~/.vnc/config

# 解像度を下げる
echo "geometry=1280x720" >> ~/.vnc/config
```

### ブラウザ別対応
- **Chrome/Chromium** ✅: 最適なパフォーマンス
- **Firefox** ✅: 良好なパフォーマンス
- **Safari** ⚠️: 基本機能のみ
- **Edge** ✅: Chrome同等の性能

### GUI Applications Not Working
```bash
# Linux/macOS: Enable X11 forwarding
xhost +local:docker

# Use web VNC at http://localhost:6081 (recommended)
```

### 一般的な問題

**1. Container Build Fails**
```bash
# Clean build
docker system prune -a
docker build --no-cache -f docker/Dockerfile.robot -t wanderline-robot .
```

**2. Port Conflicts**
```bash
# Check port usage
netstat -tulpn | grep :8888

# Modify docker-compose.robot.yml ports if needed
```

**3. ROS2 Commands Not Found**
```bash
# Inside container, source ROS2 environment
source /opt/ros/humble/setup.bash
```

### Verification Steps
```bash
# 1. Test Python environment
docker exec docker-wanderline-robot-1 python3 -c "import numpy; print('NumPy OK')"

# 2. Test ROS2 installation
docker exec docker-wanderline-robot-1 bash -c "source /opt/ros/humble/setup.bash && ros2 --version"

# 3. Test circle calculation
docker exec docker-wanderline-robot-1 python3 /workspace/robot/demos/demo_circle.py

# 4. Test UR5e robot visualization
docker exec docker-wanderline-robot-1 bash -c "source /opt/ros/humble/setup.bash && ros2 launch ur_description view_ur.launch.py ur_type:=ur5e use_fake_hardware:=true launch_rviz:=false"
```

## 🔧 高度な設定

### VNC接続パラメータ変更
```bash
# パスワード設定
docker exec -it docker-wanderline-robot-1 vncpasswd

# 解像度変更
docker exec docker-wanderline-robot-1 vncserver -kill :1
docker exec docker-wanderline-robot-1 vncserver :1 -geometry 1600x900 -depth 24
```

### 複数ユーザー接続
```bash
# 追加のVNCセッション開始
docker exec docker-wanderline-robot-1 vncserver :2 -geometry 1920x1080 -depth 24

# ポートマッピング追加が必要（docker-compose.yml編集）
```

### 追加ソフトウェア
```bash
# VS Code インストール
wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
sudo install -o root -g root -m 644 packages.microsoft.gpg /etc/apt/trusted.gpg.d/
sudo apt-get update
sudo apt-get install code
```

## 🚀 Next Steps

1. **Test Basic Demos**: Verify circle calculation works
2. **ROS2 Integration**: Test robot visualization in RViz
3. **Gazebo Simulation**: Load UR5e robot model
4. **Wanderline Integration**: Connect drawing optimization

## 📱 モバイル・セキュリティ

### モバイル対応
- **タブレット/スマートフォン**: タッチ操作対応、横画面推奨
- **iOS Safari**: ホーム画面に追加でアプリ化可能

### セキュリティ注意事項
- VNCはローカルネットワーク専用
- 外部公開は非推奨
- パスワード設定を推奨

## 📚 Additional Resources

- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Universal Robots ROS2 Packages](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)
- [Gazebo Classic Documentation](https://classic.gazebosim.org/)
- [Docker Compose Reference](https://docs.docker.com/compose/)

---
*Last updated: 2025-07-11*
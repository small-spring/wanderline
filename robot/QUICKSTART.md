# 🚀 Wanderline Robot - Quick Start

**たった3ステップで動く！**

## 🎯 Step 1: セットアップ実行
ホストマシンにて以下を実行
```bash
cd robot
bash scripts/setup.sh
```

## 🖥️ Step 2: VNC GUI開く
ブラウザで http://localhost:6081 にアクセス
→ 「Connect」をクリック

## 🤖 Step 3: UR5eパッケージ追加
VNCデスクトップ内でターミナルを開いて**一度だけ**実行：

1. デスクトップを右クリック → "Open Terminal"
2. 以下を実行：
```bash
cd /home/ubuntu/ros2_ws/src

# クリーンアップ（既存の不完全なクローンを削除）
rm -rf ur_description Universal_Rob*

# UR5eパッケージをクローン
git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git ur_description

# ビルド実行
cd ..
colcon build --packages-select ur_description

# Canvas Preview Window (Window 2) のためのOpenCV GUI対応
pip uninstall -y opencv-python-headless
pip install opencv-python
```

## 🤖 Step 4: ロボット表示
**Step 3完了後**、VNCターミナルで実行：

### デモ1: スライダーで動かせるロボット
```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
ros2 launch ur_description view_ur.launch.py ur_type:=ur5e use_fake_hardware:=true launch_rviz:=true
```

### デモ2: 円を描くロボット
```bash
cd /workspace/robot
bash scripts/auto_circle_demo.sh
```

### デモ3: Phase 1 Dual Display System（✨新機能）
**2つのウィンドウで3D+2Dビュー同時表示**

**ターミナル1**: ロボット+RViz起動
```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export DISPLAY=:1.0
ros2 launch /workspace/robot/launch/ur5e_standard.launch.py jsp_gui:=false use_rviz:=true
```

**ターミナル2**: Phase 1円描画+キャンバスプレビュー
```bash
cd /workspace/robot/demos/phase1
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
source install/setup.bash 
export ROS_DOMAIN_ID=0
export DISPLAY=:1.0
python3 main.py
```

**結果**: 
- Window 1 (RViz): 3Dロボットが円を描く動作
- Window 2 (Canvas Preview): リアルタイム2D円描画進捗


## 🎉 完了！

UR5eロボットがRVizに表示されます。


---

## 🔧 トラブルシューティング

### VNC接続できない
```bash
docker-compose -f docker/docker-compose.robot.yml restart
```

### コンテナが起動しない
```bash
docker-compose -f docker/docker-compose.robot.yml down
./scripts/setup.sh
```

### GPGエラーが出る場合
```bash
# Docker容量を空けてリセット
docker system prune -f
./scripts/setup.sh
```

### 全部やり直し
```bash
docker-compose -f docker/docker-compose.robot.yml down
docker system prune -f
./scripts/setup.sh
```

### よくある問題

**GPG署名エラー**: 
- 最新版では自動修正されます
- `APT::Get::AllowUnauthenticated "true"`でバイパス済み

**ROS2パッケージが見つからない**:
- コンテナ内で自動再試行されます  
- 基本機能には影響しません

---

**これだけ！シンプル！** 🎯
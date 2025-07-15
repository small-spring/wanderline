# 🚀 Wanderline Robot - Quick Start

**たった3ステップで動く！**

## 🎯 Step 1: セットアップ実行
ホストマシンにて以下を実行
```bash
cd robot
./scripts/setup.sh
```

## 🖥️ Step 2: VNC GUI開く
ブラウザで http://localhost:6081 にアクセス
→ 「Connect」をクリック

## 🤖 Step 3: ロボット表示
**重要**: VNCデスクトップ内でターミナルを開いて実行：

### デモ1: スライダーで動かせるロボット
1. デスクトップを右クリック → "Open Terminal"
2. 以下を実行：
```bash
source /opt/ros/humble/setup.bash
ros2 launch ur_description view_ur.launch.py ur_type:=ur5e use_fake_hardware:=true launch_rviz:=true
```

### デモ2: 円を描くロボット
```bash
./scripts/auto_circle_demo.sh
```

### デモ3: Phase 1 Dual Display System（✨新機能）
**2つのウィンドウで3D+2Dビュー同時表示**

**ターミナル1**: ロボット+RViz起動
```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export DISPLAY=:1.0
ros2 launch /workspace/robot/launch/ur5e_standard.launch.py jsp_gui:=false use_rviz:=true
```

**ターミナル2**: Phase 1円描画+キャンバスプレビュー
```bash
cd /workspace/robot/demos/phase1
source /opt/ros/humble/setup.bash
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

### 全部やり直し
```bash
docker-compose -f docker/docker-compose.robot.yml down
docker system prune -f
./scripts/setup.sh
```

---

**これだけ！シンプル！** 🎯
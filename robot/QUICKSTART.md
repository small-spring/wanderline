# 🚀 Wanderline Robot - Quick Start

**たった3ステップで動く！**

## 🎯 Step 1: セットアップ実行
```bash
cd robot
./scripts/setup.sh
```

## 🖥️ Step 2: VNC GUI開く
ブラウザで http://localhost:6081 にアクセス
→ 「Connect」をクリック

## 🤖 Step 3: ロボット表示
**重要**: VNCデスクトップ内でターミナルを開いて実行：
1. デスクトップを右クリック → "Open Terminal"
2. 以下をコピー・ペースト：
```bash
source /opt/ros/humble/setup.bash
ros2 launch ur_description view_ur.launch.py ur_type:=ur5e use_fake_hardware:=true launch_rviz:=true
```

## 🎉 完了！

UR5eロボットがRVizに表示されます。

---

## 🧪 追加テスト

### 円描画デモ
```bash
python3 /workspace/robot/demos/demo_circle.py
```

### 数学テスト
```bash
python3 /workspace/robot/tests/test_circle.py
```

### Gazeboシミュレーション（新しいターミナル）
```bash
source /opt/ros/humble/setup.bash
gazebo
```

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
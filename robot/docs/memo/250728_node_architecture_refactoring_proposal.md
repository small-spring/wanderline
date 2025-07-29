# Node Architecture Design - 250728

## 概要

ROSの分散アーキテクチャを活用した描画システムのノード設計仕様書です。単一責任原則に基づいた5ノード構成により、保守性・テスト性・拡張性を向上させます。

## アーキテクチャ設計

### ノード構成

```
CentralNode (描画制御コーディネーター)
├── CanvasNode (Canvas状態管理)
├── MotionPlannerNode (Action Server)  
├── ContactDetectionNode (接触判定・状態更新)
├── VisualizationNode (RViz可視化)
└── CoordinateTransformNode (座標変換Service)
```

### 制御フロー

```
CentralNode制御ループ:
1. ロボット状態取得 (/joint_states)
2. Canvas状態取得 (/canvas_state) 
3. 次の目標座標取得 (/get_next_target - Service), 1,2の実行結果を渡して、返り値 next_targetを得る
4. Motion計画実行 (/plan_motion - Action) next_target(x, y座標)を渡して action
5. 接触判定実行 (/check_contact - Service)
6. 状態配信 (/robot_state, /pen_tip_position)
7. 次ステップへ
```

### 1. CentralNode 🎯

**責任**: 描画全体の制御とコーディネーション

**担当機能**:
- 描画制御ループの実行
- ノード間の調整とタイミング制御
- 描画状態の管理

**ROS2通信仕様**:
- **Publishers**:
  - `/robot_state` (geometry_msgs/PoseStamped) → (VisualizationNode, Monitor tools)
  - `/pen_tip_position` (geometry_msgs/PoseStamped) → (VisualizationNode, Monitor tools)
- **Subscribers**:
  - `/joint_states` (sensor_msgs/JointState) → ロボット状態取得
  - `/canvas_state` (my_msgs/CanvasState) → Canvas状態取得
- **Services**:
  - Client: `/check_contact` (my_msgs/ContactCheck.srv) → 接触判定要求
  - Client: `/get_next_target` (my_msgs/GetNextTarget.srv) → 次の目標座標取得
- **Actions**:
  - Client: `/plan_motion` (my_msgs/PlanMotion.action) → Motion計画要求
- **Parameters**:
  - `drawing.update_rate`: 20.0
  - `drawing.circle_radius`: 100.0
  - `drawing.circle_center`: [400, 300]

### 2. CanvasNode 📋

**責任**: Canvas状態と描画データの管理

**担当機能**:
- Canvas位置・姿勢の管理
- **描画array（目標座標列）の管理**
- Canvas状態の配信
- 描画領域の定義

**ROS2通信仕様**:
- **Publishers**:
  - `/canvas_state` (my_msgs/CanvasState) → (CentralNode, ContactDetectionNode)
- **Subscribers**: なし
- **Services**:
  - Server: `/get_next_target` (my_msgs/GetNextTarget.srv) → 次の目標座標取得
- **Actions**: なし
- **Parameters**:
  - `canvas.position`: [0.4, 0.0, 0.05]
  - `canvas.width`: 0.4
  - `canvas.height`: 0.4
  - `drawing.source_file`: "drawing_path.json"

**メッセージ型定義**:
```msg
# my_msgs/CanvasState.msg
std_msgs/Header header
geometry_msgs/Pose pose  # canvasの位置姿勢 これ、canvasは大きさがあるのでどこの座標かを明治しないといけないきがする
float64 width
float64 height
geometry_msgs/Point2D[] drawing_array  # 描画目標座標列
```

```srv
# my_msgs/GetNextTarget.srv

# Request
# (空 - 次の目標を要求するだけ)

# Response
geometry_msgs/Point2D next_target
bool has_next_target  # false = 描画完了
```

### 3. MotionPlannerNode 🤖

**責任**: ロボット動作計画と実行

**担当機能**:
- Joint interpolation計算
- Smooth movement実行
- 動作計画の軌道生成
- 安全性チェック

**ROS2通信仕様**:
- **Publishers**:
  - `/joint_states` (sensor_msgs/JointState) → (CentralNode, ContactDetectionNode, VisualizationNode)
  - `/motion/status` (std_msgs/String) → Monitor tools
- **Subscribers**:
  - `/motion/emergency_stop` (std_msgs/Bool) → 緊急停止
- **Services**: なし
- **Actions**:
  - Server: `/plan_motion` (my_msgs/PlanMotion.action)
- **Parameters**:
  - `motion_planner.max_velocity`: 1.0
  - `motion_planner.max_acceleration`: 2.0
  - `motion_planner.interpolation_steps`: 25
  - `robot.joint_limits.velocity`: [3.15, 3.15, 3.15, 3.15, 3.15, 3.15]

**メッセージ型定義**:
```action
# my_msgs/PlanMotion.action

# Goal
geometry_msgs/PoseStamped[] target_poses

# Result
bool success
string message

# Feedback
geometry_msgs/PoseStamped current_pose
```

### 4. ContactDetectionNode 📍

**責任**: 接触検出とシステム状態管理

**担当機能**:
- ロボット-Canvas間の接触判定
- 接触点のシミュレーション
- 接触状態の管理

**ROS2通信仕様**:
- **Publishers**: なし
- **Subscribers**:
  - `/joint_states` (sensor_msgs/JointState) → ロボット状態取得
  - `/canvas_state` (my_msgs/CanvasState) → Canvas状態取得
- **Services**:
  - Server: `/check_contact` (my_msgs/ContactCheck.srv)
- **Actions**: なし
- **Parameters**:
  - `contact_detection.simulation_mode`: true
  - `contact_detection.contact_threshold`: 0.001
  - `canvas.tolerance`: 0.005

**メッセージ型定義**:
```srv
# my_msgs/ContactCheck.srv

# Request
geometry_msgs/PoseStamped robot_pose
my_msgs/CanvasState canvas_state

# Response
bool in_contact
```

### 5. VisualizationNode 🎨

**責任**: RViz可視化専門

**担当機能**:
- Canvas marker表示
- Pen body/tip visualization  
- Trail line strip表示
- Robot状態の可視化

**ROS2通信仕様**:
- **Publishers**:
  - `/visualization/canvas_marker` (visualization_msgs/Marker) → RViz
  - `/visualization/pen_trail` (visualization_msgs/MarkerArray) → RViz
  - `/visualization/pen_body` (visualization_msgs/Marker) → RViz
- **Subscribers**:
  - `/robot_state` (geometry_msgs/PoseStamped) → Robot状態取得
  - `/pen_tip_position` (geometry_msgs/PoseStamped) → ペン先位置取得
  - `/joint_states` (sensor_msgs/JointState) → Joint状態取得
- **Services**: なし
- **Actions**: なし
- **Parameters**:
  - `visualization.canvas_size`: [0.4, 0.4, 0.005]
  - `visualization.pen_diameter`: 0.012
  - `visualization.trail_width`: 0.002
  - `visualization.update_frequency`: 10.0

### 6. CoordinateTransformNode 📐

**責任**: 座標変換サービス提供

**担当機能**:
- Pixel → Robot → Joint座標変換
- 逆運動学計算
- ペン先オフセット計算

**ROS2通信仕様**:
- **Publishers**: なし
- **Subscribers**: なし
- **Services**:
  - Server: `/coordinate/transform` (my_msgs/CoordinateTransform.srv)
- **Actions**: なし
- **Parameters**:
  - `coordinate_transform.canvas_position`: [0.4, 0.0, 0.05]
  - `coordinate_transform.canvas_size`: [0.4, 0.4]
  - `coordinate_transform.pixel_resolution`: [800, 600]
  - `coordinate_transform.pen_tip_offset`: 0.15


## 通信設計

### ステップ別通信仕様

| ステップ | 通信方式 | トピック/サービス名 | メッセージ型 | 既存 or 自作 |
|---------|---------|-------------------|-------------|-------------|
| 1. joint取得 | Topic sub | `/joint_states` | `sensor_msgs/JointState` | 既存 ✅ |
| 2. canvas取得 | Topic sub | `/canvas_state` | `my_msgs/CanvasState` | **自作** |
| 3. 目標座標取得 | Service client | `/get_next_target` | `my_msgs/GetNextTarget.srv` | **自作** |
| 4. motion計画 | Action client | `/plan_motion` | `my_msgs/PlanMotion.action` | **自作** |
| 5. 接触判定 | Service client | `/check_contact` | `my_msgs/ContactCheck.srv` | **自作** |
| 6. 状態配信 | Topic pub | `/robot_state`, `/pen_tip_position` | `geometry_msgs/PoseStamped` | 既存 ✅ |

### ノード間通信マップ

| 送信ノード | 受信ノード | 方式 | トピック/サービス | 用途 |
|-----------|-----------|------|-----------------|------|
| MotionPlannerNode | CentralNode, ContactDetectionNode, Visualization | Topic | `/joint_states` | ロボット状態配信 |
| CanvasNode | CentralNode, ContactDetectionNode | Topic | `/canvas_state` | Canvas状態配信 |
| CanvasNode | CentralNode | Service | `/get_next_target` | 次の目標座標提供 |
| CentralNode | MotionPlannerNode | Action | `/plan_motion` | 動作計画要求 |
| CentralNode | ContactDetectionNode | Service | `/check_contact` | 接触判定要求 |
| CentralNode | VisualizationNode | Topic | `/robot_state`, `/pen_tip_position` | 状態配信 |

---

**設計者**: Claude Code  
**日付**: 2025-07-28  
**対象**: Robot Drawing System Node Architecture  
# Robot Development Problem-Solution Log

開発過程で遭遇した問題と解決策を時系列で記録

---

## (2025-07-11) UR5e Robot が RViz に表示されない

### 問題の内容
- ROS2でUR5eロボットを起動してもRVizに何も表示されない
- "No tf data" エラーが発生
- robot_state_publisherが正常に動作していない

### 解決に必要だった知識
**重要な発見**: `use_fake_hardware:=true` パラメータの存在
```bash
# 動作するコマンド
ros2 launch ur_description view_ur.launch.py ur_type:=ur5e use_fake_hardware:=true launch_rviz:=true
```

**技術的理解**:
- `use_fake_hardware:=true` がTF変換を作成する
- このパラメータなしでは視覚的なモデルのみでTFフレームが生成されない
- URDFファイルの読み込みだけでは不十分

**環境設定**:
- VNC Web GUI: http://localhost:6081
- Display設定: DISPLAY=:1
- 必要パッケージ: `ros-humble-ur-description`, `ros-humble-ur-robot-driver`

---

## (2025-07-15) Robot がジョイント指令に応答しない

### 問題の内容
- `/joint_states` トピックにメッセージを送信してもロボットが動かない
- スクリプトは正常に実行されるが視覚的な変化がない
- robot_state_publisherが動作していない

### 解決に必要だった知識
**根本原因**: 視覚モデルのみが読み込まれ、実際のロボット制御が有効になっていない

**解決策**: 適切なlaunchシーケンスの使用
```bash
# 正しいlaunch順序
ros2 launch /workspace/robot/launch/ur5e_no_gui.launch.py  # GUI競合なし
python3 /workspace/robot/demos/robot_draw_circle.py       # 制御スクリプト
```

**学んだこと**:
- launchファイルの設計がロボット制御に直接影響
- 複数のlaunchファイル同時実行は危険
- 単一のlaunch実行が安全

---

## (2025-07-15) ロボットの動きがカクカクになる

### 問題の内容
- ロボットが滑らかに動かずに点から点へジャンプする
- 円を描いているはずなのに角ばった動きになる
- 動作中にロボットが「中央に戻る」ような動作をする

### 解決に必要だった知識
**根本原因**: `joint_state_publisher_gui` との制御信号競合
```bash
# 競合する発行者
- joint_state_publisher_gui (スライダーインターフェース) - 常に「中央化」
- robot_draw_circle.py (円描画スクリプト)
```

**解決策1**: 競合するノードの除去
```bash
pkill -f joint_state_publisher_gui
```

**解決策2**: 滑らかな補間の実装
```python
def interpolate_joints(self, start_joints, target_joints, progress):
    """線形補間による滑らかな動作"""
    result = []
    for i in range(len(start_joints)):
        interpolated = start_joints[i] + (target_joints[i] - start_joints[i]) * progress
        result.append(interpolated)
    return result
```

**技術パラメータ**:
- 更新頻度: 50Hz (20ms間隔)
- ウェイポイント: 24点/円
- 補間ステップ: 25ステップ/ウェイポイント間

---

## (2025-07-15) 複数のRVizウィンドウが起動する問題

### 問題の内容
- `./scripts/auto_circle_demo.sh` 実行時に2つのRVizウィンドウが起動
- ユーザー体験が悪化
- 想定外の動作で混乱を招く

### 解決に必要だった知識
**重要な発見**: `view_ur.launch.py` にはRViz制御パラメータが存在しない

**調査方法**:
```bash
ros2 launch ur_description view_ur.launch.py --show-args
```

**利用可能パラメータ（実際）**:
```bash
'ur_type': Type/series of used UR robot
'safety_limits': Enables safety limits controller  
'safety_pos_margin': Margin to limits in safety controller
'safety_k_position': k-position factor in safety controller
'description_package': Description package with URDF/XACRO files
'description_file': URDF/XACRO description file
'tf_prefix': Prefix of joint names
```

**学んだこと**:
- `launch_rviz:=false` パラメータは存在しない（仮定していた）
- Universal Robotsの設計: `view_ur.launch.py` は常にRVizを起動する
- パラメータの存在確認は公式ソースで行う必要がある

---

## (2025-07-15) 自動化スクリプトでROS2システムが破損

### 問題の内容
- `./scripts/auto_circle_demo.sh` 実行後にROS2システムが完全破損
- `!rclpy.ok()` エラーでPythonスクリプトが停止
- ROS2デーモンが停止し全コマンドが無効になる

### 解決に必要だった知識
**根本原因**: 複数の robot_state_publisher が同時実行される

**問題のあるスクリプト動作**:
1. 既存プロセスのpkill失敗 (Operation not permitted)
2. 複数launchファイル同時実行
3. robot_state_publisher重複 (3つ同時実行確認)
4. ROS2内部状態破損

**解決策**: ROS2標準パターンの実装
```python
# 標準パターン例
DeclareLaunchArgument('jsp_gui', default_value='true', choices=['true', 'false'])

# 条件付きノード起動
joint_state_publisher = Node(condition=UnlessCondition(LaunchConfiguration('jsp_gui')))
joint_state_publisher_gui = Node(condition=IfCondition(LaunchConfiguration('jsp_gui')))
```

**実装した解決策**:
- カスタムlaunchファイル `ur5e_standard.launch.py` の作成
- 標準ROS2パターンに準拠した条件付きノード実行
- pkill不要な設計

---

## (2025-07-15) Universal Robots 公式パッケージの設計問題

### 問題の内容
- 他のROS2パッケージでは標準的な `jsp_gui:=false` が使用できない
- Universal Robotsパッケージが標準パターンを採用していない
- 一般的なROS2知識が適用できない

### 解決に必要だった知識
**調査結果**: ROS2コミュニティの標準パターンの理解

**他のパッケージの実装例**:
- **urdf_launch**: 標準的な実装リファレンス
- **mycobot_description**: `jsp_gui:=false use_rviz:=false` サポート
- **moveit_resources**: 条件付きノード起動の実装例

**Universal Robots の問題**:
- ✅ **他のパッケージ**: `ros2 launch package file.launch.py jsp_gui:=false` が標準
- ❌ **ur_description**: この標準パターンを**採用していない**

**解決策**: 標準パターンに準拠したカスタムlaunchファイル作成

---

## 学んだ重要な教訓

### ROS2開発のベストプラクティス

1. **パラメータ検証**: 
   ```bash
   ros2 launch <package> <file> --show-args  # 必須の事前確認
   ```

2. **単一制御源の原則**: 
   - `/joint_states` には必ず単一のノードのみが発行
   - 複数の発行者は制御競合を引き起こす

3. **滑らかな動作**: 
   - 高頻度補間が現実的なロボット動作に必須
   - 50Hz以上の更新頻度を推奨

4. **標準パターンの活用**: 
   - ROS2コミュニティの慣習に従う
   - 条件付きノード実行で柔軟性を確保

5. **プロセス管理**: 
   - GUI重複の検出と回避
   - 適切なクリーンアップ手順

### 開発戦略

1. **段階的アプローチ**: 
   - 単一関節 → 複数関節協調
   - 簡単な動作 → 複雑な軌道

2. **テスト駆動**: 
   - 各コンポーネントを個別に検証
   - 統合前に動作確認

3. **標準準拠**: 
   - 公式ドキュメントの確認
   - コミュニティパターンの調査

---

## (2025-07-15) Phase 1 VNC環境テスト - シークバーGUI問題の解決

### 問題の内容
- VNC環境で`ros2 launch /workspace/robot/launch/ur5e_standard.launch.py`実行時にシークバーGUIが表示される
- Phase 1円描画デモ実行時に制御信号が競合する
- ユーザーが「pkillが必要」と誤解する状況

### 解決に必要だった知識
**重要な発見**: `jsp_gui:=false`引数で制御競合を完全回避可能

**問題のあるコマンド**:
```bash
# デフォルト（jsp_gui:=true）でシークバーGUIが起動
ros2 launch /workspace/robot/launch/ur5e_standard.launch.py
```

**正しいコマンド**:
```bash
# jsp_gui:=false でGUI無効化、制御競合なし
ros2 launch /workspace/robot/launch/ur5e_standard.launch.py \
    ur_type:=ur5e \
    jsp_gui:=false \
    use_rviz:=true
```

### VNC環境での正しいPhase 1テスト手順

**ターミナル1**: ロボットシミュレーション起動
```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export DISPLAY=:1.0

ros2 launch /workspace/robot/launch/ur5e_standard.launch.py \
    jsp_gui:=false \
    use_rviz:=true
```

**ターミナル2**: Phase 1円描画デモ実行
```bash
cd /workspace/robot/demos/phase1
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
python3 main.py
```

### 期待される動作
- ✅ RVizにUR5eロボット表示（シークバーなし）
- ✅ Phase 1デモで滑らかな円描画動作
- ✅ 制御信号競合なし
- ✅ pkill不要なクリーンな実行

### 学んだこと
**`auto_circle_demo.sh`が正解を示していた**:
- スクリプト内で`jsp_gui:=false`使用
- pkillではなくlaunch引数で制御
- 標準ROS2パターンの活用

**ユーザー体験の改善**:
- デフォルトでシークバーが表示されるのは正常
- 制御競合回避は引数で解決
- Phase 1テスト時は必ず`jsp_gui:=false`を使用

---

*Last updated: 2025-07-15*

## (2025-07-15) ARM64環境でのGazebo制限とRVizベース代替アプローチ

### 問題の内容
- Ubuntu 22.04 (jammy) + ARM64 環境でGazebo11が利用できない
- 公式パッケージはAMD64のみ、非公式PPAは不安定
- `ros-humble-gazebo-ros` パッケージが見つからない

### 解決に必要だった知識
**環境制限の理解**:
- Ubuntu 22.04 (jammy) + aarch64 (ARM64)
- Gazebo Classic (gazebo11): 公式サポート終了予定（2025年1月）
- 新しいGazebo (Gz Garden): 移行推奨だが複雑

**代替アプローチの選択**:
```bash
# 既存の動作確認済みシステムを活用
ros2 launch /workspace/robot/launch/ur5e_standard.launch.py jsp_gui:=false use_rviz:=true
```

### 実装戦略：RVizベースの描画システム

**Phase 1: 座標変換関数の実装**
```python
# robot/scripts/coordinate_transform.py
def canvas_to_robot_coords(pixel_x, pixel_y, canvas_height=0.1):
    """
    Wanderline 2D座標 → UR5e 3D座標変換
    - Canvas: 40cm × 40cm, 800×600ピクセル
    - Robot base: (0, 0, 0)
    - Canvas position: (0.5, 0, 0.1)
    """
    # ピクセル → メートル変換
    canvas_width_m = 0.4  # 40cm
    canvas_height_m = 0.4  # 40cm
    
    # 正規化 (0-1)
    normalized_x = pixel_x / 800.0
    normalized_y = pixel_y / 600.0
    
    # 物理座標
    canvas_x = normalized_x * canvas_width_m
    canvas_y = normalized_y * canvas_height_m
    
    # ロボット座標系
    robot_x = 0.5 + canvas_x - (canvas_width_m / 2)
    robot_y = canvas_y - (canvas_height_m / 2)
    robot_z = canvas_height
    
    return (robot_x, robot_y, robot_z)
```

**Phase 2: Wanderline出力解析**
```python
# robot/scripts/wanderline_parser.py
def parse_stroke_data(stroke_file):
    """stroke_data.jsonから描画データを解析"""
    with open(stroke_file, 'r') as f:
        data = json.load(f)
    
    strokes = []
    for stroke in data['strokes']:
        start_3d = canvas_to_robot_coords(stroke['start_pt'][0], stroke['start_pt'][1])
        end_3d = canvas_to_robot_coords(stroke['end_pt'][0], stroke['end_pt'][1])
        strokes.append({
            'start': start_3d,
            'end': end_3d,
            'angle': stroke['angle'],
            'step': stroke['step']
        })
    
    return strokes
```

**Phase 3: RVizビジュアル確認**
```python
# robot/scripts/rviz_drawing_visualizer.py
def visualize_drawing_path(strokes):
    """RVizでの描画パス可視化"""
    marker_pub = self.create_publisher(MarkerArray, '/drawing_path', 10)
    
    for i, stroke in enumerate(strokes):
        # 線分マーカー
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.id = i
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # 開始点と終了点
        marker.points = [
            Point(x=stroke['start'][0], y=stroke['start'][1], z=stroke['start'][2]),
            Point(x=stroke['end'][0], y=stroke['end'][1], z=stroke['end'][2])
        ]
        
        marker_pub.publish(marker)
```

### 学んだこと
**環境制約の受け入れ**:
- ARM64 + Ubuntu 22.04 での制限を理解
- 既存の動作確認済みシステムを最大限活用
- 段階的アプローチで確実な進歩

**技術的利点**:
- Gazebo不要でシステムが軽量
- RVizでの3D可視化が可能
- 既存のUR5eラウンチシステムを活用

---

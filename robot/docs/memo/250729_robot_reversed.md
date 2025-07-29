# 250729 Robot Orientation Issue Investigation

## 問題概要
UR5eロボットが円描画時に不正な向きで動作している。Canvas平面とペン軌道平面が一致せず、「下から仰ぎ見る」ような姿勢で描画している。

## 現状確認済み事項

### ✅ TF2システム状況
- **robot_state_publisher**: 正常起動（ur5e_standard.launch.py経由）
- **TFチェーン**: 完全に接続確認済み（base_link → tool0）
  - **重要発見**: main.py実行中のみ接続（/joint_states依存）
  - main.py停止時はTFチェーン切断（動的変換が停止）
- **TF変換エラー**: 発生していない（意外にも以前からエラーなし）
- **main.pyのTF呼び出し**: 正常に動作している
- **TF依存関係**: ur5e_standard.launch.py + main.pyの組み合わせで完全動作

### ❌ 残存問題
- **ロボット向き**: 依然として不正（Canvas平面と軌道平面が不一致）
- **描画結果**: 円は描けているが、Canvas面に垂直ではない
- **ペン先位置**: Canvas表面に正しく接触していない

## TF2調査結果

### 発見1: TFエラーが出ない理由
```python
# main.py:312-319でのTF変換
try:
    transform = self.tf_buffer.lookup_transform("base_link", "tool0", ...)
    # 実際は成功している
except Exception as e:
    # このブロックに入らない
```

### 発見2: 手計算キネマティクスが主要計算
main.pyは以下の手計算関数を主に使用：
- `corrected_coords.pixel_to_robot_coords()` 
- `corrected_coords.wrist3_coords_to_joints()`
- `corrected_coords.joints_to_pen_tip_position()`

**TF2は補助的な用途のみ**（ペン先位置の可視化等）

### 発見3: TF2 vs 手計算の優先順位
```python
# main.pyの処理フロー
# 1. 手計算でtarget_joints算出 ← メイン処理（間違った結果）
# 2. TF2でペン位置可視化 ← 正しい結果だが使われない
```

**重要**: TF2は正しく動作しているが、手計算結果が優先使用され、TF2結果は活用されていない。

## 根本原因分析

### 1. Tool Flangeの向き計算問題
```python
# corrected_coordinate_system.py:143-153
def joints_to_tool_flange_position(self, joints):
    wrist3_pos = self.joints_to_wrist3_position(joints)
    # ❌ 常にZ軸下方向にオフセット（間違い）
    tool_flange_z = wrist3_pos[2] - tool_flange_offset
```

**問題**: `joints[4] = 1.57`（wrist_2 = 90度）の時、tool_flangeは横向きになるが、単純にZ軸方向にオフセットしている。

### 2. Canvas接触計算の矛盾
```python
# main.py:174-180, 263-269
canvas_z = self.canvas_system.canvas_position[2]  # Z=0.05
pen_tip_target_z = canvas_z + 0.001              # Z=0.051
tool_flange_z = pen_tip_target_z + self.pen_length  # Z=0.171
```

**問題**: wrist_2=90度時の実際のtool_flange向きを考慮していない。

### 3. 座標系の責任混乱
- `pixel_to_robot_coords()`: ペン先座標を返すべきだが、Z座標は手動上書きされる
- Canvas位置設定とcontact_height設定が複数箇所で不一致

## 設定値確認

### Config値（config.yaml）
```yaml
canvas:
  position: [0.6, 0.0, 0.05]  # Canvas位置: X=60cm, Y=0, Z=5cm
  contact_height: 0.02        # 接触時の高さオフセット: 2cm
pen:
  length: 0.12               # ペン長: 12cm
```

### 実際の計算値
- ペン先目標位置: Z = 0.05 + 0.001 = 0.051m
- Tool flange目標位置: Z = 0.051 + 0.12 = 0.171m
- しかし実際はwrist_2=90度で横向き

## 次のアクション

### Phase 2: CoordMapper実装 + 比較検証モード
**最優先**: `coord_mapper.py`の実装完了

#### 現在のCoordMapper状況
- ✅ 基本クラス構造作成済み
- ❌ 存在しないフレーム参照（"canvas_frame", "pen_tip"）
- ❌ 実装されていないヘルパーメソッド（`_pixel_to_canvas_position`）
- ❌ tool0フレーム活用の実装不足

#### 実装すべき機能
1. **既存フレームのみ使用**: "base_link", "tool0"のみ
2. **ヘルパーメソッド実装**: pixel→canvas座標変換
3. **TF2ベースpen_tip計算**: tool0 + pen_lengthオフセット
4. **比較機能**: 手計算 vs TF2結果の差分表示
5. **逆変換実装**: robot座標 → pixel座標

### 修正対象ファイル（優先順）
1. `coord_mapper.py`: TF2活用実装完了 ← **最優先**
2. `main.py`: CoordMapper統合 + 比較モード追加  
3. `corrected_coordinate_system.py`: Tool flange向き計算修正（必要に応じて）

## 技術的詳細

### UR5e Kinematics
- wrist_2 = 0: tool_flangeは下向き
- wrist_2 = π/2: tool_flangeは横向き（現在の設定）
- Tool flange offset: 82.5mm（wrist_3からのローカルZ軸方向）

### 現在の間違った仮定
現在のコードは「tool_flangeが常に下向き」を前提としているが、実際は関節角度により向きが変わる。

---

**結論**: TF2システムは正常に動作しているが、主要計算は手計算キネマティクス。Tool flangeの向き計算が根本的に間違っており、これが「下から仰ぎ見る」問題の直接原因。
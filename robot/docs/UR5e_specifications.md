# UR5e Robot Specifications and Structure

## 概要
UR5eは6自由度（6DOF）のシリアルロボットアームで、非球面手首設計を採用している。

## 基本仕様
- **ペイロード**: 5kg (11 lbs)
- **リーチ**: 850mm
- **重量**: 20.6kg
- **フットプリント**: Ø 149mm
- **繰り返し精度**: ± 0.03mm (ISO 9283準拠)
- **TCP速度**: 1m/s
- **消費電力**: 平均200W
- **関節可動範囲**: ± 360° (全関節)
- **最大関節速度**: ± 180°/s (全関節)

## 関節構造とリンク構成

### 関節名称
1. **Base (Shoulder Pan)** - 肩パン関節
2. **Shoulder Lift** - 肩リフト関節  
3. **Elbow** - 肘関節
4. **Wrist 1** - 手首1関節
5. **Wrist 2** - 手首2関節
6. **Wrist 3** - 手首3関節（**エンドエフェクタ**）

### DH Parameters (Denavit-Hartenberg)

| Joint | a (m) | d (m) | α (rad) | 説明 |
|-------|-------|-------|---------|------|
| 1 (Base) | 0 | 0.1625 | π/2 | ベース高さ 162.5mm |
| 2 (Shoulder) | -0.425 | 0 | 0 | 上腕長 425mm |
| 3 (Elbow) | -0.3922 | 0 | 0 | 前腕長 392.2mm |
| 4 (Wrist 1) | 0 | 0.1333 | π/2 | 手首1オフセット 133.3mm |
| 5 (Wrist 2) | 0 | 0.0997 | -π/2 | 手首2オフセット 99.7mm |
| 6 (Wrist 3) | 0 | 0.0996 | 0 | **エンドエフェクタ距離 99.6mm** |

### 重要な寸法
- **ベース高さ**: 162.5mm
- **上腕長**: 425mm  
- **前腕長**: 392.2mm
- **手首部分**: 133.3mm + 99.7mm + 99.6mm = 332.6mm
- **総リーチ**: 425 + 392.2 = 817.2mm (理論上)

### 各リンクの質量
- Link 1: 3.761 kg
- Link 2: 8.058 kg (最重量)
- Link 3: 2.846 kg
- Link 4: 1.37 kg
- Link 5: 1.3 kg
- Link 6: 0.365 kg (最軽量)

## エンドエフェクタについて

### 重要なポイント
- **Wrist 3 (Joint 6)** が実際の**エンドエフェクタ**
- エンドエフェクタはWrist 2から **99.6mm** 延長した位置
- 通常のツール取り付けポイント

### 座標フレーム
- `base_link`: ロボットベース
- `shoulder_link`: 肩関節
- `upper_arm_link`: 上腕
- `forearm_link`: 前腕  
- `wrist_1_link`: 手首1
- `wrist_2_link`: 手首2
- `wrist_3_link`: **エンドエフェクタフレーム**
- `tool0`: デフォルトツールフレーム（wrist_3_linkと同位置）

## シミュレーションでの実装確認

### 現在のコード内の値
```python
# corrected_coordinate_system.py
self.base_height = 0.163     # ✅ 162.5mm (ほぼ正確)
self.shoulder_offset = 0.138 # ❓ DH表にない値
self.upper_arm = 0.425       # ✅ 425mm (正確)
self.forearm = 0.392         # ✅ 392.2mm (ほぼ正確)
```

### 問題の可能性
1. `shoulder_offset = 0.138` が DH Parameters に対応していない
2. 手首部分の長さ（332.6mm）が考慮されていない可能性
3. エンドエフェクタの実際の位置計算に誤差

## 修正が必要な点

### 1. エンドエフェクタ位置の正確な計算
- Wrist 3までの距離を正確に計算
- 最終的なツール位置（tool0）を使用

### 2. ペン表示位置
- ペン先: エンドエフェクタ位置
- ペン本体: エンドエフェクタから上方向に延長

### 3. 座標変換の検証
- DH Parameters に基づく正確な順運動学
- 逆運動学の検証

## 参考資料
- [Universal Robots DH Parameters](https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/)
- [UR5e Technical Specifications](https://www.universal-robots.com/products/ur5e/)
- Cambridge Core研究論文: "Electro-mechanical modeling and identification of the UR5 e-series robot"
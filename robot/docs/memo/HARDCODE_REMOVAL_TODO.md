# HARDCODE REMOVAL TODO

## 現状の問題
設定値が4箇所に分散してハードコードされており、一貫性の維持が困難。

## 発見されたハードコード一覧

### 1. Canvas設定
- `config.yaml`: position: [0.6, 0.0, 0.05]
- `corrected_coordinate_system.py`: self.canvas_position = [0.6, 0.0, 0.05]
- `canvas_coordinate_system.py`: CANVAS_POSITION = [0.6, 0.0, 0.05]
- `coordinate_system_fixer.py`: self.canvas_position = [0.6, 0.0, 0.05]

### 2. ペン設定
- `config.yaml`: tip_offset: 0.12
- `corrected_coordinate_system.py`: total_pen_extension = 0.12
- Canvas接触高さがあちこちに: 0.02, 0.05

### 3. UR5e仕様
- `base_height: 0.163`
- `shoulder_offset: 0.138`
- `upper_arm: 0.425`
- `forearm: 0.392`
- `flange_offset: 0.0825`

## 理想的なアーキテクチャ

### Phase 1: Config注入パターン
```python
# config.yaml (Single Source of Truth)
robot:
  ur5e:
    base_height: 0.163
    shoulder_offset: 0.138
    upper_arm: 0.425
    forearm: 0.392
    flange_offset: 0.0825

canvas:
  position: [0.6, 0.0, 0.05]
  physical_size: 0.4
  pixel_width: 800
  pixel_height: 600
  
pen:
  tip_offset: 0.12
  body_offset: 0.06
  contact_height: 0.02
  safe_height: 0.05

# Python classes
class CorrectedCoordinateSystem:
    def __init__(self, config):
        self.canvas_position = config['canvas']['position']
        self.pen_tip_offset = config['pen']['tip_offset']
        # ...
```

### Phase 2: Dependency Injection
```python
class RobotConfigService:
    def __init__(self, config_path):
        self.config = self._load_config(config_path)
    
    def get_canvas_config(self): return self.config['canvas']
    def get_pen_config(self): return self.config['pen']
    def get_robot_config(self): return self.config['robot']

class CoordinateSystemFactory:
    def create_corrected_system(self, config_service):
        return CorrectedCoordinateSystem(
            config_service.get_canvas_config(),
            config_service.get_pen_config(),
            config_service.get_robot_config()
        )
```

## 実装優先度

### High Priority (すぐ実装)
1. ✅ ペン長の統一 (0.12m everywhere)
2. ✅ TODO コメントでハードコード明示
3. Canvas位置の統一確認

### Medium Priority (次回実装)
1. Config注入によるコンストラクタ変更
2. ファクトリーパターンでの統一生成
3. 設定値の検証機能

### Low Priority (将来実装)
1. 設定のホットリロード
2. 複数ロボット対応
3. 設定値の単位変換機能

## 緊急対応完了
- ✅ corrected_coordinate_system.py: TODOコメント付きでハードコード明示
- ✅ canvas_coordinate_system.py: TODOコメント付きでハードコード明示  
- ✅ ペン長を0.12mで統一
- ✅ 将来のConfig注入設計を文書化

## 動作確認
現在のコードは動作するが、設定変更時は以下を同期する必要:
1. config.yaml の pen.tip_offset
2. corrected_coordinate_system.py の total_pen_extension  
3. Canvas位置の各ファイル間同期
# Detailed Design

This document provides the detailed system architecture, data definitions, API specifications, and module-level design for the Wanderline project.

# wanderline 詳細設計

## 1. データ定義・フォーマット
- canvas_state
  - サイズ: W×H
  - データ型: numpy.ndarray／PIL Image／独自構造体
  - 座標系: 原点位置、単位長さ
- motif_image
  - 入力形式: RGB 画像（png／jpg）
  - 前処理: リサイズ、正規化、グレースケール変換

## 2. API 仕様
### 2.1 各モジュール共通インターフェース
```python
class Module:
    def process(self, input_data) -> output_data:
        pass
```
### 2.2 データ入力モジュール
- 関数: `load_image(path: str) -> np.ndarray`
- 引数: 画像パス
- 戻り値: motif_image (前処理済)

### 2.3 動画記録モジュール Hook
- メソッド: `record(frame: np.ndarray, step: int)`
- パラメータ: 保存先ディレクトリ、形式（mp4/gif）、fps

## 3. モジュール詳細仕様
### 3.1 画像解析モジュール
- 出力特徴量: エッジマップ、セグメンテーションマスク、ラベリング情報
- 使用ライブラリ: OpenCV, skimage

### 3.2 線描画決定モジュール
- アルゴリズム: 強化学習 
- ネットワーク構成: CNN 
    - CNN Encoder
    - 6 ch入力：[motif(3)‖canvas(3)]
    - ResNet-18 shallow 版
    Actor-Critic Head
    - μ, σ → 角度 (Gauss)
    - 共通価値関数
- 入力: [canvas_state, motif_features]
- 出力: draw_angle (float)
はじめはシンプルにCNN。でもここを世界モデル+MPCに差し替えたり、 ベクター特化 (DiffVG + RLRF)できても嬉しいよね。

### 3.3 キャンバス更新モジュール
- 関数: `apply_stroke(state: np.ndarray, angle: float) -> np.ndarray`
- 描画長さ、線幅のパラメータ定義

## 4. 報酬関数定義
- 即時報酬: Δd = d(prev_state, motif_image) - d(next_state, motif_image)
- 距離関数: 一旦L2。後で差し替えやすいように設計。 LPIPSなども使うかも？
- 累積報酬: 指定ステップ数 n_stroke で割引率 γ を適用

## 5. 環境構築
- Python 3.12 を想定
- Python仮想環境・パッケージ管理: `uv` を使用
  - 初期化: `uv init`
  - 仮想環境作成: `uv env create 3.12`
  - パッケージインストール: `uv add <package>`
- 依存パッケージ：pyproject toml

## 6. テスト・検証
- ユニットテスト: pytest で距離関数、apply_stroke の動作確認
- 統合テスト: サンプル画像に対して一定ステップ後のキャンバス比較
- CI: GitHub Actions でテスト自動化（これは追々）


## todo
- outputフォルダに、configも保存して、再現実験ができると良いかも
    - 収束したかどうかも保存しときたい
- 「続きから」描ける仕組みも作ると良いかも
- 濃度も探索するようにしてもいいのかも、でもgreedyだと探索するパラメータが爆発的に増えるので、
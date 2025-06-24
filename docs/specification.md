# Specification

This document defines the project requirements, system components, and development environment for the Wanderline project.

# wanderline プロジェクト仕様書

## 1. 概要
wanderlineは、与えられた画像（motif_image）を基に、キャンバス（canvas_state）上に線を一筆書きで描画するエージェントです。出力される角度情報をもとに、ロボットアームなどで線を描くことを想定しています。

## 2. 目的
- モチーフ画像の特徴を捉え、一筆書きスタイルで再現する。
- 将来的にロボットアーム制御への応用を想定し、角度情報を出力。

## 3. 要件
### 3.1 機能要件
- 入力: canvas_state（現在の線情報を持つキャンバス）
- 入力: motif_image（エージェントが参照する画像）
- 出力: draw_angle（描画方向の角度）
- 出力後: canvas_state を更新し、連続的に描画可能にする

### 3.2 非機能要件
- （`pyproject.toml` で管理）
- 軽量な依存ライブラリ（uvなど）
- モジュール構成の明確化

## 4. システム構成
1. データ入力モジュール
2. 画像解析モジュール
3. 線描画決定モジュール
4. キャンバス更新モジュール
5. 動画記録モジュール

## 5. 開発環境
- OS: macOS または Linux
- シェル: zsh
- Python仮想環境・パッケージ管理: `uv` を使用

## 6. 今後のタスク
- [ ] 詳細なモジュール設計
- [ ] API仕様定義
- [ ] テストケース作成
- [ ] 実装とデモ
- [ ] 動画記録機能の設計・実装

## 7. 学習設計
- ストローク (stroke): 1ステップでキャンバスに追加される線分
- 即時報酬: 各ストローク実行後における canvas_state と motif_image の距離改善量
- n_stroke パラメータ: n 回のストロークにおける累積報酬を最大化するようエージェントを訓練可能
- n_stroke の調整により短期的／長期的視点のバランスを制御
- 距離計算手法:
  - ピクセルベースのL2距離（平均二乗誤差）
  - 知覚的距離 (LPIPS など) による高次特徴の差分
- 距離計算式 (ピクセルL2 の場合):
  ```text
  d(canvas_state, motif_image) = \frac{1}{WH}\sqrt{\sum_{i=1}^{W \times H} (c_i - m_i)^2}
  ```

---

*作成日: 2025-06-24*

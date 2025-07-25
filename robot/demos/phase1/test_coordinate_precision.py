#!/usr/bin/env python3
"""
座標変換精度テスト

robot_to_pixel_coords() と pixel_to_robot_coords() の変換精度を検証し、
ペン先位置のずれの原因を特定する。
"""

import sys
import os
sys.path.append('/Users/smallspring/programs/wanderline/robot/scripts')

from corrected_coordinate_system import CorrectedCoordinateSystem
import math


def test_coordinate_precision():
    """座標変換の往復精度をテストする"""
    print("🧪 座標変換精度テスト開始")
    print("=" * 50)
    
    coords = CorrectedCoordinateSystem()
    
    # テスト用のピクセル座標（円周上の点）
    center_pixel = (400, 300)  # Canvas center
    radius_pixel = 80  # 80 pixel radius
    
    test_points = []
    for i in range(8):  # 8方向でテスト
        angle = i * (2 * math.pi / 8)
        pixel_x = center_pixel[0] + radius_pixel * math.cos(angle)
        pixel_y = center_pixel[1] + radius_pixel * math.sin(angle)
        test_points.append((pixel_x, pixel_y))
    
    print(f"テスト点数: {len(test_points)}")
    print(f"中心: {center_pixel}, 半径: {radius_pixel}px")
    print()
    
    max_error = 0.0
    total_error = 0.0
    
    for i, (orig_pixel_x, orig_pixel_y) in enumerate(test_points):
        print(f"テスト {i+1}: 原点 ({orig_pixel_x:.2f}, {orig_pixel_y:.2f})")
        
        # Step 1: pixel → robot 変換
        robot_x, robot_y, robot_z = coords.pixel_to_robot_coords(
            orig_pixel_x, orig_pixel_y, pen_down=True
        )
        print(f"  → Robot座標: ({robot_x:.6f}, {robot_y:.6f}, {robot_z:.6f})")
        
        # Step 2: robot → pixel 逆変換
        back_pixel_x, back_pixel_y = coords.robot_to_pixel_coords(robot_x, robot_y)
        print(f"  → 復元Pixel: ({back_pixel_x:.6f}, {back_pixel_y:.6f})")
        
        # 誤差計算
        error_x = abs(back_pixel_x - orig_pixel_x)
        error_y = abs(back_pixel_y - orig_pixel_y)
        total_error_point = math.sqrt(error_x**2 + error_y**2)
        
        print(f"  → 誤差: X={error_x:.6f}px, Y={error_y:.6f}px, 総誤差={total_error_point:.6f}px")
        
        max_error = max(max_error, total_error_point)
        total_error += total_error_point
        print()
    
    avg_error = total_error / len(test_points)
    
    print("📊 精度テスト結果")
    print("=" * 30)
    print(f"最大誤差: {max_error:.6f} px")
    print(f"平均誤差: {avg_error:.6f} px")
    print(f"許容誤差: 1.0 px (目安)")
    
    if max_error > 1.0:
        print("❌ 座標変換精度に問題があります")
        return False
    else:
        print("✅ 座標変換精度は許容範囲内です")
        return True


def test_pen_tip_calculation():
    """ペン先位置計算のテスト"""
    print("\n🖊️ ペン先位置計算テスト")
    print("=" * 30)
    
    coords = CorrectedCoordinateSystem()
    
    # 基準ジョイント位置
    base_joints = [0.0, -1.2, -1.0, -1.5, 1.57 + 3.14159, 0.0]
    
    # テスト用ピクセル座標
    test_pixel = (400, 300)  # Canvas center
    
    print(f"テスト座標: {test_pixel}")
    
    # Step 1: pixel → robot → joints
    robot_x, robot_y, robot_z = coords.pixel_to_robot_coords(
        test_pixel[0], test_pixel[1], pen_down=True
    )
    print(f"目標Robot座標: ({robot_x:.6f}, {robot_y:.6f}, {robot_z:.6f})")
    
    joints = coords.robot_coords_to_joints(robot_x, robot_y, robot_z, base_joints)
    print(f"計算されたJoints: {[f'{j:.6f}' for j in joints[:3]]}")
    
    # Step 2: joints → pen_tip_position
    pen_tip_pos = coords.joints_to_pen_tip_position(joints)
    print(f"ペン先位置: ({pen_tip_pos[0]:.6f}, {pen_tip_pos[1]:.6f}, {pen_tip_pos[2]:.6f})")
    
    # Step 3: pen_tip → pixel (逆変換)
    pen_tip_pixel = coords.robot_to_pixel_coords(pen_tip_pos[0], pen_tip_pos[1])
    print(f"ペン先→Pixel: ({pen_tip_pixel[0]:.6f}, {pen_tip_pixel[1]:.6f})")
    
    # 誤差計算
    error_x = abs(pen_tip_pixel[0] - test_pixel[0])
    error_y = abs(pen_tip_pixel[1] - test_pixel[1])
    total_error = math.sqrt(error_x**2 + error_y**2)
    
    print(f"ペン先位置誤差: X={error_x:.6f}px, Y={error_y:.6f}px, 総誤差={total_error:.6f}px")
    
    if total_error > 5.0:
        print("❌ ペン先位置計算に大きな誤差があります")
        return False
    else:
        print("✅ ペン先位置計算は許容範囲内です")
        return True


def analyze_scale_consistency():
    """スケールの一貫性をチェック"""
    print("\n📏 スケール一貫性チェック")
    print("=" * 30)
    
    coords = CorrectedCoordinateSystem()
    
    # pixel_to_robot_coords のスケール計算
    scale_p2r = coords.canvas_size / coords.pixel_width
    print(f"pixel_to_robot_coords スケール: {scale_p2r:.10f}")
    
    # robot_to_pixel_coords のスケール計算  
    scale_r2p = coords.canvas_size / coords.pixel_width
    print(f"robot_to_pixel_coords スケール: {scale_r2p:.10f}")
    
    # 一貫性チェック
    if abs(scale_p2r - scale_r2p) < 1e-10:
        print("✅ スケールは一貫しています")
        return True
    else:
        print("❌ スケールに不一致があります")
        return False


def main():
    """メイン実行関数"""
    print("🔍 座標変換精度検証プログラム")
    print("ペン先と軌道のずれの原因を調査します")
    print()
    
    # テスト実行
    precision_ok = test_coordinate_precision()
    pen_tip_ok = test_pen_tip_calculation()
    scale_ok = analyze_scale_consistency()
    
    print("\n📋 総合結果")
    print("=" * 20)
    print(f"座標変換精度: {'✅ 正常' if precision_ok else '❌ 問題有り'}")
    print(f"ペン先位置精度: {'✅ 正常' if pen_tip_ok else '❌ 問題有り'}")
    print(f"スケール一貫性: {'✅ 正常' if scale_ok else '❌ 問題有り'}")
    
    if not (precision_ok and pen_tip_ok and scale_ok):
        print("\n🔧 修正が必要な項目があります")
        print("詳細は上記の個別テスト結果を確認してください")
    else:
        print("\n✅ すべてのテストが正常です")
        print("ずれの原因は別の要因かもしれません")


if __name__ == "__main__":
    main()
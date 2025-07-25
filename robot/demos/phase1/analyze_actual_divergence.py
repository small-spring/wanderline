#!/usr/bin/env python3
"""
実際のペン先位置ずれ原因分析

座標変換は正常なので、実際のロボット動作でのずれ原因を特定する。
main.pyでのペン先位置とRViz表示位置の計算過程を詳しく分析。
"""

import sys
import os
sys.path.append('/Users/smallspring/programs/wanderline/robot/scripts')

from corrected_coordinate_system import CorrectedCoordinateSystem
import math


def analyze_pen_positions():
    """ペン先位置の計算過程を詳細分析"""
    print("🔍 ペン先位置ずれ原因分析")
    print("=" * 50)
    
    coords = CorrectedCoordinateSystem()
    
    # 実際のロボット動作シミュレーション
    print("1. 実際のロボット動作シミュレーション")
    print("-" * 30)
    
    # 基準ジョイント位置（main.pyより）
    base_joints = [0.0, -1.2, -1.0, -1.5, 1.57 + 3.14159, 0.0]
    print(f"基準ジョイント: {[f'{j:.3f}' for j in base_joints]}")
    
    # テスト用目標座標（円周上の点）
    target_pixel = (480, 300)  # 右端の点
    print(f"目標ピクセル座標: {target_pixel}")
    
    # main.py の処理を順番に実行
    print("\n2. main.py の処理順序を追跡")
    print("-" * 30)
    
    # Step 1: _pixel_to_joint_position() の処理
    print("Step 1: _pixel_to_joint_position() の処理")
    robot_x, robot_y, robot_z = coords.pixel_to_robot_coords(
        target_pixel[0], target_pixel[1], pen_down=True
    )
    print(f"  目標Robot座標: ({robot_x:.6f}, {robot_y:.6f}, {robot_z:.6f})")
    
    target_joints = coords.robot_coords_to_joints(robot_x, robot_y, robot_z, base_joints)
    target_joints[4] = 1.57 + 3.14159  # Wrist 2 perpendicular + 180°
    target_joints[5] = 0.0              # Wrist 3 no rotation
    print(f"  計算されたジョイント: {[f'{j:.6f}' for j in target_joints[:3]]}")
    
    # Step 2: _execute_smooth_movement() での補間
    print("\nStep 2: _execute_smooth_movement() での補間")
    current_joints = base_joints.copy()
    interpolation_steps = 25
    current_step = 10  # 補間途中の例
    
    progress = current_step / interpolation_steps
    interpolated_joints = []
    for i in range(len(current_joints)):
        interpolated = current_joints[i] + (target_joints[i] - current_joints[i]) * progress
        interpolated_joints.append(interpolated)
    
    print(f"  補間進行度: {progress:.2f} ({current_step}/{interpolation_steps})")
    print(f"  補間ジョイント: {[f'{j:.6f}' for j in interpolated_joints[:3]]}")
    
    # Step 3: 実際のロボット位置計算
    print("\nStep 3: 実際のロボット位置計算")
    actual_robot_pos = coords.joints_to_robot_position(interpolated_joints)
    print(f"  実際のRobot位置: ({actual_robot_pos[0]:.6f}, {actual_robot_pos[1]:.6f}, {actual_robot_pos[2]:.6f})")
    
    # Step 4: RViz Trail用の計算（_add_pen_trail_point）
    print("\nStep 4: RViz Trail用の計算")
    pen_tip_pos = coords.joints_to_pen_tip_position(interpolated_joints)
    print(f"  ペン先位置: ({pen_tip_pos[0]:.6f}, {pen_tip_pos[1]:.6f}, {pen_tip_pos[2]:.6f})")
    
    # Trail display position calculation
    canvas_z = coords.canvas_position[2]
    trail_display_pos = (pen_tip_pos[0], pen_tip_pos[1], canvas_z + 0.003)
    print(f"  Trail表示位置: ({trail_display_pos[0]:.6f}, {trail_display_pos[1]:.6f}, {trail_display_pos[2]:.6f})")
    
    # Step 5: Canvas Contact用の計算（_simulate_contact_detection）
    print("\nStep 5: Canvas Contact用の計算")
    # 同じペン先位置を使用
    contact_pixel = coords.robot_to_pixel_coords(pen_tip_pos[0], pen_tip_pos[1])
    print(f"  Contact pixel座標: ({contact_pixel[0]:.6f}, {contact_pixel[1]:.6f})")
    
    # 誤差計算
    print("\n3. 位置誤差の分析")
    print("-" * 30)
    
    # 目標との誤差
    target_robot_pos = coords.joints_to_robot_position(target_joints)
    robot_error = math.sqrt(
        (actual_robot_pos[0] - target_robot_pos[0])**2 + 
        (actual_robot_pos[1] - target_robot_pos[1])**2
    )
    print(f"補間による誤差: {robot_error:.6f}m ({robot_error*1000:.2f}mm)")
    
    # ピクセル誤差
    actual_pixel = coords.robot_to_pixel_coords(actual_robot_pos[0], actual_robot_pos[1])
    pixel_error = math.sqrt(
        (actual_pixel[0] - target_pixel[0])**2 + 
        (actual_pixel[1] - target_pixel[1])**2
    )
    print(f"実際位置のピクセル誤差: {pixel_error:.2f}px")
    
    # Contact計算との比較
    contact_error = math.sqrt(
        (contact_pixel[0] - target_pixel[0])**2 + 
        (contact_pixel[1] - target_pixel[1])**2
    )
    print(f"Contact位置のピクセル誤差: {contact_error:.2f}px")
    
    return robot_error, pixel_error, contact_error


def identify_divergence_source():
    """ずれの発生源を特定"""
    print("\n🎯 ずれの発生源特定")
    print("=" * 30)
    
    coords = CorrectedCoordinateSystem()
    base_joints = [0.0, -1.2, -1.0, -1.5, 1.57 + 3.14159, 0.0]
    
    # 複数の補間段階でテスト
    target_pixel = (480, 300)
    robot_x, robot_y, robot_z = coords.pixel_to_robot_coords(
        target_pixel[0], target_pixel[1], pen_down=True
    )
    target_joints = coords.robot_coords_to_joints(robot_x, robot_y, robot_z, base_joints)
    target_joints[4] = 1.57 + 3.14159
    target_joints[5] = 0.0
    
    interpolation_steps = 25
    max_error = 0.0
    
    print("補間段階別誤差:")
    for step in [0, 5, 10, 15, 20, 25]:
        if step == 0:
            progress = 0.0
        else:
            progress = step / interpolation_steps
            
        # 補間計算
        interpolated_joints = []
        for i in range(len(base_joints)):
            interpolated = base_joints[i] + (target_joints[i] - base_joints[i]) * progress
            interpolated_joints.append(interpolated)
        
        # 実際のロボット位置
        actual_pos = coords.joints_to_robot_position(interpolated_joints)
        actual_pixel = coords.robot_to_pixel_coords(actual_pos[0], actual_pos[1])
        
        # ペン先位置
        pen_tip_pos = coords.joints_to_pen_tip_position(interpolated_joints)
        pen_tip_pixel = coords.robot_to_pixel_coords(pen_tip_pos[0], pen_tip_pos[1])
        
        # 誤差計算
        actual_error = math.sqrt(
            (actual_pixel[0] - target_pixel[0])**2 + 
            (actual_pixel[1] - target_pixel[1])**2
        )
        pen_tip_error = math.sqrt(
            (pen_tip_pixel[0] - target_pixel[0])**2 + 
            (pen_tip_pixel[1] - target_pixel[1])**2
        )
        
        print(f"Step {step:2d} ({progress:.2f}): 実際={actual_error:.2f}px, ペン先={pen_tip_error:.2f}px")
        max_error = max(max_error, max(actual_error, pen_tip_error))
    
    print(f"\n最大誤差: {max_error:.2f}px")
    
    if max_error > 5.0:
        print("❌ 補間処理で大きな誤差が発生しています")
    else:
        print("✅ 補間処理の誤差は許容範囲内です")
    
    return max_error


def main():
    """メイン分析関数"""
    print("🔍 ペン先位置ずれの詳細分析")
    print("座標変換は正常なので、実際の動作での問題を調査")
    print()
    
    # 詳細分析実行
    robot_err, pixel_err, contact_err = analyze_pen_positions()
    max_interp_err = identify_divergence_source()
    
    print("\n📋 分析結果まとめ")
    print("=" * 30)
    print(f"補間による位置誤差: {robot_err*1000:.2f}mm")
    print(f"実際位置のピクセル誤差: {pixel_err:.2f}px")
    print(f"Contact計算誤差: {contact_err:.2f}px")
    print(f"補間処理での最大誤差: {max_interp_err:.2f}px")
    
    print("\n🎯 結論")
    print("-" * 10)
    if max_interp_err > 5.0:
        print("❌ 補間処理でのずれが問題の主要因")
        print("推奨対策: 補間ステップ数を増やすか、補間アルゴリズムを改善")
    elif robot_err * 1000 > 10.0:  # 10mm以上
        print("❌ ロボット動作精度に問題")
        print("推奨対策: 逆運動学計算の精度向上")
    else:
        print("✅ 計算上の誤差は許容範囲内")
        print("ずれの原因は他の要因（実機の機械精度、キャリブレーション等）かもしれません")


if __name__ == "__main__":
    main()
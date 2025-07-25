#!/usr/bin/env python3
"""
作業空間補間（Workspace Interpolation）

関節空間での線形補間ではなく、作業空間（XYZ座標）での補間を行い、
より正確なペン先軌道を実現する。
"""

import math
from typing import List, Tuple


def workspace_interpolation(start_joints: List[float], end_joints: List[float], 
                          coord_system, steps: int = 50) -> List[List[float]]:
    """
    作業空間での補間を実行
    
    Args:
        start_joints: 開始ジョイント角度
        end_joints: 終了ジョイント角度  
        coord_system: 座標変換システム
        steps: 補間ステップ数
        
    Returns:
        補間されたジョイント角度のリスト
    """
    
    # 開始・終了位置を作業空間座標に変換
    start_pos = coord_system.joints_to_robot_position(start_joints)
    end_pos = coord_system.joints_to_robot_position(end_joints)
    
    interpolated_joints = []
    
    for i in range(steps + 1):
        progress = i / steps
        
        # 作業空間での線形補間
        interp_x = start_pos[0] + (end_pos[0] - start_pos[0]) * progress
        interp_y = start_pos[1] + (end_pos[1] - start_pos[1]) * progress  
        interp_z = start_pos[2] + (end_pos[2] - start_pos[2]) * progress
        
        # 補間された作業空間座標を関節角度に逆変換
        try:
            interp_joints = coord_system.robot_coords_to_joints(
                interp_x, interp_y, interp_z, start_joints
            )
            # Wrist joints maintain orientation
            interp_joints[4] = start_joints[4]  # Wrist 2
            interp_joints[5] = start_joints[5]  # Wrist 3
            
            interpolated_joints.append(interp_joints)
            
        except Exception as e:
            # 逆運動学が失敗した場合は関節空間補間にフォールバック
            fallback_joints = []
            for j in range(len(start_joints)):
                fallback = start_joints[j] + (end_joints[j] - start_joints[j]) * progress
                fallback_joints.append(fallback)
            interpolated_joints.append(fallback_joints)
    
    return interpolated_joints


def calculate_trajectory_error(joints_sequence: List[List[float]], 
                             target_positions: List[Tuple[float, float, float]],
                             coord_system) -> float:
    """
    軌道誤差を計算
    
    Args:
        joints_sequence: ジョイント角度のシーケンス
        target_positions: 目標位置のシーケンス
        coord_system: 座標変換システム
        
    Returns:
        平均軌道誤差（メートル）
    """
    total_error = 0.0
    
    for joints, target_pos in zip(joints_sequence, target_positions):
        actual_pos = coord_system.joints_to_robot_position(joints)
        error = math.sqrt(
            (actual_pos[0] - target_pos[0])**2 +
            (actual_pos[1] - target_pos[1])**2 +
            (actual_pos[2] - target_pos[2])**2
        )
        total_error += error
    
    return total_error / len(joints_sequence)


if __name__ == "__main__":
    # テスト用コード
    print("🔧 作業空間補間テスト")
    
    import sys
    import os
    sys.path.append('/Users/smallspring/programs/wanderline/robot/scripts')
    from corrected_coordinate_system import CorrectedCoordinateSystem
    
    coords = CorrectedCoordinateSystem()
    base_joints = [0.0, -1.2, -1.0, -1.5, 1.57 + 3.14159, 0.0]
    
    # テスト用の開始・終了ジョイント
    start_joints = base_joints.copy()
    end_joints = coords.robot_coords_to_joints(0.64, 0.0, 0.07, base_joints)
    end_joints[4] = base_joints[4]
    end_joints[5] = base_joints[5]
    
    print(f"開始ジョイント: {[f'{j:.3f}' for j in start_joints[:3]]}")
    print(f"終了ジョイント: {[f'{j:.3f}' for j in end_joints[:3]]}")
    
    # 作業空間補間実行
    workspace_path = workspace_interpolation(start_joints, end_joints, coords, steps=25)
    
    print(f"補間パス生成: {len(workspace_path)}ステップ")
    
    # 誤差計算
    start_pos = coords.joints_to_robot_position(start_joints)
    end_pos = coords.joints_to_robot_position(end_joints)
    
    # 目標軌道（直線）
    target_positions = []
    for i in range(len(workspace_path)):
        progress = i / (len(workspace_path) - 1)
        target_x = start_pos[0] + (end_pos[0] - start_pos[0]) * progress
        target_y = start_pos[1] + (end_pos[1] - start_pos[1]) * progress
        target_z = start_pos[2] + (end_pos[2] - start_pos[2]) * progress
        target_positions.append((target_x, target_y, target_z))
    
    error = calculate_trajectory_error(workspace_path, target_positions, coords)
    print(f"平均軌道誤差: {error*1000:.2f}mm")
    
    print("✅ 作業空間補間テスト完了")
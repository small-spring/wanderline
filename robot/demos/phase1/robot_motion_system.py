import math
from abc import ABC, abstractmethod
from geometry_msgs.msg import PointStamped

# TODO: REFACTORING NEEDED - Mixed Responsibilities
# このファイルは複数の責務が混在しており、将来的に分離が必要:
# 1. 座標変換 (pixel ↔ robot coords)
# 2. 逆運動学計算 (IK solvers) 
# 3. TF2統合 (TF2 transforms)
# 4. 動作計画 (motion planning context)
# 5. ロボット物理パラメータ (UR5e specs)
# 
# 理想的には以下のファイルに分離:
# - coordinate_transforms.py (座標変換)
# - inverse_kinematics.py (IK + strategies)
# - motion_planner.py (動作計画統合)
# - robot_parameters.py (物理パラメータ)

class IKStrategy(ABC):
    """逆運動学ストラテジーの抽象インターフェース"""
    
    @abstractmethod
    def pixel_to_joints(self, pixel_x: float, pixel_y: float, base_joints: list) -> list:
        """ピクセル座標から関節角度を計算"""
        pass

class Wrist3BackwardStrategy(IKStrategy):
    """wrist3位置から逆算する逆運動学実装"""
    
    def __init__(self, coord_system):
        self.coord_system = coord_system
    
    def pixel_to_joints(self, pixel_x: float, pixel_y: float, base_joints: list) -> list:
        """現在の _tf2_iterative_ik 実装をそのまま移植"""
        try:
            # Step 1: 目標ペン先位置を取得
            target_pen_x, target_pen_y, target_pen_z = self.coord_system.pixel_to_robot_coords(
                pixel_x, pixel_y, pen_down=True
            )
            
            # Step 2: ペン先からwrist3位置を逆算
            tool_flange_offset = 0.0825
            wrist3_target_x = target_pen_x
            wrist3_target_y = target_pen_y
            wrist3_target_z = target_pen_z + self.coord_system.pen_length + tool_flange_offset
            
            print(f"🎯 Target pen tip: ({target_pen_x:.3f}, {target_pen_y:.3f}, {target_pen_z:.3f})")
            print(f"📍 Calculated wrist3: ({wrist3_target_x:.3f}, {wrist3_target_y:.3f}, {wrist3_target_z:.3f})")
            
            # Step 3: wrist3位置で逆運動学計算
            current_joints = self.coord_system.wrist3_coords_to_joints(
                wrist3_target_x, wrist3_target_y, wrist3_target_z, base_joints
            )
            
            # Step 4: wrist関節を固定
            current_joints[4] = -1.57  # wrist_2: -90°
            current_joints[5] = 0.0    # wrist_3: 回転なし
            
            # Step 5: TF2で検証
            if self.coord_system.tf2_is_enabled:
                tf2_pen_tip = self.coord_system._tf2_joints_to_pen_tip_position(current_joints)
                
                error_x = abs(tf2_pen_tip[0] - target_pen_x)
                error_y = abs(tf2_pen_tip[1] - target_pen_y)  
                error_z = abs(tf2_pen_tip[2] - target_pen_z)
                total_error = error_x + error_y + error_z
                
                print(f"🔍 TF2 actual pen tip: ({tf2_pen_tip[0]:.3f}, {tf2_pen_tip[1]:.3f}, {tf2_pen_tip[2]:.3f})")
                print(f"📏 Position error: {total_error:.4f}m")
                
                if total_error > 0.01:
                    print("⚠️  Large positioning error detected")
                else:
                    print("✅ Pen tip positioning successful!")
            
            return current_joints
            
        except Exception as e:
            print(f"❌ Wrist3Backward Strategy failed: {e}")
            return self.coord_system._fallback_pixel_to_joints(pixel_x, pixel_y, base_joints)

# ROBOT MOTION SYSTEM (Renamed from corrected_coordinate_system.py)
# TODO: TF2を利用した座標系の管理システムへの移行

class RobotMotionSystem:
    """Corrected coordinate system with accurate kinematics."""

    def __init__(self, config=None, tf_buffer=None):
        # Load configuration file.
        if config is None:
            raise ValueError("❌ Configuration is required for CorrectedCoordinateSystem")
        
        # Canvas parameters from config
        canvas_config = config['canvas']
        self.canvas_position = canvas_config['position']
        self.canvas_size = canvas_config['physical_size']
        self.pixel_width = canvas_config['pixel_width']
        self.pixel_height = canvas_config['pixel_height']
        self.contact_height = canvas_config['contact_height']
        self.safe_height = canvas_config['safe_height']
        
        # Pen parameters from config
        self.pen_length = config['pen']['length']

        # TODO: HARDCODE REMOVAL - UR5e parameters should come from robot spec config
        self.base_height = 0.163      # HARDCODE: UR5e spec
        self.shoulder_offset = 0.138  # HARDCODE: UR5e spec  
        self.upper_arm = 0.425        # HARDCODE: UR5e spec
        self.forearm = 0.392          # HARDCODE: UR5e spec

        # new: tf2を利用した座標変換
        self.tf_buffer = tf_buffer
        self.tf2_is_enabled = tf_buffer is not None

        if self.tf2_is_enabled:
            print("✅ RobotMotionSystem with TF2 enhancement enabled")
        else:
            print("⚠️  RobotMotionSystem: TF2 disabled, using manual kinematics only")
        
        # Strategy Pattern: IKストラテジー初期化
        self.ik_strategy = Wrist3BackwardStrategy(self)
        print(f"🔧 Using IK Strategy: Wrist3Backward")

    

    def pixel_to_robot_coords(self, pixel_x, pixel_y, pen_down=True):
        """
        Convert pixel coordinates on canvas to robot coordinates.

        Input:
        - pixel_x: X coordinate in "pixel frame"
        - pixel_y: Y coordinate in "pixel frame"
        - pen_down: True if pen is down, False if pen is up
        Output:
        - (robot_x, robot_y, robot_z): Robot coordinates in "robot base frame"
        """
        # Use same scale for both X and Y to maintain circle proportions
        # TODO: remove pixel coordinates dependency
        # TODO: use TF2 system for coordinate transformations
        scale = self.canvas_size / self.pixel_width
        
        canvas_x = (pixel_x - self.pixel_width / 2) * scale
        canvas_y = (pixel_y - self.pixel_height / 2) * scale
        
        robot_x = self.canvas_position[0] + canvas_x
        robot_y = self.canvas_position[1] + canvas_y
        
        robot_z = self.canvas_position[2] + (self.contact_height if pen_down else self.safe_height)
        
        return robot_x, robot_y, robot_z
        
    def wrist3_coords_to_joints(self, wrist3_x, wrist3_y, wrist3_z, base_joints):
        """
        Convert wrist3 coordinates to joint positions using inverse kinematics.
        Input:
        - wrist3_x: X coordinate of wrist3 in "robot base frame"
        - wrist3_y: Y coordinate of wrist3 in "robot base frame"
        - wrist3_z: Z coordinate of wrist3 in "robot base frame"
        - base_joints: Current joint positions (base, shoulder, elbow, wrist1, wrist2, wrist3): angles in radians
        Output:
        - joints: New joint positions (base, shoulder, elbow) in "robot base frame"
        """
        # Calculate base rotation
        target_pan = math.atan2(wrist3_y - self.shoulder_offset, wrist3_x)
        
        # Calculate distance and angles
        dx = wrist3_x - 0
        dy = wrist3_y - self.shoulder_offset
        horizontal_distance = math.sqrt(dx*dx + dy*dy)
        dz = wrist3_z - self.base_height
        target_distance = math.sqrt(horizontal_distance*horizontal_distance + dz*dz)
        
        # 2-link inverse kinematics
        max_reach = self.upper_arm + self.forearm
        if target_distance > max_reach:
            target_distance = max_reach * 0.95
            
        cos_shoulder = (self.upper_arm**2 + target_distance**2 - self.forearm**2) / (2 * self.upper_arm * target_distance)
        cos_shoulder = max(-1, min(1, cos_shoulder))
        
        angle_to_target = math.atan2(dz, horizontal_distance)
        shoulder_angle = angle_to_target - math.acos(cos_shoulder)
        
        cos_elbow = (self.upper_arm**2 + self.forearm**2 - target_distance**2) / (2 * self.upper_arm * self.forearm)
        cos_elbow = max(-1, min(1, cos_elbow))
        elbow_angle = math.pi - math.acos(cos_elbow)
        
        joints = base_joints.copy()
        joints[0] = target_pan
        joints[1] = shoulder_angle
        joints[2] = elbow_angle
        
        return joints

    def wrist3_coords_to_joints_fixed_shoulder(self, wrist3_x, wrist3_y, wrist3_z, base_joints, fixed_shoulder_angle):
        """
        Convert wrist3 coordinates to joint positions with fixed shoulder angle.
        Input:
        - wrist3_x, wrist3_y, wrist3_z: Target wrist3 position
        - base_joints: Current joint positions
        - fixed_shoulder_angle: Fixed shoulder angle (joint1)
        Output:
        - joints: New joint positions with fixed shoulder
        """
        # Calculate base rotation (same as original)
        target_pan = math.atan2(wrist3_y - self.shoulder_offset, wrist3_x)
        
        # Calculate target position relative to shoulder joint
        dx = wrist3_x - 0
        dy = wrist3_y - self.shoulder_offset  
        horizontal_distance = math.sqrt(dx*dx + dy*dy)
        dz = wrist3_z - self.base_height
        
        # With fixed shoulder angle, calculate required elbow position
        # Shoulder joint position
        shoulder_x = self.upper_arm * math.cos(fixed_shoulder_angle) * math.cos(target_pan)
        shoulder_y = self.shoulder_offset + self.upper_arm * math.cos(fixed_shoulder_angle) * math.sin(target_pan)
        shoulder_z = self.base_height + self.upper_arm * math.sin(fixed_shoulder_angle)
        
        # Distance from shoulder to target wrist3
        to_target_x = wrist3_x - shoulder_x
        to_target_y = wrist3_y - shoulder_y
        to_target_z = wrist3_z - shoulder_z
        forearm_distance = math.sqrt(to_target_x**2 + to_target_y**2 + to_target_z**2)
        
        # Check if target is reachable with forearm
        if forearm_distance > self.forearm:
            print(f"⚠️  Target unreachable with fixed shoulder: distance={forearm_distance:.3f}m > forearm={self.forearm:.3f}m")
            forearm_distance = self.forearm * 0.95
        
        # Calculate elbow angle to reach target
        # Vector from shoulder to wrist3 in shoulder coordinate system
        forearm_angle_3d = math.atan2(to_target_z, math.sqrt(to_target_x**2 + to_target_y**2))
        elbow_angle = fixed_shoulder_angle + forearm_angle_3d - fixed_shoulder_angle  # Relative to shoulder
        
        joints = base_joints.copy()
        joints[0] = target_pan
        joints[1] = fixed_shoulder_angle  # Fixed!
        joints[2] = elbow_angle
        
        print(f"🔒 Fixed shoulder IK: pan={target_pan:.3f}, shoulder={fixed_shoulder_angle:.3f}(fixed), elbow={elbow_angle:.3f}")
        
        return joints
        
    def joints_to_wrist3_position(self, joints):
        """
        Forward kinematics: convert joint positions to wrist3 position.
        Input:
        - joints: Joint positions (base, shoulder, elbow, wrist1, wrist2, wrist3): angles in radians
        Output:
        - (wrist3_x, wrist3_y, wrist3_z): Wrist3 position in "robot base frame"
        """
        q1, q2, q3 = joints[0], joints[1], joints[2]
        
        c1, s1 = math.cos(q1), math.sin(q1)
        
        # Position after upper arm
        upper_end_x = self.upper_arm * math.cos(q2) * c1
        upper_end_y = self.shoulder_offset + self.upper_arm * math.cos(q2) * s1
        upper_end_z = self.base_height + self.upper_arm * math.sin(q2)
        
        # Position after forearm
        forearm_angle = q2 + q3
        end_x = upper_end_x + self.forearm * math.cos(forearm_angle) * c1
        end_y = upper_end_y + self.forearm * math.cos(forearm_angle) * s1
        end_z = upper_end_z + self.forearm * math.sin(forearm_angle)
        
        return (end_x, end_y, end_z)
    
    def joints_to_tool_flange_position(self, joints):
        """
        Convert joint positions to tool_flange position (tool0 frame).
        Input:
        - joints: Joint positions (base, shoulder, elbow, wrist1, wrist2, wrist3): angles in radians
        Output:
        - (tool_flange_x, tool_flange_y, tool_flange_z): Tool_flange position in "robot base frame"
        """
        # First get wrist_3 position (end-effector)
        wrist3_pos = self.joints_to_wrist3_position(joints)
        
        # TODO: HARDCODE REMOVAL - UR5e tool_flange offset should be in robot specs
        # UR5e tool_flange is 82.5mm from wrist_3 along the Z-axis
        # With current joint configuration, tool_flange extends toward canvas
        tool_flange_offset = 0.0825  # HARDCODE: UR5e spec - 82.5mm
        
        # Calculate tool_flange position
        # For UR5e, tool_flange extends along wrist_3 Z-axis
        # Joint 4 (wrist_2) affects the tool orientation
        q4 = joints[4]  # wrist_2_joint
        q5 = joints[5]  # wrist_3_joint
        
        # Tool_flange offset in base frame coordinates
        # Simplified: assume tool points in same direction as wrist_3
        tool_flange_x = wrist3_pos[0]
        tool_flange_y = wrist3_pos[1] 
        tool_flange_z = wrist3_pos[2] - tool_flange_offset  # Tool_flange is below wrist_3
        
        return (tool_flange_x, tool_flange_y, tool_flange_z)
    
    def joints_to_pen_tip_position(self, joints):
        """
        Convert joint positions to actual pen tip position.
        TF2-enhanced version that fixes tool orientation issues.
        """
        if self.tf2_is_enabled:
            return self._tf2_joints_to_pen_tip_position(joints)
        else:
            return self._manual_joints_to_pen_tip_position(joints)

    def _tf2_joints_to_pen_tip_position(self, joints):
        """TF2-based pen tip calculation - fixes orientation issue"""
        try:
            import rclpy.time
            import rclpy.duration
            
            # Create pen tip point in tool0 coordinate system
            pen_tip_in_tool0 = PointStamped()
            pen_tip_in_tool0.header.frame_id = "tool0"
            pen_tip_in_tool0.header.stamp = rclpy.time.Time().to_msg()
            
            # Pen extends in tool0's +Z direction (handles orientation automatically!)
            pen_tip_in_tool0.point.x = 0.0
            pen_tip_in_tool0.point.y = 0.0
            pen_tip_in_tool0.point.z = self.pen_length
            
            # Transform to base_link frame
            pen_tip_in_base = self.tf_buffer.transform(
                pen_tip_in_tool0, 
                "base_link",
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            result = (
                pen_tip_in_base.point.x,
                pen_tip_in_base.point.y,
                pen_tip_in_base.point.z
            )
            
            # 比較ログ出力
            manual_result = self._manual_joints_to_pen_tip_position(joints)
            diff = [abs(result[i] - manual_result[i]) for i in range(3)]
            
            if any(d > 0.05 for d in diff):  # 5cm以上の差の場合のみログ出力
                print(f"🔍 Large pen tip difference: TF2={[f'{x:.3f}' for x in result]} Manual={[f'{x:.3f}' for x in manual_result]}")
                print(f"📏 Diff: X={diff[0]:.3f}m Y={diff[1]:.3f}m Z={diff[2]:.3f}m")
            
            return result
            
        except Exception as e:
            print(f"❌ TF2 pen tip calculation failed, using manual: {e}")
            return self._manual_joints_to_pen_tip_position(joints)

    def _manual_joints_to_pen_tip_position(self, joints):
        """Original manual calculation (with known orientation issues)"""
        # First get tool_flange position
        tool_flange_pos = self.joints_to_tool_flange_position(joints)
        
        total_pen_extension = self.pen_length    

        # Calculate pen tip position
        tip_x = tool_flange_pos[0]
        tip_y = tool_flange_pos[1]
        tip_z = tool_flange_pos[2] - total_pen_extension  # tool_flangeからさらに下方向
        
        return (tip_x, tip_y, tip_z)

    def tf2_guided_pixel_to_joints(self, pixel_x, pixel_y, base_joints):
        """
        Context method: IKストラテジーに委譲
        """
        return self.ik_strategy.pixel_to_joints(pixel_x, pixel_y, base_joints)

    # Strategy Pattern導入により、_tf2_iterative_ikは Wrist3BackwardStrategy に移行

    def _fallback_pixel_to_joints(self, pixel_x, pixel_y, base_joints):
        """既存の計算方法（フォールバック用）"""
        print(f"⚠️  Using fallback calculation for pixel ({pixel_x:.1f}, {pixel_y:.1f})")
        
        robot_x, robot_y, robot_z = self.pixel_to_robot_coords(pixel_x, pixel_y, pen_down=True)
        
        # tool_flange位置計算
        tool_flange_x = robot_x
        tool_flange_y = robot_y
        tool_flange_z = robot_z + self.pen_length
        
        # 関節角度計算
        joints = self.wrist3_coords_to_joints(tool_flange_x, tool_flange_y, tool_flange_z, base_joints)
        joints[4] = -1.57  # wrist_2: -90° (front-side approach)
        joints[5] = 0.0    # wrist_3
        
        print(f"📐 Fallback joints: {[f'{j:.3f}' for j in joints]}")
        return joints
        
    def robot_to_pixel_coords(self, robot_x, robot_y):
        """
        Convert robot coordinates back to pixel coordinates.
        Input:
        - robot_x: X coordinate in "robot base frame"
        - robot_y: Y coordinate in "robot base frame"
        Output:
        - (pixel_x, pixel_y): Pixel coordinates in "pixel frame"
        """
        # Use same scale for both X and Y to maintain circle proportions
        scale = self.canvas_size / self.pixel_width
        
        # Transform from robot to canvas coordinate system
        canvas_x = robot_x - self.canvas_position[0]
        canvas_y = robot_y - self.canvas_position[1]
        
        # Convert canvas coordinates to pixel coordinates
        pixel_x = canvas_x / scale + self.pixel_width / 2
        pixel_y = canvas_y / scale + self.pixel_height / 2
        
        return pixel_x, pixel_y

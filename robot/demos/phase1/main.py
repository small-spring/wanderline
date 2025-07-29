#!/usr/bin/env python3
"""
Phase 1 Main: Minimal Robot Circle Drawing Demo

VNC environment test implementation.
Integrates Phase 1 components with existing robot control system.
"""
# rclpy: ROS 2 Python client library
import rclpy 
from rclpy.node import Node
import rclpy.time
import rclpy.duration
from geometry_msgs.msg import PointStamped

# Message libraries
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# tf2_ros: ROS 2 TF (TF: Transform Frame) library for coordinate transformations 
from tf2_ros import TransformListener, Buffer
import tf2_geometry_msgs  # Required for PointStamped message transform support.
# although not explicitly used, it's good practice to import it for future use

# Import necessary libraries
import time

# Import Phase 1 components
from system_state import create_default_system_state, ContactPoint
from coordinate_calculator import create_coordinate_calculator
from config_loader import ConfigLoader

# Import Canvas Preview Window (conditional for environments without OpenCV)
try:
    from canvas_preview import create_preview_window
    # path: 
    CANVAS_PREVIEW_AVAILABLE = True
except ImportError:
    print("⚠️  Canvas Preview unavailable (OpenCV not found)")
    CANVAS_PREVIEW_AVAILABLE = False


from canvas_coordinate_system import CanvasCoordinateSystem
# wanderline/robot/scripts/canvas_coordinate_system.py

from robot_motion_system import RobotMotionSystem
# wanderline/robot/demos/phase1/robot_motion_system.py

# Import message types for RViz visualization (todo: move to separate file later)
from phase1_robot_drawing.msg import PenState
# robot/demos/phase1/msgs/PenState.msg
from rviz_view_node import RvizViewNode

class RobotDrawerNode(Node):
    """
    Phase 1 Robot Drawing Demo.
    Extends existing robot control with Phase 1 coordinate calculation.
    """
    
    def __init__(self):
        super().__init__('phase1_robot_drawer')
        
        # ----------- Load configuration -----------
        config_loader = ConfigLoader('config.yaml')
        self.config = config_loader.load_config()
        # wanderline/robot/demos/phase1/config.yaml
        # Pen physical specifications (from config)
        pen_config = self.config.get('pen')
        self.pen_length = pen_config.get('length') 
        # Movement control (configurable)
        movement_config = self.config.get('movement', {})
        self.interpolation_steps = movement_config.get('interpolation_steps', 25)
        
        # ------- Initialize Robot Posture -------
        # Initialize Phase 1 components
        self.system_state = create_default_system_state()
            # SystemState(@dataclass): ロボット描画システム全体の状態管理
            # - robot_joint_positions: Dict[str, float] - ジョイント角度辞書
            # - calculated_wrist3_position: Tuple[float, float, float] - wrist3位置(base_link座標系)
            # - contact_history: List[ContactPoint] - 接触履歴
            # - completion_percentage: float - 描画完了率
            # - target_shape: TargetCircle - 目標図形（中央(400,300) in pixel frame、半径80の円）

        # TF listener for accurate robot position (initialize early for CorrectedCoordinateSystem)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.coord_calculator = create_coordinate_calculator(self.config['drawing']) 
        # self.config['drawing'] とは何？ 描画ターゲットの中心点の座標など
        self.canvas_system = CanvasCoordinateSystem(self.config)
        self.motion_system = RobotMotionSystem(self.config, self.tf_buffer)  # TF2-enhanced motion system
        
        # ------- Initialize Canvas Preview Window (if available) -------
        self.canvas_preview = None
        if CANVAS_PREVIEW_AVAILABLE:
            # Check if VNC display is available
            import os
            display = os.environ.get('DISPLAY', '')
            self.get_logger().info(f"🔍 Display environment: DISPLAY={display}")
            
            try:
                self.canvas_preview = create_preview_window(self.config)
                # Check if display is actually available after initialization
                if hasattr(self.canvas_preview, 'display_available') and self.canvas_preview.display_available:
                    self.get_logger().info("🎨 Canvas Preview Window (Window 2) initialized successfully")
                else:
                    self.get_logger().info("💻 Canvas Preview Window disabled (headless environment)")
            except Exception as e:
                self.get_logger().warning(f"⚠️  Canvas Preview failed to initialize: {e}")
                self.canvas_preview = None
        
        # ----------- Initialize Robot Control Publishers -----------
        # Robot control (from existing robot_draw_circle.py)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Robot trajectory control for actual movement
        self.trajectory_pub = self.create_publisher(
            JointTrajectory, 
            '/scaled_joint_trajectory_controller/joint_trajectory', 
            10
        )
        
        # Pen state publisher for RViz visualization -> RvizViewNode
        self.penstate_pub = self.create_publisher(PenState, '/pen_state', 10)
        
        # TF listener already initialized above for CorrectedCoordinateSystem
        
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint', 
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        
        # Robot state
        # TODO: canvas位置から初期姿勢の計算関数を作る
        # self.base_joints = self.motion_system.calculate_optimal_base_joints(self.config['canvas']['position'])
        self.base_joints = [0.0, -1.0, 1.3, -1.57, -1.57, 0.0]  # Balanced posture: shoulder -57°, elbow 74°

        self.current_joints = self.base_joints.copy()
        self.target_joints = self.base_joints.copy() # ?
        
        # --------- STEP 1: Pen Down Initialization ---------
        self.current_pixel_position = (400, 300)  # Start at canvas center
        self.pen_down(400, 300)  # Set pen down at center to start drawing # TODO: not working properly
        self.drawing_active = True

        self.current_step = 0
        
        self.get_logger().info(f"🖊️  Canvas at Z={self.canvas_system.canvas_position[2]:.3f}m")
        self.get_logger().info("🎯 Phase 1 Robot Drawer started!")
        self.get_logger().info(f"📍 Starting at canvas center: {self.current_pixel_position}")
        
        # --------- STEP 2: Timer for Drawing Loop ---------
        update_rate = 1.0 / self.config['robot']['update_rate']
        self.timer = self.create_timer(update_rate, self.drawing_step)
        

    
    def pen_down(self, pixel_x:float, pixel_y:float):
        """
        Set pen down at specified pixel coordinates.
        Input:
        - pixel_x: X coordinate in "pixel frame"
        - pixel_y: Y coordinate in "pixel frame"
        """
        self.get_logger().info(f"🖊️  Pen down at pixel ({pixel_x}, {pixel_y})")
        
        # Step 1: Convert "pixel frame" to "robot base frame"
        robot_x, robot_y, robot_z = self.motion_system.pixel_to_robot_coords(
            pixel_x, pixel_y, pen_down=True
        )
        # Step 2: Canvas表面の正確な高さを取得
        canvas_z = self.canvas_system.canvas_position[2]
        pen_tip_target_z = canvas_z + 0.001  # Canvas表面に軽く接触 (1mm)

        # Step 3: ペン先位置からtool flange位置を逆算
        tool_flange_x = robot_x
        tool_flange_y = robot_y
        tool_flange_z = pen_tip_target_z + self.pen_length  # ペン長分上に移動

        # Step 4: 関節角度を計算
        target_joints = self.motion_system.wrist3_coords_to_joints(
            tool_flange_x, tool_flange_y, tool_flange_z, self.base_joints
        )
        target_joints[4] = 1.57  # Wrist 2: ペン下向き（Canvas向き）
        target_joints[5] = 0.0    # Wrist 3: 回転なし

        # Step 5: Send trajectory command to move robot
        self._send_joint_trajectory(target_joints, duration=1.0)
        
        # Update internal state
        self.current_joints = target_joints
        self.target_joints = target_joints
        self._publish_joint_state(target_joints)

        # Step 6: 位置を更新
        self.current_pixel_position = (pixel_x, pixel_y)
        # TODO: if分岐を追加。ペンがcontactしている時としていない時で分岐
        self.get_logger().info(f"✅ Pen contact established at canvas position ({pixel_x:.1f}, {pixel_y:.1f})")

    def drawing_step(self):
        """Main drawing loop step."""
        if not self.drawing_active:
            print("🚫 Drawing is not active. Skipping step.")
            return
            
        # ✅ Step 1: Get actual wrist3 position using current interpolated joints
        actual_wrist3_pos = self._joints_to_wrist3_position(self.current_joints)
        actual_pixel_pos = self.motion_system.robot_to_pixel_coords(actual_wrist3_pos[0], actual_wrist3_pos[1])
        
        # Validate pixel coordinates before proceeding
        if (actual_pixel_pos[0] < 0 or actual_pixel_pos[0] > 800 or 
            actual_pixel_pos[1] < 0 or actual_pixel_pos[1] > 600):
            self.get_logger().warning(
                f"⚠️ Invalid pixel position: ({actual_pixel_pos[0]:.1f}, {actual_pixel_pos[1]:.1f}) - using fallback"
            )
            # Fallback: use current stored position
            actual_pixel_pos = self.current_pixel_position
        
        # Step 2: Calculate next coordinate based on actual robot position
        next_coord, drawing_complete = self.coord_calculator.calculate_next_coordinate(
            actual_pixel_pos, self.system_state
        )
        
        if drawing_complete:
            self.get_logger().info("🎉 Circle drawing completed! Starting new circle...")
            # Reset for continuous circles
            self.system_state = create_default_system_state()
            self.current_pixel_position = (400, 300)  # Reset to center
            
        if next_coord is None:
            self.get_logger().warning("⚠️ No next coordinate calculated")
            return
            
        # Step 2: Convert pixel coordinate to robot joint positions
        target_joints = self._pixel_to_joint_position(next_coord[0], next_coord[1])
        
        # Step 3: Execute smooth movement to target
        self._execute_smooth_movement(target_joints)
        
        # Step 4: Simulate contact detection and update system state
        self._simulate_contact_detection(next_coord)
        
        # Step 5: Update current position
        self.current_pixel_position = next_coord
        
        # Log progress
        progress = self.system_state.completion_percentage # todo: progress is nonsense now. remove them.
        contact_count = self.system_state.get_contact_count()
        self.get_logger().info(
            f"📍 Moving to ({next_coord[0]:.1f}, {next_coord[1]:.1f}) | "
            f"Progress: {progress:.1%} | Contacts: {contact_count}"
        )
    
    def _pixel_to_joint_position(self, pixel_x: float, pixel_y: float) -> list:
        """Convert pixel coordinates to joint positions using TF2-guided calculation."""
        # TF2ガイド付き計算を使用
        joints = self.motion_system.tf2_guided_pixel_to_joints(
            pixel_x, pixel_y, self.base_joints
        )
        
        return joints
    
    def _execute_smooth_movement(self, target_joints: list):
        """Execute movement to target joints (with optional interpolation)."""
        # === INTERPOLATED MOVEMENT (Original behavior) ===
        if self.current_step >= self.interpolation_steps:
            # Start new movement
            self.current_joints = self.target_joints.copy()
            self.target_joints = target_joints
            self.current_step = 0
        
        # Interpolate between current and target
        progress = self.current_step / self.interpolation_steps
        interpolated_joints = []
        
        for i in range(len(self.current_joints)):
            interpolated = self.current_joints[i] + (self.target_joints[i] - self.current_joints[i]) * progress
            interpolated_joints.append(interpolated)
        
        active_joints = interpolated_joints
        self.current_step += 1
            
        # Common processing for both modes
        # Publish joint state
        self._publish_joint_state(active_joints)
        
        # Update robot state in system state
        joint_dict = dict(zip(self.joint_names, active_joints))
        wrist3_pos = self._joints_to_wrist3_position(active_joints)
        self.system_state.update_robot_position(joint_dict, wrist3_pos)
        

        # Get transform from base_link to tool0 (pen tip)
        try:
            transform = self.tf_buffer.lookup_transform(
                "base_link", "tool0",
                rclpy.time.Time(), 
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
        except Exception as e:
            self.get_logger().error(f"❌ TF lookup failed: {e}")
            return
        # todo: Handle exceptions if transform lookup fails

        # position of pen base = tool0 position
        pen_base_pos = (
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        )

        # compute pen tip position
        pen_tip_in_tool0 = PointStamped()
        pen_tip_in_tool0.header.frame_id = "tool0"
        pen_tip_in_tool0.header.stamp = rclpy.time.Time().to_msg()
        pen_tip_in_tool0.point.x = 0.0  # Pen tip at tool
        pen_tip_in_tool0.point.y = 0.0
        pen_tip_in_tool0.point.z = self.pen_length  # Pen length above tool flange 
        try:
            # Transform pen tip to base_link frame
            pen_tip_in_base_link = self.tf_buffer.transform(
                pen_tip_in_tool0, "base_link", 
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
        except Exception as e:
            self.get_logger().error(f"❌ TF transform failed: {e}")
            return

        pen_tip_pos = (
            pen_tip_in_base_link.point.x,
            pen_tip_in_base_link.point.y,
            pen_tip_in_base_link.point.z
        )
        # todo: Handle exceptions if transform fails

        self._publish_pen_state(
            pen_tip_pos=pen_tip_pos, 
            pen_body_pos=pen_base_pos, 
            is_contact=True # TODO: Determine actual contact state
        )

    
    def _joints_to_wrist3_position(self, joints: list) -> tuple:
        """Convert joint positions to wrist3 position using corrected forward kinematics."""
        # Use corrected forward kinematics
        return self.motion_system.joints_to_wrist3_position(joints)
        
    
    def _simulate_contact_detection(self, pixel_coord: tuple):
        """Simulate contact detection and update system state."""
        # ✅ CONSISTENT: Use pen tip position (same as trail and display)
        pen_tip_pos = self.motion_system.joints_to_pen_tip_position(self.current_joints) # ok
        
        # Calculate pen-canvas contact point using pen tip position
        pen_contact_pos = self._calculate_trail_display_position(pen_tip_pos)
        
        # Convert contact position to pixel coordinates for canvas preview
        contact_pixel = self.motion_system.robot_to_pixel_coords(
            pen_contact_pos[0], pen_contact_pos[1]
        )
        
        contact_point = ContactPoint(
            position_3d=pen_contact_pos,  # Use pen-canvas contact point
            position_2d=(int(contact_pixel[0]), int(contact_pixel[1])),  # Canvas contact pixels
            timestamp=time.time(),
            stroke_id=self.system_state.current_stroke_id
        )
        
        # Add to system state
        self.system_state.add_contact_point(contact_point)
        
        # Update Canvas Preview Window with consistent contact point
        if self.canvas_preview:
            self.canvas_preview.update_from_contact(contact_point) # ここで止まっている
        
        # Update completion percentage # TODO: 取り除く、もしくは実際の進捗計算を実装する
        # progress = self.coord_calculator._calculate_circle_progress(self.system_state)
        # self.get_logger().info(f"[DEBUG] Progress: {progress:.2%}")
        # self.system_state.update_completion_percentage(progress)
    
    def _publish_joint_state(self, joint_positions: list):
        """Publish joint state message."""
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = joint_positions
        msg.velocity = [0.0] * len(self.joint_names) # todo: dummy velocities
        msg.effort = [0.0] * len(self.joint_names) # todo: dummy efforts

        self.joint_pub.publish(msg)
        self.get_logger().info(f"📡 Joint state published: {[f'{j:.3f}' for j in joint_positions]}")
    
    def _send_joint_trajectory(self, target_joints: list, duration: float = 2.0):
        """Send joint trajectory command to move robot"""
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = target_joints
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1) * 1e9)
        
        msg.points = [point]
        self.trajectory_pub.publish(msg)
        
        self.get_logger().info(f"🚀 Trajectory command sent: duration={duration:.1f}s")

    def _publish_pen_state(self, pen_tip_pos: tuple, pen_body_pos: tuple, is_contact: bool):
        msg = PenState()
        msg.tip_position.x = pen_tip_pos[0]
        msg.tip_position.y = pen_tip_pos[1]
        msg.tip_position.z = pen_tip_pos[2]
        msg.base_position.x = pen_body_pos[0]
        msg.base_position.y = pen_body_pos[1]
        msg.base_position.z = pen_body_pos[2]
        msg.is_contact = is_contact
        self.penstate_pub.publish(msg)
    
    def _calculate_trail_display_position(self, pen_tip_pos: tuple) -> tuple:
        """Calculate trail display position for RViz visualization."""
        canvas_z = self.canvas_system.canvas_position[2]  # Canvas surface height
        
        # Project pen tip to canvas surface for trail visualization
        trail_x = pen_tip_pos[0]
        trail_y = pen_tip_pos[1] 
        trail_z = canvas_z + 0.003  # Slightly above canvas (3mm) for visibility
        
        return (trail_x, trail_y, trail_z)
    


def main(args=None):
    """Main function for Phase 1 robot drawing demo with dual nodes."""
    print("🚀 Starting Phase 1 Robot Drawing Demo!")
    print("📍 Drawing circle using Phase 1 coordinate calculation")
    print("👀 Watch in RViz - robot will draw a circle")
    print("🔄 Contact-based progress tracking enabled")
    
    rclpy.init(args=args)
    
    drawer = None
    rviz_viewer = None
    executor = None
    
    try:
        from rclpy.executors import SingleThreadedExecutor
        
        # Create both nodes
        drawer = RobotDrawerNode()
        rviz_viewer = RvizViewNode()
        # Use SingleThreadedExecutor for parallel execution
        executor = SingleThreadedExecutor()
        executor.add_node(drawer)
        executor.add_node(rviz_viewer)
        
        print("✅ Both nodes started successfully!")
        print("🎯 RobotDrawerNode: Drawing circle with Phase 1 algorithm...")
        print("🎨 RvizViewNode: Visualizing pen state in RViz...")
        print("💡 Use Ctrl+C to stop gracefully")
        
        executor.spin()
        # TODO: GUI専用スレッド実装
        # 理由: OpenCVスレッドセーフ問題の根本解決
        # 優先度: Low (現在のSingleThreadedExecutorで十分動作)
        # 実装方針: 
        #   - canvas_preview.pyにGUI専用スレッド追加
        #   - queueベースの非同期通信
        #   - MultiThreadedExecutor復活


    except KeyboardInterrupt:
        print("\n🛑 Shutting down...")
        
        # Canvas cleanup
        if drawer and hasattr(drawer, 'canvas_preview') and drawer.canvas_preview:
            try:
                drawer.canvas_preview.cleanup()
            except:
                pass
        
    except Exception as e:
        print(f"❌ Error: {e}")
        
    finally:
        # Quick cleanup
        try:
            if drawer:
                drawer.destroy_node()
            if rviz_viewer:
                rviz_viewer.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            import cv2
            cv2.destroyAllWindows()
        except:
            pass
        
        import os
        os._exit(0)


if __name__ == '__main__':
    main()
#!/usr/bin/env python3
"""
Phase 1 Main: Minimal Robot Circle Drawing Demo

VNC environment test implementation.
Integrates Phase 1 components with existing robot control system.
"""
# rclpy: ROS 2 Python client library
import rclpy 
from rclpy.node import Node

# Message libraries
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

# tf2_ros: ROS 2 TF (TF: Transform Frame) library for coordinate transformations 
from tf2_ros import TransformListener, Buffer
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

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
    CANVAS_PREVIEW_AVAILABLE = False
except ImportError:
    print("⚠️  Canvas Preview unavailable (OpenCV not found)")
    CANVAS_PREVIEW_AVAILABLE = False

# Import existing coordinate system
import sys
import os
# Support both local and VNC environments
if os.path.exists('/workspace/robot/scripts'):
    sys.path.append('/workspace/robot/scripts')  # VNC environment
else:
    sys.path.append('/Users/smallspring/programs/wanderline/robot/scripts')  # Local environment

from canvas_coordinate_system import CanvasCoordinateSystem
# wanderline/robot/scripts/canvas_coordinate_system.py

from corrected_coordinate_system import CorrectedCoordinateSystem
# wanderline/robot/scripts/corrected_coordinate_system.py

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
        
        # Load configuration
        config_loader = ConfigLoader('config.yaml')
        self.config = config_loader.load_config()
        # wanderline/robot/demos/phase1/config.yaml
        
        # Initialize Phase 1 components
        self.system_state = create_default_system_state()
        self.coord_calculator = create_coordinate_calculator(self.config['drawing'])
        self.canvas_system = CanvasCoordinateSystem(self.config)
        self.corrected_coords = CorrectedCoordinateSystem()  # Use corrected coordinate system
        
        # Initialize Canvas Preview Window (if available)
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
        
        # Robot control (from existing robot_draw_circle.py)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Pen state publisher for RViz visualization -> RvizViewNode
        self.penstate_pub = self.create_publisher(PenState, '/pen_state', 10)
        
        # TF listener for accurate robot position
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint', 
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        
        # Robot state
        self.base_joints = self.config['robot']['base_joints']
        self.current_joints = self.base_joints.copy()
        self.target_joints = self.base_joints.copy()
        
        # Drawing state
        self.current_pixel_position = (400, 300)  # Start at canvas center
        self.drawing_active = True
        
        # Movement control (configurable)
        movement_config = self.config.get('movement', {})
        

        self.interpolation_steps = movement_config.get('interpolation_steps', 25)
        self.current_step = 0
        
        # Pen physical specifications (from config)
        pen_config = self.config.get('pen', {})
        # self.pen_total_length = pen_config.get('total_length')  
        # self.pen_body_offset = pen_config.get('body_offset') 
        self.pen_tip_offset = pen_config.get('tip_offset')
        # self.pen_diameter = pen_config.get('diameter')        # 1.2cm default
        
        # Debug: Log pen configuration
        # self.get_logger().info(f"🖊️  Pen Config: Length={self.pen_total_length:.3f}m, TipOffset={self.pen_tip_offset:.3f}m")
        self.get_logger().info(f"🖊️  Canvas at Z={self.canvas_system.canvas_position[2]:.3f}m")
        
        # Timer for smooth movement
        update_rate = 1.0 / self.config['robot']['update_rate']
        self.timer = self.create_timer(update_rate, self.drawing_step)
        
        # Now that base_joints is initialized, publish canvas marker
        # self._publish_canvas_marker()
        
        self.get_logger().info("🎯 Phase 1 Robot Drawer started!")
        self.get_logger().info(f"📍 Starting at canvas center: {self.current_pixel_position}")
    
    
    def drawing_step(self):
        """Main drawing loop step."""
        if not self.drawing_active:
            return
            
        # ✅ Step 1: Get actual robot position using current interpolated joints
        actual_robot_pos = self._joints_to_robot_position(self.current_joints)
        actual_pixel_pos = self.corrected_coords.robot_to_pixel_coords(actual_robot_pos[0], actual_robot_pos[1])
        
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
            # ✅ Keep pen trail for continuous drawing visualization
            # self.pen_trail_points.clear()  # Disabled - keep trail visible
            # Don't stop drawing - continue with new circle
            
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
        """Convert pixel coordinates to joint positions considering pen tip offset."""
        # Step 1: Convert pixel to pen tip target position (Canvas面接触)
        pen_tip_target_x, pen_tip_target_y, _ = self.corrected_coords.pixel_to_robot_coords(
            pixel_x, pixel_y, pen_down=True
        )
        # Override Z coordinate to Canvas surface contact height
        canvas_z = self.canvas_system.canvas_position[2]  # Canvas surface at 5cm
        pen_tip_target_z = canvas_z + 0.001  # Just touch Canvas surface (1mm above)
        
        # Step 2: Calculate tool flange target position (reverse pen tip calculation)
        # BUG FIX: tool flangeから下方向にペンが延びるので、逆算では上方向に移動
        # Pen tip is pen_tip_offset BELOW tool flange, so tool flange should be ABOVE
        tool_flange_target_x = pen_tip_target_x
        tool_flange_target_y = pen_tip_target_y  
        tool_flange_target_z = pen_tip_target_z + self.pen_tip_offset  # Move tool flange UP by pen length
        
        # Step 3: Calculate joints for tool flange position
        joints = self.corrected_coords.robot_coords_to_joints(
            tool_flange_target_x, tool_flange_target_y, tool_flange_target_z, self.base_joints
        )
        joints[4] = 1.57 + 3.14159   # Wrist 2 perpendicular + 180° rotation (for canvas orientation)
        joints[5] = 0.0   # Wrist 3 no rotation
        
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
        robot_pos = self._joints_to_robot_position(active_joints)
        self.system_state.update_robot_position(joint_dict, robot_pos)
        
        # ✅ Both trail and pen tip use SAME pen tip position (真の一致)
        pen_tip_pos = self.corrected_coords.joints_to_pen_tip_position(active_joints)
        pen_base_pos = self._joints_to_robot_position(active_joints)
        self._publish_pen_state(
            pen_tip_pos=pen_tip_pos, 
            pen_body_pos=pen_base_pos, 
            is_contact=True # TODO: Determine actual contact state
        )
        # self._add_pen_trail_point(pen_tip_pos)  # Trail = ペン先位置
        # self._publish_pen_markers(pen_tip_pos)  # Pen tip = ペン先位置
        
        # ✅ DEBUG: Publish coordinate for external monitoring
        # self._publish_debug_coordinates(actual_robot_pos)
    
    def _joints_to_robot_position(self, joints: list) -> tuple:
        """Convert joint positions to robot position using corrected forward kinematics."""
        # Use corrected forward kinematics
        return self.corrected_coords.joints_to_robot_position(joints)
        
    
    def _simulate_contact_detection(self, pixel_coord: tuple):
        """Simulate contact detection and update system state."""
        # ✅ CONSISTENT: Use pen tip position (same as trail and display)
        pen_tip_pos = self.corrected_coords.joints_to_pen_tip_position(self.current_joints)
        
        # Calculate pen-canvas contact point using pen tip position
        pen_contact_pos = self._calculate_trail_display_position(pen_tip_pos)
        
        # Convert contact position to pixel coordinates for canvas preview
        contact_pixel = self.corrected_coords.robot_to_pixel_coords(
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
            self.canvas_preview.update_from_contact(contact_point)
        
        # Update completion percentage
        progress = self.coord_calculator._calculate_circle_progress(self.system_state)
        self.system_state.update_completion_percentage(progress)
    
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
    
    try:
        from rclpy.executors import MultiThreadedExecutor
        
        # Create both nodes
        drawer = RobotDrawerNode()
        rviz_viewer = RvizViewNode()
        
        # Use MultiThreadedExecutor for parallel execution
        executor = MultiThreadedExecutor()
        executor.add_node(drawer)
        executor.add_node(rviz_viewer)
        
        print("✅ Both nodes started successfully!")
        print("🎯 RobotDrawerNode: Drawing circle with Phase 1 algorithm...")
        print("🎨 RvizViewNode: Visualizing pen state in RViz...")
        
        executor.spin()
        
    except KeyboardInterrupt:
        print("\n🎨 Phase 1 demo stopped by user")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
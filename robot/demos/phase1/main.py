#!/usr/bin/env python3
"""
Phase 1 Main: Minimal Robot Circle Drawing Demo

VNC environment test implementation.
Integrates Phase 1 components with existing robot control system.
"""
# rclpy: ROS 2 Python client library
import rclpy 
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

# tf2_ros: ROS 2 TF (TF: Transform Frame) library for coordinate transformations 
from tf2_ros import TransformListener, Buffer
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

# Import necessary libraries
import time
import yaml
from pathlib import Path

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
        
        # Canvas visualization in RViz
        self.canvas_marker_pub = self.create_publisher(Marker, '/canvas_marker', 10)
        self.pen_trail_pub = self.create_publisher(MarkerArray, '/pen_trail', 10)
        self.pen_tip_pub = self.create_publisher(Marker, '/pen_tip', 10)  # End-effector as pen tip
        self.pen_body_pub = self.create_publisher(Marker, '/pen_body', 10)  # Pen body visualization
        
        # ✅ DEBUG: Add coordinate debugging publisher
        from geometry_msgs.msg import PointStamped
        
        self.debug_coords_pub = self.create_publisher(PointStamped, '/debug_coordinates', 10)
        
        self.pen_trail_points = []  # Store pen trail points
        
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
        self.pen_total_length = pen_config.get('total_length')  
        self.pen_body_offset = pen_config.get('body_offset') 
        self.pen_tip_offset = pen_config.get('tip_offset')
        self.pen_diameter = pen_config.get('diameter')        # 1.2cm default
        
        # Debug: Log pen configuration
        self.get_logger().info(f"🖊️  Pen Config: Length={self.pen_total_length:.3f}m, TipOffset={self.pen_tip_offset:.3f}m")
        self.get_logger().info(f"🖊️  Canvas at Z={self.canvas_system.canvas_position[2]:.3f}m")
        
        # Timer for smooth movement
        update_rate = 1.0 / self.config['robot']['update_rate']
        self.timer = self.create_timer(update_rate, self.drawing_step)
        
        # Now that base_joints is initialized, publish canvas marker
        self._publish_canvas_marker()
        
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
        progress = self.system_state.completion_percentage
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
        
        # ✅ CONSISTENT DESIGN: Both trail and pen tip use actual robot position
        actual_robot_pos = self._joints_to_robot_position(active_joints)
        
        # ✅ Both trail and pen tip use SAME pen tip position (真の一致)
        pen_tip_pos = self.corrected_coords.joints_to_pen_tip_position(active_joints)
        self._add_pen_trail_point(pen_tip_pos)  # Trail = ペン先位置
        self._publish_pen_markers(pen_tip_pos)  # Pen tip = ペン先位置
        
        # ✅ DEBUG: Publish coordinate for external monitoring
        self._publish_debug_coordinates(actual_robot_pos)
    
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
        msg.velocity = [0.0] * len(self.joint_names)
        msg.effort = [0.0] * len(self.joint_names)
        
        self.joint_pub.publish(msg)
    
    def _publish_canvas_marker(self):
        """Publish simple test canvas marker for RViz visualization."""
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "test_canvas"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        # UNIFIED: Use the SAME position as coordinate system
        canvas_pos = self.canvas_system.canvas_position
        marker.pose.position.x = canvas_pos[0]
        marker.pose.position.y = canvas_pos[1] 
        marker.pose.position.z = canvas_pos[2]
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        
        # Realistic canvas size (40cm x 40cm)
        marker.scale.x = 0.4  # 40cm width (matches CANVAS_PHYSICAL_SIZE)
        marker.scale.y = 0.4  # 40cm height
        marker.scale.z = 0.005  # 5mm thickness (realistic)
        
        # White canvas color (realistic drawing surface)
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 0.9  # Mostly opaque
        
        # Canvas persists until deleted
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        
        self.canvas_marker_pub.publish(marker)
        self.get_logger().info(f"🎯 UNIFIED Canvas at [{canvas_pos[0]:.1f}, {canvas_pos[1]:.1f}, {canvas_pos[2]:.1f}] - ALL systems use this position")
    
    def _publish_pen_markers(self, pen_tip_pos: tuple):
        """Publish pen tip marker at actual pen tip position."""
        # Publish pen tip at actual calculated position
        self._publish_pen_body(pen_tip_pos)
        # Note: pen_tip removed - trail on canvas surface shows drawing contact
    
    def _publish_pen_body(self, pen_tip_pos: tuple):
        """Publish pen attached to robot tool flange."""
        pen_body = Marker()
        pen_body.header.frame_id = "tool0"  # Attach to robot tool flange
        pen_body.header.stamp = self.get_clock().now().to_msg()
        pen_body.ns = "pen_body"
        pen_body.id = 0
        pen_body.type = Marker.CYLINDER  # Cylinder for visible pen axis
        pen_body.action = Marker.ADD
        
        # Position relative to tool flange (pen extends downward toward canvas)
        pen_body.pose.position.x = 0.0
        pen_body.pose.position.y = 0.0 
        pen_body.pose.position.z = -self.pen_body_offset  # Config-driven offset from tool flange
        
        # Orientation (pen points downward from tool flange)
        pen_body.pose.orientation.x = 0.0
        pen_body.pose.orientation.y = 0.0
        pen_body.pose.orientation.z = 0.0
        pen_body.pose.orientation.w = 1.0
        
        # Pen body dimensions (config-driven)
        pen_body.scale.x = self.pen_diameter     # Config-driven diameter
        pen_body.scale.y = self.pen_diameter     
        pen_body.scale.z = self.pen_total_length # Config-driven length
        
        # Pen color (dark blue pen body)
        pen_body.color.r = 0.0    
        pen_body.color.g = 0.2
        pen_body.color.b = 0.8    # Dark blue
        pen_body.color.a = 1.0    
        
        pen_body.lifetime.sec = 0
        pen_body.lifetime.nanosec = 0
        
        self.pen_body_pub.publish(pen_body)
    
    def _publish_pen_tip(self, robot_pos: tuple):
        """Publish pen tip marker at end-effector position (simulation contact point)."""
        pen_tip = Marker()
        pen_tip.header.frame_id = "base_link"
        pen_tip.header.stamp = self.get_clock().now().to_msg()
        pen_tip.ns = "pen_tip"
        pen_tip.id = 0
        pen_tip.type = Marker.SPHERE
        pen_tip.action = Marker.ADD
        
        # Position at end-effector (simulation contact point)
        pen_tip.pose.position.x = robot_pos[0]
        pen_tip.pose.position.y = robot_pos[1] 
        pen_tip.pose.position.z = robot_pos[2]
        pen_tip.pose.orientation.w = 1.0
        
        # Small sphere representing pen tip
        pen_tip.scale.x = 0.012   # 1.2cm diameter pen tip
        pen_tip.scale.y = 0.012   
        pen_tip.scale.z = 0.012   
        
        # Bright red for visibility (contact point)
        pen_tip.color.r = 1.0    
        pen_tip.color.g = 0.0
        pen_tip.color.b = 0.0
        pen_tip.color.a = 1.0    
        
        pen_tip.lifetime.sec = 0
        pen_tip.lifetime.nanosec = 0
        
        self.pen_tip_pub.publish(pen_tip)
    
    
    def _calculate_pen_canvas_contact(self, robot_pos: tuple) -> tuple:
        """Calculate where the pen tip contacts the canvas surface."""
        canvas_z = self.canvas_system.canvas_position[2]  # Canvas surface height
        
        # Project pen position to canvas surface (vertical projection)
        contact_x = robot_pos[0]
        contact_y = robot_pos[1] 
        contact_z = canvas_z + 0.002  # Slightly above canvas (2 mm) for visibility
        
        return (contact_x, contact_y, contact_z)
    
    def _calculate_trail_display_position(self, pen_tip_pos: tuple) -> tuple:
        """Calculate trail display position for RViz visualization."""
        canvas_z = self.canvas_system.canvas_position[2]  # Canvas surface height
        
        # Project pen tip to canvas surface for trail visualization
        trail_x = pen_tip_pos[0]
        trail_y = pen_tip_pos[1] 
        trail_z = canvas_z + 0.003  # Slightly above canvas (3mm) for visibility
        
        return (trail_x, trail_y, trail_z)
    
    def _add_pen_trail_point(self, pen_tip_pos: tuple):
        """Add pen tip point to trail and publish to RViz."""
        # Use pen tip position directly (already calculated correctly)
        contact_point = self._calculate_trail_display_position(pen_tip_pos)
        
        # Add contact point to trail
        self.pen_trail_points.append(contact_point)
        
        # Limit trail length to show complete circle (increased for full circle)
        if len(self.pen_trail_points) > 5000:  # Allow full circle trail
            self.pen_trail_points.pop(0)
        
        # Publish trail every 10 points to reduce update frequency
        if len(self.pen_trail_points) % 10 == 0:
            self._publish_pen_trail()
    
    def _publish_pen_trail(self):
        """Publish pen trail as line strip marker in RViz."""
        if len(self.pen_trail_points) < 2:
            return
            
        marker_array = MarkerArray()
        
        # Create line strip marker for pen trail
        trail_marker = Marker()
        trail_marker.header.frame_id = "base_link"
        trail_marker.header.stamp = self.get_clock().now().to_msg()
        trail_marker.ns = "pen_trail"
        trail_marker.id = 0
        trail_marker.type = Marker.LINE_STRIP
        trail_marker.action = Marker.ADD
        
        # Set line properties
        trail_marker.scale.x = 0.002  # 2mm line width
        trail_marker.color.r = 1.0    # Red color
        trail_marker.color.g = 0.0
        trail_marker.color.b = 0.0
        trail_marker.color.a = 0.8    # Semi-transparent
        
        # IMPORTANT: Set lifetime to persistent (prevent auto-deletion)
        trail_marker.lifetime.sec = 0
        trail_marker.lifetime.nanosec = 0
        
        # Add all points to the line strip - use actual robot positions
        from geometry_msgs.msg import Point
        for pos in self.pen_trail_points:
            point = Point()
            point.x = pos[0]
            point.y = pos[1] 
            point.z = pos[2]  # Use actual Z position for accurate representation
            trail_marker.points.append(point)
        
        # Current pen position marker
        current_marker = Marker()
        current_marker.header.frame_id = "base_link"
        current_marker.header.stamp = self.get_clock().now().to_msg()
        current_marker.ns = "pen_current"
        current_marker.id = 1
        current_marker.type = Marker.SPHERE
        current_marker.action = Marker.ADD
        
        # Set current pen position - use actual robot position
        current_pos = self.pen_trail_points[-1]
        current_marker.pose.position.x = current_pos[0]
        current_marker.pose.position.y = current_pos[1]
        current_marker.pose.position.z = current_pos[2]  # Use actual Z position
        current_marker.pose.orientation.w = 1.0
        
        # Set sphere properties - smaller pen tip
        current_marker.scale.x = 0.005  # 5mm diameter
        current_marker.scale.y = 0.005
        current_marker.scale.z = 0.005
        current_marker.color.r = 0.0
        current_marker.color.g = 1.0  # Green color
        current_marker.color.b = 0.0
        current_marker.color.a = 1.0
        
        # IMPORTANT: Set lifetime to persistent (prevent auto-deletion)
        current_marker.lifetime.sec = 0
        current_marker.lifetime.nanosec = 0
        
        marker_array.markers.append(trail_marker)
        marker_array.markers.append(current_marker)
        
        self.pen_trail_pub.publish(marker_array)
    
    def _get_actual_robot_position(self) -> tuple:
        """Get actual robot end-effector position from TF tree."""
        try:
            # Get transform from base_link to wrist_3_link (end effector)
            transform = self.tf_buffer.lookup_transform(
                'base_link',  # target frame
                'wrist_3_link',  # source frame
                rclpy.time.Time(),  # latest available
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z
            
            return (x, y, z)
            
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            # TF lookup failed - return None to use fallback
            return None

    def _publish_debug_coordinates(self, robot_pos: tuple):
        """Publish current robot coordinates for external monitoring."""
        from geometry_msgs.msg import PointStamped
        
        debug_msg = PointStamped()
        debug_msg.header.frame_id = "base_link"
        debug_msg.header.stamp = self.get_clock().now().to_msg()
        debug_msg.point.x = robot_pos[0]
        debug_msg.point.y = robot_pos[1] 
        debug_msg.point.z = robot_pos[2]
        
        self.debug_coords_pub.publish(debug_msg)


def main(args=None):
    """Main function for Phase 1 robot drawing demo."""
    print("🚀 Starting Phase 1 Robot Drawing Demo!")
    print("📍 Drawing circle using Phase 1 coordinate calculation")
    print("👀 Watch in RViz - robot will draw a circle")
    print("🔄 Contact-based progress tracking enabled")
    
    rclpy.init(args=args)
    
    try:
        drawer = RobotDrawerNode()
        
        print("✅ Phase 1 demo started successfully!")
        print("🎯 Drawing circle with Phase 1 algorithm...")
        
        rclpy.spin(drawer)
        
    except KeyboardInterrupt:
        print("\n🎨 Phase 1 demo stopped by user")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
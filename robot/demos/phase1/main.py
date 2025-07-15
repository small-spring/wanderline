#!/usr/bin/env python3
"""
Phase 1 Main: Minimal Robot Circle Drawing Demo

VNC environment test implementation.
Integrates Phase 1 components with existing robot control system.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
import math
import time
import yaml
from pathlib import Path

# Import Phase 1 components
from system_state import create_default_system_state, SystemState, ContactPoint
from coordinate_calculator import create_coordinate_calculator

# Import Canvas Preview Window (conditional for environments without OpenCV)
try:
    from canvas_preview import create_preview_window
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


class Phase1RobotDrawer(Node):
    """
    Phase 1 Robot Drawing Demo.
    Extends existing robot control with Phase 1 coordinate calculation.
    """
    
    def __init__(self):
        super().__init__('phase1_robot_drawer')
        
        # Load configuration
        self.config = self._load_config()
        
        # Initialize Phase 1 components
        self.system_state = create_default_system_state()
        self.coord_calculator = create_coordinate_calculator(self.config['drawing'])
        self.canvas_system = CanvasCoordinateSystem()
        
        # Initialize Canvas Preview Window (if available)
        self.canvas_preview = None
        if CANVAS_PREVIEW_AVAILABLE:
            try:
                self.canvas_preview = create_preview_window(self.config)
                self.get_logger().info("🎨 Canvas Preview Window initialized")
            except Exception as e:
                self.get_logger().warning(f"⚠️  Canvas Preview failed to initialize: {e}")
                self.canvas_preview = None
        
        # Robot control (from existing robot_draw_circle.py)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Canvas visualization in RViz
        self.canvas_marker_pub = self.create_publisher(Marker, '/canvas_marker', 10)
        self._publish_canvas_marker()
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
        self.interpolation_steps = 25
        self.current_step = 0
        
        # Timer for smooth movement
        update_rate = 1.0 / self.config['robot']['update_rate']
        self.timer = self.create_timer(update_rate, self.drawing_step)
        
        self.get_logger().info("🎯 Phase 1 Robot Drawer started!")
        self.get_logger().info(f"📍 Starting at canvas center: {self.current_pixel_position}")
        
    def _load_config(self) -> dict:
        """Load configuration from YAML file."""
        config_path = Path(__file__).parent / 'config.yaml'
        try:
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
                return config['phase1']
        except Exception as e:
            self.get_logger().error(f"Failed to load config: {e}")
            # Return minimal default config
            return {
                'robot': {
                    'base_joints': [0.0, -1.2, -1.0, -1.5, 1.57, 0.0],
                    'update_rate': 50
                },
                'drawing': {
                    'segments': 24,
                    'max_stroke_length': 0.005,
                    'closure_threshold': 0.95,
                    'perimeter_tolerance': 5.0
                }
            }
    
    def drawing_step(self):
        """Main drawing loop step."""
        if not self.drawing_active:
            return
            
        # Step 1: Calculate next coordinate using Phase 1 algorithm
        next_coord, drawing_complete = self.coord_calculator.calculate_next_coordinate(
            self.current_pixel_position, self.system_state
        )
        
        if drawing_complete:
            self.get_logger().info("🎉 Circle drawing completed!")
            self.drawing_active = False
            return
            
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
        """Convert pixel coordinates to joint positions."""
        # Convert pixel to robot coordinates
        robot_x, robot_y, robot_z = self.canvas_system.pixel_to_robot_coords(
            pixel_x, pixel_y, pen_down=True
        )
        
        # Map to joint space (from existing robot_draw_circle.py)
        canvas_center_x, canvas_center_y = 0.5, 0.0
        pan_amplitude = 0.6   # ±34 degrees base rotation
        lift_amplitude = 0.3  # ±17 degrees shoulder motion
        
        # Calculate joint offsets
        pan_offset = (robot_x - canvas_center_x) / 0.2 * pan_amplitude
        lift_offset = (robot_y - canvas_center_y) / 0.2 * lift_amplitude
        
        # Apply to base configuration
        joints = self.base_joints.copy()
        joints[0] = self.base_joints[0] + pan_offset   # shoulder_pan_joint
        joints[1] = self.base_joints[1] + lift_offset  # shoulder_lift_joint
        
        return joints
    
    def _execute_smooth_movement(self, target_joints: list):
        """Execute smooth interpolated movement to target joints."""
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
        
        # Publish joint state
        self._publish_joint_state(interpolated_joints)
        
        # Update robot state in system state
        joint_dict = dict(zip(self.joint_names, interpolated_joints))
        robot_pos = self._joints_to_robot_position(interpolated_joints)
        self.system_state.update_robot_position(joint_dict, robot_pos)
        
        self.current_step += 1
    
    def _joints_to_robot_position(self, joints: list) -> tuple:
        """Convert joint positions to estimated robot position."""
        # Simplified forward kinematics approximation
        # This is a rough estimate for testing purposes
        pan_angle = joints[0]
        lift_angle = joints[1]
        
        # Rough calculation based on UR5e kinematics
        arm_length = 0.4  # Approximate arm reach
        x = 0.5 + arm_length * math.cos(pan_angle) * math.cos(lift_angle)
        y = 0.0 + arm_length * math.sin(pan_angle) * math.cos(lift_angle)
        z = 0.1 + arm_length * math.sin(lift_angle)
        
        return (x, y, z)
    
    def _simulate_contact_detection(self, pixel_coord: tuple):
        """Simulate contact detection and update system state."""
        # Create contact point
        robot_x, robot_y, robot_z = self.canvas_system.pixel_to_robot_coords(
            pixel_coord[0], pixel_coord[1], pen_down=True
        )
        
        contact_point = ContactPoint(
            position_3d=(robot_x, robot_y, robot_z),
            position_2d=(int(pixel_coord[0]), int(pixel_coord[1])),
            timestamp=time.time(),
            stroke_id=self.system_state.current_stroke_id
        )
        
        # Add to system state
        self.system_state.add_contact_point(contact_point)
        
        # Update Canvas Preview Window
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
        """Publish canvas marker for RViz visualization."""
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "canvas"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        # Canvas position and size (from canvas_coordinate_system.py)
        marker.pose.position.x = 0.5  # Canvas center X
        marker.pose.position.y = 0.0  # Canvas center Y  
        marker.pose.position.z = 0.1  # Canvas surface Z
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        
        # Canvas size: 40cm x 40cm x 1mm
        marker.scale.x = 0.4  # 40cm width
        marker.scale.y = 0.4  # 40cm height  
        marker.scale.z = 0.001  # 1mm thickness
        
        # White canvas color
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 0.8  # Semi-transparent
        
        # Canvas persists until deleted
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        
        self.canvas_marker_pub.publish(marker)
        self.get_logger().info("📋 Canvas marker published to RViz")


def main(args=None):
    """Main function for Phase 1 robot drawing demo."""
    print("🚀 Starting Phase 1 Robot Drawing Demo!")
    print("📍 Drawing circle using Phase 1 coordinate calculation")
    print("👀 Watch in RViz - robot will draw a circle")
    print("🔄 Contact-based progress tracking enabled")
    
    rclpy.init(args=args)
    
    try:
        drawer = Phase1RobotDrawer()
        
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
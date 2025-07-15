#!/usr/bin/env python3
"""
🎯 Robot Circle Drawing Demo
Make UR5e robot draw a circle in 3D space using smooth joint movements
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math
import time

class RobotCircleDrawer(Node):
    """Draw circle with coordinated joint movements"""
    
    def __init__(self):
        super().__init__('robot_circle_drawer')
        
        # Publisher for joint states
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # UR5e joint names
        self.joint_names = [
            'shoulder_pan_joint',      # Base rotation
            'shoulder_lift_joint',     # Shoulder up/down  
            'elbow_joint',            # Elbow bend
            'wrist_1_joint',          # Wrist bend
            'wrist_2_joint',          # Wrist rotate
            'wrist_3_joint'           # End effector rotate
        ]
        
        # Circle parameters
        self.circle_points = 24  # 24 points = smooth circle
        self.current_point = 0
        self.is_drawing = True
        
        # Smooth interpolation parameters
        self.interpolation_steps = 25  # 25 steps between waypoints (smooth!)
        self.current_step = 0
        self.current_joints = None
        self.target_joints = None
        
        # Generate circle waypoints in joint space
        self.waypoints = self.generate_circle_joint_positions()
        
        # Start at first waypoint
        self.current_joints = self.waypoints[0].copy()
        self.target_joints = self.waypoints[1].copy()
        
        # Timer for smooth movement (fast interpolation)
        self.timer = self.create_timer(0.02, self.interpolation_step)  # 50Hz for smooth motion
        
        self.get_logger().info("🎯 Robot Circle Drawer started!")
        self.get_logger().info(f"🔄 Drawing circle with {len(self.waypoints)} points")
        self.get_logger().info("⏰ Smooth interpolation at 50Hz")
    
    def generate_circle_joint_positions(self):
        """Generate joint positions that create a circle motion"""
        waypoints = []
        
        # Base configuration for drawing position
        base_joints = [
            0.0,     # shoulder_pan_joint (base rotation)
            -1.2,    # shoulder_lift_joint (shoulder down)
            -1.0,    # elbow_joint (elbow bent)
            -1.5,    # wrist_1_joint (wrist down)
            1.57,    # wrist_2_joint (wrist perpendicular)
            0.0      # wrist_3_joint (no end rotation)
        ]
        
        # Circle motion parameters
        pan_amplitude = 0.6      # ±34 degrees base rotation
        lift_amplitude = 0.3     # ±17 degrees shoulder motion
        
        for i in range(self.circle_points + 1):  # +1 to close the circle
            angle = 2 * math.pi * i / self.circle_points
            
            # Create circular motion by combining:
            # - Base rotation (shoulder_pan_joint) for X motion
            # - Shoulder lift (shoulder_lift_joint) for Y motion
            joints = base_joints.copy()
            joints[0] = base_joints[0] + pan_amplitude * math.cos(angle)      # X motion
            joints[1] = base_joints[1] + lift_amplitude * math.sin(angle)     # Y motion
            
            waypoints.append(joints)
            
        return waypoints
    
    def interpolate_joints(self, start_joints, target_joints, progress):
        """Linear interpolation between joint positions"""
        result = []
        for i in range(len(start_joints)):
            interpolated = start_joints[i] + (target_joints[i] - start_joints[i]) * progress
            result.append(interpolated)
        return result
    
    def interpolation_step(self):
        """Execute one smooth interpolation step"""
        if not self.is_drawing:
            return
        
        # Calculate interpolation progress (0.0 to 1.0)
        progress = self.current_step / self.interpolation_steps
        
        # Interpolate current joint positions
        interpolated_joints = self.interpolate_joints(
            self.current_joints, 
            self.target_joints, 
            progress
        )
        
        # Create and publish joint state message
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = interpolated_joints
        msg.velocity = [0.0] * len(self.joint_names)
        msg.effort = [0.0] * len(self.joint_names)
        
        self.joint_pub.publish(msg)
        
        self.current_step += 1
        
        # Check if we reached the target
        if self.current_step >= self.interpolation_steps:
            # Move to next waypoint
            self.current_point += 1
            
            if self.current_point >= len(self.waypoints):
                self.get_logger().info("🎉 Circle complete! Starting new circle...")
                self.current_point = 0
            
            # Set up next interpolation
            self.current_joints = self.target_joints.copy()
            next_point = (self.current_point + 1) % len(self.waypoints)  # Loop back to start
            self.target_joints = self.waypoints[next_point].copy()
            self.current_step = 0
            
            # Log progress every waypoint
            pan_deg = math.degrees(self.current_joints[0])
            lift_deg = math.degrees(self.current_joints[1])
            self.get_logger().info(
                f"🎨 Point {self.current_point:2d}/{self.circle_points}: "
                f"pan={pan_deg:+5.1f}°, lift={lift_deg:+5.1f}°"
            )

def main(args=None):
    """Main function"""
    print("🚀 Starting Robot Circle Drawing!")
    print("👀 Watch the robot draw a circle in RViz!")
    print("🎨 The robot end-effector will trace a circular path")
    print("⏰ Each circle takes 12 seconds")
    
    rclpy.init(args=args)
    
    try:
        drawer = RobotCircleDrawer()
        
        print("✅ Circle drawing started!")
        print("🔄 Continuous circles - Press Ctrl+C to stop")
        
        rclpy.spin(drawer)
        
    except KeyboardInterrupt:
        print("\n🎨 Circle drawing stopped by user")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
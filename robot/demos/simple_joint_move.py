#!/usr/bin/env python3
"""
🤖 Simple Joint Movement Test
Test basic robot arm movement - just 90 degree rotation
cd /workspace/robot/demos
python3 simple_joint_move.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math
import time

class SimpleJointMover(Node):
    """Move robot joint by 90 degrees and wait"""
    
    def __init__(self):
        super().__init__('simple_joint_mover')
        
        # Publisher for joint states
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # UR5e joint names
        self.joint_names = [
            'shoulder_pan_joint',      # Base rotation (we'll move this one)
            'shoulder_lift_joint', 
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        
        # Starting position (all zeros)
        self.start_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Target position (90 degrees on first joint)
        self.target_angles = [math.pi/2, 0.0, 0.0, 0.0, 0.0, 0.0]  # 90 degrees = π/2 radians
        
        # Movement state
        self.step_count = 0
        self.max_steps = 50  # 5 seconds at 10Hz
        self.is_moving = True
        
        # Timer for movement
        self.timer = self.create_timer(0.1, self.move_step)  # 10Hz
        
        self.get_logger().info("🎯 Simple Joint Mover started!")
        self.get_logger().info("🔄 Moving shoulder_pan_joint 90 degrees over 5 seconds")
    
    def interpolate_position(self, start, target, progress):
        """Linear interpolation between start and target"""
        return start + (target - start) * progress
    
    def move_step(self):
        """Execute one movement step"""
        if not self.is_moving:
            return
            
        if self.step_count >= self.max_steps:
            # Movement complete
            self.get_logger().info("🎉 Movement complete! Holding position...")
            self.is_moving = False
            # Continue publishing target position to hold it
        
        # Calculate current progress (0.0 to 1.0)
        progress = min(self.step_count / self.max_steps, 1.0)
        
        # Interpolate joint positions
        current_angles = []
        for i in range(len(self.joint_names)):
            angle = self.interpolate_position(
                self.start_angles[i], 
                self.target_angles[i], 
                progress
            )
            current_angles.append(angle)
        
        # Create and publish joint state message
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = current_angles
        msg.velocity = [0.0] * len(self.joint_names)
        msg.effort = [0.0] * len(self.joint_names)
        
        self.joint_pub.publish(msg)
        
        # Log progress every 10 steps
        if self.step_count % 10 == 0:
            angle_degrees = math.degrees(current_angles[0])
            self.get_logger().info(f"🤖 Step {self.step_count}: shoulder_pan = {angle_degrees:.1f}°")
        
        self.step_count += 1

def main(args=None):
    """Main function"""
    print("🚀 Starting Simple Joint Movement Test!")
    print("👀 Watch the robot in RViz - shoulder should rotate 90 degrees")
    print("🕒 Movement takes 5 seconds, then holds position")
    
    rclpy.init(args=args)
    
    try:
        mover = SimpleJointMover()
        
        print("✅ Movement started!")
        print("🛑 Press Ctrl+C to stop")
        
        rclpy.spin(mover)
        
    except KeyboardInterrupt:
        print("\n🛑 Movement stopped by user")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
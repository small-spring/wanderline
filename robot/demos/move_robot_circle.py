#!/usr/bin/env python3
"""
🤖 Make UR5e Robot Draw Circle Automatically!
Takes our circle points and moves the robot through them
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
import math
import time

class CircleRobotMover(Node):
    """Move robot through circle waypoints automatically"""
    
    def __init__(self):
        super().__init__('circle_robot_mover')
        
        # Publisher for joint states
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # UR5e joint names
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint', 
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        
        # Circle parameters (same as our demo)
        self.center = [0.3, 0.0, 0.2]
        self.radius = 0.1
        self.num_points = 12
        
        # Movement state
        self.current_point = 0
        self.waypoints = self.generate_circle_waypoints()
        
        # Timer for automatic movement
        self.timer = self.create_timer(1.0, self.move_to_next_point)  # 1 second per point
        
        self.get_logger().info("🎯 Circle Robot Mover started!")
        self.get_logger().info(f"🔄 Will move through {len(self.waypoints)} points")
    
    def generate_circle_waypoints(self):
        """Generate circle waypoints (same as demo_circle_ros2.py)"""
        waypoints = []
        
        for i in range(self.num_points + 1):
            angle = 2 * math.pi * i / self.num_points
            
            point = Point()
            point.x = self.center[0] + self.radius * math.cos(angle)
            point.y = self.center[1] + self.radius * math.sin(angle)
            point.z = self.center[2]
            
            waypoints.append(point)
            
        return waypoints
    
    def point_to_joint_angles(self, point):
        """Convert 3D point to robot joint angles (simple approximation)"""
        # This is a VERY simplified inverse kinematics
        # In real applications, use MoveIt2 for proper IK
        
        x, y, z = point.x, point.y, point.z
        
        # Simple mapping for demonstration
        # Joint 0 (base rotation) - follow the angle to the point
        base_angle = math.atan2(y, x)
        
        # Joint 1 (shoulder) - adjust for height
        shoulder_angle = -1.0 + 0.5 * (z - 0.1)  # Rough height adjustment
        
        # Joint 2 (elbow) - extend toward point
        distance = math.sqrt(x*x + y*y)
        elbow_angle = -1.5 + (distance - 0.2) * 2.0
        
        # Wrist joints - keep end-effector oriented downward
        wrist1_angle = -1.0
        wrist2_angle = -1.57  # -90 degrees
        wrist3_angle = 0.0
        
        return [base_angle, shoulder_angle, elbow_angle, wrist1_angle, wrist2_angle, wrist3_angle]
    
    def move_to_next_point(self):
        """Move robot to next circle point"""
        if self.current_point >= len(self.waypoints):
            self.get_logger().info("🎉 Circle complete! Restarting...")
            self.current_point = 0
            return
        
        # Get current waypoint
        point = self.waypoints[self.current_point]
        
        # Convert to joint angles
        joint_angles = self.point_to_joint_angles(point)
        
        # Create and publish joint state message
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = joint_angles
        msg.velocity = [0.0] * len(self.joint_names)
        msg.effort = [0.0] * len(self.joint_names)
        
        self.joint_pub.publish(msg)
        
        self.get_logger().info(
            f"🤖 Point {self.current_point}: "
            f"x={point.x:.3f}, y={point.y:.3f}, z={point.z:.3f}"
        )
        
        self.current_point += 1

def main(args=None):
    """Main function"""
    print("🚀 Starting Automatic Circle Drawing!")
    print("👀 Watch the robot move in RViz!")
    
    rclpy.init(args=args)
    
    try:
        mover = CircleRobotMover()
        
        print("✅ Robot will automatically trace a circle!")
        print("🔄 1 point per second")
        print("🛑 Press Ctrl+C to stop")
        
        rclpy.spin(mover)
        
    except KeyboardInterrupt:
        print("\n🛑 Circle drawing stopped by user")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
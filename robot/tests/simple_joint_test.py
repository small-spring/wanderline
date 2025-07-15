#!/usr/bin/env python3
"""
🤖 シンプル関節テスト
1つの関節を3秒かけて90度動かします
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time

class SimpleJointTest(Node):
    """1つの関節をテストするノード"""
    
    def __init__(self):
        super().__init__('simple_joint_test')
        
        # Publisher for joint states
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # UR5e joint names
        self.joint_names = [
            'shoulder_pan_joint',      # 0: ベース回転
            'shoulder_lift_joint',     # 1: 肩上下
            'elbow_joint',            # 2: 肘
            'wrist_1_joint',          # 3: 手首1
            'wrist_2_joint',          # 4: 手首2
            'wrist_3_joint'           # 5: 手首3
        ]
        
        # Test parameters
        self.start_time = time.time()
        self.test_duration = 3.0  # 3秒
        self.target_angle = math.pi / 2  # 90度（ラジアン）
        self.test_joint = 0  # shoulder_pan_joint をテスト
        
        # 高頻度更新（20Hz）
        self.timer = self.create_timer(0.05, self.update_joint)
        
        self.get_logger().info("🎯 Simple Joint Test started!")
        self.get_logger().info(f"📐 Moving joint '{self.joint_names[self.test_joint]}' to 90 degrees in 3 seconds")
    
    def update_joint(self):
        """関節の位置を更新"""
        current_time = time.time() - self.start_time
        
        # 3秒で0から90度まで滑らかに移動
        if current_time <= self.test_duration:
            # 滑らかな補間（0 から target_angle まで）
            progress = current_time / self.test_duration
            current_angle = self.target_angle * progress
        else:
            # 3秒後は90度で固定
            progress = 1.0  # 100%完了
            current_angle = self.target_angle
        
        # 全関節の角度（テスト対象以外は0）
        joint_positions = [0.0] * len(self.joint_names)
        joint_positions[self.test_joint] = current_angle
        
        # メッセージを作成
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = joint_positions
        msg.velocity = [0.0] * len(self.joint_names)
        msg.effort = [0.0] * len(self.joint_names)
        
        # パブリッシュ
        self.joint_pub.publish(msg)
        
        # 0.5秒ごとにログ出力
        if int(current_time * 20) % 10 == 0:
            degrees = math.degrees(current_angle)
            self.get_logger().info(f"📐 Joint angle: {degrees:.1f}° (progress: {progress:.1%})")
        
        # 3秒経過で完了メッセージ
        if current_time > self.test_duration and not hasattr(self, 'completed'):
            self.completed = True
            self.get_logger().info("✅ Test completed! Joint should be at 90 degrees")

def main(args=None):
    """Main function"""
    print("🎯 Starting Simple Joint Test!")
    print("📐 Watch shoulder_pan_joint move 90° in 3 seconds")
    
    rclpy.init(args=args)
    
    try:
        test = SimpleJointTest()
        
        print("✅ Joint test running!")
        print("👀 Watch the base joint rotate in RViz!")
        print("🛑 Press Ctrl+C to stop")
        
        rclpy.spin(test)
        
    except KeyboardInterrupt:
        print("\n🛑 Joint test stopped")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
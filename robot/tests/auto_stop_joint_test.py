#!/usr/bin/env python3
"""
🤖 自動停止関節テスト
5秒後に自動で停止します
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time

class AutoStopJointTest(Node):
    """自動停止する関節テスト"""
    
    def __init__(self):
        super().__init__('auto_stop_joint_test')
        
        # Publisher for joint states
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Subscriber to monitor actual joint states
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        
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
        self.test_duration = 3.0  # 3秒で動作
        self.total_duration = 5.0  # 5秒後に自動停止
        self.target_angle = math.pi / 2  # 90度
        self.test_joint = 0  # shoulder_pan_joint
        
        # State tracking
        self.actual_joint_states = {}
        self.commanded_angle = 0.0
        
        # タイマー
        self.timer = self.create_timer(0.05, self.update_joint)
        
        self.get_logger().info("🎯 Auto-Stop Joint Test started!")
        self.get_logger().info(f"📐 Moving joint '{self.joint_names[self.test_joint]}' to 90°")
        self.get_logger().info("⏰ Will auto-stop in 5 seconds")
    
    def joint_state_callback(self, msg):
        """実際の関節状態を記録"""
        if len(msg.name) == len(msg.position):
            for name, pos in zip(msg.name, msg.position):
                self.actual_joint_states[name] = pos
    
    def update_joint(self):
        """関節の位置を更新"""
        current_time = time.time() - self.start_time
        
        # 5秒後に自動停止
        if current_time > self.total_duration:
            self.get_logger().info("⏰ Auto-stopping after 5 seconds!")
            self.report_final_states()
            rclpy.shutdown()
            return
        
        # 3秒で0から90度まで移動
        if current_time <= self.test_duration:
            progress = current_time / self.test_duration
            self.commanded_angle = self.target_angle * progress
        else:
            self.commanded_angle = self.target_angle
        
        # 関節位置を設定
        joint_positions = [0.0] * len(self.joint_names)
        joint_positions[self.test_joint] = self.commanded_angle
        
        # メッセージをパブリッシュ
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = joint_positions
        msg.velocity = [0.0] * len(self.joint_names)
        msg.effort = [0.0] * len(self.joint_names)
        
        self.joint_pub.publish(msg)
        
        # ログ出力（1秒ごと）
        if int(current_time * 20) % 20 == 0:
            commanded_deg = math.degrees(self.commanded_angle)
            actual_pos = self.actual_joint_states.get(self.joint_names[self.test_joint], 0.0)
            actual_deg = math.degrees(actual_pos)
            
            self.get_logger().info(
                f"📐 Commanded: {commanded_deg:.1f}° | "
                f"Actual: {actual_deg:.1f}° | "
                f"Time: {current_time:.1f}s"
            )
    
    def report_final_states(self):
        """最終状態をレポート"""
        self.get_logger().info("📊 Final Joint States Report:")
        for name in self.joint_names:
            actual_pos = self.actual_joint_states.get(name, 0.0)
            actual_deg = math.degrees(actual_pos)
            self.get_logger().info(f"  {name}: {actual_deg:.1f}°")

def main(args=None):
    """Main function"""
    print("🎯 Starting Auto-Stop Joint Test!")
    print("⏰ Will automatically stop in 5 seconds")
    
    rclpy.init(args=args)
    
    try:
        test = AutoStopJointTest()
        rclpy.spin(test)
        
    except KeyboardInterrupt:
        print("\n🛑 Test stopped by user")
    except rclpy.exceptions.ROSInterruptException:
        print("\n⏰ Test completed automatically")
    finally:
        print("✅ Test finished!")

if __name__ == '__main__':
    main()
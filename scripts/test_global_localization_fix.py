#!/usr/bin/env python3
"""
测试全局定位到位姿跟踪转换的修复效果
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import time
import math

class GlobalLocalizationTester(Node):
    def __init__(self):
        super().__init__('global_localization_tester')
        
        # 订阅机器人位姿
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/RobotPose',
            self.pose_callback,
            10
        )
        
        # 订阅系统状态（如果有的话）
        self.status_sub = self.create_subscription(
            String,
            '/system_status',
            self.status_callback,
            10
        )
        
        # 状态跟踪变量
        self.poses = []
        self.last_pose_time = None
        self.tracking_started = False
        self.global_loc_completed = False
        
        # 创建定时器进行状态检查
        self.timer = self.create_timer(2.0, self.check_tracking_status)
        
        self.get_logger().info("全局定位测试器已启动")
    
    def pose_callback(self, msg):
        """处理位姿消息"""
        current_time = self.get_clock().now()
        
        # 记录位姿
        pose_data = {
            'time': current_time,
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'timestamp': msg.header.stamp
        }
        self.poses.append(pose_data)
        
        # 检测全局定位完成
        if not self.global_loc_completed and len(self.poses) >= 2:
            # 如果位姿发生显著变化，可能是全局定位完成
            prev_pose = self.poses[-2]
            curr_pose = self.poses[-1]
            
            distance = math.sqrt(
                (curr_pose['x'] - prev_pose['x'])**2 + 
                (curr_pose['y'] - prev_pose['y'])**2
            )
            
            if distance > 1.0:  # 位姿跳跃超过1米
                self.global_loc_completed = True
                self.get_logger().info(f"🎯 检测到全局定位完成: 位姿从({prev_pose['x']:.2f}, {prev_pose['y']:.2f}) 跳跃到 ({curr_pose['x']:.2f}, {curr_pose['y']:.2f})")
        
        # 检测位姿跟踪开始
        if self.global_loc_completed and not self.tracking_started:
            self.tracking_started = True
            self.get_logger().info("🚀 位姿跟踪已开始")
        
        self.last_pose_time = current_time
        
        # 输出当前位姿
        self.get_logger().info(f"位姿: x={msg.pose.position.x:.3f}, y={msg.pose.position.y:.3f}")
    
    def status_callback(self, msg):
        """处理系统状态消息"""
        self.get_logger().info(f"系统状态: {msg.data}")
    
    def check_tracking_status(self):
        """检查跟踪状态"""
        current_time = self.get_clock().now()
        
        if self.last_pose_time is None:
            self.get_logger().warn("⚠️  尚未接收到位姿消息")
            return
        
        # 检查位姿更新频率
        time_since_last = (current_time - self.last_pose_time).nanoseconds / 1e9
        
        if time_since_last > 5.0:
            self.get_logger().error(f"❌ 位姿跟踪可能失败: {time_since_last:.1f}秒未收到位姿更新")
        elif time_since_last > 2.0:
            self.get_logger().warn(f"⚠️  位姿更新频率较低: {time_since_last:.1f}秒")
        
        # 分析位姿连续性
        if len(self.poses) >= 5:
            recent_poses = self.poses[-5:]
            max_distance = 0
            
            for i in range(1, len(recent_poses)):
                distance = math.sqrt(
                    (recent_poses[i]['x'] - recent_poses[i-1]['x'])**2 + 
                    (recent_poses[i]['y'] - recent_poses[i-1]['y'])**2
                )
                max_distance = max(max_distance, distance)
            
            if max_distance > 5.0:
                self.get_logger().error(f"❌ 检测到位姿跳跃: 最大距离变化 {max_distance:.2f}m")
            elif max_distance > 2.0:
                self.get_logger().warn(f"⚠️  位姿变化较大: 最大距离变化 {max_distance:.2f}m")
            else:
                self.get_logger().info(f"✅ 位姿跟踪稳定: 最大距离变化 {max_distance:.2f}m")
        
        # 输出统计信息
        if len(self.poses) > 0:
            self.get_logger().info(f"📈 统计: 已接收 {len(self.poses)} 个位姿, 全局定位完成: {self.global_loc_completed}, 跟踪开始: {self.tracking_started}")

def main(args=None):
    rclpy.init(args=args)
    
    tester = GlobalLocalizationTester()
    
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        tester.get_logger().info("测试器停止")
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

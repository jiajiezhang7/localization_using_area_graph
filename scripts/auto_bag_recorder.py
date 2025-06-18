#!/usr/bin/env python3
"""
自动rosbag录制脚本
当检测到AGLoc系统启动时，自动开始录制指定话题
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import threading
import time
import signal
import sys
import os
from datetime import datetime
import argparse

class AutoBagRecorder(Node):
    def __init__(self, output_dir="./recordings", record_duration=None, use_sim_time=True, wait_for_all_topics=False):
        super().__init__('auto_bag_recorder')
        
        # 录制参数
        self.output_dir = output_dir
        self.record_duration = record_duration
        self.use_sim_time = use_sim_time
        self.topics_to_record = ['/rss', '/RobotPath', '/RobotPathLonLat']
        self.wait_for_all_topics = wait_for_all_topics
        
        # 🔧 新增：等待时间配置
        self.topic_wait_time = 10.0  # 检测到第一个话题后等待其他话题的时间（秒）
        self.initial_detection_time = None  # 记录首次检测到话题的时间
        
        # 状态变量
        self.recording_process = None
        self.is_recording = False
        self.topics_detected = {topic: False for topic in self.topics_to_record}
        self.topics_detection_time = {topic: None for topic in self.topics_to_record}  # 记录每个话题的检测时间
        self.start_time = None
        self.clock_topic_detected = False
        self._shutdown_requested = False
        self.recording_started_topics = set()  # 记录开始录制时检测到的话题
        
        # 🔧 新增：调试和状态跟踪
        self.last_available_topics = set()
        self.topic_status_logged = False
        
        # 创建输出目录
        os.makedirs(self.output_dir, exist_ok=True)
        
        # 设置信号处理器
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        # 启动话题监测
        self.get_logger().info("🎯 开始监测AGLoc系统话题...")
        self.get_logger().info(f"📁 录制文件将保存到: {self.output_dir}")
        self.get_logger().info(f"📋 监测话题: {', '.join(self.topics_to_record)}")
        if self.wait_for_all_topics:
            self.get_logger().info("⏳ 等待模式: 所有话题都检测到后开始录制")
        else:
            self.get_logger().info(f"🚀 智能模式: 检测到话题后等待{self.topic_wait_time}秒收集更多话题")
        if self.use_sim_time:
            self.get_logger().info("⏰ 使用仿真时间模式 - 等待时钟同步")
        else:
            self.get_logger().info("⏰ 使用系统时间模式")
        
        # 定时器：定期检查话题状态
        self.timer = self.create_timer(2.0, self.check_topics_status)
        
        # 启动话题检测线程
        self.detection_thread = threading.Thread(target=self.monitor_topics)
        self.detection_thread.daemon = True
        self.detection_thread.start()

    def monitor_topics(self):
        """监测指定话题是否开始发布"""
        while rclpy.ok() and not self._shutdown_requested:
            try:
                # 获取当前可用话题列表
                topic_names_and_types = self.get_topic_names_and_types()
                available_topics = set([name for name, _ in topic_names_and_types])
                
                # 🔧 调试：记录话题变化
                new_topics = available_topics - self.last_available_topics
                if new_topics:
                    self.get_logger().debug(f"🔍 新检测到的话题: {new_topics}")
                self.last_available_topics = available_topics
                
                # 检查时钟话题（用于仿真时间同步）
                if self.use_sim_time and '/clock' in available_topics and not self.clock_topic_detected:
                    self.clock_topic_detected = True
                    self.get_logger().info("🕒 检测到时钟话题 - 仿真时间同步已建立")
                
                # 检查我们要录制的话题是否出现
                newly_detected = False
                current_time = time.time()
                
                for topic in self.topics_to_record:
                    if topic in available_topics and not self.topics_detected[topic]:
                        self.topics_detected[topic] = True
                        self.topics_detection_time[topic] = current_time
                        self.get_logger().info(f"✅ 检测到话题: {topic}")
                        newly_detected = True
                        
                        # 记录首次检测时间
                        if self.initial_detection_time is None:
                            self.initial_detection_time = current_time
                            self.get_logger().info(f"⏰ 首次话题检测时间已记录，将等待{self.topic_wait_time}秒收集其他话题")
                
                # 🔧 新增：定期显示话题检测状态
                if not self.topic_status_logged and any(self.topics_detected.values()):
                    detected_topics = [topic for topic, detected in self.topics_detected.items() if detected]
                    missing_topics = [topic for topic, detected in self.topics_detected.items() if not detected]
                    self.get_logger().info(f"📊 话题状态 - 已检测: {detected_topics}, 未检测: {missing_topics}")
                    if len(detected_topics) == len(self.topics_to_record):
                        self.topic_status_logged = True
                
                # 检查是否可以开始录制
                can_start_recording = True
                if self.use_sim_time and not self.clock_topic_detected:
                    can_start_recording = False
                    if newly_detected:
                        self.get_logger().info("⏳ 等待时钟话题同步后开始录制...")
                
                # 检查话题检测条件
                detected_count = sum(self.topics_detected.values())
                total_topics = len(self.topics_to_record)
                
                # 🔧 改进的录制决策逻辑
                should_start_recording = False
                
                if self.wait_for_all_topics:
                    # 等待所有话题模式
                    if detected_count == total_topics:
                        should_start_recording = True
                        if newly_detected:
                            self.get_logger().info(f"✅ 所有话题已检测到 ({detected_count}/{total_topics})，开始录制")
                    elif newly_detected:
                        self.get_logger().info(f"⏳ 已检测到 {detected_count}/{total_topics} 个话题，等待剩余话题...")
                else:
                    # 🔧 智能等待模式：检测到话题后等待一段时间收集更多话题
                    if detected_count > 0 and self.initial_detection_time is not None:
                        time_since_first_detection = current_time - self.initial_detection_time
                        
                        if not self.is_recording:
                            # 满足以下条件之一就开始录制：
                            # 1. 所有话题都检测到了
                            # 2. 等待时间到了且至少有一个话题
                            if detected_count == total_topics:
                                should_start_recording = True
                                self.get_logger().info(f"✅ 所有话题已检测到 ({detected_count}/{total_topics})，立即开始录制")
                            elif time_since_first_detection >= self.topic_wait_time:
                                should_start_recording = True
                                detected_topics = [topic for topic, detected in self.topics_detected.items() if detected]
                                missing_topics = [topic for topic, detected in self.topics_detected.items() if not detected]
                                self.get_logger().info(f"⏰ 等待时间已到，开始录制已检测到的话题: {detected_topics}")
                                if missing_topics:
                                    self.get_logger().warn(f"⚠️  以下话题未检测到，将不会录制: {missing_topics}")
                        else:
                            # 🔧 如果正在录制但检测到新话题，考虑重启录制
                            if newly_detected:
                                detected_topics = [topic for topic, detected in self.topics_detected.items() if detected]
                                if set(detected_topics) != self.recording_started_topics:
                                    self.get_logger().info(f"🔄 录制中检测到新话题 {[t for t in detected_topics if t not in self.recording_started_topics]}，重启录制以包含所有话题")
                                    self.stop_recording()
                                    # 给一点时间让停止完成
                                    time.sleep(1.0)
                                    should_start_recording = True
                
                # 如果满足条件且还未开始录制，并且满足时钟同步条件
                if should_start_recording and not self.is_recording and can_start_recording:
                    self.start_recording()
                
                time.sleep(1.0)  # 每秒检查一次
                
            except Exception as e:
                self.get_logger().error(f"❌ 话题监测出错: {str(e)}")
                time.sleep(2.0)

    def check_topics_status(self):
        """定时器回调：检查录制状态和时间"""
        if self.is_recording:
            # 检查录制进程是否还在运行
            if self.recording_process and self.recording_process.poll() is not None:
                self.get_logger().warn("⚠️  录制进程意外终止")
                self.is_recording = False
                return
            
            # 检查录制时间限制
            if self.record_duration and self.start_time:
                elapsed_time = time.time() - self.start_time
                if elapsed_time >= self.record_duration:
                    self.get_logger().info(f"⏰ 达到录制时间限制 ({self.record_duration}s)，停止录制")
                    self.stop_recording()
                else:
                    remaining_time = self.record_duration - elapsed_time
                    self.get_logger().info(f"🎬 录制中... 剩余时间: {remaining_time:.1f}s")
            else:
                if self.start_time:
                    elapsed_time = time.time() - self.start_time
                    self.get_logger().info(f"🎬 录制中... 已录制: {elapsed_time:.1f}s")
        else:
            # 🔧 新增：在等待期间显示状态
            if self.initial_detection_time is not None and not self.is_recording:
                detected_count = sum(self.topics_detected.values())
                time_since_first = time.time() - self.initial_detection_time
                remaining_wait = max(0, self.topic_wait_time - time_since_first)
                
                if remaining_wait > 0 and detected_count < len(self.topics_to_record):
                    self.get_logger().info(f"⏳ 等待更多话题... 已检测: {detected_count}/{len(self.topics_to_record)}, 剩余等待时间: {remaining_wait:.1f}s")

    def start_recording(self):
        """开始录制rosbag"""
        if self.is_recording:
            self.get_logger().warn("⚠️  录制已在进行中")
            return
        
        # 检查时钟同步状态
        if self.use_sim_time and not self.clock_topic_detected:
            self.get_logger().warn("⚠️  仿真时间模式下未检测到时钟话题，无法开始录制")
            return
        
        # 只录制已检测到的话题
        detected_topics = [topic for topic, detected in self.topics_detected.items() if detected]
        
        if not detected_topics:
            self.get_logger().warn("⚠️  没有检测到任何目标话题，无法开始录制")
            return
        
        # 生成输出文件名
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        bag_name = f"agloc_recording_{timestamp}"
        bag_path = os.path.join(self.output_dir, bag_name)
        
        # 构建rosbag record命令
        cmd = ['ros2', 'bag', 'record', '-o', bag_path]
        
        # 添加时间同步相关参数
        if self.use_sim_time:
            # 使用仿真时间进行录制，确保时间戳与播放的rosbag一致
            cmd.extend(['--use-sim-time'])
            self.get_logger().info("⏰ 使用仿真时间录制 - 时间戳将与原rosbag保持一致")
        
        self.recording_started_topics = set(detected_topics)  # 记录本次录制开始时的话题
        cmd.extend(detected_topics)
        
        # 设置环境变量以确保子进程也使用仿真时间
        env = os.environ.copy()
        if self.use_sim_time:
            env['ROS_PARAMETER_use_sim_time'] = 'true'
        
        # 🔧 新增：显示详细的录制信息
        missing_topics = [topic for topic, detected in self.topics_detected.items() if not detected]
        
        try:
            self.get_logger().info(f"🚀 开始录制rosbag: {bag_name}")
            self.get_logger().info(f"📋 录制话题 ({len(detected_topics)}/{len(self.topics_to_record)}): {', '.join(detected_topics)}")
            if missing_topics:
                self.get_logger().warn(f"⚠️  未录制话题: {', '.join(missing_topics)}")
            if self.use_sim_time:
                self.get_logger().info("🕒 时间同步模式: 仿真时间 (与播放rosbag同步)")
            else:
                self.get_logger().info("🕒 时间同步模式: 系统时间")
            
            # 🔧 调试：显示完整的录制命令
            self.get_logger().debug(f"🔧 录制命令: {' '.join(cmd)}")
            
            # 启动录制进程
            self.recording_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                env=env
            )
            
            self.is_recording = True
            self.start_time = time.time()
            
            self.get_logger().info("✅ rosbag录制已开始")
            
        except Exception as e:
            self.get_logger().error(f"❌ 启动录制失败: {str(e)}")

    def stop_recording(self):
        """停止录制rosbag"""
        if not self.is_recording or not self.recording_process:
            self.get_logger().warn("⚠️  没有正在进行的录制")
            return
        
        try:
            self.get_logger().info("🛑 停止rosbag录制...")
            
            # 发送SIGINT信号给录制进程
            self.recording_process.send_signal(signal.SIGINT)
            
            # 等待进程结束
            try:
                self.recording_process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.get_logger().warn("⚠️  录制进程未及时响应，强制终止")
                self.recording_process.kill()
                self.recording_process.wait()
            
            self.is_recording = False
            self.recording_process = None
            
            if self.start_time:
                total_time = time.time() - self.start_time
                self.get_logger().info(f"✅ 录制完成，总时长: {total_time:.1f}s")
            
        except Exception as e:
            self.get_logger().error(f"❌ 停止录制时出错: {str(e)}")

    def signal_handler(self, signum, frame):
        """处理中断信号"""
        self.get_logger().info("🔄 收到退出信号，正在清理...")
        self.stop_recording()
        # 不在信号处理器中调用shutdown，让main函数处理
        self._shutdown_requested = True

def main():
    parser = argparse.ArgumentParser(description='AGLoc系统自动rosbag录制工具')
    parser.add_argument('--output-dir', '-o', 
                       default='./recordings',
                       help='录制文件输出目录 (默认: ./recordings)')
    parser.add_argument('--duration', '-d',
                       type=float,
                       help='录制时长（秒），不指定则持续录制直到手动停止')
    parser.add_argument('--wait-all-topics',
                       action='store_true',
                       help='等待所有话题都检测到后再开始录制（默认：智能等待模式）')
    parser.add_argument('--topic-wait-time',
                       type=float,
                       default=10.0,
                       help='检测到第一个话题后等待其他话题的时间（秒，默认：10.0）')
    parser.add_argument('--use-system-time',
                       action='store_true',
                       help='使用系统时间而非仿真时间（默认：使用仿真时间以保持与播放rosbag时间戳一致）')
    
    args = parser.parse_args()
    
    # 初始化ROS2
    rclpy.init()
    
    try:
        # 创建录制节点
        recorder = AutoBagRecorder(
            output_dir=args.output_dir,
            record_duration=args.duration,
            use_sim_time=not args.use_system_time,  # 默认使用仿真时间
            wait_for_all_topics=args.wait_all_topics
        )
        
        # 🔧 设置等待时间
        recorder.topic_wait_time = args.topic_wait_time
        
        print("\n" + "="*60)
        print("🎬 AGLoc系统自动录制工具已启动 - 增强版")
        print("="*60)
        print(f"📁 输出目录: {args.output_dir}")
        print(f"📋 监测话题: /rss, /RobotPath, /RobotPathLonLat")
        if args.duration:
            print(f"⏰ 录制时长: {args.duration}s")
        else:
            print("⏰ 录制时长: 持续录制（直到Ctrl+C）")
        
        # 显示等待模式
        if args.wait_all_topics:
            print("📍 等待模式: 严格模式 - 必须检测到所有话题")
        else:
            print(f"📍 等待模式: 智能模式 - 等待{args.topic_wait_time}秒收集话题")
        
        # 显示时间同步模式
        if args.use_system_time:
            print("🕒 时间模式: 系统时间")
            print("⚠️  注意: 使用系统时间可能导致时间戳不同步")
        else:
            print("🕒 时间模式: 仿真时间 (推荐)")
            print("✅ 时间戳将与播放的rosbag保持一致")
        
        print("🔍 正在等待AGLoc系统启动...")
        print("💡 按Ctrl+C停止录制并退出")
        print("="*60 + "\n")
        
        # 运行节点
        try:
            while rclpy.ok() and not recorder._shutdown_requested:
                rclpy.spin_once(recorder, timeout_sec=0.1)
        except KeyboardInterrupt:
            print("\n🔄 检测到Ctrl+C，正在安全退出...")
        
    except KeyboardInterrupt:
        print("\n🔄 用户中断，正在退出...")
    except Exception as e:
        print(f"\n❌ 发生错误: {str(e)}")
    finally:
        if 'recorder' in locals():
            recorder.stop_recording()
        if rclpy.ok():
            rclpy.shutdown()
        print("👋 录制工具已退出")

if __name__ == '__main__':
    main() 
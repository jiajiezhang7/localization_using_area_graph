#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import os
import yaml
from ament_index_python.packages import get_package_share_directory

class DestinationReceiver(Node):
    def __init__(self):
        super().__init__('destination_receiver')
        
        # 获取包路径
        self.pkg_dir = get_package_share_directory('localizationUsingAreaGraph')
        self.yaml_path = os.path.join(self.pkg_dir, 'config', 'destination.yaml')
        
        # 如果文件存在，删除它
        if os.path.exists(self.yaml_path):
            try:
                os.remove(self.yaml_path)
            except Exception as e:
                self.get_logger().error(f'Failed to remove existing yaml file: {str(e)}')

        # 创建订阅者
        self.subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',  # ROS2中使用 goal_pose 而不是 move_base_simple/goal
            self.callback,
            10)
        
        self.get_logger().info('Destination receiver node started. Click the "2D Goal Pose" button in RViz2 to set destinations.')
        self.get_logger().info('The first pose will be used as the initial pose for AMCL.')

    def callback(self, msg):
        """回调函数，处理从RViz2接收的目标位置"""
        self.get_logger().info('Received new destination')
        
        # 创建要保存的数据
        pose_data = [[
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ]]
        
        # 确保目标目录存在
        os.makedirs(os.path.dirname(self.yaml_path), exist_ok=True)
        
        try:
            # 以追加模式打开文件
            with open(self.yaml_path, 'a') as f:
                yaml.dump(pose_data, f, default_flow_style=None)
            
            self.get_logger().info(f'Saved pose: Position(x:{msg.pose.position.x:.2f}, y:{msg.pose.position.y:.2f}), Orientation(w:{msg.pose.orientation.w:.2f})')
        
        except Exception as e:
            self.get_logger().error(f'Failed to save destination to yaml: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = DestinationReceiver()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in destination receiver node: {str(e)}')
    finally:
        # 清理资源
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
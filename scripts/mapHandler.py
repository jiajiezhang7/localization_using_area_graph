#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import os
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from std_msgs.msg import Header
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class MapHandler(Node):
    def __init__(self):
        super().__init__('map_handler_node')
        
        # 设置QoS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 创建发布者
        self.markers_pub = self.create_publisher(
            MarkerArray, 
            'mapMarkers', 
            qos_profile)
            
        self.map_line_pub = self.create_publisher(
            Marker, 
            'mapLine', 
            qos_profile)
            
        self.pointcloud_publisher = self.create_publisher(
            PointCloud, 
            '/mapPC', 
            qos_profile)
            
        # 加载地图数据
        try:
            self.map = np.loadtxt("../../../map/picking_list_star_center.txt", delimiter=',')
            self.map_ini = np.loadtxt("../../../map/picking_list_star_center_initialization.txt", delimiter=',')
            self.map_corridor_enlarge = np.loadtxt("../../../map/corridor_enlarge.txt", delimiter=',')
        except Exception as e:
            self.get_logger().error(f'Failed to load map files: {str(e)}')
            return

        # 创建地图标记
        self.create_map_markers()
        
        # 创建定时器，定期发布数据
        self.create_timer(1.0, self.timer_callback)
        
        self.get_logger().info('Map Handler Node initialized successfully')

    def create_map_markers(self):
        """创建地图标记"""
        self.map_markers = MarkerArray()
        self.map_line_markers = Marker()
        
        # 设置地图线条标记的基本属性
        self.map_line_markers.header.frame_id = 'map'
        self.map_line_markers.type = Marker.LINE_STRIP
        self.map_line_markers.scale.x = 0.1
        self.map_line_markers.color.b = 1.0
        self.map_line_markers.color.a = 1.0
        
        # 创建点云消息
        self.map_pc = PointCloud()
        header = Header()
        header.frame_id = 'map'
        self.map_pc.header = header
        
        # 创建走廊扩展点云
        self.corridor_enlarge_pc = PointCloud()
        self.corridor_enlarge_pc.header = header
        
        # 创建初始化点云
        self.map_pc_init = PointCloud()
        self.map_pc_init.header = header
        
        # 为每个地图点创建标记
        for i in range(len(self.map)):
            # 创建标记
            map_marker = Marker()
            map_marker.header.frame_id = "map"
            map_marker.id = i
            map_marker.type = 2  # sphere
            map_marker.action = 0
            
            # 设置位置
            map_marker.pose.position.x = self.map[i,0]
            map_marker.pose.position.y = self.map[i,1]
            map_marker.pose.position.z = 0
            
            # 设置方向
            map_marker.pose.orientation.x = 0
            map_marker.pose.orientation.y = 0
            map_marker.pose.orientation.z = 0
            map_marker.pose.orientation.w = 1.0
            
            # 设置大小
            map_marker.scale.x = 0.1
            map_marker.scale.y = 0.1
            map_marker.scale.z = 0.1
            
            # 设置颜色
            map_marker.color.r = 0.0
            map_marker.color.g = 1.0
            map_marker.color.b = 0.0
            map_marker.color.a = 1.0
            
            self.map_markers.markers.append(map_marker)
            self.map_line_markers.points.append(Point(x=self.map[i,0], y=self.map[i,1], z=0))
            
            # 添加到点云消息
            self.map_pc.points.append(Point32(x=self.map[i,0], y=self.map[i,1], z=0))

        # 添加初始化点云数据
        for i in range(len(self.map_ini)):
            self.map_pc_init.points.append(Point32(x=self.map_ini[i,0], y=self.map_ini[i,1], z=0))
            
        # 添加走廊扩展点云数据
        for i in range(len(self.map_corridor_enlarge)):
            self.corridor_enlarge_pc.points.append(
                Point32(x=self.map_corridor_enlarge[i,0], 
                       y=self.map_corridor_enlarge[i,1], 
                       z=0))

    def timer_callback(self):
        """定时器回调函数，发布地图数据"""
        # 更新时间戳
        current_time = self.get_clock().now().to_msg()
        self.map_pc.header.stamp = current_time
        self.map_pc_init.header.stamp = current_time
        self.corridor_enlarge_pc.header.stamp = current_time
        
        # 发布标记
        self.markers_pub.publish(self.map_markers)
        self.map_line_pub.publish(self.map_line_markers)
        
        # 发布点云数据
        self.get_logger().debug("Sending map pc from map handler")
        self.pointcloud_publisher.publish(self.map_pc)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        map_handler = MapHandler()
        rclpy.spin(map_handler)
    except Exception as e:
        print(f'Error in map handler node: {str(e)}')
    finally:
        # 清理资源
        map_handler.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
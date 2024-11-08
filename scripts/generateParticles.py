#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
import time
import random
from shapely import geometry as geo
from shapely.geometry import Polygon, mapping, MultiPoint
import numpy as np
import copy
import matplotlib.pyplot as plt
from sensor_msgs_py import point_cloud2  # 使用ROS2原生的point_cloud2

class ParticleGenerator(Node):
    def __init__(self):
        super().__init__('particle_generator')
        
        # 参数初始化
        self.step = 2  # 5=0.2m
        self.radius = 6
        self.AGmaps = []
        
        # 从参数服务器获取参数
        self.declare_parameter('bRescueRobot', False)
        self.bRescueRobot = self.get_parameter('bRescueRobot').value
        
        # 加载地图数据
        self.map = np.loadtxt("/home/xiefujing/research/area_graph/ws/map/picking_list_star_center.txt", 
                             delimiter=',')
        
        # 创建发布者
        self.pointcloud_publisher = self.create_publisher(
            PointCloud, 
            '/particles_for_init', 
            10)
            
        # 等待transformed AG MAP PC
        self.get_logger().info("Waiting for transformed AG MAP PC...")
        try:
            msg1 = self.create_subscription(
                PointCloud2,
                '/pubAGMapTransformedPC',
                self.generate_AG_map,
                10)
                
            # 订阅LiDAR点云
            self.subscription = self.create_subscription(
                PointCloud2,
                '/hesai/pandar',
                self.generate_particle_AG,
                10)
                
        except Exception as e:
            self.get_logger().error(f'Error during initialization: {str(e)}')

    def generate_particle_AG(self, msg):
        """生成基于区域图的粒子"""
        self.get_logger().info("Entering generate_particle_AG")
        particles_pc = PointCloud()
        particles_pc.header = msg.header
        particles_pc.header.frame_id = 'map'
        
        msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.get_logger().info(f'msg_time: {msg_time}')
        
        msg_time = round(float(msg_time), 2)
        GT = np.loadtxt("/home/xiefujing/research/area_graph/ws/GT/GTliosam2023-05-10-20-16-52.txt", 
                       delimiter='\t')
        
        where_res = np.where(GT == msg_time)
        gt_center = [GT[where_res[0][0], 1], GT[where_res[0][0], 2]]
        gt_center[0] += 0.25 * random.random()
        gt_center[1] += 0.25 * random.random()
        map_cycle = geo.Point(gt_center).buffer(self.radius)
        
        # 遍历区域，查看哪些区域与GTcycle相交
        for i in range(len(self.AGmaps)):
            np_area = np.array(self.AGmaps[i])
            poly_area = Polygon(np_area)
            x, y = poly_area.exterior.xy
            
            overlap = poly_area.intersects(map_cycle)
            if overlap:
                xmin, ymin, xmax, ymax = poly_area.bounds
                x = np.arange(np.floor(xmin * self.step) / self.step, 
                            np.ceil(xmax * self.step) / self.step, 
                            1 / self.step)
                y = np.arange(np.floor(ymin * self.step) / self.step, 
                            np.ceil(ymax * self.step) / self.step, 
                            1 / self.step)
                points = MultiPoint(np.transpose([np.tile(x, len(y)), np.repeat(y, len(x))]))

                result = points.intersection(poly_area)
                result = result.intersection(map_cycle)
                
                if isinstance(result, MultiPoint):
                    result_list = np.array([np.array((resu.x, resu.y)) for resu in np.array(result.geoms)])
                else:
                    continue

                for j in range(result_list.shape[0]):
                    particles_pc.points.append(Point32(x=result_list[j,0], 
                                                     y=result_list[j,1], 
                                                     z=float(i)))
                    
        self.pointcloud_publisher.publish(particles_pc)

    def generate_AG_map(self, msg):
        """生成所有区域的AGmaps"""
        # 使用point_cloud2转换点云数据
        pc_points = list(point_cloud2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True))
        points = np.array(pc_points)
        
        area = []
        for i in range(len(points)):
            if points[i][3] % 3 == 0:  # intensity存储在第4列
                area.clear()
                area.append([points[i][0], points[i][1]])
            elif points[i][3] % 3 == 1:
                area.append([points[i][0], points[i][1]])
            elif points[i][3] % 3 == 2:
                area.append([points[i][0], points[i][1]])
                copyarea = copy.deepcopy(area)
                self.AGmaps.append(copyarea)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        generator = ParticleGenerator()
        rclpy.spin(generator)
    except KeyboardInterrupt:
        pass
    finally:
        generator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
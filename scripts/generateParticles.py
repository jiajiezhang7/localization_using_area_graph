#!/home/xiefujing/anaconda3/bin/python
# -*- coding: UTF-8 -*-
import rospy
import std_msgs.msg
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
import time
import random
from shapely import geometry as geo
from shapely.geometry import Polygon, mapping
import numpy as np
from shapely.geometry import Polygon, MultiPoint
from math import ceil
from shapely import Polygon
import shapely
import rospy
# import pcl
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import ros_numpy
import copy

import matplotlib.pyplot as plt

map=np.loadtxt("/home/xiefujing/research/area_graph/ws/map/picking_list_star_center.txt",delimiter=',')
AGmaps=[]
bRescueRobot=rospy.get_param("bRescueRobot",False)

# step, 5=0.2m
step = 2
radius=6
# this is used in old hand made map
def generate_particle(msg):
    particlesPC = PointCloud()
    particlesPC.header = msg.header
    particlesPC.header.frame_id='/map'
    # print(1)
    msg_time=msg.header.stamp.to_sec()
    msg_time=round(float(msg_time),2)
    GT=np.loadtxt("/home/xiefujing/research/area_graph/ws/GT/GTliosam2023-05-24-20-54-47.txt",delimiter='\t')
    where_res=np.where(GT==msg_time)
    gt_center=[GT[where_res[0][0],1],GT[where_res[0][0],2]]
    #  add noise
    gt_center[0]+=0.25*random.random()
    gt_center[1]+=0.25*random.random()
    
    map_poly = Polygon(map)
    circ = geo.Point(gt_center).buffer(radius)
    xmin, ymin, xmax, ymax = map_poly.bounds  # -4.85674599573635, 37.174925051829, -4.85258684662671, 37.1842384372115
    x = np.arange(np.floor(xmin * step) / step, np.ceil(xmax * step) / step, 1 / step)  
    y = np.arange(np.floor(ymin * step) / step, np.ceil(ymax * step) / step, 1 / step)  
    points = MultiPoint(np.transpose([np.tile(x, len(y)), np.repeat(y, len(x))]))
    result = points.intersection(map_poly)
    result=result.intersection(circ)
    
    result_list=np.array( [np.array((resu.x, resu.y)) for resu in np.array(result.geoms) ] )
    for i in range(result_list.shape[0]):
        particlesPC.points.append(Point32(result_list[i,0],result_list[i,1],0))
    time.sleep(5)
    pointcloudPublisher.publish(particlesPC)
    
def generate_particle_AG(msg):
    print("entering generate_particle_AG")
    particlesPC = PointCloud()
    particlesPC.header = msg.header
    particlesPC.header.frame_id='/map'
    msg_time=msg.header.stamp.to_sec()
    print(msg_time)
    
    msg_time=round(float(msg_time),2)
    GT=np.loadtxt("/home/xiefujing/research/area_graph/ws/GT/GTliosam2023-05-10-20-16-52.txt",delimiter='\t')
    # GT=np.loadtxt("/home/xiefujing/research/area_graph/ws/GT/GTliosam2023-05-24-20-54-47.txt",delimiter='\t')
    
    where_res=np.where(GT==msg_time)
    gt_center=[GT[where_res[0][0],1],GT[where_res[0][0],2]]
    gt_center[0]+=0.25*random.random()
    gt_center[1]+=0.25*random.random()
    mapCycle = geo.Point(gt_center).buffer(radius)
    #traverse area, see which areas intersect with GTcycle
    for i in range(len(AGmaps)):
        npArea=np.array(AGmaps[i])
        polyArea = Polygon(npArea)
        x,y = polyArea.exterior.xy
        # fig=plt.plot(x,y)
        # plt.savefig("/home/xiefujing/research/area_graph/ws/map/AGmap"+str(i)+".png")
        # plt.cla()
        # maybe faster to calculate if a polygon intersect with a cycle instead of several points....so check with cycle first
        overlap=polyArea.intersects(mapCycle)
        if(overlap):
            xmin, ymin, xmax, ymax = polyArea.bounds  # -4.85674599573635, 37.174925051829, -4.85258684662671, 37.1842384372115
            x = np.arange(np.floor(xmin * step) / step, np.ceil(xmax * step) / step, 1 / step)  
            y = np.arange(np.floor(ymin * step) / step, np.ceil(ymax * step) / step, 1 / step)  
            points = MultiPoint(np.transpose([np.tile(x, len(y)), np.repeat(y, len(x))]))

            result = points.intersection(polyArea)
            result=result.intersection(mapCycle)
            #check the actually particles intersect the area
            if(isinstance(result,shapely.geometry.multipoint.MultiPoint)):
                result_list=np.array( [np.array((resu.x, resu.y)) for resu in np.array(result.geoms) ] )
            else:
                continue

            for j in range(result_list.shape[0]):
                #z coordinate means inside which area
                particlesPC.points.append(Point32(result_list[j,0],result_list[j,1],i))
                
    # time.sleep(2)
    pointcloudPublisher.publish(particlesPC)

#generate the map of all area in AGmaps, AGmaps will form different polygons
def generate_AG_map(msg):
    pc = ros_numpy.numpify(msg)
    points=np.zeros((pc.shape[0],4))
    points[:,0]=pc['x']
    points[:,1]=pc['y']
    points[:,2]=pc['z']
    points[:,3]=pc['intensity']
    # print("pc.shape",pc.shape)
    # fig=plt.plot( points[:,0], points[:,1])
    # plt.savefig("/home/xiefujing/research/area_graph/ws/map/allAGmap.png")
    # plt.cla()
    area=[]
    for i in range(pc.shape[0]):
        if(points[i][3]%3==0):
            area.clear()
            area.append([points[i][0],points[i][1]])
        elif(points[i][3]%3==1):
            area.append([points[i][0],points[i][1]])
        elif(points[i][3]%3==2):
            area.append([points[i][0],points[i][1]])
            ##################################!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!stupid python...............................
            copyarea=copy.deepcopy(area)
            AGmaps.append(copyarea)
if __name__ == '__main__':
    rospy.init_node('particlesGeneratorNode')
        #publish the particles
    pointcloudPublisher = rospy.Publisher("/particles_for_init", PointCloud, queue_size=10)
    # wait for transformed AG MAP PC, since I want to know each particle is located in which area
    msg1 = rospy.wait_for_message('/pubAGMapTransformedPC',PointCloud2, timeout=None)
    generate_AG_map(msg1)
    # getting lidar pointcloud, get GT  according to timestamp
    rospy.Subscriber("/hesai/pandar",  PointCloud2, generate_particle_AG)

    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        rospy.spin()
        rate.sleep()
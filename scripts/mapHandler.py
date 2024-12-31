#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import imp
import rospy
import numpy as np 
import os
import time

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
import std_msgs.msg

if __name__ == '__main__':
    rospy.init_node('mapHandlerNode')
    #time.sleep(5)
    markersPub = rospy.Publisher('mapMarkers', MarkerArray, queue_size=10)
    mapLinePub= rospy.Publisher('mapLine', Marker, queue_size=100)
    # map=np.loadtxt("../../../map/picking_list_old_bag.txt",delimiter=',')
    map=np.loadtxt("../../../map/picking_list_star_center.txt",delimiter=',')
    map_ini=np.loadtxt("../../../map/picking_list_star_center_initialization.txt",delimiter=',')
    map_corridor_enlarge=np.loadtxt("../../../map/corridor_enlarge.txt",delimiter=',')
    
    mapMarkers=MarkerArray()
    mapLineMarkers= Marker()
    mapLineMarkers.header.frame_id = '/map'
    mapLineMarkers.type = Marker.LINE_STRIP
    mapLineMarkers.scale.x = 0.1
    mapLineMarkers.color.b = 1.0
    mapLineMarkers.color.a = 1.0
    # use makerarray to represent map, is the area graph point organized or not?
    pointcloudPublisher = rospy.Publisher("/mapPC", PointCloud, queue_size=10)
    # corridorEnlargePublisher = rospy.Publisher("/corridorEnlargePC", PointCloud, queue_size=10)
    
    mapPC = PointCloud()
    #filling pointcloud header
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'map'
    mapPC.header = header
    corridorEnlargePC=  PointCloud()
    corridorEnlargePC.header = header
    
    # init
    pointcloudIniPublisher = rospy.Publisher("/mapPCInit", PointCloud, queue_size=10)
    mapPCInit = PointCloud()
    mapPCInit.header = header
    
#publish map pointcloud 

    for i in range(len(map)):
        mapMarker=Marker()
        mapMarker.header.frame_id="/map"
        mapMarker.header.stamp    = rospy.get_rostime()
        mapMarker.id = i
        mapMarker.type = 2 # sphere
        mapMarker.action = 0
        mapMarker.pose.position.x = map[i,0]
        mapMarker.pose.position.y = map[i,1]
        mapMarker.pose.position.z = 0
        mapMarker.pose.orientation.x = 0
        mapMarker.pose.orientation.y = 0
        mapMarker.pose.orientation.z = 0
        mapMarker.pose.orientation.w = 1.0
        mapMarker.scale.x = 0.1
        mapMarker.scale.y = 0.1
        mapMarker.scale.z = 0.1
        mapMarker.color.r = 0.0
        mapMarker.color.g = 1.0
        mapMarker.color.b = 0.0
        mapMarker.color.a = 1.0
        mapMarkers.markers.append(mapMarker)
        mapLineMarkers.points.append(Point(map[i,0],map[i,1],0))
        
        mapPC.points.append(Point32(map[i,0],map[i,1],0))
    for i in range(len(map_ini)):
        mapPCInit.points.append(Point32(map_ini[i,0],map_ini[i,1],0))
    for i in range(len(map_corridor_enlarge)):
        corridorEnlargePC.points.append(Point32(map_corridor_enlarge[i,0],map_corridor_enlarge[i,1],0))
        
        
    #mapMarker.action = Marker.ADD
    #mapMarker.lifetime = rospy.Duration()
    time.sleep(2)
    rate = rospy.Rate(1)
    pubtime=1
    markersPub.publish(mapMarkers)
    mapLinePub.publish(mapLineMarkers)
    # pointcloudPublisher.publish(mapPCInit)
    # corridorEnlargePublisher.publish(corridorEnlargePC)
    #after initialization, send whole map to algorithm
    # time.sleep(2)
    # corridorEnlargePublisher.publish(corridorEnlargePC)
    # time.sleep(2)
    print("sending map pc from map handler")
    # USE AG map now
    # pointcloudPublisher.publish(mapPC)
    
    # pointcloudIniPublisher.publish(mapPCInit)
    
    while not rospy.is_shutdown():
        rospy.spin()
        # markersPub.publish(mapMarkers)
        # mapLinePub.publish(mapLineMarkers)
        # pubtime+=1
        # if pubtime<2000:
        #     pointcloudPublisher.publish(mapPC)
        
        rate.sleep()

    ''' for i in map:
        print(i[:,0:2]) '''
    #print(map)
#! /usr/bin/env python
import rospy
import geometry_msgs.msg as geometry_msgs
import os
import yaml

def callback(msg):
    rospy.loginfo("geting destination now")
    
    #for i in range(2):  
    with open(yamlpath, "a") as f:
        yaml.dump([[msg.pose.position.x,msg.pose.position.y,msg.pose.position.z,msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w]], f)
    print(msg.pose.position.x)
    print(msg.pose.orientation.w)

if __name__ == '__main__':
    # current_path = os.path.abspath(os.path.dirname(__file__))
    # print(current_path)
    #with open(yamlpath) as f:
        #temp = yaml.safe_load(f.read())
        #print(temp[1])
        # self.init_pose=temp[0]
        # self.waypoints =temp[1:]
    curpath = os.path.dirname(os.path.realpath(__file__))
    yamlpath = os.path.join(curpath, "destination.yaml") 
    if os.path.exists(yamlpath):
        os.remove(yamlpath)
    rospy.init_node('getDestinationFromRVIZ')
    rospy.loginfo("getDestinationFromRVIZ node started, click the 2D Nav Goal on RVIZ, the first one is the initial pose for amcl")
    rospy.Subscriber("/move_base_simple/goal", geometry_msgs.PoseStamped, callback)
    rospy.spin()

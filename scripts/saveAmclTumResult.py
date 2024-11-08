#!/home/xiefujing/anaconda3/envs/py3/bin/python3.6
import rospy
# Because of transformations
# import tf_conversions
import tf
import geometry_msgs.msg
from nav_msgs.msg import Odometry
if __name__ == '__main__':
    rospy.init_node('saveAmclTumResult')
    rospy.Subscriber("/odometry/imu",
                     Odometry,
                     callback)
    rospy.spin()
#!/usr/bin/env python
import rospy
# Because of transformations
# import tf_conversions
import tf
import geometry_msgs.msg
from nav_msgs.msg import Odometry

def callback(msg):
    br = tf.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "map"
    t.child_frame_id = "odom"
    t.transform.translation.x = msg.pose.pose.position.x
    t.transform.translation.y = msg.pose.pose.position.y
    t.transform.translation.z = 0.0
    # q=tf.transformations.quaternion_from_euler(0,0,msg.theta)
    # q = tf_conversions.transformations.quaternion_from_euler(0, 0, msg.theta)
    t.transform.rotation.x = msg.pose.pose.orientation.x
    t.transform.rotation.y = msg.pose.pose.orientation.y
    t.transform.rotation.z = msg.pose.pose.orientation.z
    t.transform.rotation.w = msg.pose.pose.orientation.w
    br.sendTransform((msg.pose.pose.position.x,msg.pose.pose.position.y,0),(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w),msg.header.stamp,"base_link","odom")

if __name__ == '__main__':
    rospy.init_node('amcl_send_odom_tf')
    # turtlename = rospy.get_param('~turtle')
    rospy.Subscriber("/odometry/imu_incremental",
                     Odometry,
                     callback)
    rospy.spin()
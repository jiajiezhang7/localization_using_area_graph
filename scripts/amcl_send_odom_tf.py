#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import geometry_msgs.msg
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

class AmclOdomTFBroadcaster(Node):
    def __init__(self):
        super().__init__('amcl_send_odom_tf')
        
        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Create the subscriber
        self.subscription = self.create_subscription(
            Odometry,
            '/odometry/imu_incremental',
            self.callback,
            10)
        
        self.subscription  # prevent unused variable warning

    def callback(self, msg):
        t = TransformStamped()

        # Read message content and assign it to corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        
        # Copy translation
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = 0.0
        
        # Copy rotation
        t.transform.rotation.x = msg.pose.pose.orientation.x
        t.transform.rotation.y = msg.pose.pose.orientation.y
        t.transform.rotation.z = msg.pose.pose.orientation.z
        t.transform.rotation.w = msg.pose.pose.orientation.w
        
        # Send the transformation
        self.tf_broadcaster.sendTransform(t)
        
        # Send base_link to odom transform
        t2 = TransformStamped()
        t2.header.stamp = msg.header.stamp
        t2.header.frame_id = 'odom'
        t2.child_frame_id = 'base_link'
        
        t2.transform.translation.x = msg.pose.pose.position.x
        t2.transform.translation.y = msg.pose.pose.position.y
        t2.transform.translation.z = 0.0
        
        t2.transform.rotation.x = msg.pose.pose.orientation.x
        t2.transform.rotation.y = msg.pose.pose.orientation.y
        t2.transform.rotation.z = msg.pose.pose.orientation.z
        t2.transform.rotation.w = msg.pose.pose.orientation.w
        
        self.tf_broadcaster.sendTransform(t2)

def main(args=None):
    rclpy.init(args=args)
    
    node = AmclOdomTFBroadcaster()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
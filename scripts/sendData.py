#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Pose
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

class InitDataSender(Node):
    def __init__(self):
        super().__init__('init_data_sender')
        
        # Parameters
        self.times = 0
        self.start = 1
        self.freq = 10  # send every 3 second
        
        # You might want to make these configurable through ROS parameters
        self.declare_parameter('bag_file', '/path/to/your/bagfile.db3')
        self.declare_parameter('start_timestamp', 1684932891.20)
        
        self.bag_file = self.get_parameter('bag_file').value
        self.start_timestamp = self.get_parameter('start_timestamp').value
        
        # Set up QoS profile for better reliability
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Create publisher
        self.pub_pc = self.create_publisher(
            PointCloud2,
            '/hesai/pandar',
            qos_profile
        )
        
        # Create subscriber
        self.subscription = self.create_subscription(
            Pose,
            'doneInit',
            self.callback,
            qos_profile
        )
        
        # Initialize rosbag reader
        self.initialize_bag_reader()
        
        # Wait for CloudHandler to be ready
        self.get_logger().info("Waiting for CloudHandler to be ready...")
        time.sleep(10)
        
        # Send first message
        self.send_first_message()

    def initialize_bag_reader(self):
        """Initialize the ROS2 bag reader"""
        storage_id = 'sqlite3'
        serialization_format = 'cdr'
        
        reader = rosbag2_py.SequentialReader()
        storage_options = rosbag2_py._storage.StorageOptions(
            uri=self.bag_file,
            storage_id=storage_id
        )
        converter_options = rosbag2_py._storage.ConverterOptions(
            input_serialization_format=serialization_format,
            output_serialization_format=serialization_format
        )
        
        reader.open(storage_options, converter_options)
        self.reader = reader
        
        # Get topic type
        topic_types = reader.get_all_topics_and_types()
        # Create a map for topic name to type
        self.type_map = {topic.name: topic.type for topic in topic_types}

    def deserialize_message(self, topic_name, data):
        """Deserialize message from bag"""
        msg_type = get_message(self.type_map[topic_name])
        msg = deserialize_message(data, msg_type)
        return msg

    def send_first_message(self):
        """Send the first message from the bag"""
        while self.reader.has_next():
            topic_name, data, timestamp = self.reader.read_next()
            
            if topic_name == '/hesai/pandar':
                msg = self.deserialize_message(topic_name, data)
                timestamp_sec = timestamp * 1e-9  # Convert nanoseconds to seconds
                
                if timestamp_sec > self.start_timestamp:
                    self.get_logger().info(f"Sending first message at timestamp: {timestamp_sec}")
                    time.sleep(2)
                    self.pub_pc.publish(msg)
                    time.sleep(1)
                    break

    def callback(self, msg):
        """Callback for doneInit messages"""
        self.get_logger().info("Getting done signal")
        
        while self.reader.has_next():
            topic_name, data, timestamp = self.reader.read_next()
            
            if topic_name == '/hesai/pandar':
                msg = self.deserialize_message(topic_name, data)
                timestamp_sec = timestamp * 1e-9
                
                if timestamp_sec > self.start_timestamp:
                    self.times += 1
                    if self.times % (self.freq * self.start) == 0:
                        self.start = self.times // self.freq + 1
                        self.get_logger().info(f"Sending after {self.times}, ts = {timestamp_sec}")
                        
                        time.sleep(1)
                        self.pub_pc.publish(msg)
                        time.sleep(1)
                        break

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = InitDataSender()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
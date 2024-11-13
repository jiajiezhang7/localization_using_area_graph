/**
 * @file data_sender.cpp
 * @brief 根据指定的时间戳,读取并发布LiDAR数据
 * @author Jiajie Zhang
 * @date 2024-11-09
 */


// 关键点：
    // - 订阅 "doneInit" 话题，等待全局定位完成的信号
    // - 发布 "/hesai/pandar" 话题的点云数据
    
#include "localization_using_area_graph/data_sender.hpp"
#include <rclcpp/serialization.hpp>

DataSender::DataSender()
    : Node("data_sender"),
      times_(0),
      start_(1),
      freq_(10),
      first_message_sent_(false)
{
    initializeParameters();
    initializePublishersSubscribers();
    initializeBagReader();
    
    // Wait for CloudHandler to be ready
    RCLCPP_INFO(this->get_logger(), "Waiting for CloudHandler to be ready...");
    std::this_thread::sleep_for(std::chrono::seconds(10));
    
    // Send first message
    sendFirstMessage();
}

void DataSender::initializeParameters()
{
    // Declare and get parameters
    this->declare_parameter("bag_file", "");
    this->declare_parameter("start_timestamp", 0.0);
    this->declare_parameter("frequency", 10);
    
    bag_file_ = this->get_parameter("bag_file").as_string();
    start_timestamp_ = this->get_parameter("start_timestamp").as_double();
    freq_ = this->get_parameter("frequency").as_int();

    if (bag_file_.empty()) {
        throw std::runtime_error("Bag file path not specified!");
    }
}

void DataSender::initializePublishersSubscribers()
{
    // Create QoS profile
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
    
    // Create publisher
    pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/hesai/pandar", qos);
        
    // Create subscriber
    done_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "doneInit", qos,
        std::bind(&DataSender::doneCallback, this, std::placeholders::_1));
}

void DataSender::initializeBagReader()
{
    if (!openBag()) {
        throw std::runtime_error("Failed to open bag file!");
    }
}

bool DataSender::openBag()
{
    try {
        reader_ = std::make_unique<rosbag2_cpp::Reader>();
        
        // Set up storage options
        rosbag2_storage::StorageOptions storage_options;
        storage_options.uri = bag_file_;
        storage_options.storage_id = "sqlite3";
        
        // Open bag file with default format (no need for converter options)
        reader_->open(storage_options);
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error opening bag file: %s", e.what());
        return false;
    }
}

void DataSender::sendFirstMessage()
{
    while (reader_->has_next()) {
        auto bag_message = reader_->read_next();
        
        // Check if message is from the point cloud topic
        if (bag_message->topic_name == "/hesai/pandar") {
            double msg_time = static_cast<double>(bag_message->time_stamp) * 1e-9;
            
            if (msg_time > start_timestamp_) {
                RCLCPP_INFO(this->get_logger(), 
                           "Sending first message at timestamp: %f", msg_time);
                           
                std::this_thread::sleep_for(std::chrono::seconds(2));
                deserializeAndPublish(bag_message);
                std::this_thread::sleep_for(std::chrono::seconds(1));
                
                first_message_sent_ = true;
                break;
            }
        }
    }
}

void DataSender::doneCallback(const geometry_msgs::msg::Pose::SharedPtr /*msg*/)
{
    RCLCPP_INFO(this->get_logger(), "Getting done signal");
    
    while (reader_->has_next()) {
        auto bag_message = reader_->read_next();
        
        if (bag_message->topic_name == "/hesai/pandar") {
            double msg_time = static_cast<double>(bag_message->time_stamp) * 1e-9;
            
            if (msg_time > start_timestamp_) {
                times_++;
                if (times_ % (freq_ * start_) == 0) {
                    start_ = times_ / freq_ + 1;
                    RCLCPP_INFO(this->get_logger(), 
                               "Sending after %d, ts = %f", times_, msg_time);
                               
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                    deserializeAndPublish(bag_message);
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                    break;
                }
            }
        }
    }
}

bool DataSender::deserializeAndPublish(
    const std::shared_ptr<rosbag2_storage::SerializedBagMessage>& msg)
{
    try {
        sensor_msgs::msg::PointCloud2 pc2_msg;
        rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
        rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization;
        serialization.deserialize_message(&serialized_msg, &pc2_msg);
        pc_pub_->publish(pc2_msg);
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), 
                     "Error deserializing message: %s", e.what());
        return false;
    }
}
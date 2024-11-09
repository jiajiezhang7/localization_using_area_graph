/**
 * @file data_sender.hpp
 * @brief Node for reading and sending rosbag data
 * @author Jiajie Zhang
 * @date 2024-11-09
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <memory>
#include <string>

class DataSender : public rclcpp::Node {
public:
    DataSender();
    ~DataSender() = default;

private:
    // Node parameters
    std::string bag_file_;      // Path to bag file
    double start_timestamp_;    // Start timestamp for playback
    int times_;                 // Counter for messages sent
    int start_;                // Start factor
    int freq_;                 // Sending frequency
    bool first_message_sent_;  // Flag for first message

    // ROS publishers and subscribers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr done_sub_;
    
    // Rosbag reader
    std::unique_ptr<rosbag2_cpp::Reader> reader_;

    // Methods
    void initializeParameters();
    void initializePublishersSubscribers();
    void initializeBagReader();
    bool openBag();
    void sendFirstMessage();
    void doneCallback(const geometry_msgs::msg::Pose::SharedPtr msg);
    bool deserializeAndPublish(
        const std::shared_ptr<rosbag2_storage::SerializedBagMessage>& msg);
};
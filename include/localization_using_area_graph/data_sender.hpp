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
    // 节点参数
    std::string bag_file_;      // rosbag文件路径
    double start_timestamp_;    // 播放起始时间戳
    int times_;                 // 已发送消息计数器
    int start_;                 // 起始因子，用于控制从哪条消息开始发送
    int freq_;                  // 发送频率
    bool first_message_sent_;   // 第一条消息是否已发送的标志

    // ROS发布器和订阅器
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;  // 点云数据发布器
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr done_sub_;  // 完成信号订阅器
    
    // Rosbag读取器
    std::unique_ptr<rosbag2_cpp::Reader> reader_;  // 用于读取rosbag文件的对象

    // 方法
    void initializeParameters();              // 初始化节点参数
    void initializePublishersSubscribers();   // 初始化发布器和订阅器
    void initializeBagReader();               // 初始化rosbag读取器
    bool openBag();                           // 打开rosbag文件
    void sendFirstMessage();                  // 发送第一条消息
    void doneCallback(const geometry_msgs::msg::Pose::SharedPtr msg);  // 完成信号回调函数
    bool deserializeAndPublish(
        const std::shared_ptr<rosbag2_storage::SerializedBagMessage>& msg);  // 反序列化并发布消息
};
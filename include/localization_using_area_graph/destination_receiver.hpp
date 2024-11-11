/**
 * @file destination_receiver.hpp
 * @brief Node for receiving and saving goal poses from RViz
 * @author Jiajie Zhang
 * @date 2024-11-09 
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <string>
#include <sys/stat.h>   // for mkdir
#include <unistd.h>     // for access

class DestinationReceiver : public rclcpp::Node {
public:
    DestinationReceiver();
    ~DestinationReceiver() = default;

private:
    // ROS subscribers
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;

    // File path
    std::string yaml_path_;

    // Methods
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void initializeYamlFile();
    bool savePoseToYaml(const geometry_msgs::msg::PoseStamped& pose);
    
    // Helper methods
    bool fileExists(const std::string& path);
    bool createDirectory(const std::string& path);
    bool createDirectoryIfNeeded(const std::string& path);
    std::string getDirectoryPath(const std::string& filePath);
    std::vector<double> poseToVector(const geometry_msgs::msg::Pose& pose);
};
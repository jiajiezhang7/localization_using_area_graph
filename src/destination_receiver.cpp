/**
 * @file destination_receiver.cpp
 * @brief 核心功能：接收RViz中设置的目标点
 * @author Jiajie Zhang
 * @date 2024-11-09
 */

// 关键点：
    // - 订阅 "/goal_pose" 话题
    // - 将目标点保存到yaml文件
    
#include "localization_using_area_graph/destination_receiver.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cstdio>    // for remove
#include <errno.h>   // for errno
#include <string.h> // for strerror

DestinationReceiver::DestinationReceiver()
    : Node("destination_receiver")
{
    // Get package path and create yaml path
    std::string pkg_dir = ament_index_cpp::get_package_share_directory("localization_using_area_graph");
    yaml_path_ = pkg_dir + "/config/destination.yaml";

    // Initialize yaml file
    initializeYamlFile();

    // Create subscription
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", qos,
        std::bind(&DestinationReceiver::goalCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), 
                "Destination receiver node started. Click the '2D Goal Pose' button in RViz2 to set destinations.");
    RCLCPP_INFO(this->get_logger(),
                "The first pose will be used as the initial pose for AMCL.");
}

bool DestinationReceiver::fileExists(const std::string& path) {
    return access(path.c_str(), F_OK) != -1;
}

std::string DestinationReceiver::getDirectoryPath(const std::string& filePath) {
    size_t pos = filePath.find_last_of("/\\");
    return (pos == std::string::npos) ? "" : filePath.substr(0, pos);
}

bool DestinationReceiver::createDirectory(const std::string& path) {
    int status = mkdir(path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    if (status != 0 && errno != EEXIST) {
        RCLCPP_ERROR(this->get_logger(), 
                     "Failed to create directory %s: %s", 
                     path.c_str(), strerror(errno));
        return false;
    }
    return true;
}

bool DestinationReceiver::createDirectoryIfNeeded(const std::string& filePath) {
    std::string dirPath = getDirectoryPath(filePath);
    if (dirPath.empty()) {
        return true;
    }

    if (fileExists(dirPath)) {
        return true;
    }

    // Create parent directories recursively
    size_t pos = 1;  // Skip leading '/'
    while ((pos = dirPath.find('/', pos)) != std::string::npos) {
        std::string subPath = dirPath.substr(0, pos);
        if (!createDirectory(subPath)) {
            return false;
        }
        pos++;
    }

    // Create the final directory
    return createDirectory(dirPath);
}

void DestinationReceiver::initializeYamlFile()
{
    try {
        // Remove existing file if it exists
        if (fileExists(yaml_path_)) {
            if (remove(yaml_path_.c_str()) != 0) {
                RCLCPP_ERROR(this->get_logger(), 
                            "Failed to remove existing YAML file: %s", 
                            strerror(errno));
                throw std::runtime_error("Failed to remove existing YAML file");
            }
            RCLCPP_INFO(this->get_logger(), "Removed existing YAML file");
        }

        // Create directory if it doesn't exist
        if (!createDirectoryIfNeeded(yaml_path_)) {
            throw std::runtime_error("Failed to create directory");
        }

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), 
                     "Failed to initialize YAML file: %s", e.what());
        throw;
    }
}

void DestinationReceiver::goalCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received new destination");

    try {
        if (savePoseToYaml(*msg)) {
            RCLCPP_INFO(this->get_logger(), 
                        "Saved pose: Position(x:%.2f, y:%.2f), Orientation(w:%.2f)",
                        msg->pose.position.x,
                        msg->pose.position.y,
                        msg->pose.orientation.w);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to save pose to YAML");
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), 
                     "Error processing pose: %s", e.what());
    }
}

bool DestinationReceiver::savePoseToYaml(const geometry_msgs::msg::PoseStamped& pose)
{
    try {
        // Convert pose to vector format
        std::vector<std::vector<double>> pose_data;
        pose_data.push_back(poseToVector(pose.pose));

        // Create YAML node
        YAML::Node node;
        node.push_back(pose_data);

        // Open file in append mode
        std::ofstream fout(yaml_path_, std::ios::app);
        if (!fout.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open YAML file for writing");
            return false;
        }

        // Write to file
        fout << node;
        // Add newline for better readability
        fout << "\n";
        
        // Close file
        fout.close();

        // Verify file was written successfully
        if (fout.fail()) {
            RCLCPP_ERROR(this->get_logger(), "Error occurred while writing to file");
            return false;
        }

        return true;

    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), 
                     "YAML error: %s", e.what());
        return false;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), 
                     "Error saving to YAML: %s", e.what());
        return false;
    }
}

std::vector<double> DestinationReceiver::poseToVector(
    const geometry_msgs::msg::Pose& pose)
{
    return {
        pose.position.x,
        pose.position.y,
        pose.position.z,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w
    };
}
/**
 * @file saveAmclTumResult.cpp
 * @author Jiajie Zhang
 * @brief Save AMCL localization results in TUM format for evaluation
 * @version 0.1
 * @date 2024-11-09
 * 
 * @details This node subscribes to AMCL localization results and saves them in 
 *          TUM RGB-D benchmark format for trajectory evaluation. 
 *          Key functionality includes:
 *
 * 1. Data Recording:
 *    - Subscribes to "/amcl_pose" topic
 *    - Extracts timestamp, position and orientation
 *    - Writes to output file in TUM format:
 *      timestamp tx ty tz qx qy qz qw
 *
 * 2. File Handling:
 *    - Opens output file in append mode
 *    - Sets fixed precision for floating point numbers
 *    - Ensures proper file closure on shutdown
 *
 * 3. Data Format:
 *    - Timestamps in seconds with nanosecond precision
 *    - Position in meters (x, y, z)
 *    - Orientation as quaternion (x, y, z, w)
 *    - Space-separated values
 * 
 * File Output Example:
 * ```
 * 1305031102.175304 -1.826588 0.475166 0.000000 0.000000 0.000000 0.098224 0.995164
 * 1305031102.275304 -1.827245 0.474217 0.000000 0.000000 0.000000 0.098345 0.995152
 * ```
 *
 * Usage:
 * ```bash
 * ros2 run localization_using_area_graph save_amcl_result
 * ```
 *
 * @note Output file location: 
 *       /home/jay/AGLoc_ws/robotPoseResult/AmclTumResult.txt
 *
 * @warning Ensure write permissions for output directory
 *          Existing file will be appended to, not overwritten
 *
 * @see https://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats
 *      For TUM format specification
 *
 * @copyright Copyright (c) 2024, ShanghaiTech University
 *            All rights reserved.
 */
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <fstream>

class AmclResultSaver : public rclcpp::Node {
public:
    AmclResultSaver() : Node("amcl_result_saver") {
        // Initialize output file stream
        ofs_.setf(std::ios::fixed);
        ofs_.precision(6);
        ofs_.open("/home/jay/AGLoc_ws/robotPoseResult/AmclTumResult.txt", 
                  std::ofstream::app);

        if(!ofs_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open output file!");
            return;
        }

        // Create subscription
        subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/amcl_pose", 10,
            std::bind(&AmclResultSaver::amclResultCallback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "AmclResultSaver initialized");
    }

    ~AmclResultSaver() {
        if(ofs_.is_open()) {
            ofs_.close();
        }
    }

private:
    void amclResultCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        if(!ofs_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Output file is not open!");
            return;
        }

        // Write pose data to file in TUM format
        ofs_ << rclcpp::Time(msg->header.stamp).seconds() << " "
             << msg->pose.pose.position.x << " "
             << msg->pose.pose.position.y << " "
             << msg->pose.pose.position.z << " "
             << msg->pose.pose.orientation.x << " "
             << msg->pose.pose.orientation.y << " "
             << msg->pose.pose.orientation.z << " "
             << msg->pose.pose.orientation.w << std::endl;

        RCLCPP_DEBUG(this->get_logger(), "Saved pose at time: %f", 
                     rclcpp::Time(msg->header.stamp).seconds());
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_;
    std::ofstream ofs_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    // Create and spin node
    auto node = std::make_shared<AmclResultSaver>();
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Error during execution: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}
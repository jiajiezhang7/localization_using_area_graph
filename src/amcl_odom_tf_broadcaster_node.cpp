/**
 * @file amcl_odom_tf_broadcaster_node.cpp
 * @brief Main entry point for AMCL odometry TF broadcaster
 * @author Jiajie Zhang
 * @date 2024-11-09 
 */

#include "localization_using_area_graph/amcl_odom_tf_broadcaster.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<AmclOdomTfBroadcaster>();
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Error during execution: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}
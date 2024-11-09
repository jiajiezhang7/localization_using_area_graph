/**
 * @file data_sender_node.cpp
 * @brief Main entry point for data sender node
 * @author Jiajie Zhang
 * @date 2024-11-09
 */

#include "localization_using_area_graph/data_sender.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<DataSender>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("data_sender"), 
                     "Error: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}
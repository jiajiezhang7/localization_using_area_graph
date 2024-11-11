/**
 * @file destination_receiver_node.cpp
 * @brief Main entry point for destination receiver node
 * @author Jiajie Zhang
 * @date 2024-11-09
 */

#include "localization_using_area_graph/destination_receiver.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<DestinationReceiver>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("destination_receiver"), 
                     "Error in destination receiver node: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}
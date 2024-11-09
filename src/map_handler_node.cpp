/**
 * @file map_handler_node.cpp
 * @brief Main entry point for map handler node
 * @author Jiajie Zhang
 * @date 2024-11-09
 */

#include "localization_using_area_graph/map_handler.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<MapHandler>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("map_handler"), 
                     "Error in map handler node: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}
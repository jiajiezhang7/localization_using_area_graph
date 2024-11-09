/**
 * @file particle_generator_node.cpp
 * @brief Main entry point for particle generator node
 * @author Jiajie Zhang
 * @date 2024-11-09
 */

#include "localization_using_area_graph/particle_generator.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<ParticleGenerator>();
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Error during execution: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}
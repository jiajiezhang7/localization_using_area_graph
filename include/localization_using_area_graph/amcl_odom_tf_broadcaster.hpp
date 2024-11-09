/**
 * @file amcl_odom_tf_broadcaster.hpp
 * @brief Node that broadcasts TF from AMCL odometry
 * @author Jiajie Zhang
 * @date 2024-11-09
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

class AmclOdomTfBroadcaster : public rclcpp::Node {
public:
    AmclOdomTfBroadcaster();
    ~AmclOdomTfBroadcaster() = default;

private:
    // Callback for odometry messages
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    
    // TF2 broadcaster
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    // Subscriber
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};
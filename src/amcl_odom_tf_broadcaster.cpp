/**
 * @file amcl_odom_tf_broadcaster.cpp
 * @brief Implementation of AmclOdomTfBroadcaster class
 * @author Jiajie Zhang
 * @date 2024-11-09
 */

#include "localization_using_area_graph/amcl_odom_tf_broadcaster.hpp"

AmclOdomTfBroadcaster::AmclOdomTfBroadcaster()
    : Node("amcl_odom_tf_broadcaster")
{
    // Initialize TF broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    // Create subscription
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/imu_incremental", qos,
        std::bind(&AmclOdomTfBroadcaster::odomCallback, this, std::placeholders::_1));
        
    RCLCPP_INFO(this->get_logger(), "AmclOdomTfBroadcaster node initialized");
}

void AmclOdomTfBroadcaster::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // Create and fill map->odom transform
    geometry_msgs::msg::TransformStamped map_to_odom;
    map_to_odom.header.stamp = this->get_clock()->now();
    map_to_odom.header.frame_id = "map";
    map_to_odom.child_frame_id = "odom";
    
    // Copy translation
    map_to_odom.transform.translation.x = msg->pose.pose.position.x;
    map_to_odom.transform.translation.y = msg->pose.pose.position.y;
    map_to_odom.transform.translation.z = 0.0;
    
    // Copy rotation
    map_to_odom.transform.rotation = msg->pose.pose.orientation;
    
    // Send transform
    tf_broadcaster_->sendTransform(map_to_odom);
    
    // Create and fill odom->base_link transform
    geometry_msgs::msg::TransformStamped odom_to_base;
    odom_to_base.header.stamp = msg->header.stamp;
    odom_to_base.header.frame_id = "odom";
    odom_to_base.child_frame_id = "base_link";
    
    odom_to_base.transform.translation.x = msg->pose.pose.position.x;
    odom_to_base.transform.translation.y = msg->pose.pose.position.y;
    odom_to_base.transform.translation.z = 0.0;
    
    odom_to_base.transform.rotation = msg->pose.pose.orientation;
    
    // Send transform
    tf_broadcaster_->sendTransform(odom_to_base);
}
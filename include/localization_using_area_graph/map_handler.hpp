/**
 * @file map_handler.hpp
 * @brief Node for handling and publishing Area Graph map data
 * @author Jiajie Zhang
 * @date 2024-11-09
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Core>
#include <vector>
#include <string>

class MapHandler : public rclcpp::Node {
public:
    MapHandler();
    ~MapHandler() = default;

private:
    // Map data storage
    std::vector<Eigen::Vector3d> map_points_;
    std::vector<Eigen::Vector3d> map_init_points_;
    std::vector<Eigen::Vector3d> map_corridor_points_;

    // Publishers - 完全匹配原始代码的发布器
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;        // mapMarkers
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr map_line_pub_;           // mapLine
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;           // /mapPC
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr init_pointcloud_pub_;      // /mapPCInit

    // Timer for periodic publishing
    rclcpp::TimerBase::SharedPtr timer_;

    // Map data containers
    visualization_msgs::msg::MarkerArray map_markers_;
    visualization_msgs::msg::Marker map_line_markers_;
    
    // Point cloud messages
    sensor_msgs::msg::PointCloud2 map_pc_;          // 主地图点云
    sensor_msgs::msg::PointCloud2 map_pc_init_;     // 初始化点云
    sensor_msgs::msg::PointCloud2 corridor_enlarge_pc_; // 走廊点云(只存储不发布)

    // Methods
    void loadMapData();
    void createMapMarkers();
    void timerCallback();
    void initializePublishers();
    void initializeMapMessages();

    // Utility methods
    bool loadMapDataFromFile(const std::string& filename, 
                           std::vector<Eigen::Vector3d>& points);
    visualization_msgs::msg::Marker createMarker(int id, 
                                               const Eigen::Vector3d& point);
    geometry_msgs::msg::Point32 createPoint32(const Eigen::Vector3d& point);
    
    // 辅助方法，用于将点云数据转换为ROS2消息
    void convertToPointCloud2Message(const std::vector<Eigen::Vector3d>& points,
                                   sensor_msgs::msg::PointCloud2& cloud_msg);
};
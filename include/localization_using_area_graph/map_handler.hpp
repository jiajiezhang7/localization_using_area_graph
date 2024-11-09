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
#include <sensor_msgs/msg/point_cloud.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
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

    // Publishers
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr map_line_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pointcloud_pub_;

    // Timer for periodic publishing
    rclcpp::TimerBase::SharedPtr timer_;

    // Map data containers
    visualization_msgs::msg::MarkerArray map_markers_;
    visualization_msgs::msg::Marker map_line_markers_;
    sensor_msgs::msg::PointCloud map_pc_;
    sensor_msgs::msg::PointCloud corridor_enlarge_pc_;
    sensor_msgs::msg::PointCloud map_pc_init_;

    // Methods
    void loadMapData();
    void createMapMarkers();
    void timerCallback();
    void initializePublishers();
    void initializeMapMessages();

    // Utility methods
    bool loadMapDataFromFile(const std::string& filename, std::vector<Eigen::Vector3d>& points);
    visualization_msgs::msg::Marker createMarker(int id, const Eigen::Vector3d& point);
    geometry_msgs::msg::Point32 createPoint32(const Eigen::Vector3d& point);
};
/**
 * @file particle_generator.hpp
 * @brief Node for generating initial pose particles
 * @author Jiajie Zhang
 * @date 2024-11-09
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/geometry.h>
#include <pcl/common/common.h>
#include <vector>
#include <random>
#include <rss/msg/wifi_location.hpp>
#include "WGS84toCartesian.h"
#include <eigen3/Eigen/Dense>
#include <visualization_msgs/msg/marker.hpp>

struct GeoCoordinate {
    double longitude;
    double latitude;
    double altitude;
};

Eigen::Vector3d CoordinateTransform(const GeoCoordinate& init, const GeoCoordinate& cur);

class ParticleGenerator : public rclcpp::Node {
public:
    ParticleGenerator();
    ~ParticleGenerator() = default;

private:
    // Parameters
    double step_;              // particle generation step size 粒子采样步长
    double radius_;           // search radius 搜索半径
    bool bRescueRobot_;      // rescue robot mode flag
    std::vector<std::vector<Eigen::Vector2d>> AGmaps_; // Area Graph map data
    
    // Publishers & Subscribers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr particle_pub_;        // 发布采样得到的
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr viz_particle_pub_;    // 发布可视化粒子
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr wifi_marker_pub_;   // 发布WiFi中心点标记

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;       // 订阅雷达点云话题
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr agmap_sub_;       // 订阅 ag_map (/pubAGMapTransformedPC)
    rclcpp::Subscription<rss::msg::WifiLocation>::SharedPtr wifi_sub_;
    
    // WiFi定位结果
    std::mutex wifi_mutex_;
    rss::msg::WifiLocation::SharedPtr latest_wifi_location_;
    bool received_wifi_location_{false};  // 标记是否收到过WiFi定位结果
    
    // root node的经纬度
    double root_longitude_;
    double root_latitude_;
    
    // 地图变换参数
    std::array<double, 3> map_extrinsic_trans_;  // AGmap到map的平移变换
    double map_yaw_angle_;  // AGmap到map的旋转角度（弧度）
    
    // Callbacks
    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void agmapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void wifiCallback(const rss::msg::WifiLocation::SharedPtr msg);
    
    // Utility functions
    void generateParticles(const rclcpp::Time& stamp, const std::array<double, 2>& wifi_center);
    bool checkIntersection(const Eigen::Vector2d& point, const std::vector<Eigen::Vector2d>& polygon);
    void publishWifiCenterMarker(const rclcpp::Time& stamp, double x, double y);

    // Random number generator
    std::random_device rd_;
    std::mt19937 gen_;
    std::uniform_real_distribution<> dist_;

    // Initialize parameters
    void initializeParameters();
    void initializePublishersSubscribers();
};
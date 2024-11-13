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
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr particle_pub_;        // 发布采样得到的

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;       // 订阅雷达点云话题
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr agmap_sub_;       // 订阅 ag_map (/pubAGMapTransformedPC)

    // Callbacks
    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void agmapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    
    // Utility functions
    void loadMapData();
    void generateParticles(const rclcpp::Time& stamp, const std::array<double, 2>& gt_center);
    bool checkIntersection(const Eigen::Vector2d& point, const std::vector<Eigen::Vector2d>& polygon);

    // Random number generator
    std::random_device rd_;
    std::mt19937 gen_;
    std::uniform_real_distribution<> dist_;

    // Initialize parameters
    void initializeParameters();
    void initializePublishersSubscribers();
};
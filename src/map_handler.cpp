/**
 * @file map_handler.cpp
 * @brief 加载并发布Area Graph地图数据
 * @author Jiajie Zhang
 * @date 2024-11-09
 */

// 关键点：
    // - 加载三种地图文件：主地图、初始化地图、走廊扩展地图
    // - 将地图数据转换为ROS消息并定期发布
    // - 发布地图可视化markers

#include "localization_using_area_graph/map_handler.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>

MapHandler::MapHandler()
    : Node("map_handler")
{
    // Initialize the node
    loadMapData();
    createMapMarkers();
    initializePublishers();
    initializeMapMessages();

    // Create timer for periodic publishing
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&MapHandler::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "Map Handler Node initialized successfully");
}

void MapHandler::initializePublishers()
{
    // Create QoS profile
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

    // Create publishers
    markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "mapMarkers", qos);
    map_line_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "mapLine", qos);
    pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud>(
        "/mapPC", qos);
}

void MapHandler::loadMapData()
{
    // 加载三个不同用途的地图文件
    try {
        // Get package share directory
        std::string pkg_dir = ament_index_cpp::get_package_share_directory("localization_using_area_graph");
        
        // TODO 硬编码路径问题
        // Load main map data
        std::string map_file = "/home/jay/AGLoc_ws/map/picking_list_star_center.txt";
        if (!loadMapDataFromFile(map_file, map_points_)) {
            throw std::runtime_error("Failed to load main map file");
        }

        // Load initialization map data
        std::string init_file = pkg_dir + "/home/jay/AGLoc_ws/map/picking_list_star_center_initialization.txt";
        if (!loadMapDataFromFile(init_file, map_init_points_)) {
            throw std::runtime_error("Failed to load initialization map file");
        }

        // Load corridor map data
        std::string corridor_file = pkg_dir + "/home/jay/AGLoc_ws/map/corridor_enlarge.txt";
        if (!loadMapDataFromFile(corridor_file, map_corridor_points_)) {
            throw std::runtime_error("Failed to load corridor map file");
        }

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load map files: %s", e.what());
        throw;
    }
}

// 这个函数，把所谓的三个不同形式的map文件，读取为了一系列 3d points
bool MapHandler::loadMapDataFromFile(const std::string& filename, 
                                   std::vector<Eigen::Vector3d>& points)
{
    std::ifstream file(filename);
    if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Could not open file: %s", filename.c_str());
        return false;
    }

    points.clear();
    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        Eigen::Vector3d point;
        char comma;
        ss >> point.x() >> comma >> point.y();
        point.z() = 0.0;  // Set z coordinate to 0
        points.push_back(point);
    }

    return true;
}

void MapHandler::createMapMarkers()
{
    // Create marker for map lines
    map_line_markers_.header.frame_id = "map";
    map_line_markers_.type = visualization_msgs::msg::Marker::LINE_STRIP;
    map_line_markers_.scale.x = 0.1;
    map_line_markers_.color.b = 1.0;
    map_line_markers_.color.a = 1.0;

    // Create point cloud messages
    map_pc_.header.frame_id = "map";
    map_pc_init_.header.frame_id = "map";
    corridor_enlarge_pc_.header.frame_id = "map";

    // Create markers for map points
    for (size_t i = 0; i < map_points_.size(); ++i) {
        // Add point to markers
        map_markers_.markers.push_back(createMarker(i, map_points_[i]));
        
        // Add point to line strip
        geometry_msgs::msg::Point p;
        p.x = map_points_[i].x();
        p.y = map_points_[i].y();
        p.z = 0.0;
        map_line_markers_.points.push_back(p);

        // Add point to point cloud
        map_pc_.points.push_back(createPoint32(map_points_[i]));
    }

    // Add initialization points
    for (const auto& point : map_init_points_) {
        map_pc_init_.points.push_back(createPoint32(point));
    }

    // Add corridor points
    for (const auto& point : map_corridor_points_) {
        corridor_enlarge_pc_.points.push_back(createPoint32(point));
    }
}

visualization_msgs::msg::Marker MapHandler::createMarker(int id, const Eigen::Vector3d& point)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = point.x();
    marker.pose.position.y = point.y();
    marker.pose.position.z = 0.0;

    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    marker.color.g = 1.0;
    marker.color.a = 1.0;

    return marker;
}

geometry_msgs::msg::Point32 MapHandler::createPoint32(const Eigen::Vector3d& point)
{
    geometry_msgs::msg::Point32 p;
    p.x = point.x();
    p.y = point.y();
    p.z = 0.0;
    return p;
}

void MapHandler::timerCallback()
{
    // Update timestamp
    auto current_time = this->now();
    map_pc_.header.stamp = current_time;
    map_pc_init_.header.stamp = current_time;
    corridor_enlarge_pc_.header.stamp = current_time;

    // Publish markers
    markers_pub_->publish(map_markers_);
    map_line_pub_->publish(map_line_markers_);

    // Publish point clouds
    RCLCPP_DEBUG(this->get_logger(), "Sending map pc from map handler");
    pointcloud_pub_->publish(map_pc_);
}

void MapHandler::initializeMapMessages()
{
    // Initialize headers
    auto current_time = this->now();
    map_pc_.header.stamp = current_time;
    map_pc_init_.header.stamp = current_time;
    corridor_enlarge_pc_.header.stamp = current_time;
}
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
    pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/mapPC", qos);
    init_pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/mapPCInit", qos);
}

// 加载三个不同用途的地图文件
void MapHandler::loadMapData()
{
    try {
        // Get package share directory
        std::string pkg_dir = ament_index_cpp::get_package_share_directory("localization_using_area_graph");
        
        // Load main map data
        // TODO 这里把绝对路径改写为了pkg_dir + ""形式，竟然就无法正确读取了？
        std::string map_file = pkg_dir + "/data/map/picking_list_star_center.txt";
        if (!loadMapDataFromFile(map_file, map_points_)) {
            throw std::runtime_error("Failed to load main map file");
        }

        // Load initialization map data
        std::string init_file = pkg_dir + "/data/map/picking_list_star_center_initialization.txt";
        if (!loadMapDataFromFile(init_file, map_init_points_)) {
            throw std::runtime_error("Failed to load initialization map file");
        }

        // Load corridor map data
        std::string corridor_file = pkg_dir + "/data/map/corridor_enlarge.txt";
        if (!loadMapDataFromFile(corridor_file, map_corridor_points_)) {
            throw std::runtime_error("Failed to load corridor map file");
        }

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load map files: %s", e.what());
        throw;
    }
}

// 把每个点的z坐标设置为0的意图：picking_list中选择的点是每个ring距离最远的点，他们当然可能在不同的水平面，而ICP做的是2D的匹配
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

void MapHandler::convertToPointCloud2Message(
    const std::vector<Eigen::Vector3d>& points,
    sensor_msgs::msg::PointCloud2& cloud_msg)
{
    // 创建PCL点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl_cloud->points.resize(points.size());
    
    // 填充点云数据
    for(size_t i = 0; i < points.size(); i++) {
        pcl_cloud->points[i].x = points[i].x();
        pcl_cloud->points[i].y = points[i].y();
        pcl_cloud->points[i].z = 0.0;
    }

    // 设置点云基本属性
    pcl_cloud->width = points.size();
    pcl_cloud->height = 1;
    pcl_cloud->is_dense = true;

    // 转换为ROS2消息
    pcl::toROSMsg(*pcl_cloud, cloud_msg);
    cloud_msg.header.frame_id = "map";
}

void MapHandler::createMapMarkers()
{
    // Create marker for map lines
    map_line_markers_.header.frame_id = "map";
    map_line_markers_.type = visualization_msgs::msg::Marker::LINE_STRIP;
    map_line_markers_.scale.x = 0.1;
    map_line_markers_.color.b = 1.0;
    map_line_markers_.color.a = 1.0;

    // Convert point clouds from Eigen vectors to PointCloud2 messages
    convertToPointCloud2Message(map_points_, map_pc_);
    convertToPointCloud2Message(map_init_points_, map_pc_init_);
    convertToPointCloud2Message(map_corridor_points_, corridor_enlarge_pc_);

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
    
    // Update timestamps for all messages
    map_pc_.header.stamp = current_time;
    map_pc_init_.header.stamp = current_time;
    corridor_enlarge_pc_.header.stamp = current_time;

    // Update marker timestamps
    for (auto& marker : map_markers_.markers) {
        marker.header.stamp = current_time;
    }
    map_line_markers_.header.stamp = current_time;

    // Publish markers
    markers_pub_->publish(map_markers_);
    map_line_pub_->publish(map_line_markers_);

    // Publish point clouds
    RCLCPP_DEBUG(this->get_logger(), "Sending map pc from map handler");
    pointcloud_pub_->publish(map_pc_);         // 发布主地图点云
    init_pointcloud_pub_->publish(map_pc_init_); // 发布初始化点云
}

void MapHandler::initializeMapMessages()
{
    // Initialize all message headers
    auto current_time = this->now();
    
    // Initialize marker timestamps
    for (auto& marker : map_markers_.markers) {
        marker.header.stamp = current_time;
    }
    map_line_markers_.header.stamp = current_time;
    
    // Initialize point cloud timestamps
    map_pc_.header.stamp = current_time;
    map_pc_init_.header.stamp = current_time;
    corridor_enlarge_pc_.header.stamp = current_time;
}
/**
 * @file particle_generator.cpp
 * @brief 核心功能：为全局定位生成初始粒子猜测
 * @author Jiajie Zhang
 * @date 2024-11-09
 */

// 关键点：
    // - 根据WiFi定位的粗略位置生成候选粒子
    // - 对每个粒子检查是否在Area Graph的有效区域内
    // - 发布 "/particles_for_init" 话题

#include "localization_using_area_graph/particle_generator.hpp"

// 初始化部分
ParticleGenerator::ParticleGenerator()
    : Node("particle_generator"), 
      gen_(rd_()), 
      dist_(0.0, 1.0),
      received_wifi_location_(false)  // 新增标记变量
{
    initializeParameters();
    initializePublishersSubscribers();
    
    RCLCPP_INFO(this->get_logger(), "ParticleGenerator initialized");
}

// 参数配置
void ParticleGenerator::initializeParameters() 
{
    // Declare and get parameters
    this->declare_parameter("particle_generator_step", 2.0);  // 粒子采样步长
    this->declare_parameter("particle_generator_radius", 6.0);  //搜索半径
    this->declare_parameter("bRescueRobot", false);
    this->declare_parameter("root_long", 0.0);
    this->declare_parameter("root_lat", 0.0);
    
    // 声明地图变换参数
    this->declare_parameter("mapExtrinsicTrans", std::vector<double>{0.0, 0.0, 0.0});
    this->declare_parameter("mapYawAngle", 0.0);

    // 读取参数值
    step_ = this->get_parameter("particle_generator_step").as_double();
    radius_ = this->get_parameter("particle_generator_radius").as_double();
    bRescueRobot_ = this->get_parameter("bRescueRobot").as_bool();
    root_longitude_ = this->get_parameter("root_long").as_double();
    root_latitude_ = this->get_parameter("root_lat").as_double();
    
    // 读取地图变换参数
    auto trans_vec = this->get_parameter("mapExtrinsicTrans").as_double_array();
    if (trans_vec.size() >= 3) {
        map_extrinsic_trans_ = {trans_vec[0], trans_vec[1], trans_vec[2]};
    }
    map_yaw_angle_ = this->get_parameter("mapYawAngle").as_double() * M_PI / 180.0;  // 转换为弧度

    RCLCPP_INFO(this->get_logger(), "Loaded parameters - step: %.2f, radius: %.2f", 
                step_, radius_);
    RCLCPP_INFO(this->get_logger(), "Map transform - trans: [%.1f, %.1f, %.1f], yaw: %.1f deg", 
                map_extrinsic_trans_[0], map_extrinsic_trans_[1], map_extrinsic_trans_[2],
                map_yaw_angle_ * 180.0 / M_PI);
}

// 发布者订阅者初始化
void ParticleGenerator::initializePublishersSubscribers() 
{
    // Create QoS profile
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    
    // 发布用于初始化的采样粒子
    particle_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/particles_for_init", qos);
        
    // 订阅1: LiDAR 点云话题
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/hesai/pandar", qos,
        std::bind(&ParticleGenerator::lidarCallback, this, std::placeholders::_1));
    
    // 订阅2: osmAG地图中的node点云（经由mapAGCB处理，其坐标数值已被变换到map坐标系下）
    agmap_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/pubAGMapTransformedPC", qos,
        std::bind(&ParticleGenerator::agmapCallback, this, std::placeholders::_1));
    
    // 订阅3: WiFi定位结果
    wifi_sub_ = this->create_subscription<rss::msg::WifiLocation>(
        "/WifiLocation", qos,
        std::bind(&ParticleGenerator::wifiCallback, this, std::placeholders::_1));
}

Eigen::Vector3d CoordinateTransform(const GeoCoordinate& init, const GeoCoordinate& cur) {
    std::array<double, 2> reference{init.latitude, init.longitude};
    std::array<double, 2> current{cur.latitude, cur.longitude};
    std::array<double, 2> cur_xy;

    // 使用WGS84转换，不考虑旋转
    cur_xy = wgs84::toCartesian(reference, current);
    
    // 简单处理高度差，如果没有高度信息就默认为0
    double z = (cur.altitude - init.altitude);
    
    return {cur_xy[0], cur_xy[1], z};
}



void ParticleGenerator::wifiCallback(const rss::msg::WifiLocation::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(wifi_mutex_);
    latest_wifi_location_ = msg;
    received_wifi_location_ = true;  // 标记已收到WiFi定位结果
}

void ParticleGenerator::lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) 
{
    // 检查是否已收到WiFi定位结果
    {
        std::lock_guard<std::mutex> lock(wifi_mutex_);
        if (!received_wifi_location_) {
            RCLCPP_INFO(this->get_logger(), "等待WiFi定位结果...");
            return;  // 如果还没有收到WiFi定位结果，直接返回
        }
    }

    std::array<double, 2> wifi_center;
    
    // 获取最新的WiFi定位结果
    {
        std::lock_guard<std::mutex> lock(wifi_mutex_);
        GeoCoordinate root_coord;
        root_coord.longitude = root_longitude_;
        root_coord.latitude = root_latitude_;
        root_coord.altitude = 0.0;  // 高度设为0
        
        GeoCoordinate wifi_coord;
        wifi_coord.longitude = latest_wifi_location_->longitude;
        wifi_coord.latitude = latest_wifi_location_->latitude;
        wifi_coord.altitude = 0.0;  // 高度设为0
        
        // 转换为AGmap局部坐标
        Eigen::Vector3d local_pos = CoordinateTransform(root_coord, wifi_coord);
        
        // 应用AGmap到map的变换
        // 1. 旋转变换
        double cos_yaw = std::cos(map_yaw_angle_);
        double sin_yaw = std::sin(map_yaw_angle_);
        double x_rotated = local_pos.x() * cos_yaw - local_pos.y() * sin_yaw;
        double y_rotated = local_pos.x() * sin_yaw + local_pos.y() * cos_yaw;
        
        // 2. 平移变换
        wifi_center = {
            x_rotated + map_extrinsic_trans_[0],
            y_rotated + map_extrinsic_trans_[1]
        };
        // 已经通过检验，变换后的WiFi-location这里是正确的
        RCLCPP_INFO(this->get_logger(), "变换后的WiFi center: [%.2f, %.2f]", wifi_center[0], wifi_center[1]);
    }
    
    // 添加噪声
    wifi_center[0] += 0.25 * dist_(gen_);
    wifi_center[1] += 0.25 * dist_(gen_);
    
    // 生成并发布粒子
    generateParticles(msg->header.stamp, wifi_center);
}

void ParticleGenerator::generateParticles(const rclcpp::Time& stamp,
                                        const std::array<double, 2>& wifi_center) 
{
    // 创建PCL点云对象
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    
    for (double x = wifi_center[0] - radius_; x <= wifi_center[0] + radius_; x += 1.0/step_) {
        for (double y = wifi_center[1] - radius_; y <= wifi_center[1] + radius_; y += 1.0/step_) {
            Eigen::Vector2d point(x, y);
            
            // 2. 检查是否在搜索圆内
            if ((point - Eigen::Vector2d(wifi_center[0], wifi_center[1])).norm() > radius_) {   
                continue;
            }
            
            // 检查每个粒子是否在Area Graph的有效区域内
            for (size_t i = 0; i < AGmaps_.size(); i++) {
                if (checkIntersection(point, AGmaps_[i])) {
                    pcl::PointXYZI p;
                    p.x = x;
                    p.y = y;
                    p.z = i;  // 是 Area index 区域索引，这里的合理性在哪里？- (这里的i是区域索引，不是点云索引)
                    p.intensity = 1.0;
                    cloud->points.push_back(p);
                }
            }
        }
    }
    
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;

    // 转换为ROS消息并发布
    sensor_msgs::msg::PointCloud2 particles_msg;
    pcl::toROSMsg(*cloud, particles_msg);
    particles_msg.header.stamp = stamp;
    particles_msg.header.frame_id = "map";
    
    // Publish particles
    particle_pub_->publish(particles_msg);
}

void ParticleGenerator::agmapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) 
{
    // Convert point cloud to PCL format
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *cloud);
    
    // Process AG map points
    std::vector<Eigen::Vector2d> area;
    AGmaps_.clear();
    
    for (const auto& point : cloud->points) {
        if (static_cast<int>(point.intensity) % 3 == 0) {
            area.clear();
            area.emplace_back(point.x, point.y);
        } else if (static_cast<int>(point.intensity) % 3 == 1) {
            area.emplace_back(point.x, point.y);
        } else if (static_cast<int>(point.intensity) % 3 == 2) {
            area.emplace_back(point.x, point.y);
            AGmaps_.push_back(area);
        }
    }
}

bool ParticleGenerator::checkIntersection(const Eigen::Vector2d& point,
                                        const std::vector<Eigen::Vector2d>& polygon) {
    if (polygon.size() < 3) return false;
    
    int intersections = 0;
    size_t j = polygon.size() - 1;
    
    // 使用射线投射算法
    for (size_t i = 0; i < polygon.size(); i++) {
        // 检查点是否在多边形边界上
        if ((polygon[i][1] > point[1]) != (polygon[j][1] > point[1])) {
            double slope = (polygon[j][0] - polygon[i][0]) * (point[1] - polygon[i][1]) /
                         (polygon[j][1] - polygon[i][1]) + polygon[i][0];
            
            if (point[0] < slope)
                intersections++;
        }
        j = i;
    }
    
    // 奇数个交点表示点在多边形内部
    return (intersections % 2) == 1;
}
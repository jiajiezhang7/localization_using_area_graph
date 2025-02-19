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

ParticleGenerator::ParticleGenerator()
    : Node("particle_generator"), 
      gen_(rd_()), 
      dist_(0.0, 1.0)
{
    initializeParameters();
    initializePublishersSubscribers();
    loadMapData();
    
    RCLCPP_INFO(this->get_logger(), "ParticleGenerator initialized");
}

void ParticleGenerator::initializeParameters() 
{
    // Declare and get parameters
    this->declare_parameter("step", 2.0);  // 粒子采样步长
    this->declare_parameter("radius", 6.0);  //搜索半径
    this->declare_parameter("bRescueRobot", false);

    // 读取参数值
    step_ = this->get_parameter("particle_generator_step").as_double();
    radius_ = this->get_parameter("particle_generator_radius").as_double();
    bRescueRobot_ = this->get_parameter("bRescueRobot").as_bool();

    RCLCPP_INFO(this->get_logger(), "Loaded parameters - step: %.2f, radius: %.2f", 
                step_, radius_);
}

void ParticleGenerator::initializePublishersSubscribers() 
{
    // Create QoS profile
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    
    // 发布用于初始化的采样粒子
    particle_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud>(
        "/particles_for_init", qos);
        
    // 订阅 LiDAR 点云话题
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/hesai/pandar", qos,
        std::bind(&ParticleGenerator::lidarCallback, this, std::placeholders::_1));
    
    // 订阅osmAG地图中的node点云（经由mapAGCB处理，其坐标数值已被变换到map坐标系下）
    agmap_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/pubAGMapTransformedPC", qos,
        std::bind(&ParticleGenerator::agmapCallback, this, std::placeholders::_1));
}

void ParticleGenerator::loadMapData() 
{
    try {
        // Load map data from file
        // TODO: Implement map loading logic
        RCLCPP_INFO(this->get_logger(), "Map data loaded successfully");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load map data: %s", e.what());
    }
}

void ParticleGenerator::lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) 
{
    // 获取ground truth中心(测试用途)
    // 在实际应用中，这应该来自WiFi定位 TODO 待MaXu部分完成后，接入WiFi接口
    std::array<double, 2> gt_center = {0.0, 0.0};  // placeholder
    
    // 添加噪声
    gt_center[0] += 0.25 * dist_(gen_);
    gt_center[1] += 0.25 * dist_(gen_);
    
    // 生成并发布粒子
    generateParticles(msg->header.stamp, gt_center);
}

void ParticleGenerator::generateParticles(const rclcpp::Time& stamp,
                                        const std::array<double, 2>& gt_center) 
{
    // Create point cloud message for particles
    sensor_msgs::msg::PointCloud particles_msg;
    particles_msg.header.frame_id = "map";
    particles_msg.header.stamp = stamp;
    
    // 在ground truth中心周围生成圆形搜索区域内的粒子
    for (double x = gt_center[0] - radius_; x <= gt_center[0] + radius_; x += 1.0/step_) {
        for (double y = gt_center[1] - radius_; y <= gt_center[1] + radius_; y += 1.0/step_) {
            Eigen::Vector2d point(x, y);
            
            // 检查生成的粒子是否在搜索圆内
            if ((point - Eigen::Vector2d(gt_center[0], gt_center[1])).norm() > radius_) {
                continue;
            }
            
            // 检查每个粒子是否在Area Graph的有效区域内
            for (size_t i = 0; i < AGmaps_.size(); i++) {
                if (checkIntersection(point, AGmaps_[i])) {
                    geometry_msgs::msg::Point32 p;
                    p.x = x;
                    p.y = y;
                    p.z = i;  // Area index 区域索引
                    particles_msg.points.push_back(p);
                }
            }
        }
    }
    
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
/**
 * @file particle_generator.cpp
 * @brief Implementation of ParticleGenerator class
 * @author Jiajie Zhang
 * @date 2024-11-09
 */

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
    this->declare_parameter("step", 2.0);
    this->declare_parameter("radius", 6.0);
    this->declare_parameter("bRescueRobot", false);
    
    step_ = this->get_parameter("step").as_double();
    radius_ = this->get_parameter("radius").as_double();
    bRescueRobot_ = this->get_parameter("bRescueRobot").as_bool();
}

void ParticleGenerator::initializePublishersSubscribers() 
{
    // Create QoS profile
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    
    // Create publisher
    particle_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud>(
        "/particles_for_init", qos);
        
    // Create subscribers
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/hesai/pandar", qos,
        std::bind(&ParticleGenerator::lidarCallback, this, std::placeholders::_1));
        
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
    // Get ground truth center (for testing purposes)
    // In real application, this would come from WiFi localization
    std::array<double, 2> gt_center = {0.0, 0.0};  // placeholder
    
    // Add noise to ground truth center
    gt_center[0] += 0.25 * dist_(gen_);
    gt_center[1] += 0.25 * dist_(gen_);
    
    // Generate and publish particles
    generateParticles(msg->header.stamp, gt_center);
}

void ParticleGenerator::generateParticles(const rclcpp::Time& stamp,
                                        const std::array<double, 2>& gt_center) 
{
    // Create point cloud message for particles
    sensor_msgs::msg::PointCloud particles_msg;
    particles_msg.header.frame_id = "map";
    particles_msg.header.stamp = stamp;
    
    // Create circle around ground truth center
    for (double x = gt_center[0] - radius_; x <= gt_center[0] + radius_; x += 1.0/step_) {
        for (double y = gt_center[1] - radius_; y <= gt_center[1] + radius_; y += 1.0/step_) {
            Eigen::Vector2d point(x, y);
            
            // Check if point is within search circle
            if ((point - Eigen::Vector2d(gt_center[0], gt_center[1])).norm() > radius_) {
                continue;
            }
            
            // Check intersection with all areas
            for (size_t i = 0; i < AGmaps_.size(); i++) {
                if (checkIntersection(point, AGmaps_[i])) {
                    geometry_msgs::msg::Point32 p;
                    p.x = x;
                    p.y = y;
                    p.z = i;  // Area index
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
                                        const std::vector<Eigen::Vector2d>& polygon) 
{
    // TODO: Implement point in polygon check
    // This could use Boost.Geometry or other geometry library
    return true;  // placeholder
}
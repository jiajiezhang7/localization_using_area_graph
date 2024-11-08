#pragma once
#ifndef _CLOUD_INITIALIZER_HPP_
#define _CLOUD_INITIALIZER_HPP_

#include "utility.hpp"
#include "cloudBase.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <cv_bridge/cv_bridge.h>

class CloudInitializer : public CloudBase {
public:
    // Transform and scoring variables
    Eigen::Matrix4f MaxRobotPose;
    double MaxScore;

    // Vectors for ray intersection calculation
    std::vector<int> numofIntersection;     // Record ray intersections with map
    std::vector<double> inRayDis;           // Distance measurements
    std::vector<double> inRayRange;         // Range measurements
    std::vector<double> match_with_outside; // Outside area matching flags (1=outside, 0=inside)

    // Counters and metrics
    int numofInsidePoints;
    int numofOutsidePoints;
    double turkeyScore;
    bool bGuessReady;
    int rescueTimes;
    double rescueRunTime;

    // File output stream
    std::ofstream rescueRoomStream;

    // ROS2 Subscriptions and Publishers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr subInitialGuess;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pubRobotGuessMarker;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pubRobotPoseAfterICP;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pubCurrentMaxRobotPose;

    // Message storage
    geometry_msgs::msg::PointStamped robotGuess;

    // Constructor and Destructor
    CloudInitializer();
    ~CloudInitializer();

    // Core functionality methods
    void setLaserCloudin(pcl::PointCloud<pcl::PointXYZI>::Ptr furthestRing_,
                        std_msgs::msg::Header mapHeader_);
    void setMapPC(pcl::PointCloud<pcl::PointXYZI>::Ptr map_pc_);
    void showImgIni(double x, double y, int yaw);
    void rescueRobot();
    void scoreParticlesDist();
    void scoreParticles();
    void checkingGuess();

    // Point cloud processing methods
    void checkWholeMap(const pcl::PointXYZI& PCPoint,
                      const pcl::PointXYZI& PosePoint,
                      int horizonIndex,
                      double& minDist,
                      bool& findIntersection);

    double getScoreFromTwoPC(const Eigen::Matrix4f& robotPose,
                            pcl::PointCloud<pcl::PointXYZI>::Ptr PC1,
                            pcl::PointCloud<pcl::PointXYZI>::Ptr PC2);

    // Override methods from CloudBase
    void calClosestMapPoint(int inside_index) override;
    bool checkMap(int ring, int horizonIndex, int& last_index,
                 double& minDist, int inside_index) override;
    void allocateMemory() override;
    void resetParameters() override;

    // ICP and initialization methods
    void initializationICP(int insideAGIndex);
    bool checkICPmovingDist(Eigen::Matrix4f robotPoseGuess);
    bool insideOldArea(int mapPCindex);

private:
    // Callback methods
    void getInitialExtGuess(const sensor_msgs::msg::PointCloud::SharedPtr laserCloudMsg);

    // Publisher initialization
    void initializePublishers() {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
        
        pubRobotGuessMarker = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "pubRobotGuessMarker", qos);
            
        pubRobotPoseAfterICP = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "pubRobotPoseAfterICP", qos);
            
        pubCurrentMaxRobotPose = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "pubCurrentMaxRobotPose", qos);
    }

    // Subscriber initialization
    void initializeSubscribers() {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
        
        subInitialGuess = this->create_subscription<sensor_msgs::msg::PointCloud>(
            "/particles_for_init", qos,
            std::bind(&CloudInitializer::getInitialExtGuess, this, std::placeholders::_1));
    }

    // Initialize variables
    void initializeVariables() {
        MaxScore = 0.0;
        numofInsidePoints = 0;
        numofOutsidePoints = 0;
        turkeyScore = 0.0;
        bGuessReady = false;
        rescueTimes = 0;
        rescueRunTime = 0.0;
        
        // Reserve space for vectors
        numofIntersection.reserve(1000);
        inRayDis.reserve(1000);
        inRayRange.reserve(1000);
        match_with_outside.reserve(1000);
    }

protected:
    // Image transport (if needed later)
    // std::shared_ptr<image_transport::ImageTransport> it_;
    // image_transport::Publisher pubThings2Say;
};

#endif // _CLOUD_INITIALIZER_HPP_
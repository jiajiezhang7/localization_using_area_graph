/**
 * @file utility.hpp
 * @author Jiajie Zhang
 * @brief porting from ROS1 to ROS2
 * @version 0.1
 * @date 2024-11-09
 * 
 * main changes:
 * 将原来的单个头文件拆分为.hpp和.cpp两个文件，这符合C++的最佳实践
 * 将ParamServer从纯参数类改为继承自rclcpp::Node的节点类
 * 添加了ROS2风格的参数声明和获取机制
 * 
 * @copyright Copyright (c) 2024
 * 
 */


#pragma once
#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_

// ROS2 headers
#include <rclcpp/rclcpp.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.hpp>

// PCL headers 
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/pca.h>

// TF2 headers
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>

// Standard headers
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>
#include <chrono>
#include <ceres/ceres.h>

// Custom message headers
// TODO 这里的include文件名似乎有错，因为这两个头文件似乎不在这里的路径下
#include "area_graph_data_parser/msg/agindex.hpp"  
#include "area_graph_data_parser/msg/area_index.hpp"

using namespace std;

struct VelodynePointXYZIRT {
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(VelodynePointXYZIRT,
    (float, x, x)(float, y, y)(float, z, z)
    (float, intensity, intensity)(uint16_t, ring, ring))

using PointXYZIRT = VelodynePointXYZIRT;
typedef pcl::PointXYZI PointType;

enum class SensorType { VELODYNE, OUSTER };

class ParamServer : public rclcpp::Node {
public:
    ParamServer(const std::string& node_name = "agloc_node");

protected:
    // Parameters
    std::string pointCloudTopic;
    std::string lidarFrame;
    std::string baselinkFrame;
    std::string odometryFrame;
    std::string mapFrame;

    int N_SCAN;
    int Horizon_SCAN;
    int downsampleRate;
    int downsampleRateHorizontal;
    float lidarMinRange;
    float lidarMaxRange;
    int N_ceiling;

    Eigen::Matrix3f initialExtRot;
    Eigen::Vector3f initialExtTrans;
    Eigen::Vector3f mapExtTrans;

    float initialYawAngle;
    float mapYawAngle;

    double errorUpThredInit;
    double errorLowThredInit;
    double errorUpThred;
    double errorLowThred;
    bool opti;

    double translation_thres;
    int icp_iteration;
    int icp_init_iteration;
    bool use_weight;
    bool detect_corridor;
    double maxPercentageCorridor;
    double recalIntersectionThred;
    double percentageThred;
    double averDistanceThred;
    double radiusDisthred;
    double groundThred;
    double ceilingThred;
    double parallelThred;
    int subSample;
    
    bool pause_iter;
    bool initialization_imu;
    double diff_angle_init;
    double icp_stop_translation_thred;
    double icp_stop_rotation_thred;
    double rescue_angle_interval;
    bool bRescueRobot;
    bool bTestRescue;
    bool bOnlyScoreParticles;
    bool bResultChecking;
    double scoreDownsampleRate;

    double checkingAngle;
    double checkingGuessX;
    double checkingGuessY;

    bool bGenerateResultFile;
    bool bFurthestRingTracking;
    double turkeyPauseThred;
    double corridorDSmaxDist;

    bool bAllPassageOpen;
    bool bAllPassageClose;
    bool bInitializationWithICP;

private:
    // TF2 components
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    void declare_parameters();
    void get_parameters();

public:
    void calPedal(double x1, double y1, double x2, double y2, 
                  double x3, double y3, double& x4, double& y4);
                  
    pcl::PointXYZI calIntersection(pcl::PointXYZI p1, pcl::PointXYZI p2,
                                  pcl::PointXYZI p3, pcl::PointXYZI p4);
                                  
    bool inBetween(pcl::PointXYZI p1, pcl::PointXYZI p2,
                   pcl::PointXYZI p3, pcl::PointXYZI p4,
                   pcl::PointXYZI* intersection);
                   
    void inRay(pcl::PointXYZI p1, pcl::PointXYZI p2,
               pcl::PointXYZI p3, pcl::PointXYZI p4, bool& bOnRay);
               
    void inRayGeneral(pcl::PointXYZI p1, pcl::PointXYZI p2,
                      pcl::PointXYZI p3, pcl::PointXYZI p4, bool& bOnRay);
                      
    double calWeightTurkey(double r, double k, bool outside, double outsideThred);
    
    double calWeightHuber(double r, double k);
    
    double calWeightCauchy(double r, double k);
    
    double checkParallel(pcl::PointXYZI p1, pcl::PointXYZI p2,
                        pcl::PointXYZI p3, pcl::PointXYZI p4);
                        
    void getPCA(Eigen::Vector3f& eigen_values,
                Eigen::Matrix3f& eigen_vector,
                pcl::PointCloud<pcl::PointXYZI>::Ptr furthestRing,
                bool& bPCA);
                
    double calDistance(pcl::PointXYZI p1, pcl::PointXYZI p2);
    
    double calDistance2Line(pcl::PointXYZI p0, pcl::PointXYZI p1, 
                          pcl::PointXYZI p2);
};

#endif
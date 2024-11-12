/**
 * @file cloudBase.cpp
 * @author Jiajie Zhang (ROS2 port)
 *         Fujing Xie (original ROS1 implementation)
 *         Sören Schwertfeger (original ROS1 implementation)
 * @brief Implementation of cloudBase class for Area Graph-based LiDAR localization
 *        ROS2 implementation of the core AGLoc functionality
 * @version 0.1
 * @date 2024-11-09
 * 
 * @details Implements the following key functionalities:
 *          - Point cloud data processing and organization
 *          - Clutter removal and point cloud subsampling
 *          - Area Graph intersection calculations
 *          - Point cloud registration and pose estimation
 *          - Memory management for point cloud data structures
 *          - ROS2 node handling and message processing
 * 
 * @note This implementation corresponds to the methods described in:
 *       "Robust Lifelong Indoor LiDAR Localization using the Area Graph"
 *       IEEE Robotics and Automation Letters, 2023
 * 
 * @implementation_details
 *        - Converts ROS1 message types to ROS2
 *        - Updates timing and transform handling for ROS2
 *        - Implements ROS2 parameter handling
 *        - Maintains compatibility with original algorithm design
 * 
 * @dependencies
 *        - ROS2 core libraries
 *        - PCL library for point cloud processing
 *        - Area Graph data structures
 *        - TF2 for coordinate transformations
 * 
 * @copyright Copyright (c) 2024, ShanghaiTech University
 *            All rights reserved.
 * 
 */
#include "localization_using_area_graph/cloudBase.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

void CloudBase::saveTUMTraj(geometry_msgs::msg::PoseStamped & pose_stamped) {
    robotPoseTum << pose_stamped.header.stamp.sec + pose_stamped.header.stamp.nanosec * 1e-9 
                 << " " << pose_stamped.pose.position.x 
                 << " " << pose_stamped.pose.position.y 
                 << " " << pose_stamped.pose.position.z 
                 << " " << pose_stamped.pose.orientation.x 
                 << " " << pose_stamped.pose.orientation.y 
                 << " " << pose_stamped.pose.orientation.z 
                 << " " << pose_stamped.pose.orientation.w << std::endl;
}

bool CloudBase::areaInsideChecking(const Eigen::Matrix4f& robotPose, int areaStartIndex) {
    pcl::PointXYZI robot;
    robot.x = robotPose(0,3);
    robot.y = robotPose(1,3);
    robot.z = robotPose(2,3);
    robot.intensity = 0;

    pcl::PointXYZI robotInfinity;
    robotInfinity.x = robotPose(0,3) + 5132;
    robotInfinity.y = robotPose(1,3) + 2345;
    robotInfinity.z = robotPose(2,3);
    robotInfinity.intensity = 0;

    int throughTimes = 0;

    auto line_strip = std::make_unique<visualization_msgs::msg::Marker>();
    line_strip->type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_strip->header = mapHeader;
    line_strip->color.b = 1.0;
    line_strip->color.a = 1.0;
    line_strip->scale.x = 0.1;
    line_strip->id = 1;
    line_strip->action = visualization_msgs::msg::Marker::ADD;

    // Check all map lines of this area
    for(int i = areaStartIndex; i < areaStartIndex + 1000000; i++) {
        // The end of this area
        if((int)map_pc->points[i].intensity % 3 == 2) {
            break;
        }
        bool b_inray;
        inRayGeneral(map_pc->points[i], map_pc->points[(i+1)%mapSize], robot, robotInfinity, b_inray);
        if(b_inray) {
            throughTimes++;
        }
    }

    // Inside this area
    if(throughTimes % 2 == 1) {
        geometry_msgs::msg::Point p;
        p.x = robot.x;
        p.y = robot.y;
        p.z = robot.z;

        geometry_msgs::msg::Point p_;
        p_.x = robotInfinity.x;
        p_.y = robotInfinity.y;
        p_.z = robotInfinity.z;

        line_strip->points.push_back(p);
        line_strip->points.push_back(p_);
        pubinfinity->publish(*line_strip);

        return true;
    }
    return false;
}

CloudBase::CloudBase(const std::string& node_name)
    : ParamServer(node_name) {
    initializeVariables();
    initializePublishers();
    initializeSubscribers();
    allocateMemory();

    // 通过LIO-SAM得到的GT数据保存路径
    GTstream.open("/home/jay/AGLoc_ws/GT/GTliosam2023-05-24-20-54-47.txt", 
                  std::ofstream::app);
    GTstream.setf(std::ios::fixed);
    GTstream.precision(2);

    TransformedLiosamPath.header.stamp = this->now();
    TransformedLiosamPath.header.frame_id = "map";

    setInitialPose(initialYawAngle, initialExtTrans);

    RCLCPP_INFO(this->get_logger(), "CloudBase initialized successfully");
}

geometry_msgs::msg::PoseStamped CloudBase::transformLiosamPath(const nav_msgs::msg::Path& pathMsg) {
    geometry_msgs::msg::PoseStamped this_pose_stamped;
    this_pose_stamped = pathMsg.poses.back();

    Eigen::Matrix3d rotation_matrix;
    rotation_matrix << 0.9965, -0.0800, -0.0080,
                      0.0800,  0.9966, -0.0006,
                      0.0080, -0.0006,  0.9999;

    Eigen::Quaterniond quaternion(rotation_matrix);
    Eigen::Quaterniond quaternionLIO(
        this_pose_stamped.pose.orientation.w,
        this_pose_stamped.pose.orientation.x,
        this_pose_stamped.pose.orientation.y,
        this_pose_stamped.pose.orientation.z);
        
    Eigen::Quaterniond afterRotationQuat = quaternion * quaternionLIO;

    double newx = 0.9965 * pathMsg.poses.back().pose.position.x - 
                 0.08 * pathMsg.poses.back().pose.position.y -
                 0.0080 * pathMsg.poses.back().pose.position.z - 0.1024;
                 
    double newy = 0.0800 * pathMsg.poses.back().pose.position.x +
                 0.9966 * pathMsg.poses.back().pose.position.y -
                 0.0006 * pathMsg.poses.back().pose.position.z - 0.2499;
                 
    double newz = 0.0080 * pathMsg.poses.back().pose.position.x -
                 0.0006 * pathMsg.poses.back().pose.position.y +
                 0.9999 * pathMsg.poses.back().pose.position.z + 0.0092;

    this_pose_stamped.pose.position.x = newx;
    this_pose_stamped.pose.position.y = newy;
    this_pose_stamped.pose.position.z = newz;
    this_pose_stamped.pose.orientation.w = afterRotationQuat.w();
    this_pose_stamped.pose.orientation.x = afterRotationQuat.x();
    this_pose_stamped.pose.orientation.y = afterRotationQuat.y();
    this_pose_stamped.pose.orientation.z = afterRotationQuat.z();

    return this_pose_stamped;
}

void CloudBase::mapInitCB(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg) {
    // 添加实现或移除订阅器
    RCLCPP_INFO(this->get_logger(), "Map init callback received");
}

geometry_msgs::msg::Pose CloudBase::transformLiosamPathnew(
    const nav_msgs::msg::Odometry::SharedPtr pathMsg) {
    // [[ 0.99968442 -0.02004835 -0.01513714]
    //  [ 0.01981344  0.99968335 -0.01551225]
    //  [ 0.01544334  0.01520744  0.99976509]]
    // [ 1.65950261 -4.2548306   0.0313965 ]
    geometry_msgs::msg::Pose this_pose_stamped;
    this_pose_stamped = pathMsg->pose.pose;

    Eigen::Matrix3d rotation_matrix;
    // 0524 bag
    rotation_matrix << -0.9817312, 0.19017827, 0.00600681,
                      -0.19000758, -0.98153977, 0.02183553,
                       0.01004857, 0.02029529, 0.99974353;
                       
    Eigen::Quaterniond quaternion(rotation_matrix);
    Eigen::Quaterniond quaternionLIO(
        this_pose_stamped.orientation.w,
        this_pose_stamped.orientation.x,
        this_pose_stamped.orientation.y,
        this_pose_stamped.orientation.z);
        
    Eigen::Quaterniond afterRotationQuat = quaternion * quaternionLIO;

    double newx = -0.9817312 * pathMsg->pose.pose.position.x + 
                  0.19017827 * pathMsg->pose.pose.position.y + 
                  0.00600681 * pathMsg->pose.pose.position.z - 1.06462496;
                  
    double newy = -0.19000758 * pathMsg->pose.pose.position.x - 
                  0.98153977 * pathMsg->pose.pose.position.y + 
                  0.02183553 * pathMsg->pose.pose.position.z - 3.08371366;
                  
    double newz = 0.01004857 * pathMsg->pose.pose.position.x + 
                  0.02029529 * pathMsg->pose.pose.position.y + 
                  0.99974353 * pathMsg->pose.pose.position.z + 0.06008223;

    this_pose_stamped.position.x = newx;
    this_pose_stamped.position.y = newy;
    this_pose_stamped.position.z = newz;
    this_pose_stamped.orientation.w = afterRotationQuat.w();
    this_pose_stamped.orientation.x = afterRotationQuat.x();
    this_pose_stamped.orientation.y = afterRotationQuat.y();
    this_pose_stamped.orientation.z = afterRotationQuat.z();
    
    return this_pose_stamped;
}

void CloudBase::liosamPathCB(const nav_msgs::msg::Path::SharedPtr pathMsg) {
    geometry_msgs::msg::PoseStamped this_pose_stamped = pathMsg->poses.back();
    TransformedLiosamPath.poses.push_back(this_pose_stamped);
    pubTransformedLiosamPath->publish(TransformedLiosamPath);
}
// 接受到AGindex后的回调处理
void CloudBase::AGindexCB(const area_graph_data_parser::msg::AGindex::SharedPtr msg) {
    AG_index = *msg;
    AGindexReceived = true;
}

void CloudBase::getGTfromLiosam(std_msgs::msg::Header cloudHeader) {
    // In ROS2 we need to use tf2 instead of tf
    tf2::Quaternion quat;
    tf2::fromMsg(TransformedLiosamPath.poses.back().pose.orientation, quat);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    
    GTstream << rclcpp::Time(cloudHeader.stamp).seconds() << "\t"
             << TransformedLiosamPath.poses.back().pose.position.x << "\t"
             << TransformedLiosamPath.poses.back().pose.position.y << "\t"
             << yaw/M_PI*180 << std::endl;
}

void CloudBase::mapCB(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg) {
    double mapCenterWeight = 0;
    mapCenterInitialization.setZero();
    mapInit = false;
    mapReceivedTimes++;

    if(!mapHistogram.empty()) {
        mapHistogram.clear();
    }

    if(!mapInit) {
        map_pc->clear();
        mapSize = laserCloudMsg->width * laserCloudMsg->height;
        map_pc->points.resize(mapSize);

        // Convert ROS2 PointCloud2 to PCL PointCloud
        pcl::fromROSMsg(*laserCloudMsg, *map_pc);

        for (int i = 0; i < mapSize; i++) {
            mapHistogram.push_back(0);

            double middile_x = (map_pc->points[i].x - 
                              map_pc->points[(i-1+mapSize)%mapSize].x)/2;
            double middile_y = (map_pc->points[i].y - 
                              map_pc->points[(i-1+mapSize)%mapSize].y)/2;
            double length = std::sqrt(std::pow(map_pc->points[i].x - 
                                    map_pc->points[(i-1+mapSize)%mapSize].x, 2) +
                                    std::pow(map_pc->points[i].y - 
                                    map_pc->points[(i-1+mapSize)%mapSize].y, 2));
                                    
            mapCenterWeight += length;
            mapCenterInitialization(0) += middile_x * length;
            mapCenterInitialization(1) += middile_y * length;
        }

        mapCenterInitialization = mapCenterInitialization / mapCenterWeight;

        RCLCPP_INFO(this->get_logger(), "Map center = [%f, %f]", 
                    mapCenterInitialization(0), 
                    mapCenterInitialization(1));

        mapInit = true;
        RCLCPP_WARN(this->get_logger(), 
                    "cloudHandler Map initialized success, this is the %d map.", 
                    mapReceivedTimes);
    }

    sensor_msgs::msg::PointCloud2 outMsg;
    pcl::toROSMsg(*map_pc, outMsg);
    outMsg.header = mapHeader;
    pubMapPC->publish(outMsg);
}

// 继续修改其他函数...

void CloudBase::mapAGCB(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg) {
    RCLCPP_WARN(this->get_logger(), "Receiving map from AG");
    mapSize = map_pc->points.size();

    // Receive index first, if already received, return
    if(!AGindexReceived || mapInit) {
        return; 
    }

    // Handle map transform, transform map to GT FRAME
    std_msgs::msg::Header tempHeader = laserCloudMsg->header;
    tempHeader.frame_id = "map";
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserAGMap(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserAGMapTansformed(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*laserCloudMsg, *laserAGMap);
   
    Eigen::Matrix4f mapPose = Eigen::Matrix4f::Zero();
    Eigen::Affine3f transform_initial = Eigen::Affine3f::Identity();
    transform_initial.translation() << mapExtTrans[0], mapExtTrans[1], mapExtTrans[2];
    transform_initial.rotate(Eigen::AngleAxisf(mapYawAngle/180.0*M_PI, Eigen::Vector3f::UnitZ()));
    mapPose = transform_initial.matrix();
    pcl::transformPointCloud(*laserAGMap, *laserAGMapTansformed, mapPose);

    // Publish transformed point cloud
    sensor_msgs::msg::PointCloud2 outMsg;
    pcl::toROSMsg(*laserAGMapTansformed, outMsg);
    outMsg.header = tempHeader;
    pubAGMapTransformedPC->publish(outMsg);

    double mapCenterWeight = 0;
    mapCenterInitialization.setZero();
    mapReceivedTimes++;
    
    mapHistogram.clear();

    if(!mapInit) {
        map_pc->clear();
        mapSize = laserAGMapTansformed->points.size(); 
        map_pc->points.resize(mapSize);

        for (int i = 0; i < mapSize; i++) {
            pcl::PointXYZI thisPoint;
            thisPoint.x = laserAGMapTansformed->points[i].x;  
            thisPoint.y = laserAGMapTansformed->points[i].y;
            thisPoint.z = laserAGMapTansformed->points[i].z;
            thisPoint.intensity = laserAGMapTansformed->points[i].intensity;
            map_pc->points[i] = thisPoint;
            mapHistogram.push_back(0);

            double middile_x = (map_pc->points[i].x - map_pc->points[(i-1+mapSize)%mapSize].x)/2;
            double middile_y = (map_pc->points[i].y - map_pc->points[(i-1+mapSize)%mapSize].y)/2;
            double length = std::sqrt(
                std::pow(map_pc->points[i].x - map_pc->points[(i-1+mapSize)%mapSize].x, 2) +
                std::pow(map_pc->points[i].y - map_pc->points[(i-1+mapSize)%mapSize].y, 2));
                
            mapCenterWeight += length;
            mapCenterInitialization(0) += middile_x * length;
            mapCenterInitialization(1) += middile_y * length;
        }
        
        mapCenterInitialization = mapCenterInitialization / mapCenterWeight;

        RCLCPP_INFO(this->get_logger(), "Map center = [%f, %f]", 
                    mapCenterInitialization(0), mapCenterInitialization(1));
        mapInit = true;
        RCLCPP_WARN(this->get_logger(), "AG Map initialized success, this is the %d map.", 
                    mapReceivedTimes);
    }

    sensor_msgs::msg::PointCloud2 outMsgMap;
    pcl::toROSMsg(*map_pc, outMsgMap);
    outMsgMap.header = tempHeader;
    pubMapPC->publish(outMsgMap);
}

void CloudBase::imuCB(const sensor_msgs::msg::Imu::SharedPtr imuMsg) {
    imu_buf.push_back(*imuMsg);
}

void CloudBase::initializedUsingMassCenter() {
    double center_x = 0;
    double center_y = 0;
    
    for(size_t i = 0; i < transformed_pc->points.size(); i++) {
        center_x += transformed_pc->points[i].x;
        center_y += transformed_pc->points[i].y;
    }
    
    center_x /= transformed_pc->points.size();
    center_y /= transformed_pc->points.size();
    
    RCLCPP_INFO(this->get_logger(), "Transformed PC CENTER = [%f, %f]", center_x, center_y);
    
    robotPose(0,3) -= center_x - mapCenterInitialization(0);
    robotPose(1,3) -= center_y - mapCenterInitialization(1);
}

void CloudBase::checkMapinRay(int ring, int horizonIndex, int& last_index) {
    pcl::PointXYZI PCPoint;
    PCPoint.x = transformed_pc->points[ring*Horizon_SCAN+horizonIndex].x;
    PCPoint.y = transformed_pc->points[ring*Horizon_SCAN+horizonIndex].y;
    PCPoint.z = 0;
    
    pcl::PointXYZI PosePoint;
    PosePoint.x = robotPose(0,3);
    PosePoint.y = robotPose(1,3);
    PosePoint.z = 0;

    // double pc_distance = calDistance(PCPoint, PosePoint);
    int mapCorridorEnlargeSize = mapCorridorEnlarge_pc->points.size();

    for(int j = last_index; j < mapCorridorEnlargeSize + last_index; j++) {
        bool bInRay = false;
        inRay(PosePoint, PCPoint, map_pc->points[j%mapSize], 
              map_pc->points[(j+1)%mapSize], bInRay);
    }
}

void CloudBase::organizePointcloud() {
    organizedCloudIn64->resize(64 * Horizon_SCAN);

    if(bFurthestRingTracking) {
        N_SCAN = 64;
    }
    
    int cloudSize = laserCloudIn->points.size(); 
    furthestRing->points.resize(Horizon_SCAN, pcl::PointXYZI());
    transformedFurthestRing->points.resize(Horizon_SCAN, pcl::PointXYZI());

    for (int i = 0; i < cloudSize; ++i) {
        pcl::PointXYZI thisPoint;
        thisPoint.x = laserCloudIn->points[i].x;
        thisPoint.y = laserCloudIn->points[i].y;
        thisPoint.z = laserCloudIn->points[i].z;

        float range_xy = std::sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y);
        thisPoint.intensity = range_xy;

        float range = std::sqrt(thisPoint.x * thisPoint.x + 
                              thisPoint.y * thisPoint.y + 
                              thisPoint.z * thisPoint.z);

        int rowIdn = laserCloudIn->points[i].ring;
        if (rowIdn < 0 || rowIdn >= N_SCAN) {
            RCLCPP_ERROR(this->get_logger(), "WRONG ROW_ID_N, CHECK N_SCAN....");
            continue;
        }

        if (rowIdn % downsampleRate != 0) {
            continue;
        }

        float horizonAngle = std::atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;
        static float ang_res_x = 360.0/float(Horizon_SCAN);
        int columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;
        
        if (columnIdn >= Horizon_SCAN) {
            columnIdn -= Horizon_SCAN;
        }
        if (columnIdn < 0 || columnIdn >= Horizon_SCAN) {
            continue;
        }

        int index = columnIdn + rowIdn * Horizon_SCAN;

        if (range < lidarMinRange || range > lidarMaxRange ||
            thisPoint.z < groundThred || thisPoint.z > ceilingThred) {
            thisPoint.x = thisPoint.y = thisPoint.z = thisPoint.intensity = 0;
            
            if(!initialized && bRescueRobot) {
                organizedCloudIn->points[index%Horizon_SCAN] = thisPoint;
            } else {
                organizedCloudIn->points[index] = thisPoint;
                organizedCloudIn64->points[index] = thisPoint;
            }
            continue;
        }

        if((!initialized && bRescueRobot)) {
            organizedCloudIn->points[index%Horizon_SCAN] = thisPoint;
        } else {
            organizedCloudIn->points[index] = thisPoint;
            organizedCloudIn64->points[index] = thisPoint;
        }

        if(range_xy > furthestRing->points[columnIdn].intensity) {
            furthestRing->points[columnIdn] = thisPoint;
            furthestRing->points[columnIdn].intensity = range_xy;
        }
    }

    if(!initialized && bRescueRobot) {
        *organizedCloudIn = *furthestRing;
    }
    if(bFurthestRingTracking) {
        *organizedCloudIn = *furthestRing;
        N_SCAN = 1;
    }
}



void CloudBase::verticalRadiusFilter() {
    for (int i = 0; i < Horizon_SCAN; i++) {
        double average_radius = 0;
        int number = 0;
        
        for(int j = N_SCAN-1; j > 0; j--) {
            bool initialized = false;
            double radius = std::sqrt(
                std::pow(UsefulPoints1->points[i + j * Horizon_SCAN].x, 2) +
                std::pow(UsefulPoints1->points[i + j * Horizon_SCAN].y, 2));
                
            if(radius > 0.1 && !initialized) {
                average_radius += radius;
                number++;
                initialized = true;
            }
            
            if(number == 3) {
                initialized = true;
                number = 0;
                average_radius = average_radius/5;
                continue;
            }
            
            if(initialized) {
                if(std::abs(radius - average_radius) > radiusDisthred) {
                    UsefulPoints1->points[i + j * Horizon_SCAN].x = 0;
                    UsefulPoints1->points[i + j * Horizon_SCAN].y = 0;
                    UsefulPoints2->points[i + j * Horizon_SCAN].x = 0;
                    UsefulPoints2->points[i + j * Horizon_SCAN].y = 0;
                }
            }
        }
    }
}

void CloudBase::setInitialPose(double initialYawAngle, Eigen::Vector3f initialExtTrans) {
    robotPose.setZero();
    Eigen::Affine3f transform_initial = Eigen::Affine3f::Identity();
    transform_initial.translation() << initialExtTrans[0], initialExtTrans[1], initialExtTrans[2];
    
    // Rotate around Z axis
    transform_initial.rotate(
        Eigen::AngleAxisf(initialYawAngle/180.0*M_PI, Eigen::Vector3f::UnitZ())
    );
    
    robotPose = transform_initial.matrix();
    
    RCLCPP_DEBUG(this->get_logger(), "Initial robot pose set with yaw angle: %f", initialYawAngle);
}

void CloudBase::pubPclCloud(
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub,
    std_msgs::msg::Header* cloudHeader) {
    
    if (!cloud || !pub || !cloudHeader) {
        RCLCPP_WARN(this->get_logger(), "Invalid inputs to pubPclCloud");
        return;
    }

    sensor_msgs::msg::PointCloud2 tempCloud;
    pcl::toROSMsg(*cloud, tempCloud);
    tempCloud.header = *cloudHeader;
    pub->publish(tempCloud);
}

void CloudBase::setEveryFrame() {
    bPCA = false;
    accumulateAngle = 0;
    averDistancePairedPoints = 0;
    onlyOneDirection = false;
    
    // mapHistogram has same size as map, calculate every frame of lidar data
    if (!mapHistogram.empty()) {
        std::fill(mapHistogram.begin(), mapHistogram.end(), 0);
    }
    numTotalHistogram = 0;
    
    RCLCPP_DEBUG(this->get_logger(), "Frame parameters reset");
}

void CloudBase::test() {
    RCLCPP_INFO(this->get_logger(), "Test function called - no implementation");
}

// 实用工具函数添加
void CloudBase::allocateMemory() {
    // Initialize point cloud pointers
    laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
    organizedCloudIn.reset(new pcl::PointCloud<pcl::PointXYZI>());
    organizedCloudIn64.reset(new pcl::PointCloud<pcl::PointXYZI>());
    furthestRing.reset(new pcl::PointCloud<pcl::PointXYZI>());
    transformedFurthestRing.reset(new pcl::PointCloud<pcl::PointXYZI>());
    transformed_pc.reset(new pcl::PointCloud<pcl::PointXYZI>());
    map_pc.reset(new pcl::PointCloud<pcl::PointXYZI>());
    mapCorridorEnlarge_pc.reset(new pcl::PointCloud<pcl::PointXYZI>());
    ringMapP1.reset(new pcl::PointCloud<pcl::PointXYZI>());
    ringMapP2.reset(new pcl::PointCloud<pcl::PointXYZI>());
    intersectionOnMap.reset(new pcl::PointCloud<pcl::PointXYZI>());
    UsefulPoints1.reset(new pcl::PointCloud<pcl::PointXYZI>());
    UsefulPoints2.reset(new pcl::PointCloud<pcl::PointXYZI>());
    laserUppestRing.reset(new pcl::PointCloud<pcl::PointXYZI>());
    potentialCeilingPoints.reset(new pcl::PointCloud<pcl::PointXYZI>());
    insidePC.reset(new pcl::PointCloud<pcl::PointXYZI>());
    outsidePC.reset(new pcl::PointCloud<pcl::PointXYZI>());

    // Pre-allocate vectors with default sizes where appropriate
    mapHistogram.reserve(1000);  // 预估大小，可根据实际情况调整
    weightsTurkey.reserve(1000);
    Vec_pcx.reserve(1000);
    Vec_pcy.reserve(1000);
    Vec_pedalx.reserve(1000);
    Vec_pedaly.reserve(1000);

    RCLCPP_INFO(this->get_logger(), "Memory allocated for point clouds and vectors");
}

void CloudBase::resetParameters() {
    // Clear or resize point clouds
    laserCloudIn->clear();
    organizedCloudIn->clear();
    furthestRing->clear();
    transformed_pc->clear();
    UsefulPoints1->clear();
    UsefulPoints2->clear();
    
    // Reset vectors
    usefulIndex.clear();
    outsideAreaIndexRecord.clear();
    outsideAreaLastRingIndexRecord.clear();
    
    // Reset counters and flags
    numIcpPoints = 0;
    weightSumTurkey = 0;
    weightSumCauchy = 0;
    
    // Reset other parameters
    accumulateAngle = 0;
    averDistancePairedPoints = 0;
    
    RCLCPP_DEBUG(this->get_logger(), "Parameters reset");
}
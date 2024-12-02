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

// 检查机器人是否在区域内
bool CloudBase::areaInsideChecking(const Eigen::Matrix4f& robotPose, int areaStartIndex) {
    // 创建机器人当前位置点
    pcl::PointXYZI robot;
    robot.x = robotPose(0,3);
    robot.y = robotPose(1,3);
    robot.z = robotPose(2,3);
    robot.intensity = 0;

    // 创建一个远处的点,用于射线检测
    pcl::PointXYZI robotInfinity;
    robotInfinity.x = robotPose(0,3) + 5132;
    robotInfinity.y = robotPose(1,3) + 2345;
    robotInfinity.z = robotPose(2,3);
    robotInfinity.intensity = 0;

    // 记录射线穿过边界的次数
    int throughTimes = 0;

    // 创建可视化标记
    auto line_strip = std::make_unique<visualization_msgs::msg::Marker>();
    line_strip->type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_strip->header = mapHeader;
    line_strip->color.b = 1.0;
    line_strip->color.a = 1.0;
    line_strip->scale.x = 0.1;
    line_strip->id = 1;
    line_strip->action = visualization_msgs::msg::Marker::ADD;

    // 检查该区域所有边界线段
    for(int i = areaStartIndex; i < areaStartIndex + 1000000; i++) {
        // 到达区域末尾时退出
        if((int)map_pc->points[i].intensity % 3 == 2) {
            break;
        }
        bool b_inray;
        inRayGeneral(map_pc->points[i], map_pc->points[(i+1)%mapSize], robot, robotInfinity, b_inray);
        if(b_inray) {
            throughTimes++;
        }
    }

    // 如果射线穿过边界次数为奇数,说明点在区域内部
    if(throughTimes % 2 == 1) {
        // 可视化射线
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

// 初始化CloudBase类
CloudBase::CloudBase(const std::string& node_name)
    : ParamServer(node_name) {
    initializeVariables();
    initializePublishers();
    initializeSubscribers();
    allocateMemory();

    // 通过LIO-SAM得到的GT数据保存路径
    GTstream.open("/home/jay/AGLoc_ws/src/localization_using_area_graph/data/GT/GTliosam.txt", 
                  std::ofstream::app);
    GTstream.setf(std::ios::fixed);
    GTstream.precision(2);

    TransformedLiosamPath.header.stamp = this->now();
    TransformedLiosamPath.header.frame_id = "map";

    setInitialPose(initialYawAngle, initialExtTrans);

    RCLCPP_INFO(this->get_logger(), "CloudBase initialized successfully");
}

// only call this for 2022-11-14-19-07-34 bag
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

geometry_msgs::msg::Pose CloudBase::transformLiosamPathnew(const nav_msgs::msg::Odometry::SharedPtr pathMsg) {
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

//map and bag are not recorded in the same time, therefore bag's lio sam path has a transform with line map
// 接受到LIO-SAM路径后的回调处理
void CloudBase::liosamPathCB(const nav_msgs::msg::Path::SharedPtr pathMsg) {
    geometry_msgs::msg::PoseStamped this_pose_stamped = pathMsg->poses.back();
    TransformedLiosamPath.poses.push_back(this_pose_stamped);
    pubTransformedLiosamPath->publish(TransformedLiosamPath);
}

// Fujing源代码里没有实现
void CloudBase::mapInitCB(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg) {
    // 添加实现或移除订阅器
    RCLCPP_INFO(this->get_logger(), "Map init callback received");
}

// 接受到AGindex后的回调处理
void CloudBase::AGindexCB(const area_graph_data_parser::msg::AGindex::SharedPtr msg) {
    AG_index = *msg;
    AGindexReceived = true;
}

// 从LIO-SAM中获取GT数据
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

// 接受到地图数据后的回调处理，这是对于来自map_Handler的地图数据的处理，但是由于map_Handler没有被使用，所以这个函数没有被调用
void CloudBase::mapCB(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg) {
    // 地图中心点的权重
    double mapCenterWeight = 0;
    // 地图中心点的初始化向量(x,y)
    mapCenterInitialization.setZero();
    // 地图初始化标志位
    mapInit = false;
    // 地图接收次数计数器加1
    mapReceivedTimes++;

    // 如果直方图不为空,清空直方图
    if(!mapHistogram.empty()) {
        mapHistogram.clear();
    }

    // 如果地图未初始化
    if(!mapInit) {
        // 清空地图点云
        map_pc->clear();
        // 计算地图点云大小(宽x高)
        mapSize = laserCloudMsg->width * laserCloudMsg->height;
        // 调整地图点云大小
        map_pc->points.resize(mapSize);

        // 将ROS2 PointCloud2消息转换为PCL点云格式
        pcl::fromROSMsg(*laserCloudMsg, *map_pc);

        // 遍历所有点
        for (int i = 0; i < mapSize; i++) {
            // 初始化直方图
            mapHistogram.push_back(0);

            // 计算当前点与前一个点的中点x坐标
            double middile_x = (map_pc->points[i].x - 
                              map_pc->points[(i-1+mapSize)%mapSize].x)/2;
            // 计算当前点与前一个点的中点y坐标                              
            double middile_y = (map_pc->points[i].y - 
                              map_pc->points[(i-1+mapSize)%mapSize].y)/2;
            // 计算当前点与前一个点的距离                              
            double length = std::sqrt(std::pow(map_pc->points[i].x - 
                                    map_pc->points[(i-1+mapSize)%mapSize].x, 2) +
                                    std::pow(map_pc->points[i].y - 
                                    map_pc->points[(i-1+mapSize)%mapSize].y, 2));
            
            // 累加权重                                
            mapCenterWeight += length;
            // 累加x方向加权和
            mapCenterInitialization(0) += middile_x * length;
            // 累加y方向加权和
            mapCenterInitialization(1) += middile_y * length;
        }

        // 计算地图中心点坐标(加权平均)
        mapCenterInitialization = mapCenterInitialization / mapCenterWeight;

        // 输出地图中心点坐标信息
        RCLCPP_INFO(this->get_logger(), "Map center = [%f, %f]", 
                    mapCenterInitialization(0), 
                    mapCenterInitialization(1));

        // 设置地图初始化完成标志
        mapInit = true;
        // 输出地图初始化成功信息
        RCLCPP_WARN(this->get_logger(), 
                    "cloudHandler Map initialized success, this is the %d map.", 
                    mapReceivedTimes);
    }

    // 将PCL点云转换回ROS2消息格式并发布
    sensor_msgs::msg::PointCloud2 outMsg;
    pcl::toROSMsg(*map_pc, outMsg);
    outMsg.header = mapHeader;
    pubMapPC->publish(outMsg);
}



// 真实被使用的map来自area_graph_data_parser的mapPC_AG
// AGLoc系统中，地图数据的流向：  topology_publisher -> /mapPC_AG -> CloudBase::mapAGCB()
/**
 * @brief 处理来自Area Graph的地图点云数据的回调函数
 * @details 该函数负责:
 *          1. 接收和转换Area Graph地图点云
 *          2. 对地图进行坐标变换
 *          3. 计算地图中心点
 *          4. 初始化地图相关参数
 * @param laserCloudMsg 输入的地图点云消息(ROS PointCloud2格式)
 */
void CloudBase::mapAGCB(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg) {
    RCLCPP_INFO(this->get_logger(), "Receiving map from AG");

    // 获取当前地图点云的大小
    mapSize = map_pc->points.size();

    // 检查是否已接收到AG索引数据和地图是否已初始化
    if(!AGindexReceived || mapInit) {
        return; 
    }

    // 设置地图坐标系为"map"
    std_msgs::msg::Header tempHeader = laserCloudMsg->header;
    tempHeader.frame_id = "map";
    
    // 创建PCL点云对象用于存储原始和变换后的地图
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserAGMap(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserAGMapTansformed(new pcl::PointCloud<pcl::PointXYZI>());

    // 将ROS消息转换为PCL点云格式
    try {
        RCLCPP_INFO(this->get_logger(), "Starting PCL conversion...");
        pcl::fromROSMsg(*laserCloudMsg, *laserAGMap);
        RCLCPP_INFO(this->get_logger(), "PCL conversion successful. Points: %zu", 
                    laserAGMap->points.size());

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "PCL conversion failed: %s", e.what());
        return;
    }
   
    // 构建地图变换矩阵
    Eigen::Matrix4f mapPose = Eigen::Matrix4f::Zero();
    Eigen::Affine3f transform_initial = Eigen::Affine3f::Identity();
    transform_initial.translation() << mapExtTrans[0], mapExtTrans[1], mapExtTrans[2];
    transform_initial.rotate(Eigen::AngleAxisf(mapYawAngle/180.0*M_PI, Eigen::Vector3f::UnitZ()));
    mapPose = transform_initial.matrix();

    // 对点云进行坐标变换
    try {
        pcl::transformPointCloud(*laserAGMap, *laserAGMapTansformed, mapPose);
        RCLCPP_INFO(this->get_logger(), "Point cloud transformation successful");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Transform failed: %s", e.what());
        return;
    }

    // 发布变换后的点云
    sensor_msgs::msg::PointCloud2 outMsg;
    pcl::toROSMsg(*laserAGMapTansformed, outMsg);
    outMsg.header = tempHeader;
    pubAGMapTransformedPC->publish(outMsg);

    // 初始化地图中心计算相关变量
    double mapCenterWeight = 0;
    mapCenterInitialization.setZero();
    mapReceivedTimes++;
    
    mapHistogram.clear();

    // 如果地图未初始化，进行初始化处理
    if(!mapInit) {
        map_pc->clear();
        mapSize = laserAGMapTansformed->points.size(); 
        map_pc->points.resize(mapSize);

        // 遍历所有点，进行地图初始化
        for (int i = 0; i < mapSize; i++) {
            // 创建并设置点的属性
            pcl::PointXYZI thisPoint;
            thisPoint.x = laserAGMapTansformed->points[i].x;  
            thisPoint.y = laserAGMapTansformed->points[i].y;
            thisPoint.z = laserAGMapTansformed->points[i].z;
            thisPoint.intensity = laserAGMapTansformed->points[i].intensity;

            // 验证点的intensity值是否有效
            int intensityMod3 = static_cast<int>(thisPoint.intensity) % 3;
            if(intensityMod3 < 0 || intensityMod3 > 2) {
                RCLCPP_WARN(this->get_logger(), 
                            "Invalid intensity value at point %d: %.3f (mod 3 = %d)",
                            i, thisPoint.intensity, intensityMod3);
            }

            // 存储点并初始化直方图
            map_pc->points[i] = thisPoint;
            mapHistogram.push_back(0);

            // 计算相邻点的中点和距离
            double middile_x = (map_pc->points[i].x - map_pc->points[(i-1+mapSize)%mapSize].x)/2;
            double middile_y = (map_pc->points[i].y - map_pc->points[(i-1+mapSize)%mapSize].y)/2;
            double length = std::sqrt(
                std::pow(map_pc->points[i].x - map_pc->points[(i-1+mapSize)%mapSize].x, 2) +
                std::pow(map_pc->points[i].y - map_pc->points[(i-1+mapSize)%mapSize].y, 2));
                
            // 累加权重和加权坐标
            mapCenterWeight += length;
            mapCenterInitialization(0) += middile_x * length;
            mapCenterInitialization(1) += middile_y * length;
        }
        
        // 计算地图中心点的加权平均坐标
        mapCenterInitialization = mapCenterInitialization / mapCenterWeight;

        // 输出地图中心点信息并设置初始化标志
        RCLCPP_INFO(this->get_logger(), "Map center = [%f, %f]", 
                    mapCenterInitialization(0), mapCenterInitialization(1));
        mapInit = true;
        RCLCPP_INFO(this->get_logger(), "AG Map initialized success, this is the %d map.", 
                    mapReceivedTimes);
    }

    // 发布处理后的地图点云
    sensor_msgs::msg::PointCloud2 outMsgMap;
    pcl::toROSMsg(*map_pc, outMsgMap);
    outMsgMap.header = tempHeader;
    pubMapPC->publish(outMsgMap);
}

void CloudBase::imuCB(const sensor_msgs::msg::Imu::SharedPtr imuMsg) {
    imu_buf.push_back(*imuMsg);
}

/**
 * @brief 使用质心法初始化机器人位姿
 * @details 该函数通过计算转换后点云的质心位置，并与地图的质心位置进行对比，
 *          来调整机器人的位姿。主要步骤包括:
 *          1. 计算转换后点云的质心坐标
 *          2. 输出质心位置信息
 *          3. 根据两个质心的差值调整机器人位姿
 */
void CloudBase::initializedUsingMassCenter() {
    // 初始化质心坐标
    double center_x = 0;
    double center_y = 0;
    
    // 累加所有点的坐标来计算质心
    for(size_t i = 0; i < transformed_pc->points.size(); i++) {
        center_x += transformed_pc->points[i].x;
        center_y += transformed_pc->points[i].y;
    }
    
    // 计算平均值得到质心坐标
    center_x /= transformed_pc->points.size();
    center_y /= transformed_pc->points.size();
    
    // 输出转换后点云的质心位置
    RCLCPP_INFO(this->get_logger(), "Transformed PC CENTER = [%f, %f]", center_x, center_y);
    
    // 根据质心差值调整机器人位姿
    // robotPose(0,3)和robotPose(1,3)分别表示机器人位姿矩阵中的x和y平移分量
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


/**
 * @brief 组织点云数据的主要函数
 * @details 该函数负责将无序的点云数据组织成有序结构,主要功能包括:
 *          1. 验证扫描参数的有效性
 *          2. 记录点云统计信息
 *          3. 初始化数据结构
 *          4. 处理每个点并计算相关参数
 *          5. 根据ring信息组织点云
 * 
 * @note 该函数期望输入的点云数据包含ring信息(如Velodyne点云)
 *       如果输入数据不包含ring信息可能会导致错误
 */
void CloudBase::organizePointcloud() {
    // 添加参数验证
    if (N_SCAN <= 0 || Horizon_SCAN <= 0) {
        RCLCPP_ERROR(get_logger(), "Invalid scan parameters: N_SCAN=%d, Horizon_SCAN=%d", 
                     N_SCAN, Horizon_SCAN);
        return;
    }

    if (!laserCloudIn || laserCloudIn->empty()) {
        RCLCPP_ERROR(get_logger(), "Empty input point cloud");
        return;
    }
    // 为点云分配内存前进行大小检查
    if (transformed_pc && transformed_pc->width > 0) {
        size_t cloud_size = transformed_pc->width * transformed_pc->height;
        if (cloud_size == 0) {
            RCLCPP_ERROR(get_logger(), "Invalid cloud size: width=%d, height=%d",
                        transformed_pc->width, transformed_pc->height);
            return;
        }
    }
    try {
        // 3. 初始化数据结构
        RCLCPP_DEBUG(get_logger(), "Resizing point clouds for organization...");
        organizedCloudIn64->resize(64 * Horizon_SCAN);
        
        // 正确初始化 furthestRing
        furthestRing->clear();
        furthestRing->width = Horizon_SCAN;   // 设置宽度
        furthestRing->height = 1;             // 设置高度为1（无序点云）
        furthestRing->points.resize(Horizon_SCAN);
        furthestRing->is_dense = false;       // 可能包含无效点

        // 2. 同样初始化transformedFurthestRing
        transformedFurthestRing->clear();
        transformedFurthestRing->width = Horizon_SCAN;
        transformedFurthestRing->height = 1;
        transformedFurthestRing->is_dense = false;
        transformedFurthestRing->points.resize(Horizon_SCAN);

        // 3. 添加验证
        if (furthestRing->width != Horizon_SCAN) {
            RCLCPP_ERROR(get_logger(), 
            "FurthestRing initialization failed: width=%zu, expected=%d",
            furthestRing->width, Horizon_SCAN);
            return;
        
        }
        if(bFurthestRingTracking) {
            N_SCAN = 64;
            RCLCPP_DEBUG(get_logger(), "FurthestRingTracking enabled, N_SCAN set to 64");
        }
        
        int cloudSize = laserCloudIn->points.size();

        furthestRing->points.resize(Horizon_SCAN, pcl::PointXYZI());
        transformedFurthestRing->points.resize(Horizon_SCAN, pcl::PointXYZI());

        // 4. 处理每个点
        int validPoints = 0;
        int filteredPoints = 0;
        
        for (int i = 0; i < cloudSize; ++i) {
            pcl::PointXYZI thisPoint;
            thisPoint.x = laserCloudIn->points[i].x;
            thisPoint.y = laserCloudIn->points[i].y;
            thisPoint.z = laserCloudIn->points[i].z;

            // 5. 计算点的距离信息
            float range_xy = std::sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y);
            thisPoint.intensity = range_xy;
            float range = std::sqrt(range_xy * range_xy + thisPoint.z * thisPoint.z);

            // 6. 获取行索引并验证
            int rowIdn = laserCloudIn->points[i].ring;
            if (rowIdn < 0 || rowIdn >= N_SCAN) {
                RCLCPP_ERROR(get_logger(), 
                    "Invalid ring index %d at point %d [x:%.2f, y:%.2f, z:%.2f]",
                    rowIdn, i, thisPoint.x, thisPoint.y, thisPoint.z);
                filteredPoints++;
                continue;
            }

            // 7. 应用降采样
            if (rowIdn % downsampleRate != 0) {
                filteredPoints++;
                continue;
            }

            // 8. 计算列索引
            float horizonAngle = std::atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;
            static float ang_res_x = 360.0/float(Horizon_SCAN);
            int columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;
            
            // 9. 验证列索引
            if (columnIdn >= Horizon_SCAN) {
                columnIdn -= Horizon_SCAN;
            }
            if (columnIdn < 0 || columnIdn >= Horizon_SCAN) {
                RCLCPP_DEBUG(get_logger(), 
                    "Point %d filtered: column index %d out of range", i, columnIdn);
                filteredPoints++;
                continue;
            }


            // 11. 处理点的存储
            int index = columnIdn + rowIdn * Horizon_SCAN;
            if (range < lidarMinRange || range > lidarMaxRange ||
                thisPoint.z < groundThred || thisPoint.z > ceilingThred) {
                // 清零点并存储
                thisPoint.x = thisPoint.y = thisPoint.z = thisPoint.intensity = 0;
                
                if(!initialized && bRescueRobot) {
                    organizedCloudIn->points[index%Horizon_SCAN] = thisPoint;
                } else {
                    organizedCloudIn->points[index] = thisPoint;
                    organizedCloudIn64->points[index] = thisPoint;
                }
                filteredPoints++;
                continue;
            }

            // 12. 存储有效点
            if((!initialized && bRescueRobot)) {
                organizedCloudIn->points[index%Horizon_SCAN] = thisPoint;
            } else {
                organizedCloudIn->points[index] = thisPoint;
                organizedCloudIn64->points[index] = thisPoint;
            }
            validPoints++;

            // 13. 更新最远点
            if(range_xy > furthestRing->points[columnIdn].intensity) {
                furthestRing->points[columnIdn] = thisPoint;
                furthestRing->points[columnIdn].intensity = range_xy;
            }
        }

        // 14. 记录处理结果
        RCLCPP_INFO(get_logger(), 
            "Point cloud organized: %d valid points, %d filtered points",
            validPoints, filteredPoints);

        // rescue without initialization
        if(!initialized && bRescueRobot) {

            if (furthestRing->width == 0) {
                RCLCPP_ERROR(get_logger(), "FurthestRing width is zero before assignment");
                return;
            }

            *organizedCloudIn = *furthestRing; 
            RCLCPP_DEBUG(get_logger(), "Rescue mode: Using furthestRing as organizedCloudIn");
        }
        if(bFurthestRingTracking) {
            if (furthestRing->width == 0 || furthestRing->points.empty()) {
                RCLCPP_ERROR(get_logger(), 
                    "Invalid furthestRing: width=%zu, points=%zu",
                    furthestRing->width, furthestRing->points.size());
                return;
            }
            *organizedCloudIn = *furthestRing; 
            N_SCAN = 1;
            RCLCPP_DEBUG(get_logger(), "FurthestRingTracking: Reset N_SCAN to 1");
        }


    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Error in organizePointcloud: %s", e.what());
        throw;
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
    
    
    transform_initial.rotate(
        Eigen::AngleAxisf(initialYawAngle/180.0*M_PI, Eigen::Vector3f::UnitZ())
    );
    // 仅PoseTracking模式下，直接将yaml中的参数转换为初始位姿矩阵
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
    // 添加前置检查
    if (N_SCAN <= 0 || Horizon_SCAN <= 0) {
        RCLCPP_ERROR(get_logger(), "Invalid N_SCAN(%d) or Horizon_SCAN(%d)", 
                     N_SCAN, Horizon_SCAN);
        return;
    }
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
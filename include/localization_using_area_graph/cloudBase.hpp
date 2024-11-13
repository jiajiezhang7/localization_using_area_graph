/**
 * @file cloudBase.hpp
 * @author Jiajie Zhang (ROS2 port)
 *         Fujing Xie (original ROS1 implementation)
 *         Sören Schwertfeger (original ROS1 implementation)
 * @brief Base class for Area Graph-based LiDAR localization system (AGLoc)
 *        Provides core functionality for indoor localization using Area Graph map representation
 *        Including global localization and pose tracking with clutter removal
 * @version 0.1
 * @date 2024-11-09
 * 
 * @details This class implements the base functionality for:
 *          - Point cloud processing and clutter removal
 *          - Area Graph map handling
 *          - Ray tracing and intersection calculation
 *          - Point-to-line ICP registration
 *          - Corridorness detection and scoring
 *          Ported from ROS1 to ROS2 maintaining full functionality
 * 
 * @note Part of the AGLoc system as described in:
 *       "Robust Lifelong Indoor LiDAR Localization using the Area Graph"
 *       Published in IEEE Robotics and Automation Letters, 2023
 * 
 * @copyright Copyright (c) 2024, ShanghaiTech University
 *            All rights reserved.
 * 
 */
#pragma once
#ifndef _CLOUD_BASE_HPP_
#define _CLOUD_BASE_HPP_

#include "utility.hpp"
class CloudBase : public ParamServer {
public:
    // ROS2 Subscribers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subMap;               //订阅 /mapPC     --- from map_handler
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subMapAG;             //订阅 /mapPC_AG  --- from topology_publisher (area_graph_data_parser)
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subMapInit;           //订阅 /AGindex   --- from topology_publisher (area_graph_data_parser)
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subLiosamPath;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subLiosamodometry_incremental;
    // 订阅AG_index话题 -- 来自 area_graph_data_parser
    rclcpp::Subscription<area_graph_data_parser::msg::AGindex>::SharedPtr subAGindex;

    // Headers
    std_msgs::msg::Header cloudHeader;
    std_msgs::msg::Header mapHeader;
    nav_msgs::msg::Path globalPath;

    // ROS2 Publishers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubUppestRing;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubFurthestRing;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubPotentialCeiling;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubtest;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubOrganizedCloudIn;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubIntersection;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubTransformedPC;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubTransformedWholePC;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubUsefulPoints1;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubUsefulPoints2;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubMapPC;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubAGMapTransformedPC;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubOptiPC;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubRobotPath;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubTransformedLiosamPath;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pubDONEsignal;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubInsidePC;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubOutsidePC;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pubinfinity;

    nav_msgs::msg::Path TransformedLiosamPath;

    // Flags and state variables
    bool bUseFurestestRing;
    bool bPCA;  // hope to use PCA to tell if the robot is inside corridor when rescue
    bool AGindexReceived;
    int lastInsideIndex;  // record current area index the robot is in
    int globalImgTimes;

    // Scoring variables
    double insideScore;
    double outsideScore;
    double insideTotalRange;
    double outsideTotalScore;

    /*
    organizedCloudIn size:
    Horizon_SCAN:
    N_SCAN*Horizon_SCAN: initialized or case3 initialization
     */
    /*
    three cases:
    reset every frame:
            laserCloudIn
            organizedCloudIn
            furthestRing
    reset every guess:
            transformed_pc
            ringMapP1
            ringMapP2
            intersectionOnMap
            insidePC
            outsidePC
    reset every icp iteration:
            UsefulPoints1
            UsefulPoints2
    */

    // Point cloud containers
    pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;
    pcl::PointCloud<pcl::PointXYZI>::Ptr organizedCloudIn;
    pcl::PointCloud<pcl::PointXYZI>::Ptr organizedCloudIn64;
    pcl::PointCloud<pcl::PointXYZI>::Ptr furthestRing;
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformedFurthestRing;
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_pc;
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_pc;
    pcl::PointCloud<pcl::PointXYZI>::Ptr mapCorridorEnlarge_pc;
    pcl::PointCloud<pcl::PointXYZI>::Ptr ringMapP1;
    pcl::PointCloud<pcl::PointXYZI>::Ptr ringMapP2;
    pcl::PointCloud<pcl::PointXYZI>::Ptr intersectionOnMap;
    pcl::PointCloud<pcl::PointXYZI>::Ptr UsefulPoints1;
    pcl::PointCloud<pcl::PointXYZI>::Ptr UsefulPoints2;
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserUppestRing;
    pcl::PointCloud<pcl::PointXYZI>::Ptr potentialCeilingPoints;
    pcl::PointCloud<pcl::PointXYZI>::Ptr insidePC;
    pcl::PointCloud<pcl::PointXYZI>::Ptr outsidePC;

    // State variables
    int mapSize;
    bool mapInit;
    std::deque<sensor_msgs::msg::Imu> imu_buf;
    Eigen::Matrix4f robotPose;

    // Centers and metrics
    Eigen::Vector2d mapCenter;
    Eigen::Vector2d PCCenter;
    int numIcpPoints;
    double IcpPointsPercentage;
    int mapReceivedTimes;
    double averDistancePairedPoints;

    Eigen::Vector2d mapCenterInitialization;
    Eigen::Vector2d PCCenterInitialization;

    // Index and record vectors
    std::vector<int> usefulIndex;
    std::vector<int> outsideAreaIndexRecord;
    std::vector<int> outsideAreaLastRingIndexRecord;

    // Thresholds and weights
    double errorUpThredCurr;
    double errorLowThredCurr;
    double weightSumTurkey;
    double weightSumCauchy;
    std::vector<double> weightsTurkey;
    double accumulateAngle;
    bool initialized;
    bool onlyOneDirection;

    // Vectors for computation
    std::vector<int> mapHistogram;
    std::vector<double> Vec_pcx;
    std::vector<double> Vec_pcy;
    std::vector<double> Vec_pedalx;
    std::vector<double> Vec_pedaly;
    std::vector<Eigen::Vector3f> corridorGuess;
    std::vector<Eigen::Vector3f> roomGuess;
    area_graph_data_parser::msg::AGindex AG_index;
    
    // Counters
    int numTotalHistogram;
    int currentIteration;
    int runTime;

    // Output streams
    std::ofstream GTstream;
    std::ofstream robotPoseTum;
    std::ofstream LiosamPoseTum;


    

    // 将默认构造函数改为接受节点名的构造函数
    explicit CloudBase(const std::string& node_name);
    
    // 添加虚析构函数
    virtual ~CloudBase() = default;

    // 处理map相关info的回调函数
    void mapCB(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg);
    void mapAGCB(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg);
    void AGindexCB(const area_graph_data_parser::msg::AGindex::SharedPtr msg);

    void organizePointcloud();
    void setInitialPose(double initialYawAngle, Eigen::Vector3f initialExtTrans);
    void pubPclCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub,
                    std_msgs::msg::Header* cloudHeader);
    void setEveryFrame();

    bool areaInsideChecking(const Eigen::Matrix4f& robotPose, int areaStartIndex);
    geometry_msgs::msg::PoseStamped transformLiosamPath(const nav_msgs::msg::Path& pathMsg);
    geometry_msgs::msg::Pose transformLiosamPathnew(const nav_msgs::msg::Odometry::SharedPtr pathMsg);
    // VIRTUAL functions
    virtual void calClosestMapPoint(int inside_index) = 0;
    virtual bool checkMap(int ring, int horizonIndex, int& last_index, double& minDist, int inside_index) = 0;
    virtual void allocateMemory() = 0;
    virtual void resetParameters() = 0;

    // Core processing functions
    void initializedUsingMassCenter();
    void liosamPathCB(const nav_msgs::msg::Path::SharedPtr pathMsg);

    // NOT USED functions (keeping for compatibility)
    void verticalRadiusFilter();
    void test();
    void glassReflectionFilter();
    void optimization();
    void checkMapinRay(int ring, int horizonIndex, int& last_index);
    void mapInitCB(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg);
    void imuCB(const sensor_msgs::msg::Imu::SharedPtr imuMsg);
    void getGTfromLiosam(std_msgs::msg::Header cloudHeader);
    void formFurthestRing();
    double calParticleDist(int ring, int horizonIndex, int& last_index);
    void saveTUMTraj(geometry_msgs::msg::PoseStamped& pose_stamped);

private:
    void initializePublishers() {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
        
        pubUppestRing = this->create_publisher<sensor_msgs::msg::PointCloud2>("uppestRing", qos);
        pubFurthestRing = this->create_publisher<sensor_msgs::msg::PointCloud2>("FurthestRing", qos);
        pubPotentialCeiling = this->create_publisher<sensor_msgs::msg::PointCloud2>("potentialCeiling", qos);
        pubtest = this->create_publisher<sensor_msgs::msg::PointCloud2>("pubtest", qos);
        pubOrganizedCloudIn = this->create_publisher<sensor_msgs::msg::PointCloud2>("pubOrganizedCloudIn", qos);
        pubIntersection = this->create_publisher<sensor_msgs::msg::PointCloud2>("pubIntersection", qos);
        pubTransformedPC = this->create_publisher<sensor_msgs::msg::PointCloud2>("pubTransformedPC", qos);
        pubTransformedWholePC = this->create_publisher<sensor_msgs::msg::PointCloud2>("pubTransformedWholePC", qos);
        pubUsefulPoints1 = this->create_publisher<sensor_msgs::msg::PointCloud2>("pubUsefulPoints1", qos);
        pubUsefulPoints2 = this->create_publisher<sensor_msgs::msg::PointCloud2>("pubUsefulPoints2", qos);
        pubMapPC = this->create_publisher<sensor_msgs::msg::PointCloud2>("pubMapPC", qos);
        pubAGMapTransformedPC = this->create_publisher<sensor_msgs::msg::PointCloud2>("pubAGMapTransformedPC", qos);
        pubOptiPC = this->create_publisher<sensor_msgs::msg::PointCloud2>("pubOptiPC", qos);
        pubRobotPath = this->create_publisher<nav_msgs::msg::Path>("RobotPath", qos);
        pubTransformedLiosamPath = this->create_publisher<nav_msgs::msg::Path>("TransformedLiosamPath", qos);
        pubDONEsignal = this->create_publisher<geometry_msgs::msg::Pose>("doneInit", qos);
        pubInsidePC = this->create_publisher<sensor_msgs::msg::PointCloud2>("pubInsidePC", qos);
        pubOutsidePC = this->create_publisher<sensor_msgs::msg::PointCloud2>("pubOutsidePC", qos);
        pubinfinity = this->create_publisher<visualization_msgs::msg::Marker>("pubinfinity", qos);
    }

    void initializeSubscribers() {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
        
        subMap = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/mapPC", qos,
            std::bind(&CloudBase::mapCB, this, std::placeholders::_1));
            
        subMapAG = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/mapPC_AG", qos,
            std::bind(&CloudBase::mapAGCB, this, std::placeholders::_1));
            
        subMapInit = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/mapPCInit", qos,
            std::bind(&CloudBase::mapInitCB, this, std::placeholders::_1));
            
        subImu = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu/data", qos,
            std::bind(&CloudBase::imuCB, this, std::placeholders::_1));
            
        subLiosamPath = this->create_subscription<nav_msgs::msg::Path>(
            "/lio_sam/mapping/path", qos,
            std::bind(&CloudBase::liosamPathCB, this, std::placeholders::_1));
            
            
        subAGindex = this->create_subscription<area_graph_data_parser::msg::AGindex>(
            "/AGindex", qos,
            std::bind(&CloudBase::AGindexCB, this, std::placeholders::_1));
    }

    void initializeVariables() {
        // Initialize flags
        mapInit = false;
        bUseFurestestRing = false;
        bPCA = false;
        AGindexReceived = false;
        initialized = false;
        onlyOneDirection = false;
        
        // Initialize counters
        mapReceivedTimes = 0;
        lastInsideIndex = -1;
        globalImgTimes = 0;
        numIcpPoints = 0;
        numTotalHistogram = 0;
        currentIteration = 0;
        runTime = 0;
        
        // Initialize metrics
        IcpPointsPercentage = 0.0;
        averDistancePairedPoints = 0.0;
        
        // Initialize transformation matrices and vectors
        robotPose.setZero();
        mapCenter.setZero();
        PCCenter.setZero();
        mapCenterInitialization.setZero();
        PCCenterInitialization.setZero();
        
        // Initialize scoring variables
        insideScore = 0.0;
        outsideScore = 0.0;
        insideTotalRange = 0.0;
        outsideTotalScore = 0.0;
        
        // Initialize weights
        weightSumTurkey = 0.0;
        weightSumCauchy = 0.0;
        accumulateAngle = 0.0;
        
        // Initialize vectors
        usefulIndex.clear();
        outsideAreaIndexRecord.clear();
        outsideAreaLastRingIndexRecord.clear();
        mapHistogram.clear();
        weightsTurkey.clear();
        Vec_pcx.clear();
        Vec_pcy.clear();
        Vec_pedalx.clear();
        Vec_pedaly.clear();
        corridorGuess.clear();
        roomGuess.clear();
    }
};
#endif
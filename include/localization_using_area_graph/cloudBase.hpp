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
#include <mutex> // 添加互斥锁头文件

class CloudBase : public ParamServer {
public:
    // ROS2 Subscribers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subMap;               //订阅 /mapPC     --- from map_handler
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subMapInit;           //订阅 /mapPC_Init --- from map_handler

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subMapAG;             //订阅 /mapPC_AG  --- 来自 topology_publisher (area_graph_data_parser)
    rclcpp::Subscription<area_graph_data_parser::msg::AGindex>::SharedPtr subAGindex;    // 订阅AG_index话题 -- 来自 topology_publisher (area_graph_data_parser)



    // 消息头部信息
    std_msgs::msg::Header cloudHeader;      // 点云数据的消息头
    std_msgs::msg::Header mapHeader;        // 地图数据的消息头
    nav_msgs::msg::Path globalPath;         // 全局路径

    // ROS2 发布器
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubFurthestRing;        // 发布最远激光环
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubtest;                // 测试用发布器
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubOrganizedCloudIn;    // 发布组织化的输入点云

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubTransformedPC;       // 发布变换后的点云



    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubMapPC;               // 发布地图点云（后续定位实际使用的AGMap）
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubAGMapTransformedPC;  // 发布变换后的AG地图点云
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubRobotPath;                     // 发布机器人路径

    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pubDONEsignal;               // 发布完成信号


    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pubinfinity;          // 发布无穷远点标记


    // 标志位和状态变量
    bool bUseFurestestRing;      // 是否使用最远环点云
    bool bPCA;                   // 是否使用PCA分析判断机器人是否在走廊中(用于救援模式)
    int lastInsideIndex;         // 记录机器人当前所在区域的索引
    int globalImgTimes;          // 全局定位迭代次数计数器

    // 评分相关变量
    double insideScore;          // 区域内匹配得分
    double outsideScore;         // 区域外匹配得分
    double insideTotalRange;     // 区域内总距离范围
    double outsideTotalScore;    // 区域外总评分

    /*
    organizedCloudIn size:
    Horizon_SCAN:
    N_SCAN*Horizon_SCAN: initialized or case3 initialization
     */
    /*
    three cases:
    reset every frame:        // 每帧重置
        laserCloudIn         // 原始激光点云
        organizedCloudIn     // 组织化的点云
        furthestRing         // 最远环

    reset every guess:        // 每次位姿猜测重置
        transformed_pc       // 变换后的点云
        ringMapP1           // 地图环点1
        ringMapP2           // 地图环点2
        intersectionOnMap   // 地图上的交点
        insidePC           // 内部点云
        outsidePC          // 外部点云

    reset every icp iteration:  // 每次ICP迭代重置
        UsefulPoints1         // 有用点1
        UsefulPoints2         // 有用点2
    */


    // Point cloud containers
    pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;  // 原始激光点云  --- 在PCL中注册的自定义点云类型
    pcl::PointCloud<pcl::PointXYZI>::Ptr organizedCloudIn;  // 组织化的点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr organizedCloudIn64;  // 64线组织化点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr furthestRing;  // 最远环
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformedFurthestRing;  // 变换后的最远环
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_pc;  // 变换后的点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_pc;  // 地图点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr mapCorridorEnlarge_pc;  // 扩大的走廊地图点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr ringMapP1;  // 地图环点1
    pcl::PointCloud<pcl::PointXYZI>::Ptr ringMapP2;  // 地图环点2
    pcl::PointCloud<pcl::PointXYZI>::Ptr intersectionOnMap;  // 地图上的交点
    pcl::PointCloud<pcl::PointXYZI>::Ptr UsefulPoints1;  // 有用点1
    pcl::PointCloud<pcl::PointXYZI>::Ptr UsefulPoints2;  // 有用点2
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserUppestRing;  // 激光最上环
    pcl::PointCloud<pcl::PointXYZI>::Ptr potentialCeilingPoints;  // 潜在天花板点
    pcl::PointCloud<pcl::PointXYZI>::Ptr insidePC;  // 内部点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr outsidePC;  // 外部点云

    // 地图和机器人状态相关变量
    int mapSize;                                    // 地图大小
    bool mapInit;                                   // 地图是否初始化
    std::deque<sensor_msgs::msg::Imu> imu_buf;      // IMU数据缓冲
    Eigen::Matrix4f robotPose;                      // 机器人位姿

    // 中心点和度量相关变量
    Eigen::Vector2d mapCenter;                      // 地图中心
    Eigen::Vector2d PCCenter;                       // 点云中心
    int numIcpPoints;                               // ICP点数量
    double IcpPointsPercentage;                     // ICP点百分比
    int mapReceivedTimes;                           // 地图接收次数
    double averDistancePairedPoints;                // 配对点平均距离

    Eigen::Vector2d mapCenterInitialization;        // 地图中心初始化
    Eigen::Vector2d PCCenterInitialization;         // 点云中心初始化

    // 索引和记录向量
    std::vector<int> usefulIndex;                   // 有用点索引
    std::vector<int> outsideAreaIndexRecord;        // 外部区域索引记录
    std::vector<int> outsideAreaLastRingIndexRecord;// 外部区域最后一环索引记录

    // 阈值和权重相关变量
    double errorUpThredCurr;                        // 当前上限阈值
    double errorLowThredCurr;                       // 当前下限阈值
    double weightSumTurkey;                         // Turkey权重和
    double weightSumCauchy;                         // Cauchy权重和
    std::vector<double> weightsTurkey;              // Turkey权重向量
    double accumulateAngle;                         // 累积角度
    bool initialized;                               // 是否初始化
    bool onlyOneDirection;                          // 是否只有一个方向

    // 计算用向量
    std::vector<int> mapHistogram;                  // 地图直方图
    std::vector<double> Vec_pcx;                    // 点云X坐标向量
    std::vector<double> Vec_pcy;                    // 点云Y坐标向量
    std::vector<double> Vec_pedalx;                 // 垂足X坐标向量
    std::vector<double> Vec_pedaly;                 // 垂足Y坐标向量
    std::vector<Eigen::Vector3f> corridorGuess;     // 走廊猜测
    std::vector<Eigen::Vector3f> roomGuess;         // 房间猜测
    static area_graph_data_parser::msg::AGindex AG_index;  // 区域图索引

    // 计数器
    int numTotalHistogram;                          // 总直方图数
    int currentIteration;                           // 当前迭代次数
    int runTime;                                    // 运行时间

    // 输出流
    std::ofstream GTstream;                         // 地面真值输出流
    std::ofstream robotPoseTum;                     // 机器人位姿TUM格式输出流



    // 将默认构造函数改为接受节点名的构造函数
    explicit CloudBase(const std::string& node_name);

    // 添加虚析构函数
    virtual ~CloudBase() = default;

    void mapAGCB(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg);
    void AGindexCB(const area_graph_data_parser::msg::AGindex::SharedPtr msg);

    void organizePointcloud();
    void setInitialPose(double initialYawAngle, Eigen::Vector3f initialExtTrans);
    void pubPclCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub,
                    std_msgs::msg::Header* cloudHeader);
    void setEveryFrame();

    bool areaInsideChecking(const Eigen::Matrix4f& robotPose, int areaStartIndex);
    // VIRTUAL functions
    virtual void calClosestMapPoint(int inside_index) = 0;
    virtual bool checkMap(int ring, int horizonIndex, int& last_index, double& minDist, int inside_index) = 0;
    virtual void allocateMemory() = 0;
    virtual void resetParameters() = 0;



    // NOT USED functions (keeping for compatibility)
    void verticalRadiusFilter();

    void saveTUMTraj(geometry_msgs::msg::PoseStamped& pose_stamped);


protected:
    // 互斥锁用于保护AGindexReceived
    static std::mutex agIndexMutex;
    static bool AGindexReceived;

    // 线程安全地获取AGindexReceived的值
    bool isAGIndexReceived() const;

private:
    void initializePublishers() {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

        pubFurthestRing = this->create_publisher<sensor_msgs::msg::PointCloud2>("FurthestRing", qos);
        pubtest = this->create_publisher<sensor_msgs::msg::PointCloud2>("pubtest", qos);
        pubOrganizedCloudIn = this->create_publisher<sensor_msgs::msg::PointCloud2>("pubOrganizedCloudIn", qos);

        pubTransformedPC = this->create_publisher<sensor_msgs::msg::PointCloud2>("pubTransformedPC", qos);



        pubMapPC = this->create_publisher<sensor_msgs::msg::PointCloud2>("pubMapPC", qos);
        pubAGMapTransformedPC = this->create_publisher<sensor_msgs::msg::PointCloud2>("pubAGMapTransformedPC", qos);
        pubRobotPath = this->create_publisher<nav_msgs::msg::Path>("RobotPath", qos);

        pubDONEsignal = this->create_publisher<geometry_msgs::msg::Pose>("doneInit", qos);


        pubinfinity = this->create_publisher<visualization_msgs::msg::Marker>("pubinfinity", qos);
    }

    void initializeSubscribers() {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));


        subMapAG = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/mapPC_AG", qos,
            std::bind(&CloudBase::mapAGCB, this, std::placeholders::_1));

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
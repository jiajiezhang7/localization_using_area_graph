/**
 * @file cloudInitializer.hpp
 * @author Jiajie Zhang (ROS2 port)
 *         Fujing Xie (original ROS1 implementation)
 *         Sören Schwertfeger (original ROS1 implementation)
 * @brief Global localization initialization for Area Graph based indoor localization
 * @version 0.1
 * @date 2024-11-09
 *
 * @details Global localization class that provides initial pose estimation in Area Graph maps.
 *          This class inherits from CloudBase and implements:
 *
 *          1. Pose Initialization:
 *             - WiFi/barometer based coarse initialization
 *             - Uniform pose sampling around initial guess
 *             - Scoring function for pose evaluation
 *             - Best pose selection mechanism
 *
 *          2. Score Functions:
 *             - Distance based scoring
 *             - Outside/inside point ratio evaluation
 *
 * 多线程优化控制宏
 * 取消注释以下宏定义可启用调试功能
 *
 *             - Turkey weight robust scoring
 *             - Hybrid scoring approaches
 *
 *          3. Rescue Functions:
 *             - Recovery from localization failures
 *             - Multi-hypothesis tracking
 *             - Passage and corridor handling
 *             - Initial ICP refinement
 *
 *          4. Data Management:
 *             - Point cloud filtering and organization
 *             - Map intersection calculations
 *             - Score recording and analysis
 *             - Result visualization
 *
 * Class Variables:
 * @var MaxRobotPose      Best estimated robot pose matrix
 * @var MaxScore          Highest pose evaluation score
 * @var numofIntersection Ray intersection count per point
 * @var inRayDis         Ray intersection distances
 * @var inRayRange       Range measurements for rays
 * @var match_with_outside Outside area matching flags
 *
 * Public Methods:
 * @fn setLaserCloudin() Sets input point cloud data
 * @fn setMapPC()        Sets reference map point cloud
 * @fn rescueRobot()     Performs recovery localization

 *
 * Key Features:
 * - Robust to partial map observations
 * - Handles symmetric environments
 * - Efficient pose sampling strategy
 * - Multi-modal pose distribution handling
 *
 * @note Implements global localization as described in paper Section III.D
 *       "Guess Scoring" and III.E "Weight Function"
 *
 * @see cloudBase.hpp for base functionality
 * @see utility.hpp for common utilities
 *
 * @warning Requires:
 * - Dense point cloud input (e.g., 64 beam LiDAR)
 * - WiFi/barometer rough position estimate
 * - Well-defined Area Graph map
 *
 * @copyright Copyright (c) 2024, ShanghaiTech University
 *            All rights reserved.
 */
#pragma once
#ifndef _CLOUD_INITIALIZER_HPP_
#define _CLOUD_INITIALIZER_HPP_

// 多线程优化控制宏
// 取消注释以下宏定义可启用调试功能
#define DEBUG_PUBLISH    // 启用此宏将在多线程中发布中间结果
#define DETAILED_TIMING  // 启用此宏将记录详细的时间信息

#include "utility.hpp"
#include "cloudBase.hpp"
#include "thread_pool.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>

class CloudInitializer : public CloudBase,
                        public std::enable_shared_from_this<CloudInitializer> {
public:
    // 变换和评分变量
    Eigen::Matrix4f MaxRobotPose;                    // 最佳位姿估计
    double MaxScore;                                 // 最佳得分
    bool bGuessReady;                                // 粒子初始化就绪标志
    bool isRescueFinished = false;    // 标记rescueRobot是否已完成

    // 射线交叉计算的向量
    std::vector<int> numofIntersection;     // 记录射线与地图的交叉次数
    std::vector<double> inRayDis;           // 距离测量
    std::vector<double> inRayRange;         // 范围测量
    std::vector<double> match_with_outside; // 外部区域匹配标志 (1=外部, 0=内部)

    // 计数器和度量
    int numofInsidePoints;    // 内部点数量
    int numofOutsidePoints;   // 外部点数量
    double turkeyScore;       // Turkey权重得分

    int rescueTimes;          // 救援次数
    double rescueRunTime;     // 救援运行时间

    // 文件输出流
    std::ofstream rescueRoomStream;  // 用于记录救援房间信息的输出流
    std::ofstream perf_log;          // 用于记录性能数据的输出流
    std::time_t now_time_t;          // 用于时间戳记录

    // 时间计算变量
    rclcpp::Time fine_search_start; // 精细搜索开始时间
    double fine_search_time;        // 精细搜索总耗时

    // ROS2 订阅和发布器
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subInitialGuess;  // 初始猜测订阅器
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pubRobotGuessMarker;  // 机器人猜测标记发布器
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pubRobotPoseAfterICP;  // ICP后机器人位姿发布器
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pubCurrentMaxRobotPose;  // 当前最佳机器人位姿发布器
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pubGlobalLocMarker;  // 全局定位结果标记发布器

    // 消息存储
    geometry_msgs::msg::PointStamped robotGuess;  // 机器人猜测位置

    // ImageTransport 和发布者
    std::shared_ptr<image_transport::ImageTransport> image_transport_;
    image_transport::Publisher image_pub_;

    // 构造函数和析构函数
    explicit CloudInitializer();
    ~CloudInitializer() override = default;

    // 核心功能方法
    void setLaserCloudin(pcl::PointCloud<pcl::PointXYZI>::Ptr furthestRing_,
                        std_msgs::msg::Header mapHeader_);  // 设置输入激光点云数据
    void setMapPC(pcl::PointCloud<pcl::PointXYZI>::Ptr map_pc_);  // 设置参考地图点云
    void rescueRobot();  // 执行恢复定位

    // 点云处理方法
    bool checkWholeMap(const pcl::PointXYZI& PCPoint,
                      const pcl::PointXYZI& PosePoint,
                      int horizonIndex,
                      double& minDist,
                      bool& findIntersection);  // 检查整个地图

    // 点云验证方法
    bool isValidPoint(const pcl::PointXYZI& point);  // 验证点是否有效


    // 重写CloudBase方法
    void calClosestMapPoint(int inside_index) override;  // 计算最近的地图点
    bool checkMap(int ring, int horizonIndex, int& last_index,
                 double& minDist, int inside_index) override;  // 检查地图
    void allocateMemory() override;  // 分配内存
    void resetParameters() override;  // 重置参数

    // ICP和初始化方法
    void initializationICP(int insideAGIndex);  // 使用ICP进行初始化
    bool checkICPmovingDist(Eigen::Matrix4f robotPoseGuess);  // 检查ICP移动距离
    bool insideOldArea(int mapPCindex);  // 检查是否在旧区域内

    // 双峰候选位姿评估方法
    /**
     * @brief 使用ICP评估位姿，不修改全局状态
     *
     * @param initialPose 初始位姿估计
     * @param areaId 区域ID
     * @param inputCloud 输入点云
     * @return std::pair<Eigen::Matrix4f, double> 优化后的位姿和评分
     */
    std::pair<Eigen::Matrix4f, double> evaluatePoseWithICP(
        const Eigen::Matrix4f& initialPose,
        int areaId,
        pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud);

    // 多线程优化相关方法
    void rescueRobotMultiThread(); // 使用多线程的救援机器人方法

    // 线程安全的方法
    void setInitialPoseThreadSafe(int yaw, const Eigen::Vector3f& trans, Eigen::Matrix4f& localRobotPose);
    void initializationICPThreadSafe(int insideAGIndex, Eigen::Matrix4f& localRobotPose,
                                  pcl::PointCloud<pcl::PointXYZI>::Ptr& localTransformedPC,
                                  double& localInsideScore, double& localOutsideScore,
                                  int& localNumofInsidePoints, int& localNumofOutsidePoints,
                                  double& localTurkeyScore);
    void calClosestMapPointThreadSafe(int inside_index, Eigen::Matrix4f& localRobotPose,
                                    pcl::PointCloud<pcl::PointXYZI>::Ptr& localTransformedPC,
                                    double& localInsideScore, double& localOutsideScore,
                                    int& localNumofInsidePoints, int& localNumofOutsidePoints,
                                    double& localTurkeyScore);
    void writeResultToFile(double result_angle, size_t particleIdx, const Eigen::Matrix4f& localRobotPose,
                         int localNumofInsidePoints, double localInsideScore,
                         int localNumofOutsidePoints, double localOutsideScore,
                         double localInsideTotalRange, double localOutsideTotalScore,
                         double localTurkeyScore);
    bool checkMapThreadSafe(int ring, int horizonIndex, int& last_index, double& minDist, int inside_index,
                         pcl::PointCloud<pcl::PointXYZI>::Ptr& localRingMapP1,
                         pcl::PointCloud<pcl::PointXYZI>::Ptr& localRingMapP2,
                         pcl::PointCloud<pcl::PointXYZI>::Ptr& localTransformedPC);
    // 回调方法
    void getInitialExtGuess(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg);  // 获取初始外部猜测

private:
    // 成员变量
    double intersectionx;  // 交叉点x坐标
    double intersectiony;  // 交叉点y坐标

    // 多线程同步相关
    std::mutex score_mutex;     // 用于保护MaxScore和MaxRobotPose
    std::mutex publish_mutex;   // 用于保护发布操作
    std::mutex file_mutex;      // 用于保护文件写入
    std::mutex map_mutex;       // 用于保护地图数据

    // 初始化发布器
    void initializePublishers() {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

        pubRobotGuessMarker = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "pubRobotGuessMarker", qos);

        pubRobotPoseAfterICP = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "pubRobotPoseAfterICP", qos);

        pubCurrentMaxRobotPose = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "pubCurrentMaxRobotPose", qos);

        pubGlobalLocMarker = this->create_publisher<visualization_msgs::msg::Marker>(
            "global_loc_marker", qos);
    }

    // 初始化订阅器
    void initializeSubscribers() {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

        subInitialGuess = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/particles_for_init", qos,
            std::bind(&CloudInitializer::getInitialExtGuess, this, std::placeholders::_1));
    }

    // 初始化变量
    void initializeVariables() {
        MaxScore = 0.0;
        numofInsidePoints = 0;
        numofOutsidePoints = 0;
        turkeyScore = 0.0;
        bGuessReady = false;
        rescueTimes = 0;
        rescueRunTime = 0.0;

        // 为向量预留空间
        numofIntersection.reserve(1000);
        inRayDis.reserve(1000);
        inRayRange.reserve(1000);
        match_with_outside.reserve(1000);
    }

protected:
    // 可视化相关成员变量和方法
    bool visualization_enabled_;

    /**
     * @brief 生成并保存可视化图像，包含地图、WiFi定位和全局定位结果
     *
     * @param wifi_position WiFi定位的位置 (x,y)
     * @param final_position 全局定位后的最终位置 (x,y)
     * @param output_path 输出图像文件的路径
     */
    void saveVisualizationImage(const std::vector<float>& wifi_position,
                               const std::vector<float>& final_position,
                               const std::string& output_path);

    /**
     * @brief 在指定图像上绘制点云地图
     *
     * @param image 要绘制的图像
     * @param scale 缩放因子
     * @param offset_x X轴偏移
     * @param offset_y Y轴偏移
     */
    void drawMapOnImage(cv::Mat& image, float scale, float offset_x, float offset_y);
};

#endif // _CLOUD_INITIALIZER_HPP_
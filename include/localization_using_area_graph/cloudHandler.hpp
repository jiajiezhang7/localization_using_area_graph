/**
 * @file cloudHandler.hpp
 * @author Jiajie Zhang (ROS2 port)
 *         Fujing Xie (original ROS1 implementation)
 *         Sören Schwertfeger (original ROS1 implementation)
 * @brief Point cloud processing and localization using Area Graph map representation
 * @version 0.1
 * @date 2024-12-02
 *
 */
#pragma once
#ifndef _CLOUD_HANDLER_HPP_
#define _CLOUD_HANDLER_HPP_

#include "utility.hpp"
#include "cloudBase.hpp"
#include "cloudInitializer.hpp"
#include "odom_fusion.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

// Message filters for ROS2
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
// 并没有直接使用消息同步机制，允许各个消息独立处理
#include "message_filters/sync_policies/approximate_time.h"

class CloudHandler : public CloudBase {
public:
    // ROS2 订阅器
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloud;    // 订阅激光点云数据
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subInitialGuess;  // 订阅初始位姿猜测
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subManualInitialPose;  // 订阅手动设置的初始位姿

    // ROS2 发布器
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pubRobotPose;      // 发布机器人位姿

    // cloudInitializer时在CloudHandler类构造时就已经被实例化了的
    std::shared_ptr<CloudInitializer> cloudInitializer;  // 使用智能指针管理云初始化器对象

    // 地图相关索引
    int insideAreaStartIndex;  // 地图点云索引
    int insideAreaID;          // AG索引数据结构

    // 处理变量
    std::vector<bool> vbHistogramRemain;  // 直方图剩余标志
    std::chrono::steady_clock::time_point sumFrameRunTime; // 使用ROS2时间，累计帧运行时间
    int numofFrame;       // 帧数
    bool hasGlobalPoseEstimate;    // 是否已从全局定位获得位姿估计
    bool hasManualInitialPose;     // 是否有手动设置的初始位姿
    int globalImgTimes;   // 全局图像次数

    explicit CloudHandler();  // 显式构造函数
    ~CloudHandler() override = default;  // 默认析构函数

    // 点云处理方法
    void filterUsefulPoints();  // 过滤有用点
    void optimizationICP();     // ICP优化
    void showImg1line(const std::string& words);  // 显示一行图像

    // 获取机器人位姿 - 为Nav2接口添加
    Eigen::Matrix4f getRobotPose() const { return robotPose; }  // 返回当前机器人位姿

    // 设置手动初始位姿 - 为Nav2接口添加
    void setManualInitialPose(double yaw, const Eigen::Vector3f& position);  // 设置手动初始位姿

    // 地图和直方图处理
    void mergeMapHistogram();  // 合并地图直方图
    double corridornessDSRate(double maxPercentage);  // 计算走廊度下采样率
    void gettingInsideWhichArea();  // 确定所在区域

    // 点云检查方法
    bool checkWholeMap(int pc_index,
                      const pcl::PointXYZI& PCPoint,
                      double &map1x,
                      double &map1y,
                      double &map2x,
                      double &map2y,
                      double &intersectionx,
                      double &intersectiony);  // 检查整个地图

    // 重写CloudBase方法
    void calClosestMapPoint(int inside_index) override;  // 计算最近地图点
    bool checkMap(int ring,
                 int horizonIndex,
                 int &last_index,
                 double &minDist,
                 int inside_index) override;  // 检查地图
    void allocateMemory() override;  // 分配内存
    void resetParameters() override;  // 重置参数

private:
    // ========== 里程计融合相关 ==========
    std::unique_ptr<agloc_fusion::OdomFusion> odom_fusion_;  // 里程计融合模块
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;  // 里程计订阅器
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr predicted_pose_pub_;  // 预测位姿发布器

    // 融合状态变量
    Eigen::Matrix4f last_fused_pose_;           // 上一次融合后的位姿
    rclcpp::Time last_fusion_time_;             // 上一次融合的时间
    bool fusion_initialized_;                   // 融合模块是否已初始化
    double last_icp_score_;                     // 上一次ICP得分

    // 回调方法
    void cloudHandlerCB(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg);  // 处理接收到的点云数据
    void setInitialGuessFlag(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg);  // 设置初始猜测标志
    void manualInitialPoseCB(const std::shared_ptr<geometry_msgs::msg::PoseWithCovarianceStamped> poseMsg);  // 处理手动设置的初始位姿
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg);  // 里程计回调函数

    // ========== 里程计融合方法 ==========
    void initializeOdomFusion();                                          // 初始化里程计融合模块
    Eigen::Matrix4f applyOdomFusion(const Eigen::Matrix4f& icp_pose,     // 应用里程计融合
                                   double icp_score,
                                   const rclcpp::Time& timestamp);
    double computeICPScore();                                             // 计算ICP匹配得分
    void publishPredictedPose(const Eigen::Matrix4f& predicted_pose,     // 发布预测位姿
                             const rclcpp::Time& timestamp);

    // 初始化发布器和订阅器
    void initializePublishers() {
        // 创建机器人位姿发布者
        pubRobotPose = create_publisher<geometry_msgs::msg::PoseStamped>(
            "/cloud_handler/pose", rclcpp::QoS(1).reliable());

        // 如果启用位姿预测发布，创建预测位姿发布者
        if (publish_prediction) {
            predicted_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
                "/cloud_handler/predicted_pose", rclcpp::QoS(1).reliable());
        }
    }

    void initializeSubscribers() {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

        // 订阅点云话题
        subLaserCloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            pointCloudTopic, qos,
            std::bind(&CloudHandler::cloudHandlerCB, this, std::placeholders::_1));

        // 订阅初始猜测粒子，一旦检测到生成的粒子，则使得标识符 hasGlobalPoseEstimate == True
        subInitialGuess = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/particles_for_init", qos,
            std::bind(&CloudHandler::setInitialGuessFlag, this, std::placeholders::_1));

        // 订阅手动设置的初始位姿话题
        subManualInitialPose = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose_agloc", qos,
            std::bind(&CloudHandler::manualInitialPoseCB, this, std::placeholders::_1));

        // 如果启用里程计融合，订阅里程计话题
        if (enable_odom_fusion) {
            odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                odom_topic, qos,
                std::bind(&CloudHandler::odomCallback, this, std::placeholders::_1));
        }
    }

    // 初始化变量
    void initializeVariables() {
        // 初始化时间跟踪
        sumFrameRunTime = std::chrono::steady_clock::now();
        insideAreaStartIndex = 0;  // 内部区域起始索引
        insideAreaID = 0;          // 内部区域ID
        numofFrame = 0;            // 帧数
        hasGlobalPoseEstimate = false;      // 是否已从全局定位获得位姿估计
        hasManualInitialPose = false;       // 是否有手动设置的初始位姿
        globalImgTimes = 0;        // 全局图像次数

        // 初始化里程计融合相关变量
        fusion_initialized_ = false;
        last_fused_pose_ = Eigen::Matrix4f::Identity();
        last_fusion_time_ = this->now();
        last_icp_score_ = 0.0;

        // 如果启用里程计融合，初始化融合模块
        if (enable_odom_fusion) {
            initializeOdomFusion();
        }
    }

protected:
    // Add any protected members if needed
};

#endif // _CLOUD_HANDLER_HPP_
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

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "nav_msgs/msg/odometry.hpp"

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
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subLiosamOdometry;      // 订阅LIO-SAM里程计数据

    // ROS2 发布器
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubinsideAreaPC;     // 发布区域内点云

    // 核心功能对象
    CloudInitializer cloudInitializer;  // 云初始化器对象

    // 地图相关索引
    int insideAreaStartIndex;  // 地图点云索引
    int insideAreaID;          // AG索引数据结构

    // 处理变量
    std::vector<bool> vbHistogramRemain;  // 直方图剩余标志
    std::chrono::steady_clock::time_point sumFrameRunTime; // 使用ROS2时间，累计帧运行时间
    int numofFrame;       // 帧数
    bool getGuessOnce;    // 是否已获得一次猜测
    int globalImgTimes;   // 全局图像次数

    explicit CloudHandler();  // 显式构造函数
    ~CloudHandler() override = default;  // 默认析构函数

    // 点云处理方法
    void filterUsefulPoints();  // 过滤有用点
    void optimizationICP();     // ICP优化
    void showImg1line(const std::string& words);  // 显示一行图像

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


    // 回调方法
    void cloudHandlerCB(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg);  // 处理接收到的点云数据
    void liosamOdometryIncrementalCB(const nav_msgs::msg::Odometry::SharedPtr odomMsg);  // 处理LIO-SAM里程计增量数据
    void getInitialExtGuess(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg);  // 获取初始外部猜测

    // 初始化发布器和订阅器
    void initializePublishers() {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
        
        // 发布内部区域点云
        pubinsideAreaPC = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "insideAreaPC", qos);
    }

    void initializeSubscribers() {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
        
        // 订阅点云话题
        subLaserCloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            pointCloudTopic, qos,
            std::bind(&CloudHandler::cloudHandlerCB, this, std::placeholders::_1));
            
        // 订阅初始猜测粒子
        subInitialGuess = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/particles_for_init", qos,
            std::bind(&CloudHandler::getInitialExtGuess, this, std::placeholders::_1));
            
        // 订阅LIO-SAM里程计数据
        subLiosamOdometry = this->create_subscription<nav_msgs::msg::Odometry>(
            "/lio_sam/mapping/odometry", qos,
            std::bind(&CloudHandler::liosamOdometryIncrementalCB, this, std::placeholders::_1));

        // 订阅LIO-SAM增量里程计数据（在Fujing的代码中没有使用）
        // subLiosamodometry_incremental = this->create_subscription<nav_msgs::msg::Odometry>(
        //     "/lio_sam/mapping/odometry_incremental", qos,
        //     std::bind(&CloudHandler::liosamOdometryIncrementalCB, 
        //              this, 
        //              std::placeholders::_1));
    }

    // 初始化变量
    void initializeVariables() {
        // 初始化时间跟踪
        sumFrameRunTime = std::chrono::steady_clock::now();
        insideAreaStartIndex = 0;  // 内部区域起始索引
        insideAreaID = 0;          // 内部区域ID
        numofFrame = 0;            // 帧数
        getGuessOnce = false;      // 是否已获得一次猜测
        globalImgTimes = 0;        // 全局图像次数
        

    }

protected:
    // Add any protected members if needed
};

#endif // _CLOUD_HANDLER_HPP_
/**
 * @file cloudHandler.cpp
 * @author Jiajie Zhang
 * @brief Implementation of CloudHandler class for Area Graph based localization
 * @version 0.1
 * @date 2024-12-02
 *
 * @note Implementation corresponds to algorithms described in the AGLoc paper
 *       Section III.C-G for point cloud processing and pose tracking
 *
 *
 * @copyright Copyright (c) 2024, ShanghaiTech University
 *            All rights reserved.
 */
#include "localization_using_area_graph/cloudHandler.hpp"
#include "localization_using_area_graph/utility.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

/**
 * @brief 计算走廊场景的点云降采样率
 * @details 基于场景的走廊特征程度(corridorness)自适应计算降采样率：
 *          1. 当场景不具有明显走廊特征时(maxPercentage < 0.5)不进行降采样
 *          2. 当场景越像走廊时，采用更高的降采样率以提高效率
 *
 * @param maxPercentage 走廊特征程度，范围[0,1]
 *                      - 0表示完全不具有走廊特征
 *                      - 1表示完全符合走廊特征
 *                      该值通过直方图分析得到，表示主导方向上的点的占比
 *
 * @return double 降采样率
 *         - 返回0表示不进行降采样
 *         - 返回值越大表示降采样程度越高
 *         - 实际降采样率 = 10 * maxPercentage - 4
 *           例如：maxPercentage = 0.7 时，降采样率 = 3
 */
double CloudHandler::corridornessDSRate(double maxPercentage) {
    // 如果走廊特征程度小于50%，认为不是走廊场景，不进行降采样
    if(maxPercentage < 0.5) {
        return 0;
    }

    // 线性计算降采样率
    // 走廊特征越明显(maxPercentage越大)，降采样率越高
    // 降采样率范围：1(当maxPercentage=0.5) 到 6(当maxPercentage=1.0)
    return 10 * maxPercentage - 4;
}

/**
 * @brief 检查机器人当前所在区域
 * @details 此函数通过以下步骤确定机器人的当前位置:
 *          1. 首先检查是否仍在上一个已知区域内
 *          2. 如果不在，则遍历所有区域进行检查
 *          3. 收集当前区域的点云用于可视化
 *          4. 处理多区域和无区域的错误情况
 */
void CloudHandler::gettingInsideWhichArea() {
    auto insideAreaPC = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

    // 检查是否仍在上一个区域内
    if(lastInsideIndex != -1) {
        bool binside = areaInsideChecking(robotPose, lastInsideIndex);
        // 仍在旧区域内
        if(binside) {
            // 收集点云用于可视化
            for(int j = lastInsideIndex; j < lastInsideIndex + 100000; j++) {
                if((int)map_pc->points[j].intensity % 3 == 2) {
                    break;
                }
                insideAreaPC->points.push_back(map_pc->points[j]);
            }

            // 发布可视化信息并返回 (已删除pubinsideAreaPC发布器)
            // sensor_msgs::msg::PointCloud2 outMsg;
            // pcl::toROSMsg(*insideAreaPC, outMsg);
            // outMsg.header = mapHeader;
            // pubinsideAreaPC->publish(outMsg);

            RCLCPP_INFO(get_logger(), "---------------------Inside old area---------------------");
            return;
        }
    }

    // 搜索所有区域
    int insideTime = 0;
    int temp = -1;

    for(size_t i = 0; i < map_pc->points.size(); i++) {
        bool binside = false;

        // 检查新区域的开始
        if((int)map_pc->points[i].intensity % 3 == 0) {
            // intensity % 3 == 0 --> Area的起始点
            binside = areaInsideChecking(robotPose, i);
            temp++;
        }

        if(binside) {
            insideTime++;
            insideAreaStartIndex = i;
            insideAreaID = temp;

            // 收集区域点云用于可视化
            for(size_t j = i; j < static_cast<size_t>(i + 100000) && j < map_pc->points.size(); j++) {
                // 区域结束
                if((int)map_pc->points[j].intensity % 3 == 2) {
                    break;
                }
                insideAreaPC->points.push_back(map_pc->points[j]);
            }
            lastInsideIndex = i;
        }
    }

    // 错误检查 - 应该只在一个区域内
    if(insideTime > 1) {
        RCLCPP_ERROR(get_logger(), "错误: 机器人位置在多个区域内!");
    } else if(insideTime == 0) {
        RCLCPP_ERROR(get_logger(), "错误: 机器人位置在所有区域外!");
    } else {
        RCLCPP_INFO(get_logger(), "机器人位置在区域 %d 内", insideAreaStartIndex);
    }

    // 发布可视化信息 (已删除pubinsideAreaPC发布器)
    // sensor_msgs::msg::PointCloud2 outMsg;
    // pcl::toROSMsg(*insideAreaPC, outMsg);
    // outMsg.header = mapHeader;
    // pubinsideAreaPC->publish(outMsg);

    // 处理多区域错误
    if(insideTime > 1) {
        RCLCPP_ERROR(get_logger(), "错误: 机器人位置在多个区域内!");
        std::cout << "按回车键继续" << std::endl;
        std::getchar();
    }
}

CloudHandler::CloudHandler()
    : CloudBase("cloud_handler_node") {  // 只初始化基类
    // 初始化变量
    globalImgTimes = 0;  // 全局图像计数器
    hasGlobalPoseEstimate = false;  // 是否已获取全局位姿估计标志
    hasManualInitialPose = false;   // 初始化手动位姿标志为false
    sumFrameRunTime = std::chrono::steady_clock::now();  // 累计帧运行时间
    numofFrame = 0;  // 帧数计数器

    // 初始化CloudHandler中的发布者和订阅者
    initializePublishers();
    initializeSubscribers();

    // 打开文件以保存机器人位姿结果（TUM格式）
    robotPoseTum.open("/home/jay/AGLoc_ws/robotPoseResult/robotPoseTum.txt",
                      std::ios::ate);
    robotPoseTum << std::fixed;
    robotPoseTum.precision(6);

    // 分配内存
    allocateMemory();

    // 从params.yaml文件中读取并设置初始位姿
    // 在模式1和模式2 -- 开启全局定位下， 造时设置初始位姿（会被覆盖），会通过getInitialExtGuess和rescueRobot重新估计位姿
    setInitialPose(initialYawAngle, initialExtTrans);

    // 创建 CloudInitializer 的智能指针
    cloudInitializer = std::make_shared<CloudInitializer>();
}

void CloudHandler::setInitialGuessFlag(
    const sensor_msgs::msg::PointCloud2::SharedPtr /* laserCloudMsg */) {
    hasGlobalPoseEstimate = true;
}

/**
 * @brief 点云处理的主回调函数
 * @details 该函数负责处理接收到的激光雷达点云数据,主要功能包括:
 *          1. 检查地图初始化状态
 *          2. 对点云数据进行预处理和组织化
 *          3. 执行全局定位
 *          4. 发布处理后的点云和定位结果
 *
 * @param laserCloudMsg 输入的激光雷达点云消息(ROS PointCloud2格式)
 *
 * @note 该函数是点云处理的核心,会在每帧点云数据到来时被调用
 *       需要地图已经初始化才能正常工作
 */
void CloudHandler::cloudHandlerCB(
    const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg) {

    // 输出当前地图初始化状态和大小
    // RCLCPP_DEBUG(get_logger(), "Received map message, mapInit=%d", mapInit);
    // RCLCPP_DEBUG(get_logger(), "Map size: %zu", map_pc->points.size());

    // 显示全局定位开始的分隔线
    if(globalImgTimes == 0) {
        RCLCPP_INFO(get_logger(), "---------------------------Global localizing---------------------------");
    }
    globalImgTimes++;

    // 初始化计时器
    auto startC = std::chrono::high_resolution_clock::now();
    auto startTime = this->now();
    auto startTimecb = this->now();

    // 清除上一帧的记录
    outsideAreaIndexRecord.clear();
    outsideAreaLastRingIndexRecord.clear();

    // 检查地图是否初始化，未初始化则返回
    if(!mapInit) {
        RCLCPP_INFO_ONCE(get_logger(), "Map not initialized yet, waiting for map!");
        return;
    }

    // 打印当前状态 - 增强调试信息
    RCLCPP_INFO(get_logger(), "📊 系统状态: bRescueRobot=%s, isRescueFinished=%s, initialized=%s, hasGlobalPoseEstimate=%s",
                bRescueRobot ? "true" : "false",
                cloudInitializer->isRescueFinished ? "true" : "false",
                initialized ? "true" : "false",
                hasGlobalPoseEstimate ? "true" : "false");

    // 添加错误阈值状态信息
    RCLCPP_DEBUG(get_logger(), "错误阈值: errorUpThred=%.1f, errorLowThred=%.1f, errorUpThredCurr=%.1f, errorLowThredCurr=%.1f",
                errorUpThred, errorLowThred, errorUpThredCurr, errorLowThredCurr);

    // 准备新帧的处理
    setEveryFrame();
    cloudInitializer->setMapPC(map_pc);
    cloudHeader = laserCloudMsg->header;
    mapHeader = cloudHeader;
    mapHeader.frame_id = "map";
    globalPath.header = mapHeader;

    // lidar_points -> laserCloudMsg -> laserCloudIn
    sensor_msgs::msg::PointCloud2 temp_msg = *laserCloudMsg;
    pcl::fromROSMsg(temp_msg, *laserCloudIn);

    // laserCloudIn -> organizedCloudIn
    organizePointcloud();

    // 发布组织化后的点云
    sensor_msgs::msg::PointCloud2 outMsg;
    pcl::toROSMsg(*organizedCloudIn, outMsg);
    outMsg.header = cloudHeader;
    pubOrganizedCloudIn->publish(outMsg);

    // 根据初始化状态设置误差阈值
    if(initialized) {
        errorUpThredCurr = errorUpThred;
        errorLowThredCurr = errorLowThred;
    } else {
        errorUpThredCurr = errorUpThredInit;
        errorLowThredCurr = errorLowThredInit;
    }

    // 模式1: 测试全局定位 - 每帧都执行全局定位
    if(bTestRescue) {
        if (!isAGIndexReceived()) {
            RCLCPP_ERROR(get_logger(), "AG_index not initialized yet!");
            RCLCPP_WARN(get_logger(), "CloudBase::AGindexReceived: %d", isAGIndexReceived());
            throw std::bad_weak_ptr();
        }


        RCLCPP_INFO(get_logger(), "----------TEST RESCUE ROBOT, EVERY FRAME GOES TO RESCUE----------");

        // 设置初始位姿估计的回调函数 --- 包装器
        auto initialGuessCallback = std::bind(&CloudInitializer::getInitialExtGuess,
                                            cloudInitializer.get(),
                                            std::placeholders::_1);

        // 实际上调用的就是CloudInitializer::getInitialExtGuess
        cloudInitializer->subInitialGuess = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/particles_for_init", 10, initialGuessCallback);

        // 发布最远环点云
        sensor_msgs::msg::PointCloud2 furthestMsg;
        pcl::toROSMsg(*furthestRing, furthestMsg);
        furthestMsg.header = mapHeader;
        pubtest->publish(furthestMsg);

        cloudInitializer->setLaserCloudin(furthestRing, mapHeader);
        resetParameters();
        return;
    }
    // 模式2: 救援机器人模式 - 全局定位(仅一次)，然后跳转到模式3 - 位姿跟踪
    else if(bRescueRobot) {
        // 首次执行：创建订阅并处理点云
        if(!hasGlobalPoseEstimate) {
            RCLCPP_INFO(get_logger(), "-------------STARTING RESCUE ROBOT (ONCE)---------------");

            // 设置初始位姿估计的回调函数
            auto initialGuessCallback = std::bind(&CloudInitializer::getInitialExtGuess,
                                              cloudInitializer.get(),
                                              std::placeholders::_1);

            cloudInitializer->subInitialGuess = create_subscription<sensor_msgs::msg::PointCloud2>(
                "/particles_for_init", 10, initialGuessCallback);

            // 发布最远环点云
            sensor_msgs::msg::PointCloud2 furthestMsg;
            pcl::toROSMsg(*furthestRing, furthestMsg);
            furthestMsg.header = mapHeader;
            pubtest->publish(furthestMsg);

            cloudInitializer->setLaserCloudin(furthestRing, mapHeader);

            // 标记已开始全局定位流程
            hasGlobalPoseEstimate = true;

            // 注意：主动触发rescueRobot流程
            static int trigger_count = 0;
            trigger_count++;

            if(trigger_count >= 5 && !cloudInitializer->isRescueFinished) {
                RCLCPP_WARN(get_logger(), "强制触发rescueRobot流程...");
                cloudInitializer->rescueRobot();
                trigger_count = 0;
            }

            return;
        }

        // 检查rescueRobot是否已完成
        if(cloudInitializer->isRescueFinished) {
            RCLCPP_INFO(get_logger(), "-------------RESCUE ROBOT COMPLETED---------------");

            // 应用最佳估计位姿
            robotPose = cloudInitializer->MaxRobotPose;

            RCLCPP_INFO(get_logger(), "Setting robot pose in rescue robot: [%f, %f]",
                        robotPose(0,3), robotPose(1,3));

            // 发布机器人位姿
            geometry_msgs::msg::PoseStamped pose_msg;
            pose_msg.header = mapHeader;
            pose_msg.pose.position.x = robotPose(0,3);
            pose_msg.pose.position.y = robotPose(1,3);
            pose_msg.pose.position.z = robotPose(2,3);

            // 从旋转矩阵转换为四元数
            Eigen::Matrix3f rot = robotPose.block<3,3>(0,0);
            Eigen::Quaternionf q(rot);
            pose_msg.pose.orientation.x = q.x();
            pose_msg.pose.orientation.y = q.y();
            pose_msg.pose.orientation.z = q.z();
            pose_msg.pose.orientation.w = q.w();

            pubRobotPose->publish(pose_msg);

            // 关闭救援模式，但不重置isRescueFinished，让下一帧进入位姿跟踪
            bRescueRobot = false;

            // 设置全局定位完成标志，让下一帧使用全局定位结果
            hasGlobalPoseEstimate = true;

            // 取消对粒子消息的订阅，防止再次触发rescueRobot
            cloudInitializer->subInitialGuess.reset();

            subInitialGuess = create_subscription<sensor_msgs::msg::PointCloud2>(
                "/none", 10, std::bind(&CloudHandler::setInitialGuessFlag,
                                     this, std::placeholders::_1));

            RCLCPP_INFO(get_logger(), "全局定位完成，下一帧将开始位姿跟踪");
            return;
        }

        // 等待rescueRobot完成
        RCLCPP_INFO_ONCE(get_logger(), "-------------WAITING FOR RESCUE ROBOT TO COMPLETE---------------");
        return;
    }
    // 模式3: 纯位姿跟踪模式 - 使用固定初始位姿 （目前能跑通的模式）
    else {
        // 如果从模式2的全局定位环节计算得到了初始位姿，则使用它，并且覆盖从params中读取的默认值
        // 优先级：手动设置位姿 > 全局定位结果 > params默认值
        if(hasManualInitialPose) {
            // 使用手动设置的位姿，已在manualInitialPoseCB中设置robotPose
            errorUpThred = 3;

            RCLCPP_INFO(get_logger(), "使用手动设置的初始位姿: [%f, %f]",
                         robotPose(0,3), robotPose(1,3));

            // 清除标志，避免重复使用
            hasManualInitialPose = false;
        }
        else if(hasGlobalPoseEstimate) {
            // 使用全局定位的结果
            robotPose = cloudInitializer->MaxRobotPose;

            // 重要修复：设置合适的跟踪阈值，而非硬编码的3
            errorUpThred = 1.5;  // 稍微宽松一点，便于从全局定位过渡到精确跟踪
            errorLowThred = 1.0;

            // 重要修复：标记系统已初始化，启用正常的跟踪模式
            initialized = true;

            // 重要修复：重置isRescueFinished，防止重复使用全局定位结果
            cloudInitializer->isRescueFinished = false;

            cloudInitializer->subInitialGuess = create_subscription<sensor_msgs::msg::PointCloud2>(
                "/none", 10, std::bind(&CloudInitializer::getInitialExtGuess,
                                     cloudInitializer.get(),
                                     std::placeholders::_1));

            RCLCPP_INFO(get_logger(), "使用全局定位结果作为初始位姿: [%f, %f], 系统已初始化，开始位姿跟踪",
                         robotPose(0,3), robotPose(1,3));
            RCLCPP_INFO(get_logger(), "跟踪阈值设置: errorUpThred=%.1f, errorLowThred=%.1f",
                         errorUpThred, errorLowThred);

            hasGlobalPoseEstimate = false;
        } else {
            // 如果既没有手动设置也没有全局定位结果，则使用params中读取的默认值
            RCLCPP_INFO_ONCE(get_logger(), "------未提供初始位姿，使用params中的默认值--------");
        }

        // 使用当前机器人位姿变换点云
        pcl::transformPointCloud(*organizedCloudIn, *transformed_pc, robotPose);
        RCLCPP_INFO(get_logger(), "Robot pose in tracking: [%f, %f]",
                    robotPose(0,3), robotPose(1,3));

        // 发布变换后的点云(1*600)
        sensor_msgs::msg::PointCloud2 transformedMsg;
        pcl::toROSMsg(*transformed_pc, transformedMsg);
        transformedMsg.header = mapHeader;
        pubTransformedPC->publish(transformedMsg);

        // 记录准备阶段的运行时间
        auto endTime = this->now();
        RCLCPP_DEBUG(get_logger(), "Prepare run time: %f ms",
                     (endTime - startTime).seconds() * 1000);

        // 初始化点云处理标记
        vbHistogramRemain.resize(transformed_pc->points.size(), true);

        // 确定机器人所在区域
        startTime = this->now();
        gettingInsideWhichArea();
        endTime = this->now();
        RCLCPP_DEBUG(get_logger(), "GettingInsideWhichArea run time: %f ms",
                     (endTime - startTime).seconds() * 1000);

        // 计算与地图点云的最近点
        startTime = this->now();
        calClosestMapPoint(insideAreaStartIndex);
        endTime = this->now();
        RCLCPP_DEBUG(get_logger(), "CalClosestMapPoint run time: %f ms",
                     (endTime - startTime).seconds() * 1000);

        // 执行ICP优化
        startTime = this->now();
        optimizationICP();

        // 发布优化后的变换点云(1*600)
        pcl::toROSMsg(*transformed_pc, transformedMsg);
        transformedMsg.header = mapHeader;
        pubTransformedPC->publish(transformedMsg);

        // 变换并发布完整点云(64*600)
        auto transformed_pc_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        transformed_pc_->resize(64 * Horizon_SCAN);
        pcl::transformPointCloud(*organizedCloudIn64, *transformed_pc_, robotPose);

        // 移除完整变换点云发布代码，保留核心功能

        // 变换并发布最远环点云
        transformed_pc_->clear();
        transformed_pc_->resize(Horizon_SCAN);
        pcl::transformPointCloud(*furthestRing, *transformed_pc_, robotPose);

        sensor_msgs::msg::PointCloud2 ringMsg;
        pcl::toROSMsg(*transformed_pc_, ringMsg);
        ringMsg.header = cloudHeader;
        pubFurthestRing->publish(ringMsg);

        endTime = this->now();
        RCLCPP_DEBUG(get_logger(), "OptimizationICP run time: %f ms",
                     (endTime - startTime).seconds() * 1000);
    }

    // 重置参数
    resetParameters();

    // 计算并记录总运行时间
    auto endTimecb = this->now();
    auto finishC = std::chrono::high_resolution_clock::now();

    double cb_duration = (endTimecb - startTimecb).seconds() * 1000;
    RCLCPP_DEBUG(get_logger(), "Pointcloud_CB run time: %f ms", cb_duration);

    // 检查处理时间是否过长
    if(cb_duration > 100) {
        RCLCPP_ERROR(get_logger(), "TAKES TOO LONG!");
    }

    // 计算平均运行时间
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(finishC - startC);
    sumFrameRunTime += std::chrono::duration_cast<std::chrono::steady_clock::duration>(duration);
    numofFrame++;

    RCLCPP_DEBUG(get_logger(), "Average cloudhandler run time: %f ns",
                 std::chrono::duration_cast<std::chrono::duration<double>>(sumFrameRunTime - std::chrono::steady_clock::now()).count() / numofFrame);
}

/**
 * @brief 计算激光点云与地图之间的最近点/交点
 * @details 此函数通过以下步骤处理激光点云与地图的匹配：
 *          1. 遍历每个水平角度的激光束
 *          2. 根据模式选择使用单环或多环进行匹配
 *          3. 计算激光束与地图的交点
 *          4. 发布交点结果用于可视化
 *
 * @param inside_index 机器人当前所在区域的起始索引
 */
void CloudHandler::calClosestMapPoint(int inside_index) {
    // 记录上一次找到交点的地图索引，用于加速后续搜索
    int last_index = 0;

    // 遍历激光雷达的每一个水平角度
    for(int i = 0; i < Horizon_SCAN; i++) {
        bool findIntersection = false;

        // 最远环模式：只使用最远的那一环进行匹配
        if(bFurthestRingTracking) {
            double minDist;  // 存储最小距离
            // 检查第0环（最远环）与地图的交点
            findIntersection = checkMap(0, i, last_index, minDist, inside_index);
        }
        // 多环模式：使用多个环进行匹配
        else {
            // 从底部向上每隔5环选择一个环进行匹配
            for(int chose_ring = 0; chose_ring < N_SCAN/5; chose_ring++) {
                double minDist;
                // 确保选择的环不超过激光雷达的最大线数
                if((10 + 5 * chose_ring) < N_SCAN) {
                    // 检查当前选择的环与地图的交点
                    findIntersection = checkMap(10 + 5 * chose_ring, i, last_index,
                                              minDist, inside_index);
                }
                // 如果找到交点就不再检查其他环
                if(findIntersection) {
                    break;
                }
            }
        }

        // 如果没有找到交点，将对应位置的点坐标设为零
        if(!findIntersection) {
            intersectionOnMap->points[i].x = 0;
            intersectionOnMap->points[i].y = 0;
            intersectionOnMap->points[i].z = 0;
        }
    }

    // 将计算得到的交点转换为ROS消息并发布
    // 移除交点点云发布代码，保留核心功能
}

// 类似于状态监视器，把当前进程状态发布到"Things2say"话题
void CloudHandler::showImg1line(const std::string& words) {
    // 创建图像传输对象
    image_transport::ImageTransport it(shared_from_this());
    // 创建发布器，发布到"Things2say"话题
    auto pub = it.advertise("Things2say", 1);

    // 创建一个黑色背景的图像
    cv::Mat image(200, 600, CV_8UC3, cv::Scalar(0,0,0));
    // 在图像上绘制白色文字
    cv::putText(image, words, cv::Point(20,100), cv::FONT_HERSHEY_DUPLEX,
                2, cv::Scalar(255,255,255), 2, 8);

    // 将OpenCV图像转换为ROS消息
    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
    // 等待500毫秒，确保消息能被接收
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    // 发布图像消息
    pub.publish(*msg);

    // 使用ROS日志输出显示的消息内容
    RCLCPP_INFO(get_logger(), "Display message: %s", words.c_str());
}

/**
 * @brief 检查激光线与地图的交点
 * @details 此函数通过以下步骤检查激光线与地图的交点：
 *          1. 获取当前点的坐标
 *          2. 获取机器人的当前位置
 *          3. 遍历地图点，检查是否有交点
 *          4. 返回是否找到交点
 *
 * @param ring 激光线所在的环
 * @param horizonIndex 激光线在环中的索引
 * @param last_index 上一次找到交点的地图索引
 * @param minDist 最小距离
 * @param inside_index 机器人当前所在区域的起始索引
 */
bool CloudHandler::checkMap(int ring,
                          int horizonIndex,
                          int& last_index,
                          double& minDist,
                          int inside_index) {
    // 获取当前点的坐标
    pcl::PointXYZI PCPoint;
    PCPoint.x = transformed_pc->points[ring * Horizon_SCAN + horizonIndex].x;
    PCPoint.y = transformed_pc->points[ring * Horizon_SCAN + horizonIndex].y;
    PCPoint.z = 0;  // 设置z坐标为0，因为我们只关心2D平面上的交点

    // 获取机器人的当前位置
    pcl::PointXYZI PosePoint;
    PosePoint.x = robotPose(0,3);  // 从机器人姿态矩阵中提取x标
    PosePoint.y = robotPose(1,3);  // 从机器人姿态矩阵中提取y坐标
    PosePoint.z = 0;  // 设置z坐标为0，保持2D平面一致性

    bool findIntersection = false;  // 标记是否找到交点
    minDist = 0;  // 初始化最小距离

    // 遍历地图点，检查是否有交点
    for(int j = inside_index; j < mapSize; j++) {
        // 检查是否到达区域边界
        if((int)map_pc->points[j % mapSize].intensity % 3 == 2) {
            break;  // 如果是区域边界，停止搜索
        }

        // 跳过玻璃点（z坐标不为0的点被认为是玻璃）
        if(map_pc->points[j % mapSize].z != 0) {
            continue;
        }

        // 计算激光射线与地图线段的交点
        pcl::PointXYZI intersectionOnMapThisLine;
        bool inbetween = inBetween(PosePoint,
                                 PCPoint,
                                 map_pc->points[j % mapSize],
                                 map_pc->points[(j + 1) % mapSize],
                                 &intersectionOnMapThisLine);

        if(inbetween) {
            // 计算交点到机器人位置的距离平方
            double distSq = std::pow(intersectionOnMapThisLine.x - PosePoint.x, 2) +
                          std::pow(intersectionOnMapThisLine.y - PosePoint.y, 2);

            // 由于地图多边形的形状不保证是凸的，激光线可能与地图多次相交，选择最近的交点
            if(minDist == 0 || minDist > distSq) {
                findIntersection = true;
                minDist = distSq;

                // 存储交点信息
                intersectionOnMap->points[horizonIndex] = intersectionOnMapThisLine;

                // 标记通道交点
                if((int)map_pc->points[j % mapSize].intensity > 2 &&
                   (int)map_pc->points[(j + 1) % mapSize].intensity > 2) {
                    // 当测试所有通道打开时，标记此点为通道交点，使其有所不同
                    intersectionOnMap->points[horizonIndex].intensity = -1;
                }

                // 记录此激光线与地图线段的交点信息，用于后续的pedal计算
                ringMapP1->points[horizonIndex] = map_pc->points[j % mapSize];
                ringMapP2->points[horizonIndex] = map_pc->points[(j + 1) % mapSize];
                last_index = j % mapSize;

                // 根据不同情况更新transformed_pc中的intensity值
                if(initialized || (!bTestRescue && !bRescueRobot)) {
                    // 对于已初始化或非救援情，更新整列的intensity
                    for(int i = 0; i < N_SCAN; i++) {
                        transformed_pc->points[i * Horizon_SCAN + horizonIndex].intensity =
                            j % mapSize;
                    }
                }
                else {
                    // 对于其他情况，只更新当前点的intensity
                    transformed_pc->points[horizonIndex].intensity = j % mapSize;
                }
            }
        }
    }

    return findIntersection;  // 返回是否找到交点
}

/**
 * @brief 检查激光线与整个地图的交点，特别是处理穿过通道和玻璃的情况
 * @details 此函数通过以下步骤检查激光线与整个地图的交点：
 *          1. 获取机器人的当前位置
 *          2. 遍历地图点，检查是否有交点
 *          3. 返回是否找到交点
 *
 * @param pc_index 激光线在transformed_pc中的索引
 * @param PCPoint 激光线上的当前点
 * @param map1x 地图线段的第一个点的x坐标
 * @param map1y 地图线段的第一个点的y坐标
 * @param map2x 地图线段的第二个点的x坐标
 * @param map2y 地图线段的第二个点的y坐标
 * @param intersectionx 交点的x坐标
 * @param intersectiony 交点的y坐标
 */
// 由于机器人可以穿过通道和玻璃，这次找到与激光点最接近的交点，而不是射线追踪的第一个交点
bool CloudHandler::checkWholeMap(int pc_index,
                               const pcl::PointXYZI& PCPoint,
                               double& map1x,
                               double& map1y,
                               double& map2x,
                               double& map2y,
                               double& intersectionx,
                               double& intersectiony) {
    // 获取机器人当前位置（从变换矩阵中提取）
    pcl::PointXYZI PosePoint;
    PosePoint.x = robotPose(0,3);
    PosePoint.y = robotPose(1,3);
    PosePoint.z = 0;  // 2D平面定位，z坐标置0

    // 初始化搜索参数
    double min_error = 0;        // 最小匹配误差
    double min_PCLength = 0;     // 最小激光点距离
    double min_mapLength = 0;    // 最小地图点距离
    bool bMatchWithPass = false; // 是否与通道匹配的标志
    int start_index = 0;         // 搜索起始索引

    // 优化搜索起点：使用历史记录加速搜索
    if(outsideAreaIndexRecord[pc_index] != 0) {
        // 使用当前点的历史匹配记录
        start_index = outsideAreaIndexRecord[pc_index];
    }
    else if(outsideAreaLastRingIndexRecord[pc_index % Horizon_SCAN] != 0 &&
            calDistance(transformed_pc->points[pc_index - Horizon_SCAN],
                       transformed_pc->points[pc_index]) < 0.8) {
        // 如果相邻两圈的激光点距离小于0.8米，使用上一圈的匹配记录
        start_index = outsideAreaLastRingIndexRecord[pc_index % Horizon_SCAN];
    }

    // 遍历地图点寻找最优交点
    for(size_t i = start_index; i < map_pc->size() + start_index; i++) {
        // 通道处理模式：如果设置为开放所有通道，则跳过通道线段
        if(bAllPassageOpen) {
            if((int)map_pc->points[i % mapSize].intensity > 2 &&
               (int)map_pc->points[(i + 1) % mapSize].intensity > 2) {
                continue;
            }
        }

        // 跳过区域边界点（intensity % 3 == 2 表示区域边界）
        if(((int)map_pc->points[i % mapSize].intensity) % 3 == 2) {
            continue;
        }

        // 计算激光射线与地图线段的交点
        pcl::PointXYZI intersectionOnMapThisLine;
        bool inbetween = inBetween(PosePoint,  // 射线起点（机器人位置）
                                 PCPoint,      // 射线终点（激光点）
                                 map_pc->points[i % mapSize],         // 地图线段起点
                                 map_pc->points[(i + 1) % mapSize],   // 地图线段终点
                                 &intersectionOnMapThisLine);         // 输出：交点

        // 如果找到交点，更新最优匹配
        if(inbetween) {
            // 计算交点到激光点的距离
            double dist = calDistance(intersectionOnMapThisLine, PCPoint);

            // 更新最优匹配（距离更小的交点）
            if(min_error == 0 || min_error > dist) {
                min_error = dist;

                // 记录最优匹配的地图线段端点
                map1x = map_pc->points[i % mapSize].x;
                map1y = map_pc->points[i % mapSize].y;
                map2x = map_pc->points[(i + 1) % mapSize].x;
                map2y = map_pc->points[(i + 1) % mapSize].y;

                // 计算射线和地图的长度
                double mapLength = calDistance(intersectionOnMapThisLine, PosePoint);
                double PCLength = calDistance(PCPoint, PosePoint);
                min_mapLength = mapLength;
                min_PCLength = PCLength;

                // 更新索引记录（用于下次搜索优化）
                outsideAreaIndexRecord[pc_index] = i % mapSize;
                outsideAreaLastRingIndexRecord[pc_index % Horizon_SCAN] = i % mapSize;

                // 记录交点坐标
                intersectionx = intersectionOnMapThisLine.x;
                intersectiony = intersectionOnMapThisLine.y;

                // 检查是否与通道匹配（intensity > 2 表示通道）
                if(map_pc->points[i % mapSize].intensity > 2) {
                    bMatchWithPass = true;
                }
            }

            // 如果使用了优化起点，找到第一个交点后就退出
            if(start_index) {
                break;
            }
        }
    }

    // 返回是否找到有效的通道穿透点：
    // 1. 必须与通道匹配(bMatchWithPass为true)
    // 2. 误差必须大于1米(min_error > 1)
    return (bMatchWithPass && min_error > 1);
}


// 每帧和每次猜测时调用，用于重置参数
void CloudHandler::resetParameters() {
    // 1. 清空临时点云数据
    laserCloudIn->clear();        // 清空输入激光点云
    UsefulPoints1->clear();       // 清空有用点集1
    UsefulPoints2->clear();       // 清空有用点集2

    // 2. 重置环形扫描相关的点云大小
    ringMapP1->points.resize(Horizon_SCAN, 0);  // 重置环形地图点云1的大小
    ringMapP2->points.resize(Horizon_SCAN, 0);  // 重置环形地图点云2的大小

    intersectionOnMap->clear();                       // 清空地图上的交点
    intersectionOnMap->points.resize(Horizon_SCAN, 0);  // 重置交点大小
    numIcpPoints = 0;                                 // 重置ICP点数
    furthestRing->clear();                            // 清空最远环
    furthestRing->points.resize(Horizon_SCAN);        // 重置最远环大小
    intersectionOnMap->points.resize(Horizon_SCAN, 0);  // 再次重置交点大小（可能是冗余的，但按照Fujing的代码，这里需要重置）

    // 3. 重置组织化点云
    organizedCloudIn->clear();                                // 清空组织化输入点云
    organizedCloudIn->points.resize(N_SCAN * Horizon_SCAN, 0);  // 重置组织化输入点云大小

    organizedCloudIn64->clear();                              // 清空64线组织化输入点云
    organizedCloudIn64->points.resize(N_SCAN * Horizon_SCAN, 0);  // 重置64线组织化输入点云大小

    transformed_pc->clear();                                  // 清空变换后的点云
    transformed_pc->points.resize(N_SCAN * Horizon_SCAN, 0);    // 重置变换后点云大小

    UsefulPoints1->points.resize(N_SCAN * Horizon_SCAN, 0);     // 重置有用点集1大小
    UsefulPoints2->points.resize(N_SCAN * Horizon_SCAN, 0);     // 重置有用点集2大小
}

// 过滤有用点
void CloudHandler::filterUsefulPoints() {
    // 记录开始时间
    auto startTime = this->now();
    if (!transformed_pc || transformed_pc->empty()) {
        RCLCPP_ERROR(get_logger(), "Invalid transformed point cloud");
        return;
    }

    // 确保width不为0
    if (transformed_pc->width == 0) {
        RCLCPP_ERROR(get_logger(), "Point cloud width cannot be zero");
        return;
    }

    // 重置每次迭代的中心点和权重相关参数
    PCCenter.setZero();          // 点云中心点
    mapCenter.setZero();         // 地图中心点
    numIcpPoints = 0;            // ICP点数量
    usefulIndex.clear();         // 有用点索引
    weightSumTurkey = 0;         // Turkey权重和
    weightSumCauchy = 0;         // Cauchy权重和
    weightsTurkey.clear();       // Turkey权重列表

    // 调整记录数组大小
    outsideAreaIndexRecord.resize(transformed_pc->points.size(), 0);        // 区域外点索引记录
    outsideAreaLastRingIndexRecord.resize(Horizon_SCAN, 0);

    // Debug: 在 filterUsefulPoints() 函数中添加以下调试信息
    RCLCPP_DEBUG(this->get_logger(), "transformed_pc size: %zu", transformed_pc->points.size());
    RCLCPP_DEBUG(this->get_logger(), "intersectionOnMap size: %zu", intersectionOnMap->points.size());
    RCLCPP_DEBUG(this->get_logger(), "Horizon_SCAN value: %d", Horizon_SCAN);
    RCLCPP_DEBUG(this->get_logger(), "Valid points before filtering: %zu", transformed_pc->points.size());

    // 遍历所有变换后的点云
    for(size_t i = 0; i < transformed_pc->points.size(); i++) {
        // 添加索引安全检查
        if (i >= intersectionOnMap->points.size()) {
            RCLCPP_ERROR(this->get_logger(), "Index out of bounds: i=%zu, intersectionOnMap size=%zu",
                        i, intersectionOnMap->points.size());
            continue;
        }

        // 检查点是否为NaN
        if(std::isnan(transformed_pc->points[i].x) ||
           std::isnan(transformed_pc->points[i].y) ||
           std::isnan(intersectionOnMap->points[i % Horizon_SCAN].x) ||
           std::isnan(intersectionOnMap->points[i % Horizon_SCAN].y)) {
            RCLCPP_INFO(this->get_logger(), "NaN point detected at index %zu", i);
            continue;
        }

        // 获取点到激光雷达的距离
        double distance = organizedCloudIn->points[i].intensity;

        // 处理有效的交点
        if(abs(intersectionOnMap->points[i % Horizon_SCAN].x) > 1e-6 &&
           abs(intersectionOnMap->points[i % Horizon_SCAN].y) > 1e-6) {

            double pedalx, pedaly;                // 垂足坐标
            double intersectionx, intersectiony;   // 交点坐标

            // 计算地图上的距离
            double temp_map_length = std::sqrt(
                std::pow(intersectionOnMap->points[i % Horizon_SCAN].x - robotPose(0,3), 2) +
                std::pow(intersectionOnMap->points[i % Horizon_SCAN].y - robotPose(1,3), 2));
            double match_difference = distance - temp_map_length;  // 距离差值

            // 根据通道处理模式进行处理
            if(!bAllPassageClose && !bAllPassageOpen) {  // 正常模式
                // 处理通道点
                if(match_difference > 0.1 &&
                   ringMapP1->points[i % Horizon_SCAN].intensity > 2 &&
                   ringMapP2->points[i % Horizon_SCAN].intensity > 2) {

                    // 检查整个地图上的穿透情况
                    double map1x = 0, map1y = 0, map2x = 0, map2y = 0;
                    bool countGoingthrough = checkWholeMap(i, transformed_pc->points[i],
                                                         map1x, map1y, map2x, map2y,
                                                         intersectionx, intersectiony);
                    if(countGoingthrough) continue;

                    // 计算垂足
                    calPedal(map1x, map1y, map2x, map2y,
                            transformed_pc->points[i].x, transformed_pc->points[i].y,
                            pedalx, pedaly);
                }
                else {
                    // 计算普通点的垂足
                    calPedal(ringMapP1->points[i % Horizon_SCAN].x,
                            ringMapP1->points[i % Horizon_SCAN].y,
                            ringMapP2->points[i % Horizon_SCAN].x,
                            ringMapP2->points[i % Horizon_SCAN].y,
                            transformed_pc->points[i].x,
                            transformed_pc->points[i].y,
                            pedalx, pedaly);

                    intersectionx = intersectionOnMap->points[i % Horizon_SCAN].x;
                    intersectiony = intersectionOnMap->points[i % Horizon_SCAN].y;
                }
            }
            // 处理关闭的通道
            else if(bAllPassageClose) {
                calPedal(ringMapP1->points[i % Horizon_SCAN].x,
                        ringMapP1->points[i % Horizon_SCAN].y,
                        ringMapP2->points[i % Horizon_SCAN].x,
                        ringMapP2->points[i % Horizon_SCAN].y,
                        transformed_pc->points[i].x,
                        transformed_pc->points[i].y,
                        pedalx, pedaly);

                intersectionx = intersectionOnMap->points[i % Horizon_SCAN].x;
                intersectiony = intersectionOnMap->points[i % Horizon_SCAN].y;
            }
            // 处理开放的通道
            else if(bAllPassageOpen) {
                if(intersectionOnMap->points[i % Horizon_SCAN].intensity == -1) {
                    if(match_difference > 0.5) {
                        // 检查整个地图上的穿透情况
                        double map1x = 0, map1y = 0, map2x = 0, map2y = 0;
                        bool countGoingthrough = checkWholeMap(i, transformed_pc->points[i],
                                                             map1x, map1y, map2x, map2y,
                                                             intersectionx, intersectiony);
                        if(countGoingthrough) continue;

                        // 计算垂足
                        calPedal(map1x, map1y, map2x, map2y,
                                transformed_pc->points[i].x, transformed_pc->points[i].y,
                                pedalx, pedaly);
                    }
                    else continue;
                }
                else {
                    // 计算普通点的垂足
                    calPedal(ringMapP1->points[i % Horizon_SCAN].x,
                            ringMapP1->points[i % Horizon_SCAN].y,
                            ringMapP2->points[i % Horizon_SCAN].x,
                            ringMapP2->points[i % Horizon_SCAN].y,
                            transformed_pc->points[i].x,
                            transformed_pc->points[i].y,
                            pedalx, pedaly);

                    intersectionx = intersectionOnMap->points[i % Horizon_SCAN].x;
                    intersectiony = intersectionOnMap->points[i % Horizon_SCAN].y;
                }
            }

            // 计算点的度量
            double pcx = transformed_pc->points[i].x;
            double pcy = transformed_pc->points[i].y;
            // 计算地图长度
            double maplength = std::sqrt(std::pow(intersectionx - robotPose(0,3), 2) +
                                       std::pow(intersectiony - robotPose(1,3), 2));
            double error = distance - maplength;  // 距离误差
            // 计算垂直误差
            double error_vertical = std::sqrt(std::pow(pedalx - pcx, 2) +
                                            std::pow(pedaly - pcy, 2));

            // 处理有用的点
            if((error < 0.0 && error_vertical < errorLowThredCurr) ||
               (error > 0.0 && error_vertical < errorUpThredCurr)) {
                numIcpPoints++;
                usefulIndex.push_back(i);

                // 保存有用点和其垂足
                UsefulPoints1->points[i] = transformed_pc->points[i];
                UsefulPoints2->points[i].x = pedalx;
                UsefulPoints2->points[i].y = pedaly;
                UsefulPoints2->points[i].z = transformed_pc->points[i].z;

                // 计算Turkey权重
                double weight = calWeightTurkey(error_vertical, errorLowThredCurr,
                                              (error > 0), errorUpThredCurr);
                weightSumTurkey += weight;
                weightsTurkey.push_back(weight);

                // 更新中心点
                if(use_weight && initialized) {
                    // 使用权重更新
                    PCCenter(0) += weight * pcx;
                    PCCenter(1) += weight * pcy;
                    mapCenter(0) += weight * pedalx;
                    mapCenter(1) += weight * pedaly;
                }
                else {
                    // 不使用权重更新
                    PCCenter(0) += pcx;
                    PCCenter(1) += pcy;
                    mapCenter(0) += pedalx;
                    mapCenter(1) += pedaly;
                }

                // 保存带权重的坐标
                Vec_pcx.push_back(weight * pcx);
                Vec_pcy.push_back(weight * pcy);
                Vec_pedalx.push_back(weight * pedalx);
                Vec_pedaly.push_back(weight * pedaly);

                // 更新配对点的平均距离
                averDistancePairedPoints += error_vertical;

                // 第一次迭代时更新地图直方图
                if(currentIteration == 0) {
                    mapHistogram[transformed_pc->points[i].intensity]++;
                    numTotalHistogram++;
                }
            }
        }
    }

    // 在计算完所有点后添加安全检查和调试信息
    RCLCPP_DEBUG(this->get_logger(), "Number of ICP points: %d", numIcpPoints);
    RCLCPP_DEBUG(this->get_logger(), "Turkey weight sum: %f", weightSumTurkey);

    // 增强的位姿跟踪失败检测
    if (numIcpPoints == 0) {
        RCLCPP_ERROR(this->get_logger(), "❌ 位姿跟踪失败: 没有找到有效的ICP点!");

        // 如果是从全局定位刚切换过来，可能需要调整阈值
        if (initialized && errorUpThred < 2.0) {
            RCLCPP_WARN(this->get_logger(), "尝试放宽误差阈值以恢复跟踪...");
            errorUpThred = std::min(errorUpThred * 1.5, 3.0);
            errorLowThred = std::min(errorLowThred * 1.2, 1.5);
            RCLCPP_INFO(this->get_logger(), "调整后阈值: errorUpThred=%.1f, errorLowThred=%.1f",
                         errorUpThred, errorLowThred);
        }
        return;
    }

    if (weightSumTurkey < 1e-6) {
        RCLCPP_WARN(this->get_logger(), "Turkey weight sum is too small: %f", weightSumTurkey);
    }

    // 检测跟踪质量
    if (numIcpPoints < 10) {
        RCLCPP_WARN(this->get_logger(), "⚠️  跟踪质量较差: ICP点数量过少 (%d < 10)", numIcpPoints);
    } else if (numIcpPoints > 50) {
        RCLCPP_INFO(this->get_logger(), "✅ 跟踪质量良好: ICP点数量充足 (%d)", numIcpPoints);
    }
}

// 根据相似角度合并地图直方图，用于通道检测
void CloudHandler::mergeMapHistogram() {
    // 用于存储有用点的索引和权重
    std::vector<int> usefulIndexHistogram;
    std::vector<double> weightsTurkeyHistogram;

    // 设置角度间隔和总区间数
    double intervalDeg = 5;  // 每个区间5度
    int interval = ceil(180/intervalDeg);  // 总共36个区间(0-180度)
    std::vector<double> histogram(interval, 0);  // 初始化直方图数组

    // 初始化统计指标
    int total_hit_points = 0;  // 总命中点数

    // 处理地图点,计算每条线段的角度并更新直方图
    for(size_t i = 0; i < map_pc->points.size(); i++) {
        // 计算当前点与下一个点形成的线段角度
        double angle = std::atan2(map_pc->points[i].y - map_pc->points[(i+1) % map_pc->points.size()].y,
                                map_pc->points[i].x - map_pc->points[(i+1) % map_pc->points.size()].x);

        // 将角度转换到[0,180]区间
        angle = (angle + M_PI/2) / M_PI * 180;
        int index = floor(angle/intervalDeg);

        // 更新直方图和总点数
        histogram[index] += mapHistogram[i];
        total_hit_points += mapHistogram[i];
    }

    // 寻找直方图最大值及其索引
    int max_value = 0;
    int max_index = 0;
    int total_points = 0;

    for(size_t i = 0; i < histogram.size(); i++) {
        total_points += histogram[i];
        if(histogram[i] > max_value) {
            max_value = histogram[i];
            max_index = i;
        }
    }

    // 找出与最大角度对应的地图线段
    std::vector<int> mapLineIndex;
    for(size_t i = 0; i < map_pc->points.size(); i++) {
        double angle = std::atan2(map_pc->points[i].y - map_pc->points[(i+1) % map_pc->points.size()].y,
                                map_pc->points[i].x - map_pc->points[(i+1) % map_pc->points.size()].x);
        angle = (angle + M_PI/2) / M_PI * 180;
        int index = floor(angle/intervalDeg);

        // 保存最大角度对应的非空线段索引
        if(index == max_index && mapHistogram[i] != 0) {
            mapLineIndex.push_back(i);
        }
    }

    // 计算通道性指标
    double maxPercentage = static_cast<double>(max_value) / (total_points + 0.001);  // 最大方向占比
    double DSrate = corridornessDSRate(maxPercentage);  // 计算降采样率

    // 输出通道性指标信息
    RCLCPP_INFO(this->get_logger(),
                "Corridor metrics - Total points: %d, Max percentage: %f, DS rate: %f",
                total_points, maxPercentage, DSrate);

    // 如果通道性指标超过阈值,进行点云降采样处理
    if(DSrate > maxPercentageCorridor) {
        onlyOneDirection = true;
        int temp_times = 0;  // 用于控制降采样频率
        int minus_times = 0;  // 统计被移除的点数

        // 处理每个有用点
        for(size_t i = 0; i < usefulIndex.size(); i++) {
            bool find = false;
            // 检查点是否属于主方向
            for(size_t j = 0; j < mapLineIndex.size(); j++) {
                if(mapLineIndex[j] == int(UsefulPoints1->points[usefulIndex[i]].intensity)) {
                    find = true;
                    double distance = organizedCloudIn->points[usefulIndex[i]].intensity;

                    // 根据距离和降采样率处理点
                    if(distance < corridorDSmaxDist) {
                        if((int)(i/DSrate) != temp_times) {
                            // 保留该点
                            temp_times = (int)(i/DSrate);
                            usefulIndexHistogram.push_back(usefulIndex[i]);
                            weightsTurkeyHistogram.push_back(weightsTurkey[i]);
                        }
                        else {
                            // 移除该点并更新相关统计量
                            vbHistogramRemain[usefulIndex[i]] = false;
                            numIcpPoints--;
                            UsefulPoints1->points[usefulIndex[i]].x = 0;
                            UsefulPoints1->points[usefulIndex[i]].y = 0;
                            UsefulPoints1->points[usefulIndex[i]].z = 0;
                            UsefulPoints1->points[usefulIndex[i]].intensity = -1;

                            // 更新中心点坐标
                            if(use_weight && initialized) {
                                PCCenter(0) -= Vec_pcx[i];
                                PCCenter(1) -= Vec_pcy[i];
                                mapCenter(0) -= Vec_pedalx[i];
                                mapCenter(1) -= Vec_pedaly[i];
                                weightSumTurkey -= weightsTurkey[i];
                            }
                            else {
                                PCCenter(0) -= Vec_pcx[i];
                                PCCenter(1) -= Vec_pcy[i];
                                mapCenter(0) -= Vec_pedalx[i];
                                mapCenter(1) -= Vec_pedaly[i];
                            }
                            minus_times++;
                        }
                    }
                    else {
                        // 距离超过阈值的点直接保留
                        usefulIndexHistogram.push_back(usefulIndex[i]);
                        weightsTurkeyHistogram.push_back(weightsTurkey[i]);
                    }
                }
            }
            // 不属于主方向的点直接保留
            if(!find) {
                usefulIndexHistogram.push_back(usefulIndex[i]);
                weightsTurkeyHistogram.push_back(weightsTurkey[i]);
            }
        }

        // 更新有用点索引和权重
        usefulIndex = usefulIndexHistogram;
        weightsTurkey = weightsTurkeyHistogram;
    }
}

// 分配内存
void CloudHandler::allocateMemory() {

    // 添加前置检查,确保激光雷达参数有效
    if (N_SCAN <= 0 || Horizon_SCAN <= 0) {
        RCLCPP_ERROR(get_logger(), "Invalid N_SCAN(%d) or Horizon_SCAN(%d)",
                     N_SCAN, Horizon_SCAN);
        return;
    }

    // 初始化原始点云数据的智能指针
    //使用PointXYZIRT类型以保留Velodyne激光雷达的ring和time信息
    laserCloudIn = std::make_shared<pcl::PointCloud<PointXYZIRT>>();  // 原始激光雷达数据
    laserUppestRing = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();  // 最上层激光点云
    potentialCeilingPoints = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();  // 潜在天花板点云
    organizedCloudIn = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();  // 组织化的输入点云
    organizedCloudIn64 = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();  // 64线组织化点云
    transformed_pc = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();  // 变换后的点云
    UsefulPoints1 = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();  // ICP配准用的源点云
    UsefulPoints2 = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();  // ICP配准用的目标点云
    map_pc = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();  // 地图点云
    mapCorridorEnlarge_pc = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();  // 扩大后的走廊地图点云

    // 初始化地图相关点云的智能指针
    ringMapP1 = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();  // 环形地图点云1
    ringMapP2 = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();  // 环形地图点云2
    intersectionOnMap = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();  // 地图上的交点
    furthestRing = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();  // 最远环
    transformedFurthestRing = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();  // 变换后的最远环
    insidePC = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();  // 内部点云
    outsidePC = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();  // 外部点云

    // 预分配点云大小以提高性能
    potentialCeilingPoints->points.resize(N_SCAN * Horizon_SCAN);  // 基于扫描参数分配天花板点云大小
    ringMapP1->points.resize(Horizon_SCAN);  // 基于水平扫描分配环形地图大小
    ringMapP2->points.resize(Horizon_SCAN);
    intersectionOnMap->points.resize(Horizon_SCAN);
    furthestRing->points.resize(Horizon_SCAN);
    transformedFurthestRing->points.resize(Horizon_SCAN);
    insidePC->points.resize(Horizon_SCAN);
    outsidePC->points.resize(Horizon_SCAN);

    // 分配组织化点云和ICP相关点云的大小
    organizedCloudIn->points.resize(N_SCAN * Horizon_SCAN);
    organizedCloudIn64->points.resize(64 * Horizon_SCAN);  // 专门为64线激光雷达设置
    transformed_pc->points.resize(N_SCAN * Horizon_SCAN);
    UsefulPoints1->points.resize(N_SCAN * Horizon_SCAN);
    UsefulPoints2->points.resize(N_SCAN * Horizon_SCAN);

    // 输出调试信息
    RCLCPP_DEBUG(this->get_logger(),
                 "Allocated pointclouds - Organized size: %lu, Intersection size: %lu",
                 organizedCloudIn->points.size(),
                 intersectionOnMap->points.size());
}


// optimizationICP()函数本身没有显式的输入参数，但它使用了类内的成员变量作为输入数据：

    // 输入（类成员变量）：
    // transformed_pc：需要被配准的点云
    // UsefulPoints1和UsefulPoints2：用于ICP匹配的源点云和目标点云
    // robotPose：当前机器人的位姿估计

    // 输出（更新的类成员变量）：
    // robotPose：更新后的机器人位姿矩阵（4x4的变换矩阵）
    // transformed_pc：经过新位姿变换后的点云
    // globalPath：将新的位姿添加到全局路径中
    // initialized：如果ICP收敛，会将这个标志设置为true
// ICP优化函数，用于对点云进行迭代最近点匹配优化
void CloudHandler::optimizationICP() {
    // 根据是否已初始化决定迭代次数
    int totalIteration = initialized ? icp_iteration : icp_init_iteration;

    // 开始ICP迭代
    for(int iteration = 0; iteration < totalIteration; iteration++) {
        auto startTime = this->now();

        // 在每次ICP迭代开始时，函数会清空上一次迭代的数据
        averDistancePairedPoints = 0;  // 配对点的平均距离
        currentIteration = iteration;  // 当前迭代次数
        Vec_pcx.clear();  // 清空点云x坐标向量
        Vec_pcy.clear();  // 清空点云y坐标向量
        Vec_pedalx.clear();  // 清空垂足x坐标向量
        Vec_pedaly.clear();  // 清空垂足y坐标向量

        // 过滤和处理用于ICP匹配的有效点： 通过筛选得到高质量的点对（UsefulPoints1和UsefulPoints2 && usefulIndex）
        filterUsefulPoints();

        // 在计算中心点之前添加检查
        if (numIcpPoints == 0) {
            RCLCPP_ERROR(this->get_logger(), "No valid points for ICP, skipping optimization");
            return;
        }

        if (use_weight && weightSumTurkey < 1e-6) {
            RCLCPP_ERROR(this->get_logger(), "Invalid weight sum for Turkey weights");
            return;
        }

        // 如果启用了走廊检测，则处理走廊检测
        if(detect_corridor) {
            RCLCPP_INFO(this->get_logger(), "Detecting corridor...");
            mergeMapHistogram();
        }

        // 计算点云中心点，根据是否使用权重采用不同计算方式
        if(use_weight) {
            mapCenter = mapCenter / weightSumTurkey;  // 使用Turkey权重的地图中心
            PCCenter = PCCenter / weightSumTurkey;    // 使用Turkey权重的点云中心
        } else {
            mapCenter = mapCenter / numIcpPoints;     // 使用点数平均的地图中心
            PCCenter = PCCenter / numIcpPoints;       // 使用点数平均的点云中心
        }

        // 计算变换矩阵W
        Eigen::Matrix2d W = Eigen::Matrix2d::Zero();

        // 遍历所有ICP点对，构建变换矩阵
        for(int i = 0; i < numIcpPoints; i++) {
            if(UsefulPoints1->points[usefulIndex[i]].x != 0 ||
               UsefulPoints1->points[usefulIndex[i]].y != 0) {

                Eigen::Vector2d PCVec, MapVec;

                // 根据是否使用权重和是否初始化选择不同的计算方式
                if(use_weight && initialized) {
                    // 使用原始坐标计算
                    PCVec << UsefulPoints1->points[usefulIndex[i]].x,
                            UsefulPoints1->points[usefulIndex[i]].y;
                    MapVec << UsefulPoints2->points[usefulIndex[i]].x,
                            UsefulPoints2->points[usefulIndex[i]].y;
                    W += weightsTurkey[i] * MapVec * PCVec.transpose();
                } else {
                    // 使用去中心化的坐标计算
                    PCVec << UsefulPoints1->points[usefulIndex[i]].x - PCCenter(0),
                            UsefulPoints1->points[usefulIndex[i]].y - PCCenter(1);
                    MapVec << UsefulPoints2->points[usefulIndex[i]].x - mapCenter(0),
                            UsefulPoints2->points[usefulIndex[i]].y - mapCenter(1);
                    W += MapVec * PCVec.transpose();
                }
            }
        }

        // 如果使用权重且已初始化，对W矩阵进行额外处理
        if(use_weight && initialized) {
            W = 1/weightSumTurkey * W - mapCenter * PCCenter.transpose();
        }

        // 使用SVD分解求解最优旋转矩阵
        Eigen::JacobiSVD<Eigen::Matrix2d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix2d rotationMatrix = svd.matrixU() * svd.matrixV().transpose();
        // 计算平移向量
        Eigen::Vector2d translation = mapCenter - rotationMatrix * PCCenter;

        // 输出调试信息
        RCLCPP_DEBUG(this->get_logger(),
                     "ICP iteration %d - Translation norm: %f, Threshold: %f",
                     iteration, translation.norm(), errorLowThredCurr);

        // 发布中间结果点云
        sensor_msgs::msg::PointCloud2 cloud_msg;
        cloud_msg.header = mapHeader;

        // UsefulPoints1是源点云 --- 代表LiDAR帧点云中被筛选后的用于ICP的有效点，UsefulPoints2是目标点云 --- 来自地图
        // 移除调试用点云发布代码，保留核心功能

        // 更新机器人位姿
        Eigen::Matrix4f robotPoseOldInv = robotPose.inverse();
        robotPose(0,3) += translation(0);  // 更新x方向平移
        robotPose(1,3) += translation(1);  // 更新y方向平移
        robotPose(3,3) = 1;
        robotPose.topLeftCorner(2,2) = rotationMatrix.cast<float>() * robotPose.topLeftCorner(2,2);  // 更新旋转部分
        robotPose(2,2) = 1;

        // 根据新的位姿变换点云
        pcl::transformPointCloud(*transformed_pc, *transformed_pc, robotPose * robotPoseOldInv);

        // 每3次迭代重置点云大小，防止内存占用过大
        if(iteration % 3 == 0) {
            UsefulPoints1->points.resize(N_SCAN * Horizon_SCAN);
            UsefulPoints2->points.resize(N_SCAN * Horizon_SCAN);
        }

        // 检查收敛条件：如果平移量小于阈值且旋转角度小于阈值，则认为收敛
        if(std::isnan(translation.norm()) ||
           (translation.norm() < icp_stop_translation_thred &&
            acos(std::abs(rotationMatrix(0,0)))/M_PI*180 < icp_stop_rotation_thred)) {
            if(!bTestRescue) {
                initialized = true;
                RCLCPP_DEBUG(get_logger(), "ICP收敛: 平移=%.4f, 旋转=%.2f度, 迭代次数=%d",
                           translation.norm(),
                           acos(std::abs(rotationMatrix(0,0)))/M_PI*180,
                           iteration + 1);
            }
            break;
        }

        // 添加发散检测
        if(translation.norm() > 5.0) {
            RCLCPP_WARN(get_logger(), "⚠️  ICP可能发散: 平移量过大 (%.2f > 5.0)", translation.norm());
            // 如果是刚从全局定位切换过来，可能需要更多迭代
            if(iteration < 3) {
                RCLCPP_INFO(get_logger(), "继续迭代以稳定位姿...");
                continue;
            } else {
                RCLCPP_ERROR(get_logger(), "❌ ICP发散，停止迭代");
                break;
            }
        }
    }

    // 准备并发布机器人位姿
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = mapHeader;
    pose_stamped.pose.position.x = robotPose(0,3);
    pose_stamped.pose.position.y = robotPose(1,3);
    pose_stamped.pose.position.z = 0;

    // 将旋转矩阵转换为四元数
    Eigen::Matrix3d rotation3d = Eigen::Matrix3d::Identity();
    rotation3d.topLeftCorner(3,3) = robotPose.topLeftCorner(3,3).cast<double>();
    Eigen::Quaterniond quaternion(rotation3d);

    // 设置位姿的方向信息
    pose_stamped.pose.orientation.x = quaternion.x();
    pose_stamped.pose.orientation.y = quaternion.y();
    pose_stamped.pose.orientation.z = quaternion.z();
    pose_stamped.pose.orientation.w = quaternion.w();

    // 发布当前位姿给AGLoc定位器
    pubRobotPose->publish(pose_stamped);

    // ========== 应用里程计融合 ==========
    if (enable_odom_fusion && fusion_initialized_) {
        // 计算ICP得分
        double icp_score = computeICPScore();

        // 应用里程计融合
        Eigen::Matrix4f fused_pose = applyOdomFusion(robotPose, icp_score, mapHeader.stamp);

        // 更新机器人位姿为融合后的结果
        robotPose = fused_pose;

        // 重新计算位姿消息
        pose_stamped.pose.position.x = robotPose(0,3);
        pose_stamped.pose.position.y = robotPose(1,3);
        pose_stamped.pose.position.z = 0;

        // 重新计算四元数
        Eigen::Matrix3d rotation3d_fused = Eigen::Matrix3d::Identity();
        rotation3d_fused.topLeftCorner(3,3) = robotPose.topLeftCorner(3,3).cast<double>();
        Eigen::Quaterniond quaternion_fused(rotation3d_fused);

        pose_stamped.pose.orientation.x = quaternion_fused.x();
        pose_stamped.pose.orientation.y = quaternion_fused.y();
        pose_stamped.pose.orientation.z = quaternion_fused.z();
        pose_stamped.pose.orientation.w = quaternion_fused.w();

        if (debug_fusion) {
            RCLCPP_DEBUG(get_logger(), "应用里程计融合: ICP得分=%.6f, 融合位姿=[%.3f, %.3f]",
                         icp_score, robotPose(0,3), robotPose(1,3));
        }
    }

    // 将位姿添加到全局路径并发布
    globalPath.poses.push_back(pose_stamped);
    pubRobotPath->publish(globalPath);
    saveTUMTraj(pose_stamped);  // 保存轨迹到TUM格式文件
}

/**
 * @brief 设置手动初始位姿
 * @details 此函数用于手动设置机器人的初始位姿
 *
 * @param yaw 机器人朝向（偏航角），单位弧度
 * @param position 机器人位置，3D向量(x,y,z)
 */
void CloudHandler::setManualInitialPose(double yaw, const Eigen::Vector3f& position) {
    // 创建旋转矩阵（绕Z轴旋转yaw弧度）
    Eigen::Matrix3f rot;
    rot << cos(yaw), -sin(yaw), 0,
           sin(yaw), cos(yaw), 0,
           0, 0, 1;

    // 设置robotPose矩阵的旋转部分
    robotPose.block<3,3>(0,0) = rot;

    // 设置robotPose矩阵的平移部分
    robotPose.block<3,1>(0,3) = position;

    // 标记已有手动设置的初始位姿
    hasManualInitialPose = true;

    RCLCPP_INFO(get_logger(), "手动设置初始位姿: 位置[%f, %f, %f], 偏航角[%f]",
                position[0], position[1], position[2], yaw);
}

// ========== 里程计融合相关实现 ==========

/**
 * @brief 初始化里程计融合模块
 */
void CloudHandler::initializeOdomFusion() {
    if (!enable_odom_fusion) {
        return;
    }

    try {
        odom_fusion_ = std::make_unique<agloc_fusion::OdomFusion>(this);
        fusion_initialized_ = true;

        RCLCPP_INFO(get_logger(), "里程计融合模块初始化成功");
        RCLCPP_INFO(get_logger(), "里程计话题: %s, 超时时间: %.2f秒",
                    odom_topic.c_str(), odom_timeout);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "里程计融合模块初始化失败: %s", e.what());
        fusion_initialized_ = false;
    }
}

/**
 * @brief 里程计回调函数
 * @param odom_msg 里程计消息
 */
void CloudHandler::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
    if (!enable_odom_fusion || !fusion_initialized_ || !odom_fusion_) {
        return;
    }

    try {
        odom_fusion_->updateOdometry(odom_msg);

        if (debug_fusion) {
            RCLCPP_DEBUG(get_logger(), "更新里程计数据: pos=[%.3f, %.3f], vel=[%.3f, %.3f]",
                         odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y,
                         odom_msg->twist.twist.linear.x, odom_msg->twist.twist.angular.z);
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "处理里程计数据时出错: %s", e.what());
    }
}

/**
 * @brief 应用里程计融合
 * @param icp_pose ICP优化后的位姿
 * @param icp_score ICP匹配得分
 * @param timestamp 当前时间戳
 * @return 融合后的位姿
 */
Eigen::Matrix4f CloudHandler::applyOdomFusion(const Eigen::Matrix4f& icp_pose,
                                             double icp_score,
                                             const rclcpp::Time& timestamp) {
    if (!enable_odom_fusion || !fusion_initialized_ || !odom_fusion_) {
        return icp_pose;  // 如果未启用融合，直接返回ICP结果
    }

    try {
        // 检查里程计数据是否有效
        if (!odom_fusion_->isOdomValid(odom_timeout)) {
            if (debug_fusion) {
                RCLCPP_WARN(get_logger(), "里程计数据无效或超时，使用纯ICP结果");
            }
            return icp_pose;
        }

        // 基于里程计预测位姿
        Eigen::Matrix4f predicted_pose = odom_fusion_->predictPose(last_fused_pose_, timestamp);

        // 发布预测位姿（如果启用）
        if (publish_prediction && predicted_pose_pub_) {
            publishPredictedPose(predicted_pose, timestamp);
        }

        // 融合ICP结果与里程计预测
        Eigen::Matrix4f fused_pose = odom_fusion_->fusePoses(icp_pose, predicted_pose, icp_score, timestamp);

        // 更新多假设跟踪（如果启用）
        if (enable_multi_hypothesis) {
            odom_fusion_->updateHypotheses(fused_pose, icp_score, timestamp);

            // 获取最佳假设
            auto best_hypothesis = odom_fusion_->getBestHypothesis();
            if (best_hypothesis.weight > hypothesis_weight_threshold) {
                fused_pose = best_hypothesis.pose;
            }
        }

        // 更新状态
        last_fused_pose_ = fused_pose;
        last_fusion_time_ = timestamp;
        last_icp_score_ = icp_score;

        if (debug_fusion) {
            double pos_diff = (icp_pose.block<3,1>(0,3) - predicted_pose.block<3,1>(0,3)).norm();
            RCLCPP_DEBUG(get_logger(), "位姿融合完成: ICP-预测差=%.3f, ICP得分=%.6f", pos_diff, icp_score);
        }

        return fused_pose;

    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "位姿融合时出错: %s", e.what());
        return icp_pose;  // 出错时返回ICP结果
    }
}

/**
 * @brief 计算ICP匹配得分
 * @return ICP得分 (0.0-1.0)
 */
double CloudHandler::computeICPScore() {
    if (numIcpPoints == 0) {
        return 0.0;
    }

    // 基于有效ICP点数和权重计算得分
    double point_ratio = static_cast<double>(numIcpPoints) / (N_SCAN * Horizon_SCAN);
    double weight_score = use_weight ? (weightSumTurkey / numIcpPoints) : 1.0;

    // 综合得分：点数比例 * 权重得分
    double score = point_ratio * weight_score;

    // 限制在[0, 1]范围内
    return std::max(0.0, std::min(1.0, score));
}

/**
 * @brief 发布预测位姿用于可视化
 * @param predicted_pose 预测的位姿
 * @param timestamp 时间戳
 */
void CloudHandler::publishPredictedPose(const Eigen::Matrix4f& predicted_pose,
                                       const rclcpp::Time& timestamp) {
    if (!predicted_pose_pub_) {
        return;
    }

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = timestamp;
    pose_msg.header.frame_id = "map";

    // 设置位置
    pose_msg.pose.position.x = predicted_pose(0,3);
    pose_msg.pose.position.y = predicted_pose(1,3);
    pose_msg.pose.position.z = predicted_pose(2,3);

    // 设置方向
    Eigen::Matrix3f rot = predicted_pose.block<3,3>(0,0);
    Eigen::Quaternionf q(rot);
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    predicted_pose_pub_->publish(pose_msg);
}

/**
 * @brief 处理手动设置的初始位姿回调
 * @details 从/initialpose_agloc话题接收初始位姿消息，并设置robotPose
 *
 * @param poseMsg 包含位姿和协方差的消息
 */
void CloudHandler::manualInitialPoseCB(const std::shared_ptr<geometry_msgs::msg::PoseWithCovarianceStamped> poseMsg) {
    // 从消息中提取位置
    Eigen::Vector3f position;
    position[0] = poseMsg->pose.pose.position.x;
    position[1] = poseMsg->pose.pose.position.y;
    position[2] = poseMsg->pose.pose.position.z;

    // 从四元数中提取偏航角
    tf2::Quaternion q;
    tf2::fromMsg(poseMsg->pose.pose.orientation, q);

    // 获取欧拉角
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // 调用setManualInitialPose设置初始位姿
    setManualInitialPose(yaw, position);

    RCLCPP_INFO(get_logger(), "收到初始位姿消息，设置初始位姿: 位置[%f, %f], 偏航角[%f]",
                position[0], position[1], yaw);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // 创建CloudHandler实例
    auto cloudHandler = std::make_shared<CloudHandler>();

    // Set logging level
    if(rcutils_logging_set_logger_level(
        cloudHandler->get_logger().get_name(),
        RCUTILS_LOG_SEVERITY_INFO)) {
        auto logger = rclcpp::get_logger("CloudHandler");
        RCLCPP_INFO(logger, "Logger level set to DEBUG");
    }

    RCLCPP_INFO(cloudHandler->get_logger(), "CloudHandler node started");

    rclcpp::spin(cloudHandler);
    rclcpp::shutdown();
    return 0;
}
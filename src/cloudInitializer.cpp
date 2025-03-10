/**
 * @file cloudInitializer.cpp
 * @author Jiajie Zhang (ROS2 port)
 *         Fujing Xie (original ROS1 implementation)
 *         Sören Schwertfeger (original ROS1 implementation)
 * @brief Implementation of global localization algorithms for AGLoc
 * @version 0.1
 * @date 2024-11-09
 * 
 * @details Implementation details of the global localization algorithms
 *
 * @copyright Copyright (c) 2024, ShanghaiTech University
 *            All rights reserved.
 */

#include "localization_using_area_graph/cloudInitializer.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <thread>

// 函数声明
bool isValidPoint(const pcl::PointXYZI& point);

// 函数实现
bool isValidPoint(const pcl::PointXYZI& point) {
    // 检查点是否为NaN
    if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z)) {
        return false;
    }
    // 检查点是否为无穷大
    if (std::isinf(point.x) || std::isinf(point.y) || std::isinf(point.z)) {
        return false;
    }
    // 检查点是否为零点（可选，取决于您的应用）
    if (point.x == 0 && point.y == 0 && point.z == 0) {
        return false;
    }
    return true;
}

CloudInitializer::CloudInitializer() : CloudBase("cloud_initializer_node") {
    RCLCPP_DEBUG(get_logger(), "Constructing CloudInitializer");
    RCLCPP_INFO(this->get_logger(), "start of construct main CloudInitializer");
    
    bGuessReady = false;
    MaxRobotPose.setZero();
    MaxScore = 0;
    rescueRunTime = 0;
    rescueTimes = 0;
    
    // Initialize publishers and subscribers
    initializePublishers();
    initializeSubscribers();
    
    // Initialize variables
    initializeVariables();
    
    // Allocate memory for point clouds
    allocateMemory();
    
    // 添加调试信息
    RCLCPP_INFO(get_logger(), "CloudInitializer constructed. AGindexReceived=%d, AG_index size=%zu",
                isAGIndexReceived(), AG_index.area_index.size());
}

void CloudInitializer::setLaserCloudin(
    pcl::PointCloud<pcl::PointXYZI>::Ptr furthestRing_,
    std_msgs::msg::Header mapHeader_) {
    
    *furthestRing = *furthestRing_;
    mapHeader = mapHeader_;
    mapSize = map_pc->points.size();
    numofInsidePoints = 0;
    numofOutsidePoints = 0;
}

void CloudInitializer::setMapPC(pcl::PointCloud<pcl::PointXYZI>::Ptr map_pc_) {
    *map_pc = *map_pc_;
}

void CloudInitializer::getInitialExtGuess(
    // 这里的laserCloudMsg 即为 particle_for_init
    const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg) {
    
    RCLCPP_WARN(this->get_logger(), "CloudInitializer GETTING INITIAL GUESS");

    // 处理 particle_generator 提供的初始粒子
        // 创建一个PCL点云指针
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        // 点云格式转换： PointCloud2 -> PCL
    pcl::fromROSMsg(*laserCloudMsg, *cloud);
        // 获取粒子数量
    int GuessSize = cloud->points.size();

    // 清空之前的猜测数据
    corridorGuess.clear();

    // 遍历所有点,构建3D向量 -- x,y表示猜测的位置坐标， z表示猜测属于哪个Area( TODO 为什么？这个信息最初来自哪里？)
    for (int i = 0; i < GuessSize; i++) {
        Eigen::Vector3f tempGuess;
        tempGuess << cloud->points[i].x,
                    cloud->points[i].y,
                    cloud->points[i].z;
        corridorGuess.push_back(tempGuess);
    }

    // 设置准备标志
    bGuessReady = true;

    // 计算救援时间
    auto startTime = this->now();
    // 重点处理步骤在此
    rescueRobot();
    RCLCPP_INFO(this->get_logger(), "-------------------rescueRobot finished-------------------");
    robotPose = MaxRobotPose;
    auto endTime = this->now();

    RCLCPP_INFO(this->get_logger(), 
                "Number of guesses: %d, Rescue robot run time: %f ms",
                GuessSize,
                (endTime - startTime).seconds() * 1000);
}

void CloudInitializer::rescueRobot() {
    // 添加静态变量来跟踪调用情况
    static int call_count = 0;
    static bool is_running = false;
    static auto last_call_time = this->now();
    auto current_time = this->now();
    
    // 当前调用ID
    int current_call_id = ++call_count;
    
    RCLCPP_WARN(this->get_logger(), 
               "[调用 #%d] 开始执行rescueRobot，距离上次调用间隔: %.3f秒，是否已有实例在运行: %s", 
               current_call_id,
               (current_time - last_call_time).seconds(),
               is_running ? "是" : "否");
               
    // 检查是否有其他实例正在运行
    if (is_running) {
        RCLCPP_ERROR(this->get_logger(), 
                    "[调用 #%d] *** 警告: rescueRobot被重复调用! 前一个实例尚未完成 ***", 
                    current_call_id);
    }
    
    // 标记为正在运行
    is_running = true;
    last_call_time = current_time;

    RCLCPP_INFO(this->get_logger(), "----------- Guess is ready, start rescue -----------------");

    // Prepare output file
    std::ostringstream ts;
    ts.precision(2);
    ts << std::fixed << rclcpp::Time(mapHeader.stamp).seconds();
    // TODO 全局定位结果保存路径
    std::string filename = "/home/jay/AGLoc_ws/frameResult" + 
                          ts.str() + "rescueRoom.txt";
                          
    rescueRoomStream.open(filename, std::ofstream::out | std::ofstream::app);
    rescueRoomStream.setf(std::ios::fixed);
    rescueRoomStream.precision(2);
    
    if (!rescueRoomStream.good()) {
        RCLCPP_ERROR(this->get_logger(), "ERROR OPENING FILE");
        return;
    }

    auto startC = std::chrono::high_resolution_clock::now();
    
    // 如果 initialization_imu 为 false，才会进行角度遍历，否则将会有先验角度信息？-maybe
    if (!initialization_imu) {
        size_t try_time = static_cast<size_t>(360/rescue_angle_interval);
        auto organizedCloudInDS = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        
        // Downsample point cloud
        pcl::VoxelGrid<pcl::PointXYZI> downSizeFurthestRing;
        downSizeFurthestRing.setLeafSize(scoreDownsampleRate, 
                                        scoreDownsampleRate, 
                                        scoreDownsampleRate);
        downSizeFurthestRing.setInputCloud(furthestRing);
        downSizeFurthestRing.filter(*organizedCloudInDS);
        

        RCLCPP_INFO(this->get_logger(), "Downsample size = %lu", 
                    organizedCloudInDS->points.size());

        // FIXME: 两重循环 -- Maxu:多线程
        // 
        // Try different angles， 360 /2 = 180个角度
        for(size_t i = 0; i < try_time; i++) {
            RCLCPP_DEBUG(this->get_logger(), "开始尝试第 %zu 个角度", i);
            // 遍历采样生成的粒子 (在本次循环中，即针对某个角度下的某个粒子)
            for(size_t j = 0; j < corridorGuess.size(); j++) {
                RCLCPP_DEBUG(this->get_logger(), "处理第 %zu 个粒子猜测", j);
                auto startTime = this->now();
                
                // Publish current guess
                auto this_guess_stamped = geometry_msgs::msg::PointStamped();
                this_guess_stamped.header.frame_id = "map";
                this_guess_stamped.point.x = corridorGuess[j][0];
                this_guess_stamped.point.y = corridorGuess[j][1];
                this_guess_stamped.point.z = 0;
                // RCLCPP_INFO(this->get_logger(), "当前猜测位置: x=%.2f, y=%.2f", corridorGuess[j][0], corridorGuess[j][1]);
                pubRobotGuessMarker->publish(this_guess_stamped);

                // Reset parameters for new guess
                // RCLCPP_INFO(this->get_logger(), "重置参数");
                accumulateAngle = 0;
                averDistancePairedPoints = 0;
                numofInsidePoints = 0;
                numofOutsidePoints = 0;
                insideScore = 0;
                outsideScore = 0;
                insideTotalRange = 0;
                outsideTotalScore = 0;
                turkeyScore = 0;

                // Set initial pose for this guess
                // RCLCPP_INFO(this->get_logger(), "设置初始位姿, 角度=%.2f", std::fmod(initialYawAngle + i * rescue_angle_interval, 360.0));
                Eigen::Vector3f tempinitialExtTrans;
                tempinitialExtTrans << corridorGuess[j][0], corridorGuess[j][1], 0;
                setInitialPose(static_cast<int>(std::fmod(initialYawAngle + i * rescue_angle_interval, 360.0)),
                             tempinitialExtTrans);
                             
                // RCLCPP_INFO(this->get_logger(), "robotPose矩阵:\n%.2f %.2f %.2f %.2f\n%.2f %.2f %.2f %.2f\n%.2f %.2f %.2f %.2f\n%.2f %.2f %.2f %.2f",
                //     robotPose(0,0), robotPose(0,1), robotPose(0,2), robotPose(0,3),
                //     robotPose(1,0), robotPose(1,1), robotPose(1,2), robotPose(1,3),
                //     robotPose(2,0), robotPose(2,1), robotPose(2,2), robotPose(2,3),
                //     robotPose(3,0), robotPose(3,1), robotPose(3,2), robotPose(3,3));
                
                // Transform point cloud
                transformed_pc->points.clear();  // 清空点云
                transformed_pc->points.resize(organizedCloudInDS->points.size());  // 调整大小以匹配输入点云
                pcl::transformPointCloud(*organizedCloudInDS, *transformed_pc, robotPose);
                
                // Publish transformed point cloud
                sensor_msgs::msg::PointCloud2 outMsg;
                pcl::toROSMsg(*transformed_pc, outMsg);
                outMsg.header = mapHeader;
                pubTransformedPC->publish(outMsg);


                if(bInitializationWithICP) {
                    if(pause_iter) {
                        RCLCPP_INFO(this->get_logger(), "Press enter to continue, before icp");
                        std::getchar();
                    }
                    
                    auto icpStart = this->now();
                    initializationICP(corridorGuess[j][2]);
                    auto icpEnd = this->now();
                    
                    // Publish updated transformed point cloud
                    pcl::toROSMsg(*transformed_pc, outMsg);
                    outMsg.header = mapHeader;
                    pubTransformedPC->publish(outMsg);
                } else {
                    // If not using ICP initialization
                    // RCLCPP_INFO(this->get_logger(), "corridorGuess[j][2] = %.2f", corridorGuess[j][2]);
                    calClosestMapPoint(corridorGuess[j][2]);
                }

                double result_angle = 0;
                if(bInitializationWithICP) {
                    Eigen::Matrix4d temp = robotPose.cast<double>();
                    Eigen::Matrix3d temp_ = temp.topLeftCorner(3,3);
                    Eigen::Quaterniond quaternion(temp_);
                    Eigen::Vector3d eulerAngle = quaternion.matrix().eulerAngles(1,2,0);
                    result_angle = eulerAngle[1]/M_PI*180;
                } else {
                    result_angle = static_cast<int>(std::fmod(initialYawAngle + i * rescue_angle_interval, 360.0));
                }

                initialized = false;

                if(bGenerateResultFile) {
                    rescueRoomStream << "timestamp:" << rclcpp::Time(mapHeader.stamp).seconds() << ","
                                   << "angle:" << result_angle << ","
                                   << "guess_x:" << corridorGuess[j](0) << ","
                                   << "guess_y:" << corridorGuess[j](1) << ","
                                   << "pose_x:" << robotPose(0,3) << ","
                                   << "pose_y:" << robotPose(1,3) << ","
                                   << "inside_points:" << numofInsidePoints << ","
                                   << "inside_score:" << insideScore << ","
                                   << "outside_points:" << numofOutsidePoints << ","
                                   << "outside_score:" << outsideScore << ","
                                   << "inside_range:" << insideTotalRange << ","
                                   << "outside_total:" << outsideTotalScore << ","
                                   << "turkey_score:" << turkeyScore << std::endl;
                }
                // Update best pose if current score is better
                if(MaxScore < 1.0/(insideScore + outsideScore)) {
                    MaxScore = 1.0/(insideScore + outsideScore);
                    MaxRobotPose = robotPose;
                    
                    // Publish current max pose
                    auto pose_max_stamped = geometry_msgs::msg::PointStamped();
                    pose_max_stamped.header.frame_id = "map";
                    pose_max_stamped.point.x = robotPose(0,3);
                    pose_max_stamped.point.y = robotPose(1,3);
                    pose_max_stamped.point.z = 0;
                    pubCurrentMaxRobotPose->publish(pose_max_stamped);
                    
                    RCLCPP_INFO(this->get_logger(), 
                        "Current best guess: x=%.2f, y=%.2f, yaw=%d",
                        robotPose(0,3), robotPose(1,3), 
                        static_cast<int>(std::fmod(initialYawAngle + i * rescue_angle_interval, 360.0)));
                }
                else {
                    // 添加调试信息：为什么当前粒子没有成为最佳位姿
                    // RCLCPP_INFO(this->get_logger(),
                    //     "粒子位姿 x=%.2f, y=%.2f, yaw=%d 评分=%.6f < 当前最佳评分=%.6f (insideScore=%.2f, outsideScore=%.2f)",
                    //     robotPose(0,3), robotPose(1,3), 
                    //     static_cast<int>(std::fmod(initialYawAngle + i * rescue_angle_interval, 360.0)),
                    //     1.0/(insideScore + outsideScore), MaxScore,
                    //     insideScore, outsideScore);
                }

                if(pause_iter) {
                    RCLCPP_INFO(this->get_logger(), "Press enter to continue, after icp");
                    std::getchar();
                }

                if(turkeyPauseThred > 0 && turkeyScore > turkeyPauseThred) {
                    RCLCPP_INFO(this->get_logger(), "Press enter to continue");
                    std::getchar();
                }

                resetParameters();
                
                auto endTime = this->now();
                // RCLCPP_DEBUG(this->get_logger(), 
                //             "One guess run time: %f ms", 
                //             (endTime - startTime).seconds() * 1000);
            }
        }

        RCLCPP_INFO(this->get_logger(), "----------------------resuceRobot finished!!!--------------------------");
        bRescueRobot = false;
        errorUpThred = 3;
        
        if(bRescueRobot) {
            if(rclcpp::ok()) {
                system("ros2 lifecycle set particlesGeneratorNode shutdown");
            }
        }

        auto finishC = std::chrono::high_resolution_clock::now();
        rescueTimes++;
        rescueRunTime += std::chrono::duration_cast<std::chrono::nanoseconds>
                        (finishC - startC).count();
                        
        RCLCPP_DEBUG(this->get_logger(), 
                    "Average rescue run time = %f ns, rescue times = %d",
                    rescueRunTime/rescueTimes, 
                    rescueTimes);
                    
        rescueRoomStream.flush();
        rescueRoomStream.close();
        
        auto pose = geometry_msgs::msg::Pose();
        pubDONEsignal->publish(pose);

        bGuessReady = false;
    }
    // 函数结束前添加调试语句，输出最终的MaxScore和对应位姿
    RCLCPP_WARN(this->get_logger(), 
                "最终评分: MaxScore=%.6f，对应位姿: x=%.2f, y=%.2f, yaw=%d",
                MaxScore, MaxRobotPose(0,3), MaxRobotPose(1,3),
                static_cast<int>(std::fmod(atan2(MaxRobotPose(1,0), MaxRobotPose(0,0)) * 180.0 / M_PI, 360.0)));

    // 函数结束前标记为未运行并记录完成信息
    auto end_time = this->now();
    RCLCPP_WARN(this->get_logger(), 
               "[调用 #%d] 完成执行rescueRobot，运行时间: %.3f秒", 
               current_call_id,
               (end_time - current_time).seconds());
               
    // 标记为未运行
    is_running = false;
    
    // 标记rescueRobot已完成
    isRescueFinished = true;

    // 重置MaxScore为一个较低的值，确保下次能够更新
    MaxScore = 0.0;
}

bool CloudInitializer::insideOldArea(int mapPCindex) {
    int throughTimes = 0;
    for(size_t i = static_cast<size_t>(mapPCindex); i < map_pc->points.size(); i++) {
        // End of this area
        if((int)map_pc->points[i].intensity % 3 == 2) {
            break;
        }
        
        pcl::PointXYZI temp;
        temp.x = robotPose(0,3);
        temp.y = robotPose(1,3);
        pcl::PointXYZI temp_;
        temp_.x = robotPose(0,3) + 1000;
        temp_.y = robotPose(1,3) + 1000;
        
        bool inray;
        inRayGeneral(map_pc->points[i],
                     map_pc->points[(i+1) % map_pc->points.size()],
                     temp, temp_, inray);
        if(inray) {
            throughTimes++;
        }
    }
    
    return (throughTimes % 2 == 1);
}

bool CloudInitializer::checkICPmovingDist(Eigen::Matrix4f robotPoseGuess) {
    return sqrt(pow(robotPoseGuess(0,3) - robotPose(0,3), 2) +
               pow(robotPoseGuess(1,3) - robotPose(1,3), 2)) > 5;
}

void CloudInitializer::allocateMemory() {
    laserCloudIn = std::make_shared<pcl::PointCloud<PointXYZIRT>>();
    laserUppestRing = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    potentialCeilingPoints = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    potentialCeilingPoints->points.resize(N_SCAN * Horizon_SCAN);
    
    organizedCloudIn = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    transformed_pc = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    UsefulPoints1 = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    UsefulPoints2 = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    
    map_pc = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    mapCorridorEnlarge_pc = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    
    // Initialize point clouds with proper sizes
    ringMapP1 = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    ringMapP1->points.resize(Horizon_SCAN);
    ringMapP2 = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    ringMapP2->points.resize(Horizon_SCAN);
    intersectionOnMap = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    intersectionOnMap->points.resize(Horizon_SCAN);
    furthestRing = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    furthestRing->points.resize(Horizon_SCAN);
    insidePC = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    insidePC->points.resize(Horizon_SCAN);
    outsidePC = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    outsidePC->points.resize(Horizon_SCAN);
    
    organizedCloudIn->points.resize(Horizon_SCAN);
    transformed_pc->points.resize(Horizon_SCAN);
    UsefulPoints1->points.resize(Horizon_SCAN);
    UsefulPoints2->points.resize(Horizon_SCAN);

    RCLCPP_INFO(get_logger(), 
                "Allocate organize size= %lu, intersectionOnMap size = %lu",
                organizedCloudIn->points.size(),
                intersectionOnMap->points.size());
}

void CloudInitializer::resetParameters() {
    laserCloudIn->clear();
    UsefulPoints1->clear();
    UsefulPoints2->clear();
    potentialCeilingPoints->clear();
    potentialCeilingPoints->points.resize(N_SCAN * Horizon_SCAN);
    
    transformed_pc->clear();
    ringMapP1->clear();
    ringMapP1->points.resize(Horizon_SCAN);
    ringMapP2->clear();
    ringMapP2->points.resize(Horizon_SCAN);
    
    intersectionOnMap->clear();
    intersectionOnMap->points.resize(Horizon_SCAN);
    numIcpPoints = 0;
    
    insidePC->clear();
    outsidePC->clear();
    insidePC = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    insidePC->points.resize(Horizon_SCAN);
    outsidePC = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    outsidePC->points.resize(Horizon_SCAN);
}

void CloudInitializer::initializationICP(int insideAGIndex) {
    int totalIteration = icp_init_iteration;
    Eigen::Matrix4f robotPoseGuess = robotPose;

    for (int iteration = 0; iteration < totalIteration; iteration++) {
        averDistancePairedPoints = 0;
        numofInsidePoints = 0;
        numofOutsidePoints = 0;
        insideScore = 0;
        outsideScore = 0;
        insideTotalRange = 0;
        outsideTotalScore = 0;
        turkeyScore = 0;

        Vec_pcx.clear();
        Vec_pcy.clear();
        Vec_pedalx.clear();
        Vec_pedaly.clear();
        RCLCPP_WARN(get_logger(), "Useful POINTS SIZE 1 = %zu", UsefulPoints1->points.size());

        calClosestMapPoint(insideAGIndex);
        RCLCPP_WARN(get_logger(), "Useful POINTS SIZE 2 = %zu", UsefulPoints1->points.size());

        if(use_weight) {
            mapCenter = mapCenter/weightSumTurkey;
            PCCenter = PCCenter/weightSumTurkey;          
        } else {
            mapCenter = mapCenter/numIcpPoints;
            PCCenter = PCCenter/numIcpPoints;
        }

        Eigen::Matrix2d W;
        W.setZero();
        auto startTime = this->now();

        for (int i = 0; i < numIcpPoints; i++) {
            Eigen::Vector2d PCVec;
            Eigen::Vector2d MapVec;
            if(UsefulPoints1->points[usefulIndex[i]].x != 0 || 
               UsefulPoints1->points[usefulIndex[i]].y != 0) {
                if(use_weight && initialized) {
                    PCVec << UsefulPoints1->points[usefulIndex[i]].x,
                            UsefulPoints1->points[usefulIndex[i]].y;
                    MapVec << UsefulPoints2->points[usefulIndex[i]].x,
                            UsefulPoints2->points[usefulIndex[i]].y;
                    W += weightsTurkey[i] * MapVec * PCVec.transpose();
                } else {
                    PCVec << UsefulPoints1->points[usefulIndex[i]].x - PCCenter(0),
                            UsefulPoints1->points[usefulIndex[i]].y - PCCenter(1);
                    MapVec << UsefulPoints2->points[usefulIndex[i]].x - mapCenter(0),
                            UsefulPoints2->points[usefulIndex[i]].y - mapCenter(1);
                    W += MapVec * PCVec.transpose();
                }
            }
        }

        if(use_weight && initialized) {
            W = 1/weightSumTurkey * W - mapCenter * PCCenter.transpose();
        }

        Eigen::JacobiSVD<Eigen::MatrixXd> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix2d U = svd.matrixU();
        Eigen::Matrix2d V = svd.matrixV();
        Eigen::Matrix2d rotationMatrix = U * V.transpose();
        Eigen::Vector2d translation = mapCenter - rotationMatrix * PCCenter;
        
        auto endTime = this->now();
        RCLCPP_INFO(get_logger(), 
                    "The for loop + svd run time is: %f ms",
                    (endTime - startTime).seconds() * 1000);

        RCLCPP_WARN(get_logger(), "---------------iteration = %d", iteration);
        RCLCPP_INFO(get_logger(), 
                    "translation norm = %f threshold = %f",
                    translation.norm(),
                    errorLowThredCurr);

        // Publish point clouds for visualization
        sensor_msgs::msg::PointCloud2 outMsg;
        pcl::toROSMsg(*UsefulPoints1, outMsg);
        outMsg.header = mapHeader;
        pubUsefulPoints1->publish(outMsg);

        pcl::toROSMsg(*UsefulPoints2, outMsg);
        outMsg.header = mapHeader;
        pubUsefulPoints2->publish(outMsg);

        // Update robot pose
        Eigen::Matrix4f robotPoseOldInv = robotPose.inverse();
        robotPose(0,3) += translation(0);
        robotPose(1,3) += translation(1);
        robotPose(3,3) = 1;
        robotPose.topLeftCorner(2,2) = rotationMatrix.cast<float>() * robotPose.topLeftCorner(2,2);
        robotPose(2,2) = 1;

        // Transform point cloud
        pcl::transformPointCloud(*transformed_pc, *transformed_pc, robotPose * robotPoseOldInv);

        // Reset useful points for next iteration
        UsefulPoints1->clear();
        UsefulPoints2->clear();
        UsefulPoints1->points.resize(Horizon_SCAN, pcl::PointXYZI());
        UsefulPoints2->points.resize(Horizon_SCAN, pcl::PointXYZI());

        geometry_msgs::msg::PointStamped pose_after_icp_stamped;
        pose_after_icp_stamped.header.frame_id = "map";
        pose_after_icp_stamped.point.x = robotPose(0,3);
        pose_after_icp_stamped.point.y = robotPose(1,3);
        pose_after_icp_stamped.point.z = 0;
        pubRobotPoseAfterICP->publish(pose_after_icp_stamped);

        if(checkICPmovingDist(robotPoseGuess)) {
            return;
        }
    }

    // Publish final pose after ICP
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = mapHeader;
    pose_stamped.pose.position.x = robotPose(0,3);
    pose_stamped.pose.position.y = robotPose(1,3);
    pose_stamped.pose.position.z = 0;

    Eigen::Matrix3d rotation3d;
    rotation3d.setIdentity();
    rotation3d.topLeftCorner(3,3) = robotPose.topLeftCorner(3,3).cast<double>();
    Eigen::Quaterniond quaternion(rotation3d);
    pose_stamped.pose.orientation.x = quaternion.x();
    pose_stamped.pose.orientation.y = quaternion.y();
    pose_stamped.pose.orientation.z = quaternion.z();
    pose_stamped.pose.orientation.w = quaternion.w();

    globalPath.poses.push_back(pose_stamped);
    pubRobotPath->publish(globalPath);
    saveTUMTraj(pose_stamped);
}

void CloudInitializer::calClosestMapPoint(int inside_index) {
    // 首先检查AG_index是否已初始化
    if (!isAGIndexReceived()) {
        RCLCPP_ERROR(get_logger(), "AG_index not initialized yet!, CloudBase::AGindexReceived = %d", isAGIndexReceived());
        throw std::runtime_error("AG_index not initialized");
    }

    const int MAX_ITERATIONS = 1000;
    int iteration_count = 0;

    // 获取点云大小
    size_t transformed_pc_size = transformed_pc->points.size();

    int last_index = 0;
    numIcpPoints = 0;
    weightSumTurkey = 0;
    weightSumCauchy = 0;
    weightsTurkey.clear();
    outsideAreaIndexRecord.resize(transformed_pc->points.size(), 0);
    outsideAreaLastRingIndexRecord.resize(Horizon_SCAN, 0);

    numofIntersection.resize(transformed_pc_size);
    std::fill(numofIntersection.begin(), numofIntersection.end(), 0);
    inRayDis.resize(transformed_pc_size);
    std::fill(inRayDis.begin(), inRayDis.end(), 0);
    inRayRange.resize(transformed_pc_size);
    std::fill(inRayRange.begin(), inRayRange.end(), 0);
    match_with_outside.resize(transformed_pc_size);
    std::fill(match_with_outside.begin(), match_with_outside.end(), 0);

    insidePC->clear();
    insidePC->points.resize(transformed_pc_size);
    outsidePC->clear();
    outsidePC->points.resize(transformed_pc_size);

    // RCLCPP_INFO(get_logger(), "transformed_pc_size = %zu", transformed_pc_size);

    for(size_t i = 0; i < transformed_pc_size; i++) {

        iteration_count++;
        if (iteration_count > MAX_ITERATIONS) {
            RCLCPP_ERROR(get_logger(), "Maximum iterations reached, breaking loop");
            break;
        }

        // 添加点的有效性检查
        if (!isValidPoint(transformed_pc->points[i])) {
            RCLCPP_WARN(get_logger(), "Skipping invalid point at index %zu", i);
            continue;
        }

        // 检查索引访问的合法性
        if (i >= ringMapP1->points.size() || i >= ringMapP2->points.size()) {
            RCLCPP_ERROR(get_logger(), "Index out of bounds");
            break;
        }


        // RCLCPP_DEBUG(get_logger(), "处理第 %zu 个点", i);
        
        // 检查点云数据的有效性
        // RCLCPP_DEBUG(get_logger(), "当前点坐标: x=%f, y=%f", 
        //              transformed_pc->points[i].x, 
        //              transformed_pc->points[i].y);
        
        bool findIntersection = false;
        double minDist = 0;
        
        
        findIntersection = checkMap(0, i, last_index, minDist, inside_index);
        // RCLCPP_DEBUG(get_logger(), "checkMap结果: findIntersection=%d, minDist=%f", 
        //              findIntersection, minDist);
        
        double pedalx, pedaly;
        if (ringMapP1->points[i].x == ringMapP2->points[i].x && 
            ringMapP1->points[i].y == ringMapP2->points[i].y) {
            continue;
        }
        calPedal(ringMapP1->points[i].x,
                 ringMapP1->points[i].y,
                 ringMapP2->points[i].x,
                 ringMapP2->points[i].y,
                 transformed_pc->points[i].x,
                 transformed_pc->points[i].y,
                 pedalx,
                 pedaly);
        // RCLCPP_DEBUG(get_logger(), "垂足计算结果: x=%f, y=%f", pedalx, pedaly);

        double error = sqrt(pow(pedalx-transformed_pc->points[i].x, 2) +
                          pow(pedaly-transformed_pc->points[i].y, 2));
        // RCLCPP_DEBUG(get_logger(), "计算得到的error=%f", error);

        if(!findIntersection) {
            RCLCPP_ERROR_ONCE(get_logger(), 
                             "NO MATCHING MAP INTERSECTION FOR THIS POINT");
            continue;
        }

        RCLCPP_DEBUG(get_logger(), "检查点 %zu: inRayDis=%f", i, inRayDis[i]);
        if(inRayDis[i] < 1e-6 && 
           (transformed_pc->points[i].x != 0 || transformed_pc->points[i].y != 0) && 
           findIntersection) {
            inRayDis[i] = error;
            inRayRange[i] = sqrt(minDist);
            RCLCPP_DEBUG(get_logger(), "更新inRayDis[%zu]=%f, inRayRange[%zu]=%f", 
                         i, inRayDis[i], i, inRayRange[i]);
        }

        if(!findIntersection) {
            RCLCPP_DEBUG(get_logger(), "设置intersection点 %zu 为零", i);
            intersectionOnMap->points[i].x = 0;
            intersectionOnMap->points[i].y = 0;
            intersectionOnMap->points[i].z = 0;
        }
        
        // RCLCPP_DEBUG(get_logger(), "完成处理第 %zu 个点", i);
    }
    
    // 只有当所有处理都成功完成时才发布消息
    try {
        if (pubIntersection && pubIntersection.get()) {  // 添加发布器有效性检查
            sensor_msgs::msg::PointCloud2 outMsg;
            pcl::toROSMsg(*intersectionOnMap, outMsg);
            outMsg.header = mapHeader;
            pubIntersection->publish(outMsg);
        } else {
            RCLCPP_WARN(get_logger(), "pubIntersection is not valid, skipping publish");
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Error publishing intersection points: %s", e.what());
    }

    // 添加第二个循环，用于计算评分和分类点云
    // RCLCPP_INFO(get_logger(), "开始第二个循环，计算评分...");
    usefulIndex.clear();
    Vec_pcx.clear();
    Vec_pcy.clear();
    Vec_pedalx.clear();
    Vec_pedaly.clear();
    
    // 初始化中心点坐标
    PCCenter.setZero();
    mapCenter.setZero();
    
    // 记录外部区域总分
    outsideTotalScore = 0;

    for(size_t i = 0; i < transformed_pc_size; i++) {
        // 累加外部区域分数
        outsideTotalScore += match_with_outside[i];
        
        // 获取当前点坐标
        double pcx = transformed_pc->points[i].x;
        double pcy = transformed_pc->points[i].y;
        
        // 计算垂足
        double pedalx, pedaly;
        calPedal(ringMapP1->points[i].x,
                 ringMapP1->points[i].y,
                 ringMapP2->points[i].x,
                 ringMapP2->points[i].y,
                 transformed_pc->points[i].x,
                 transformed_pc->points[i].y,
                 pedalx,
                 pedaly);
        
        // 只处理有效点
        if(transformed_pc->points[i].x != 0 || transformed_pc->points[i].y != 0) {
            // 根据交叉点数量判断点是内部点还是外部点
            // 偶数次交叉（加上机器人位置）意味着在地图内部
            if(numofIntersection[i] % 2 == 0 && intersectionOnMap->points[i].intensity == 1) {
                // 处理内部点
                insidePC->points[i] = transformed_pc->points[i];
                numofInsidePoints++;
                
                // 计算Turkey权重
                double weight = calWeightTurkey(inRayDis[i], errorLowThred, false, errorUpThred);
                
                // 计算内部点得分
                if(inRayDis[i] < 50) {
                    if(inRayDis[i] < 0.8) {
                        insideScore += inRayDis[i];
                    } else {
                        insideScore += 2;
                    }
                    turkeyScore += weight;
                }
                
                // 累加内部范围
                insideTotalRange += inRayRange[i];
                
                // 如果误差小于初始化阈值，用于ICP
                if(inRayDis[i] < errorLowThredInit) {
                    numIcpPoints++;
                    usefulIndex.push_back(i);
                    
                    // 保存有用点
                    UsefulPoints1->points[i] = transformed_pc->points[i];
                    UsefulPoints2->points[i].x = pedalx;
                    UsefulPoints2->points[i].y = pedaly;
                    UsefulPoints2->points[i].z = transformed_pc->points[i].z;
                    
                    // 累加权重
                    weightSumTurkey += weight;
                    weightsTurkey.push_back(weight);
                    
                    // 如果使用ICP初始化
                    if(bInitializationWithICP) {
                        if(use_weight) {
                            // 使用权重
                            double tempx = weight * pcx;
                            double tempy = weight * pcy;
                            PCCenter(0) += tempx;
                            PCCenter(1) += tempy;
                            
                            double tempx_ = weight * pedalx;
                            double tempy_ = weight * pedaly;
                            mapCenter(0) += tempx_;
                            mapCenter(1) += tempy_;
                            
                            // 保存向量
                            Vec_pcx.push_back(tempx);
                            Vec_pcy.push_back(tempy);
                            Vec_pedalx.push_back(tempx_);
                            Vec_pedaly.push_back(tempy_);
                        } else {
                            // 不使用权重
                            PCCenter(0) += pcx;
                            PCCenter(1) += pcy;
                            mapCenter(0) += pedalx;
                            mapCenter(1) += pedaly;
                            
                            // 保存向量
                            Vec_pcx.push_back(pcx);
                            Vec_pcy.push_back(pcy);
                            Vec_pedalx.push_back(pedalx);
                            Vec_pedaly.push_back(pedaly);
                        }
                    }
                }
            } else {
                // 处理外部点
                outsidePC->points[i] = transformed_pc->points[i];
                numofOutsidePoints++;
                
                // 计算Turkey权重
                double weight = calWeightTurkey(inRayDis[i], errorLowThred, true, errorUpThred);
                
                // 计算外部点得分
                if(inRayDis[i] < 50) {
                    turkeyScore += weight;
                    if(inRayDis[i] < 0.8) {
                        outsideScore += inRayDis[i];
                    } else {
                        outsideScore += 2;
                    }
                    
                    // 如果误差小于初始化上限阈值，用于ICP
                    if(inRayDis[i] < errorUpThredInit) {
                        numIcpPoints++;
                        usefulIndex.push_back(i);
                        
                        // 保存有用点
                        UsefulPoints1->points[i] = transformed_pc->points[i];
                        UsefulPoints2->points[i].x = pedalx;
                        UsefulPoints2->points[i].y = pedaly;
                        UsefulPoints2->points[i].z = transformed_pc->points[i].z;
                        
                        // 累加权重
                        weightSumTurkey += weight;
                        weightsTurkey.push_back(weight);
                        
                        // 如果使用ICP初始化
                        if(bInitializationWithICP) {
                            if(use_weight) {
                                // 使用权重
                                double tempx = weight * pcx;
                                double tempy = weight * pcy;
                                PCCenter(0) += tempx;
                                PCCenter(1) += tempy;
                                
                                double tempx_ = weight * pedalx;
                                double tempy_ = weight * pedaly;
                                mapCenter(0) += tempx_;
                                mapCenter(1) += tempy_;
                                
                                // 保存向量
                                Vec_pcx.push_back(tempx);
                                Vec_pcy.push_back(tempy);
                                Vec_pedalx.push_back(tempx_);
                                Vec_pedaly.push_back(tempy_);
                            } else {
                                // 不使用权重
                                PCCenter(0) += pcx;
                                PCCenter(1) += pcy;
                                mapCenter(0) += pedalx;
                                mapCenter(1) += pedaly;
                                
                                // 保存向量
                                Vec_pcx.push_back(pcx);
                                Vec_pcy.push_back(pcy);
                                Vec_pedalx.push_back(pedalx);
                                Vec_pedaly.push_back(pedaly);
                            }
                        }
                    }
                }
                
                // 如果误差小于0.5，累加内部范围
                if(inRayDis[i] < 0.5) {
                    insideTotalRange += inRayRange[i];
                }
            }
        }
    }
    
    // 输出评分结果
    // RCLCPP_INFO(get_logger(), "评分结果: 内部点=%d, 内部得分=%.2f, 外部点=%d, 外部得分=%.2f, 内部范围=%.2f",
    //             numofInsidePoints, insideScore, numofOutsidePoints, outsideScore, insideTotalRange);
    
    // 发布内部和外部点云
    try {
        if(pubInsidePC && pubInsidePC.get()) {
            sensor_msgs::msg::PointCloud2 outMsg;
            pcl::toROSMsg(*insidePC, outMsg);
            outMsg.header = mapHeader;
            pubInsidePC->publish(outMsg);
        } else {
            RCLCPP_WARN(get_logger(), "pubInsidePC is not valid, skipping publish");
        }
        
        if(pubOutsidePC && pubOutsidePC.get()) {
            sensor_msgs::msg::PointCloud2 outMsg;
            pcl::toROSMsg(*outsidePC, outMsg);
            outMsg.header = mapHeader;
            pubOutsidePC->publish(outMsg);
        } else {
            RCLCPP_WARN(get_logger(), "pubOutsidePC is not valid, skipping publish");
        }
    } catch(const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Error publishing inside/outside points: %s", e.what());
    }
}

bool CloudInitializer::checkWholeMap(const pcl::PointXYZI& PCPoint, 
                                   const pcl::PointXYZI &PosePoint,
                                   int horizonIndex,
                                   double & minDist,
                                   bool& findIntersection) {
    double min_error = 0;
    double min_PCLength = 0;
    double min_mapLength = 0;
    bool first_ring_find = false;
    bool done_checking_ring = false;
    int start_index = 0;
    bool bMatchWithPass = false;

    if(outsideAreaIndexRecord[horizonIndex] != 0) {
        start_index = outsideAreaIndexRecord[horizonIndex];
    }
    else if(outsideAreaLastRingIndexRecord[horizonIndex % Horizon_SCAN] != 0 &&
            calDistance(transformed_pc->points[horizonIndex-Horizon_SCAN],
                       transformed_pc->points[horizonIndex]) < 0.8) {
        start_index = outsideAreaLastRingIndexRecord[horizonIndex % Horizon_SCAN];
    }
    else {
        start_index = 0;
    }

    for(size_t i = start_index; i < map_pc->size() + start_index; i++) {
        bool bOnRay = false;
        inRay(PosePoint, PCPoint,
              map_pc->points[i % mapSize],
              map_pc->points[(i+1) % mapSize],
              bOnRay);

        if(bAllPassageOpen) {
            if((int)map_pc->points[i % mapSize].intensity > 2 && 
               (int)map_pc->points[(i+1) % mapSize].intensity > 2) {
                continue;
            }
        }

        if(((int)map_pc->points[i % mapSize].intensity) % 3 == 2) {
            continue;
        }

        pcl::PointXYZI intersectionOnMapThisLine;
        bool inbetween = inBetween(PosePoint, PCPoint,
                                 map_pc->points[i % mapSize],
                                 map_pc->points[(i+1) % mapSize],
                                 &intersectionOnMapThisLine);

        if(inbetween) {
            double dist = calDistance(intersectionOnMapThisLine, PCPoint);
            if(min_error == 0 || min_error > dist) {
                min_error = dist;
                double mapLength = calDistance(intersectionOnMapThisLine, PosePoint);
                double PCLength = calDistance(PCPoint, PosePoint);

                min_mapLength = mapLength;
                min_PCLength = PCLength;
                outsideAreaIndexRecord[horizonIndex] = i % mapSize;
                outsideAreaLastRingIndexRecord[horizonIndex % Horizon_SCAN] = i % mapSize;

                intersectionx = intersectionOnMapThisLine.x;
                intersectiony = intersectionOnMapThisLine.y;
                
                if(map_pc->points[i % mapSize].intensity > 2) {
                    bMatchWithPass = true;
                }
            }

            if(start_index) {
                break;
            }
        }
    }

    return bMatchWithPass && (min_error > 1);
}

bool CloudInitializer::checkMap(int ring, 
                              int horizonIndex, 
                              int& last_index,
                              double& minDist,
                              int inside_index) {
    // 添加调试信息
    RCLCPP_DEBUG(get_logger(), "checkMap: ring=%d, horizonIndex=%d, inside_index=%d", 
                 ring, horizonIndex, inside_index);
    RCLCPP_DEBUG(get_logger(), "mapSize=%d, map_pc size=%zu", 
                 mapSize, map_pc ? map_pc->size() : 0);
    
    // 验证输入参数
    if (inside_index < 0 || inside_index >= AG_index.area_index.size()) {
        RCLCPP_ERROR(get_logger(), "Invalid inside_index: %d, area_index size: %zu", 
                    inside_index, AG_index.area_index.size());
        return false;
    } else {
        // RCLCPP_DEBUG(get_logger(), "we have got inside_index: %d, area_index size: %zu", 
        //     inside_index, AG_index.area_index.size());
    }
    
    if (!map_pc || map_pc->empty()) {
        RCLCPP_ERROR(get_logger(), "map_pc is null or empty");
        return false;
    }
    
    RCLCPP_DEBUG(get_logger(), "AG_index range: start=%d, end=%d", 
                 AG_index.area_index[inside_index].start,
                 AG_index.area_index[inside_index].end);
    
    // Get current point coordinates
    pcl::PointXYZI PCPoint;
    PCPoint.x = transformed_pc->points[horizonIndex].x;
    PCPoint.y = transformed_pc->points[horizonIndex].y;
    PCPoint.z = 0;
    
    // Get robot pose as point
    pcl::PointXYZI PosePoint;
    PosePoint.x = robotPose(0,3);
    PosePoint.y = robotPose(1,3);
    PosePoint.z = 0;

    bool findIntersection = false;
    minDist = 0;

    // Parallelize map traversal for better performance
    #pragma omp parallel for num_threads(8)
    for(int j = AG_index.area_index[inside_index].start; 
        j < AG_index.area_index[inside_index].end; 
        j++) {

        // Check for area boundary
        if((int)map_pc->points[j % mapSize].intensity % 3 == 2) {
            continue;
        }

        // Calculate ray intersections
        bool bOnRay = false;
        inRay(PosePoint, 
              PCPoint,
              map_pc->points[j % mapSize],
              map_pc->points[(j+1) % mapSize],
              bOnRay);
              
        if(bOnRay && (PCPoint.x != 0 || PCPoint.y != 0)) {
            numofIntersection[horizonIndex]++;
        }

        // Find intersection point
        pcl::PointXYZI intersectionOnMapThisLine;
        bool inbetween = inBetween(PosePoint,
                                 PCPoint,
                                 map_pc->points[j % mapSize],
                                 map_pc->points[(j+1) % mapSize],
                                 &intersectionOnMapThisLine);

        if(inbetween) {
            // Handle passage (door) intersections
            if((int)map_pc->points[j % mapSize].intensity > 2 && 
               (int)map_pc->points[(j+1) % mapSize].intensity > 2 &&
               (int)map_pc->points[j % mapSize].intensity % 3 != 2) {
                
                checkWholeMap(PCPoint, PosePoint, horizonIndex, minDist, findIntersection);
                continue;
            }

            // Find closest intersection
            double squaredDist = std::pow(intersectionOnMapThisLine.x - PosePoint.x, 2) +
                               std::pow(intersectionOnMapThisLine.y - PosePoint.y, 2);
                               
            if(minDist == 0 || minDist > squaredDist) {
                findIntersection = true;
                minDist = squaredDist;
                
                // Store intersection point
                intersectionOnMap->points[horizonIndex] = intersectionOnMapThisLine;
                
                // Mark passage intersections
                if((int)map_pc->points[j % mapSize].intensity > 2 && 
                   (int)map_pc->points[(j+1) % mapSize].intensity > 2) {
                    intersectionOnMap->points[horizonIndex].intensity = -1;
                }

                // Store map points for this intersection
                ringMapP1->points[horizonIndex] = map_pc->points[j % mapSize];
                ringMapP2->points[horizonIndex] = map_pc->points[(j+1) % mapSize];
                last_index = j % mapSize;

                // Update point intensity based on initialization state
                if(initialized || (!bTestRescue && !bRescueRobot)) {
                    for(int i = 0; i < N_SCAN; i++) {
                        transformed_pc->points[i * Horizon_SCAN + horizonIndex].intensity = 
                            j % mapSize;
                    }
                } else {
                    transformed_pc->points[horizonIndex].intensity = j % mapSize;
                }
            }
        }
    }

    // If no intersection found, clear the intersection point
    if(!findIntersection) {
        intersectionOnMap->points[horizonIndex].x = 0;
        intersectionOnMap->points[horizonIndex].y = 0;
        intersectionOnMap->points[horizonIndex].z = 0;
    }

    return findIntersection;
}
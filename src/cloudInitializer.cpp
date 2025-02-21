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

// void CloudInitializer::showImgIni(double x, double y, int yaw) {
//     // 懒加载方式初始化 image_transport
//     if (!image_transport_) {
//         image_transport_ = std::make_shared<image_transport::ImageTransport>(Node::shared_from_this());
//         image_pub_ = image_transport_->advertise("Things2say", 1);
//     }
    
//     // Create image
//     cv::Mat image(200, 600, CV_8UC3, cv::Scalar(0,0,0));
    
//     // Format text
//     std::string text1 = "Initializing,";
//     std::string text2 = "current best guess: ";
//     std::string text3 = "x=" + std::to_string(x).substr(0, std::to_string(y).size()-5) + 
//                        ", y=" + std::to_string(y).substr(0, std::to_string(y).size()-5) + 
//                        ", yaw=" + std::to_string(yaw).substr(0, std::to_string(y).size()-5);
                       
//     // Add text to image
//     cv::putText(image, text1, cv::Point(20,60), cv::FONT_HERSHEY_DUPLEX, 1, 
//                 cv::Scalar(255,255,255), 1, 8);
//     cv::putText(image, text2, cv::Point(20,100), cv::FONT_HERSHEY_DUPLEX, 1, 
//                 cv::Scalar(255,255,255), 1, 8);
//     cv::putText(image, text3, cv::Point(20,140), cv::FONT_HERSHEY_DUPLEX, 1, 
//                 cv::Scalar(255,255,255), 1, 8);

//     // Convert to ROS message and publish
//     sensor_msgs::msg::Image::SharedPtr msg = 
//         cv_bridge::CvImage(mapHeader, "bgr8", image).toImageMsg();
        
//     std::this_thread::sleep_for(std::chrono::milliseconds(50));
//     image_pub_.publish(*msg);
    
//     RCLCPP_ERROR(this->get_logger(), "THINGS TO SAY!!!!!!!!!!!!!!!!!!!!!! ini");
// }

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

    // 遍历所有点,构建3D向量 -- x,y表示猜测的位置坐标， z表示猜测属于哪个Area
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
    RCLCPP_INFO(this->get_logger(), "-------------------rescueRobot finished---------");
    robotPose = MaxRobotPose;
    auto endTime = this->now();

    // 线索：在这行输出之前，程序已经崩溃了
    RCLCPP_INFO(this->get_logger(), 
                "Number of guesses: %d, Rescue robot run time: %f ms",
                GuessSize,
                (endTime - startTime).seconds() * 1000);
}

void CloudInitializer::rescueRobot() {
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
    
    // 如果 initialization_imu 为 false，才会进行救援
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
        
        // 线索：这一行被打印了
        RCLCPP_INFO(this->get_logger(), "Downsample size = %lu", 
                    organizedCloudInDS->points.size());

        // 线索：在以下程序中崩溃了，没有出这个函数
        // Try different angles
        for(size_t i = 0; i < try_time; i++) {
            RCLCPP_INFO(this->get_logger(), "开始尝试第 %zu 个角度", i);
            for(size_t j = 0; j < corridorGuess.size(); j++) {
                RCLCPP_INFO(this->get_logger(), "处理第 %zu 个粒子猜测", j);
                auto startTime = this->now();
                
                // Publish current guess
                auto this_guess_stamped = geometry_msgs::msg::PointStamped();
                this_guess_stamped.header.frame_id = "map";
                this_guess_stamped.point.x = corridorGuess[j][0];
                this_guess_stamped.point.y = corridorGuess[j][1];
                this_guess_stamped.point.z = 0;
                RCLCPP_INFO(this->get_logger(), "当前猜测位置: x=%.2f, y=%.2f", corridorGuess[j][0], corridorGuess[j][1]);
                pubRobotGuessMarker->publish(this_guess_stamped);

                // Reset parameters for new guess
                RCLCPP_INFO(this->get_logger(), "重置参数");
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
                RCLCPP_INFO(this->get_logger(), "设置初始位姿, 角度=%.2f", std::fmod(initialYawAngle + i * rescue_angle_interval, 360.0));
                Eigen::Vector3f tempinitialExtTrans;
                tempinitialExtTrans << corridorGuess[j][0], corridorGuess[j][1], 0;
                setInitialPose(static_cast<int>(std::fmod(initialYawAngle + i * rescue_angle_interval, 360.0)),
                             tempinitialExtTrans);
                             
                RCLCPP_INFO(this->get_logger(), "robotPose矩阵:\n%.2f %.2f %.2f %.2f\n%.2f %.2f %.2f %.2f\n%.2f %.2f %.2f %.2f\n%.2f %.2f %.2f %.2f",
                    robotPose(0,0), robotPose(0,1), robotPose(0,2), robotPose(0,3),
                    robotPose(1,0), robotPose(1,1), robotPose(1,2), robotPose(1,3),
                    robotPose(2,0), robotPose(2,1), robotPose(2,2), robotPose(2,3),
                    robotPose(3,0), robotPose(3,1), robotPose(3,2), robotPose(3,3));
                
                // Transform point cloud
                RCLCPP_INFO(this->get_logger(), "开始点云变换, 输入点云大小=%zu", organizedCloudInDS->points.size());
                transformed_pc->points.clear();  // 清空点云
                transformed_pc->points.resize(organizedCloudInDS->points.size());  // 调整大小以匹配输入点云
                pcl::transformPointCloud(*organizedCloudInDS, *transformed_pc, robotPose);
                RCLCPP_INFO(this->get_logger(), "点云变换完成, 输出点云大小=%zu", transformed_pc->points.size());
                
                // Publish transformed point cloud
                sensor_msgs::msg::PointCloud2 outMsg;
                pcl::toROSMsg(*transformed_pc, outMsg);
                outMsg.header = mapHeader;
                pubTransformedPC->publish(outMsg);

                RCLCPP_INFO(this->get_logger(), "当前corridorGuess[%zu]的值: x=%.2f, y=%.2f, z=%.2f",
                           j, corridorGuess[j][0], corridorGuess[j][1], corridorGuess[j][2]);



                RCLCPP_INFO(this->get_logger(), "organizedCloudInDS->is_dense: %d", organizedCloudInDS->is_dense);
                RCLCPP_INFO(this->get_logger(), "transformed_pc->is_dense: %d", transformed_pc->is_dense);
                RCLCPP_INFO(this->get_logger(), "organizedCloudInDS->size: %zu", organizedCloudInDS->size());
                RCLCPP_INFO(this->get_logger(), "transformed_pc->size: %zu", transformed_pc->size());


                // Check if corridorGuess[j] is valid
                if (std::isnan(corridorGuess[j][0]) || std::isnan(corridorGuess[j][1]) || std::isnan(corridorGuess[j][2])) {
                    RCLCPP_ERROR(this->get_logger(), "corridorGuess[%zu] contains NaN values!", j);
                    continue; // Skip this guess
                }
                if (std::isinf(corridorGuess[j][0]) || std::isinf(corridorGuess[j][1]) || std::isinf(corridorGuess[j][2])) {
                    RCLCPP_ERROR(this->get_logger(), "corridorGuess[%zu] contains Inf values!", j);
                    continue; // Skip this guess
                }
                //线索： 打印完以上语句后，程序崩溃

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
                    RCLCPP_INFO(this->get_logger(), "corridorGuess[j][2] = %.2f", corridorGuess[j][2]);
                    //  ----------------------------------出错点-----------------------------------
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
                    rescueRoomStream << rclcpp::Time(mapHeader.stamp).seconds() << ","
                                   << result_angle << ","
                                   << corridorGuess[j](0) << ","
                                   << corridorGuess[j](1) << ","
                                   << robotPose(0,3) << ","
                                   << robotPose(1,3) << ","
                                   << numofInsidePoints << ","
                                   << insideScore << ","
                                   << numofOutsidePoints << ","
                                   << outsideScore << ","
                                   << insideTotalRange << ","
                                   << outsideTotalScore << ","
                                   << turkeyScore << std::endl;
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
                    
                    // 定位：就是这个函数的问题
                    // showImgIni(robotPose(0,3), 
                    //           robotPose(1,3),
                    //           static_cast<int>(std::fmod(initialYawAngle + i * rescue_angle_interval, 360.0)));
                    RCLCPP_INFO(this->get_logger(), 
                        "Current best guess: x=%.2f, y=%.2f, yaw=%d",
                        robotPose(0,3), robotPose(1,3), 
                        static_cast<int>(std::fmod(initialYawAngle + i * rescue_angle_interval, 360.0)));
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
                RCLCPP_DEBUG(this->get_logger(), 
                            "One guess run time: %f ms", 
                            (endTime - startTime).seconds() * 1000);
            }
        }

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
                        
        RCLCPP_INFO(this->get_logger(), 
                    "Average rescue run time = %f ns, rescue times = %d",
                    rescueRunTime/rescueTimes, 
                    rescueTimes);
                    
        rescueRoomStream.flush();
        rescueRoomStream.close();
        
        auto pose = geometry_msgs::msg::Pose();
        pubDONEsignal->publish(pose);

        bGuessReady = false;
    }
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

// 线索，问题出现在这个函数中
void CloudInitializer::calClosestMapPoint(int inside_index) {
    // 首先检查AG_index是否已初始化
    if (!isAGIndexReceived()) {
        RCLCPP_ERROR(get_logger(), "AG_index not initialized yet!, CloudBase::AGindexReceived = %d", isAGIndexReceived());
        throw std::runtime_error("AG_index not initialized");
    }

    // 检查发布器是否有效
    if (!pubIntersection) {
        RCLCPP_ERROR(get_logger(), "Publisher not initialized!");
        return;
    }

    int last_index = 0;
    size_t transformed_pc_size = transformed_pc->points.size();
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

    RCLCPP_WARN(get_logger(), "--------------------------程序运行到了for循环前还没有崩溃--------------------------");
    RCLCPP_INFO(get_logger(), "transformed_pc_size = %zu", transformed_pc_size);
    for(size_t i = 0; i < transformed_pc_size; i++) {
        RCLCPP_DEBUG(get_logger(), "处理第 %zu 个点", i);
        
        // 检查点云数据的有效性
        RCLCPP_DEBUG(get_logger(), "当前点坐标: x=%f, y=%f", 
                     transformed_pc->points[i].x, 
                     transformed_pc->points[i].y);
        
        bool findIntersection = false;
        double minDist = 0;
        
        // 检查ringMapP1和ringMapP2的访问
        RCLCPP_DEBUG(get_logger(), "ringMapP1点 %zu: x=%f, y=%f", 
                     i, ringMapP1->points[i].x, ringMapP1->points[i].y);
        RCLCPP_DEBUG(get_logger(), "ringMapP2点 %zu: x=%f, y=%f", 
                     i, ringMapP2->points[i].x, ringMapP2->points[i].y);
        
        findIntersection = checkMap(0, i, last_index, minDist, inside_index);
        RCLCPP_DEBUG(get_logger(), "checkMap结果: findIntersection=%d, minDist=%f", 
                     findIntersection, minDist);
        
        double pedalx, pedaly;
        RCLCPP_DEBUG(get_logger(), "开始计算垂足");
        if (ringMapP1->points[i].x == ringMapP2->points[i].x && 
            ringMapP1->points[i].y == ringMapP2->points[i].y) {
            RCLCPP_DEBUG(get_logger(), "跳过重合点 %zu", i);
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
        RCLCPP_DEBUG(get_logger(), "垂足计算结果: x=%f, y=%f", pedalx, pedaly);

        double error = sqrt(pow(pedalx-transformed_pc->points[i].x, 2) +
                          pow(pedaly-transformed_pc->points[i].y, 2));
        RCLCPP_DEBUG(get_logger(), "计算得到的error=%f", error);

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
        
        RCLCPP_DEBUG(get_logger(), "完成处理第 %zu 个点", i);
    }
    RCLCPP_WARN(get_logger(), "--------------------------程序运行到了for循环后还没有崩溃--------------------------");
    
    // // 只有当所有处理都成功完成时才发布消息
    // try {
    //     if (pubIntersection && pubIntersection.get()) {  // 添加发布器有效性检查
    //         sensor_msgs::msg::PointCloud2 outMsg;
    //         pcl::toROSMsg(*intersectionOnMap, outMsg);
    //         outMsg.header = mapHeader;
    //         pubIntersection->publish(outMsg);
    //     } else {
    //         RCLCPP_WARN(get_logger(), "pubIntersection is not valid, skipping publish");
    //     }
    // } catch (const std::exception& e) {
    //     RCLCPP_ERROR(get_logger(), "Error publishing intersection points: %s", e.what());
    // }
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

void CloudInitializer::scoreParticlesDist() {
    std::ostringstream ts;
    ts.precision(2);
    ts << std::fixed << rclcpp::Time(cloudHeader.stamp).seconds();
    std::string filename = "/home/jay/AGLoc_ws/frameResult/" +
                          ts.str() + "rescueRoom.txt";
                          
    rescueRoomStream.open(filename, std::ofstream::out | std::ofstream::app);
    rescueRoomStream.setf(std::ios::fixed);
    rescueRoomStream.precision(2);

    pcl::PointCloud<pcl::PointXYZI>::Ptr organizedCloudInDS(
        new pcl::PointCloud<pcl::PointXYZI>());

    pcl::VoxelGrid<pcl::PointXYZI> downSizeFurthestRing;
    downSizeFurthestRing.setLeafSize(scoreDownsampleRate, 
                                    scoreDownsampleRate,
                                    scoreDownsampleRate);
    downSizeFurthestRing.setInputCloud(furthestRing);
    downSizeFurthestRing.filter(*organizedCloudInDS);

    RCLCPP_INFO(get_logger(), "Downsample size = %lu", 
                organizedCloudInDS->points.size());

    for(const auto& guess : corridorGuess) {
        insideScore = 0;
        outsideScore = 0;
        insideTotalRange = 0;
        numofInsidePoints = 0;
        numofOutsidePoints = 0;

        setInitialPose(initialYawAngle, guess);
        pcl::transformPointCloud(*organizedCloudInDS, *transformed_pc, robotPose);

        sensor_msgs::msg::PointCloud2 outMsg;
        pcl::toROSMsg(*transformed_pc, outMsg);
        outMsg.header = mapHeader;
        pubTransformedPC->publish(outMsg);

        calClosestMapPoint(guess[2]);

        initialized = false;

        if(bGenerateResultFile) {
            rescueRoomStream << rclcpp::Time(mapHeader.stamp).seconds() << ","
                           << initialYawAngle << ","
                           << guess(0) << "," << guess(1) << ","
                           << robotPose(0,3) << "," << robotPose(1,3) << ","
                           << numofInsidePoints << "," << insideScore << ","
                           << numofOutsidePoints << "," << outsideScore << ","
                           << insideTotalRange << std::endl;
        }
        resetParameters();
    }

    rescueRoomStream.close();
    geometry_msgs::msg::Pose pose;
    pubDONEsignal->publish(pose);
}

void CloudInitializer::scoreParticles() {
    RCLCPP_INFO(get_logger(), "Starting scoreParticles");
    
    double currentTime = rclcpp::Time(cloudHeader.stamp).seconds();

    if(bTestRescue || bRescueRobot) {
        int try_time = 360 / rescue_angle_interval;
        
        auto organizedCloudInRecord = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        pcl::VoxelGrid<pcl::PointXYZI> downSizeFurthestRing;
        downSizeFurthestRing.setLeafSize(scoreDownsampleRate, 
                                        scoreDownsampleRate,
                                        scoreDownsampleRate);
        downSizeFurthestRing.setInputCloud(furthestRing);
        downSizeFurthestRing.filter(*organizedCloudInRecord);

        for(size_t i = 0; i < static_cast<size_t>(try_time); i++) {
            for(size_t j = 0; j < corridorGuess.size(); j++) {
                insideScore = 0;
                outsideScore = 0;
                insideTotalRange = 0;
                numofInsidePoints = 0;
                numofOutsidePoints = 0;

                Eigen::Vector3f tempinitialExtTrans = corridorGuess[j];
                double currentAngle = initialYawAngle + i * rescue_angle_interval;
                int discreteAngle = static_cast<int>(currentAngle) % 360;
                setInitialPose(discreteAngle, tempinitialExtTrans);
                                        
                pcl::transformPointCloud(*organizedCloudInRecord, 
                                       *transformed_pc,
                                       robotPose);

                sensor_msgs::msg::PointCloud2 outMsg;
                pcl::toROSMsg(*transformed_pc, outMsg);
                outMsg.header = mapHeader;
                pubTransformedPC->publish(outMsg);

                calClosestMapPoint(j);
                initialized = false;

                double inoutsideRatio = numofInsidePoints != 0 ? 
                    static_cast<double>(numofOutsidePoints) / numofInsidePoints : 10000;

                if(bGenerateResultFile) {
                    rescueRoomStream << rclcpp::Time(mapHeader.stamp).seconds() << ","
                                   << static_cast<int>(std::fmod(initialYawAngle + i * rescue_angle_interval, 360.0)) << ","
                                   << corridorGuess[j](0) << "," 
                                   << corridorGuess[j](1) << ","
                                   << robotPose(0,3) << "," 
                                   << robotPose(1,3) << ","
                                   << inoutsideRatio << ","
                                   << numofInsidePoints << "," 
                                   << insideScore << ","
                                   << numofOutsidePoints << "," 
                                   << outsideScore << ","
                                   << insideTotalRange << std::endl;
                }

                resetParameters();
            }
        }

        if(!bTestRescue) {
            initialized = true;
        }
    }

    rescueRoomStream.close();
    geometry_msgs::msg::Pose pose;
    pubDONEsignal->publish(pose);
}


void CloudInitializer::checkingGuess() {
    insideScore = 0;
    outsideScore = 0;
    insideTotalRange = 0;

    auto organizedCloudInRecord = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    
    pcl::VoxelGrid<pcl::PointXYZI> downSizeFurthestRing;
    downSizeFurthestRing.setLeafSize(scoreDownsampleRate, 
                                    scoreDownsampleRate,
                                    scoreDownsampleRate);
    downSizeFurthestRing.setInputCloud(furthestRing);
    downSizeFurthestRing.filter(*organizedCloudInRecord);

    RCLCPP_INFO(get_logger(), "Downsample size = %lu", 
                organizedCloudInRecord->points.size());

    Eigen::Vector3f guessTemp;
    guessTemp << checkingGuessX, checkingGuessY, 0;
    
    setInitialPose(checkingAngle, guessTemp);
    pcl::transformPointCloud(*organizedCloudInRecord, *transformed_pc, robotPose);
    
    sensor_msgs::msg::PointCloud2 outMsg;
    pcl::toROSMsg(*transformed_pc, outMsg);
    outMsg.header = mapHeader;
    pubTransformedPC->publish(outMsg);

    initialized = false;
    resetParameters();
    
    RCLCPP_INFO(get_logger(), "CheckingGuess completed");
}

double CloudInitializer::getScoreFromTwoPC(
    const Eigen::Matrix4f & robotPose,
    pcl::PointCloud<pcl::PointXYZI>::Ptr PC1,
    pcl::PointCloud<pcl::PointXYZI>::Ptr PC2) {
    
    // Currently returns 0 as per original implementation
    return 0;
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
    // TODO 这里出现了错误，AG_index.area_index.size() == 0 
    if (inside_index < 0 || inside_index >= AG_index.area_index.size()) {
        RCLCPP_ERROR(get_logger(), "Invalid inside_index: %d, area_index size: %zu", 
                    inside_index, AG_index.area_index.size());
        return false;
    } else {
        RCLCPP_INFO(get_logger(), "we have got inside_index: %d, area_index size: %zu", 
            inside_index, AG_index.area_index.size());
    }
    
    if (!map_pc || map_pc->empty()) {
        RCLCPP_ERROR(get_logger(), "map_pc is null or empty");
        return false;
    }
    
    // RCLCPP_DEBUG(get_logger(), "AG_index range: start=%d, end=%d", 
    //              AG_index.area_index[inside_index].start,
    //              AG_index.area_index[inside_index].end);
    
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
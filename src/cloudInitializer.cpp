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
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <filesystem>

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
    
    // 声明参数
    this->declare_parameter("particle_generator_radius", 6.0);
    
    // 初始化可视化参数
    visualization_enabled_ = true;
    
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
    
    RCLCPP_INFO(this->get_logger(), "CloudInitializer GETTING INITIAL GUESS");

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
    RCLCPP_INFO(this->get_logger(), "-------------------rescueRobot started-------------------");
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
    
    RCLCPP_DEBUG(this->get_logger(), 
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
    
    // 如果启用了多线程，则调用多线程版本
    if(use_multithread) {
        RCLCPP_INFO(this->get_logger(), "使用多线程模式执行救援机器人");
        rescueRobotMultiThread();
        is_running = false; // 恢复运行标志
        return;
    }

    RCLCPP_INFO(this->get_logger(), "----------- Guess is ready, start rescue -----------------");

    // 创建性能日志文件
    std::string log_dir = "/home/jay/AGLoc_ws/performance_logs_global_loc";
    // 创建目录（如果不存在）
    std::filesystem::create_directories(log_dir);
    
    // 生成带时间戳的文件名
    auto now = std::chrono::system_clock::now();
    now_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << log_dir << "/performance_" << std::put_time(std::localtime(&now_time_t), "%Y%m%d_%H%M%S") << ".log";
    std::string log_file = ss.str();
    
    // 打开日志文件
    perf_log.open(log_file);
    if (!perf_log.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "无法创建性能日志文件: %s", log_file.c_str());
    } else {
        RCLCPP_INFO(this->get_logger(), "性能日志将输出到: %s", log_file.c_str());
        perf_log << "========== 粒子定位算法性能日志 ==========" << std::endl;
        perf_log << "开始时间: " << std::put_time(std::localtime(&now_time_t), "%Y-%m-%d %H:%M:%S") << std::endl;
        perf_log << "----------------------------------------" << std::endl;
    }

    // Prepare output file
    std::ostringstream ts;
    ts.precision(2);
    ts << std::fixed << rclcpp::Time(mapHeader.stamp).seconds();
    // TODO 全局定位结果保存路径
    // std::string filename = "/home/jay/AGLoc_ws/frameResult" + 
    //                       ts.str() + "rescueRoom.txt";
                          
    // rescueRoomStream.open(filename, std::ofstream::out | std::ofstream::app);
    // rescueRoomStream.setf(std::ios::fixed);
    // rescueRoomStream.precision(2);
    
    // if (!rescueRoomStream.good()) {
    //     RCLCPP_ERROR(this->get_logger(), "ERROR OPENING FILE");
    //     return;
    // }

    auto startC = std::chrono::high_resolution_clock::now();
    
    // 声明所有需要的变量
    std::vector<Eigen::Vector3f> sparse_particles;
    std::vector<Eigen::Vector3f> local_particles;
    Eigen::Vector3f best_sparse_position;
    best_sparse_position << 0.0, 0.0, 0.0;
    int best_sparse_area_id = 0;
    double best_sparse_score = 0.0;
    double best_sparse_angle = 0.0;
    double coarse_search_time = 0.0;
    double fine_search_time = 0.0;
    double best_fine_score = 0.0;
    Eigen::Matrix4f best_fine_pose = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f initialPose = Eigen::Matrix4f::Identity();
    
    // 如果 initialization_imu 为 false，才会进行角度遍历，否则将会有先验角度信息？-maybe
    if (!initialization_imu) {
        // 第一步：粒子稀疏采样 - 隔行隔列取出1/2的粒子
        RCLCPP_INFO(this->get_logger(), "原始粒子数量: %zu", corridorGuess.size());
        
        // 执行稀疏采样
        for (size_t j = 0; j < corridorGuess.size(); j += 2) {
            sparse_particles.push_back(corridorGuess[j]);
        }
        
        RCLCPP_INFO(this->get_logger(), "稀疏采样后粒子数量: %zu (减少了%d%%)", 
                    sparse_particles.size(), 
                    static_cast<int>((1.0 - static_cast<double>(sparse_particles.size()) / corridorGuess.size()) * 100));
        
        // 第二步：粗粒度角度搜索 - 使用较大角度步长
        const size_t coarse_try_time = static_cast<size_t>(360/12); // 使用12度的角度步长
        auto organizedCloudInDS = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        
        // Downsample point cloud - 体素滤波器降采样
        // 若需减少降采样 - 保留更多点，则需要减小scoreDownsampleRate的值
        pcl::VoxelGrid<pcl::PointXYZI> downSizeFurthestRing;
        downSizeFurthestRing.setLeafSize(scoreDownsampleRate, 
                                        scoreDownsampleRate, 
                                        scoreDownsampleRate);
        downSizeFurthestRing.setInputCloud(furthestRing);
        downSizeFurthestRing.filter(*organizedCloudInDS);
        
        RCLCPP_INFO(this->get_logger(), "Downsample size = %lu", 
                    organizedCloudInDS->points.size());

        // 记录粗粒度搜索的开始时间
        auto coarse_search_start_time = this->now();
        
        // 初始化稀疏粒子的最佳评分和对应位姿
        best_sparse_score = 0.0;
        Eigen::Matrix4f best_sparse_pose = Eigen::Matrix4f::Identity();
        best_sparse_position << 0.0, 0.0, 0.0;
        best_sparse_angle = 0;
        best_sparse_area_id = -1;
        
        // 对稀疏采样的粒子执行粗粒度角度搜索
        RCLCPP_INFO(this->get_logger(), "开始粗粒度角度搜索，角度步长: 12度");
        
        // 按Area对稀疏粒子进行分组
        std::map<int, std::vector<size_t>> sparse_area_particles;
        for(size_t j = 0; j < sparse_particles.size(); j++) {
            int area_id = static_cast<int>(sparse_particles[j][2]);
            sparse_area_particles[area_id].push_back(j);
            
            RCLCPP_DEBUG(this->get_logger(), "稀疏粒子 %zu: x=%.2f, y=%.2f, z=%.2f, area_id=%d", 
                         j, sparse_particles[j][0], sparse_particles[j][1], sparse_particles[j][2], area_id);
        }
        
        RCLCPP_INFO(this->get_logger(), "稀疏粒子分布在 %zu 个不同的Area", sparse_area_particles.size());
        
        // 遍历每个Area的稀疏粒子
        for(auto& [area_id, particles] : sparse_area_particles) {
            // 记录区域处理开始时间
            auto area_start_time = this->now();
            
            RCLCPP_INFO(this->get_logger(), "开始处理Area %d的%zu个粒子", area_id, particles.size());
            
            // 遍历该Area的所有稀疏粒子
            for(size_t particle_idx : particles) {
                // 用于记录该粒子在不同角度下的评分
                std::vector<double> angle_scores(coarse_try_time, 0.0);
                int best_angle_idx = -1;
                double best_particle_score = 0.0;
            
                // 内循环：对粒子进行粗粒度角度搜索
                for(size_t i = 0; i < coarse_try_time; i++) {
                    RCLCPP_DEBUG(this->get_logger(), "粗搜索: 处理稀疏粒子 %zu 的第 %zu 个角度", particle_idx, i);
                    auto startTime = this->now();
                    
                    // Publish current guess
                    auto this_guess_stamped = geometry_msgs::msg::PointStamped();
                    this_guess_stamped.header.frame_id = "map";
                    this_guess_stamped.point.x = sparse_particles[particle_idx][0];
                    this_guess_stamped.point.y = sparse_particles[particle_idx][1];
                    this_guess_stamped.point.z = 0;
                    pubRobotGuessMarker->publish(this_guess_stamped);

                    // Reset parameters for new guess
                    accumulateAngle = 0;
                    averDistancePairedPoints = 0;
                    numofInsidePoints = 0;
                    numofOutsidePoints = 0;
                    insideScore = 0;
                    outsideScore = 0;
                    insideTotalRange = 0;
                    outsideTotalScore = 0;
                    turkeyScore = 0;

                    // 使用粗粒度角度步长(12度)
                    Eigen::Vector3f tempinitialExtTrans;
                    tempinitialExtTrans << sparse_particles[particle_idx][0], sparse_particles[particle_idx][1], 0;
                    // 计算当前角度 (12度步长)
                    int current_angle = static_cast<int>(std::fmod(initialYawAngle + i * 12.0, 360.0));
                    setInitialPose(current_angle, tempinitialExtTrans);
                             
                // RCLCPP_DEBUG(this->get_logger(), "robotPose矩阵:\n%.2f %.2f %.2f %.2f\n%.2f %.2f %.2f %.2f\n%.2f %.2f %.2f %.2f\n%.2f %.2f %.2f %.2f",
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
                    initializationICP(sparse_particles[particle_idx][2]);
                    auto icpEnd = this->now();
                    
                    // Publish updated transformed point cloud
                    pcl::toROSMsg(*transformed_pc, outMsg);
                    outMsg.header = mapHeader;
                    pubTransformedPC->publish(outMsg);
                } else {
                    // If not using ICP initialization
                    calClosestMapPoint(sparse_particles[particle_idx][2]);
                }

                double result_angle = 0;
                if(bInitializationWithICP) {
                    Eigen::Matrix4d temp = robotPose.cast<double>();
                    Eigen::Matrix3d temp_ = temp.topLeftCorner(3,3);
                    Eigen::Quaterniond quaternion(temp_);
                    Eigen::Vector3d eulerAngle = quaternion.matrix().eulerAngles(1,2,0);
                    result_angle = eulerAngle[1]/M_PI*180;
                } else {
                    result_angle = current_angle;
                }

                initialized = false;

                if(bGenerateResultFile) {
                    rescueRoomStream << "timestamp:" << rclcpp::Time(mapHeader.stamp).seconds() << ","
                                   << "angle:" << result_angle << ","
                                   << "guess_x:" << sparse_particles[particle_idx](0) << ","
                                   << "guess_y:" << sparse_particles[particle_idx](1) << ","
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
                // 计算当前角度的评分
                double current_score = 1.0/(insideScore + outsideScore);
                angle_scores[i] = current_score;
                
                // 更新该粒子的最佳角度
                if(best_angle_idx == -1 || current_score > best_particle_score) {
                    best_angle_idx = i;
                    best_particle_score = current_score;
                }
                
                // 更新粗搜索阶段的最佳位姿
                if(best_sparse_score < current_score) {
                    best_sparse_score = current_score;
                    best_sparse_pose = robotPose;
                    best_sparse_position = sparse_particles[particle_idx];
                    best_sparse_angle = current_angle;
                    best_sparse_area_id = area_id;
                    
                    // 发布当前最佳位姿
                    auto pose_max_stamped = geometry_msgs::msg::PointStamped();
                    pose_max_stamped.header.frame_id = "map";
                    pose_max_stamped.point.x = robotPose(0,3);
                    pose_max_stamped.point.y = robotPose(1,3);
                    pose_max_stamped.point.z = 0;
                    pubCurrentMaxRobotPose->publish(pose_max_stamped);
                    
                    RCLCPP_INFO(this->get_logger(), 
                        "粗搜索阶段最佳猜测: x=%.2f, y=%.2f, yaw=%d, 评分=%.6f",
                        robotPose(0,3), robotPose(1,3), current_angle, current_score);
                }
                else {
                    // 添加调试信息
                    RCLCPP_DEBUG(this->get_logger(),
                        "粒子位姿 x=%.2f, y=%.2f, yaw=%d 评分=%.6f < 当前最佳评分=%.6f (insideScore=%.2f, outsideScore=%.2f)",
                        robotPose(0,3), robotPose(1,3), current_angle,
                        current_score, best_sparse_score, insideScore, outsideScore);
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
                            "粗搜索: 一次评估耗时: %f ms", 
                            (endTime - startTime).seconds() * 1000);
            }
            
                RCLCPP_DEBUG(this->get_logger(), 
                    "粒子 %zu (x=%.2f, y=%.2f, area=%d) 在粗角度搜索中的最佳角度: %d度, 评分: %.6f",
                    particle_idx, sparse_particles[particle_idx][0], sparse_particles[particle_idx][1], area_id,
                    static_cast<int>(std::fmod(initialYawAngle + best_angle_idx * 12.0, 360.0)),
                    best_particle_score);
            }
            
            // 计算当前Area处理时间
            auto area_end_time = this->now();
            double area_total_time = (area_end_time - area_start_time).seconds() * 1000.0;
            RCLCPP_INFO(this->get_logger(), "Area %d 粗搜索总处理时间: %.2f ms", area_id, area_total_time);
            
            // 将Area总处理时间输出到日志文件
            if (perf_log.is_open()) {
                perf_log << "[Area 粗搜索] Area " << area_id << std::endl;
                perf_log << "  总处理时间: " << std::fixed << std::setprecision(2) << area_total_time << " ms" << std::endl;
                perf_log << "  总粒子数量: " << particles.size() << std::endl;
                perf_log << "  平均每粒子时间: " << std::fixed << std::setprecision(2) << area_total_time / particles.size() << " ms" << std::endl;
                perf_log << "  时间点: " << std::put_time(std::localtime(&now_time_t), "%H:%M:%S") << std::endl;
                perf_log << "========================================" << std::endl;
            }
            
        }

        // 记录粗粒度搜索的结束时间
        auto coarse_search_end_time = this->now();
        coarse_search_time = (coarse_search_end_time - coarse_search_start_time).seconds() * 1000.0;
        RCLCPP_INFO(this->get_logger(), "粗粒度搜索总耗时: %.2f ms", coarse_search_time);
        
        // 将粗搜索时间输出到日志文件
        if (perf_log.is_open()) {
            perf_log << "[粗粒度搜索总结]" << std::endl;
            perf_log << "  总处理时间: " << std::fixed << std::setprecision(2) << coarse_search_time << " ms" << std::endl;
            perf_log << "  稀疏粒子数量: " << sparse_particles.size() << std::endl;
            perf_log << "  平均每粒子时间: " << std::fixed << std::setprecision(2) << coarse_search_time / sparse_particles.size() << " ms" << std::endl;
            perf_log << "  最佳评分: " << std::fixed << std::setprecision(6) << best_sparse_score << std::endl;
            perf_log << "  最佳位置: x=" << best_sparse_position[0] << ", y=" << best_sparse_position[1] << ", 角度=" << best_sparse_angle << std::endl;
            perf_log << "  时间点: " << std::put_time(std::localtime(&now_time_t), "%H:%M:%S") << std::endl;
            perf_log << "========================================" << std::endl;
        }
        

        
        // 第三步：局部精细搜索 - 以最高分粒子为中心进行精细搜索
        // 记录精细搜索的开始时间
        fine_search_start = this->now();
        RCLCPP_INFO(this->get_logger(), "开始局部精细搜索，以最佳粒子(x=%.2f, y=%.2f, area=%d)为中心", 
                     best_sparse_position[0], best_sparse_position[1], best_sparse_area_id);
        
        // 从参数文件读取的原始采样半径
        const float original_radius = this->get_parameter("particle_generator_radius").as_double();
        // 计算局部搜索半径 - 为原采样半径的1/2
        const float local_search_radius = original_radius / 3.0f;
        
        // 创建局部搜索粒子列表
        local_particles.clear();
        
        // 将最佳粒子添加到局部搜索列表
        local_particles.push_back(best_sparse_position);
        
        // 从原始粒子中筛选在局部搜索范围内的粒子
        for (size_t i = 0; i < corridorGuess.size(); i++) {
            // 计算当前粒子到最佳粒子的距离
            float dx = corridorGuess[i][0] - best_sparse_position[0];
            float dy = corridorGuess[i][1] - best_sparse_position[1];
            float distance = std::sqrt(dx*dx + dy*dy);
            
            // 如果在局部搜索半径内且不是最佳粒子本身，则添加到局部搜索列表
            if (distance <= local_search_radius && distance > 0.001f) {  // 使用小阈值避免添加重复的最佳粒子
                local_particles.push_back(corridorGuess[i]);
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "局部精细搜索留下了 %zu 个粒子", local_particles.size());
        
        // 使用更小的角度步长进行精细搜索
        const size_t fine_try_time = static_cast<size_t>(360/5); // 使用5度的角度步长
        

        
        // 对每个局部粒子进行精细角度搜索
        for(size_t particle_idx = 0; particle_idx < local_particles.size(); particle_idx++) {
            RCLCPP_DEBUG(this->get_logger(), "精搜索: 处理局部粒子 %zu (x=%.2f, y=%.2f)", 
                          particle_idx, local_particles[particle_idx][0], local_particles[particle_idx][1]);
            
            // 记录该粒子的最佳评分和角度
            double particle_best_score = 0.0;
            int particle_best_angle_idx = -1;
            
            // 对当前粒子进行精细角度搜索
            // 如果是最佳粒子，则使用其最佳角度附近进行搜索
            int start_angle_idx, end_angle_idx;
            
            if (particle_idx == 0) {
                // 对于最佳粒子，在其最佳角度附近搜索
                int best_angle_idx_fine = static_cast<int>(best_sparse_angle / 5.0); // 转换为5度步长的索引
                start_angle_idx = std::max(0, best_angle_idx_fine - 6); // 左右30度
                end_angle_idx = std::min(static_cast<int>(fine_try_time) - 1, best_angle_idx_fine + 6); // 右30度
            } else {
                // 对于其他局部粒子，进行全角度搜索
                start_angle_idx = 0;
                end_angle_idx = static_cast<int>(fine_try_time) - 1;
            }
            
            // 对当前粒子进行角度搜索
            for(int angle_idx = start_angle_idx; angle_idx <= end_angle_idx; angle_idx++) {
                auto startTime = this->now();
                
                // 发布当前猜测位置
                auto this_guess_stamped = geometry_msgs::msg::PointStamped();
                this_guess_stamped.header.frame_id = "map";
                this_guess_stamped.point.x = local_particles[particle_idx][0];
                this_guess_stamped.point.y = local_particles[particle_idx][1];
                this_guess_stamped.point.z = 0;
                pubRobotGuessMarker->publish(this_guess_stamped);
                
                // 重置参数
                accumulateAngle = 0;
                averDistancePairedPoints = 0;
                numofInsidePoints = 0;
                numofOutsidePoints = 0;
                insideScore = 0;
                outsideScore = 0;
                insideTotalRange = 0;
                outsideTotalScore = 0;
                turkeyScore = 0;
                
                // 设置初始位姿
                Eigen::Vector3f tempinitialExtTrans;
                tempinitialExtTrans << local_particles[particle_idx][0], local_particles[particle_idx][1], 0;
                float current_angle = std::fmod(initialYawAngle + angle_idx * 5.0, 360.0); // 使用5度步长
                setInitialPose(static_cast<int>(current_angle), tempinitialExtTrans);
                
                // Transform point cloud
                transformed_pc->points.clear();
                    transformed_pc->points.resize(organizedCloudInDS->points.size());
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
                        initializationICP(local_particles[particle_idx][2]);
                        auto icpEnd = this->now();
                        
                        // Publish updated transformed point cloud
                        pcl::toROSMsg(*transformed_pc, outMsg);
                        outMsg.header = mapHeader;
                        pubTransformedPC->publish(outMsg);
                    } else {
                        // If not using ICP initialization
                        calClosestMapPoint(static_cast<int>(local_particles[particle_idx][2]));
                    }
                    
                    double result_angle = 0;
                    if(bInitializationWithICP) {
                        Eigen::Matrix4d temp = robotPose.cast<double>();
                        Eigen::Matrix3d temp_ = temp.topLeftCorner(3,3);
                        Eigen::Quaterniond quaternion(temp_);
                        Eigen::Vector3d eulerAngle = quaternion.matrix().eulerAngles(1,2,0);
                        result_angle = eulerAngle[1]/M_PI*180;
                    } else {
                        result_angle = static_cast<int>(std::fmod(initialYawAngle + angle_idx * 5.0, 360.0));
                    }
                    
                    initialized = false;
                    
                    if(bGenerateResultFile) {
                        rescueRoomStream << "timestamp:" << rclcpp::Time(mapHeader.stamp).seconds() << ","
                                       << "angle:" << result_angle << ","
                                       << "guess_x:" << corridorGuess[particle_idx](0) << ","
                                       << "guess_y:" << corridorGuess[particle_idx](1) << ","
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
                    
                    // 计算当前角度的评分
                    double current_score = 1.0/(insideScore + outsideScore);
                    
                    // 更新该粒子的最佳角度
                    if(particle_best_angle_idx == -1 || current_score > particle_best_score) {
                        particle_best_angle_idx = angle_idx;
                        particle_best_score = current_score;
                    }
                    
                    // 更新精细搜索的最佳位姿和评分
                    if(best_fine_score < current_score) {
                        best_fine_score = current_score;
                        best_fine_pose = robotPose;
                        
                        // 发布当前最佳位姿
                        auto pose_max_stamped = geometry_msgs::msg::PointStamped();
                        pose_max_stamped.header.frame_id = "map";
                        pose_max_stamped.point.x = robotPose(0,3);
                        pose_max_stamped.point.y = robotPose(1,3);
                        pose_max_stamped.point.z = 0;
                        pubCurrentMaxRobotPose->publish(pose_max_stamped);
                        
                        RCLCPP_INFO(this->get_logger(), 
                            "精细搜索更新最佳位姿: x=%.2f, y=%.2f, 角度=%d度, 评分=%.6f",
                            robotPose(0,3), robotPose(1,3), 
                            static_cast<int>(result_angle), current_score);
                    }
                    else {
                        // 添加调试信息
                        RCLCPP_DEBUG(this->get_logger(),
                            "粒子位姿 x=%.2f, y=%.2f, yaw=%d 评分=%.6f < 当前最佳评分=%.6f (insideScore=%.2f, outsideScore=%.2f)",
                            robotPose(0,3), robotPose(1,3), 
                            static_cast<int>(result_angle),
                            current_score, best_fine_score, insideScore, outsideScore);
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
                                "精细搜索: 粒子 %zu, 角度 %d度, 时间 %.2f ms, 评分 %.6f", 
                                particle_idx, static_cast<int>(result_angle),
                                (endTime - startTime).seconds() * 1000, current_score);
                }
                
                RCLCPP_DEBUG(this->get_logger(), 
                    "局部粒子 %zu (x=%.2f, y=%.2f) 的最佳角度: %d度, 评分: %.6f",
                    particle_idx, local_particles[particle_idx][0], local_particles[particle_idx][1],
                    static_cast<int>(std::fmod(initialYawAngle + particle_best_angle_idx * 5.0, 360.0)),
                    particle_best_score);
            }
            
            // 记录当前精细搜索的完成状态
            RCLCPP_DEBUG(this->get_logger(), "完成粒子的精细搜索");
        }

        // 计算局部精细搜索总耗时
        auto fine_search_end = this->now();
        fine_search_time = (fine_search_end - fine_search_start).seconds() * 1000.0;
        
        // 计算平均每粒子处理时间
        size_t total_particles = local_particles.size();
        double avg_particle_time = total_particles > 0 ? fine_search_time / total_particles : 0.0;
        
        RCLCPP_INFO(this->get_logger(), "局部精细搜索总耗时: %.2f ms, 共 %zu 个粒子, 平均每粒子 %.2f ms", 
                    fine_search_time, total_particles, avg_particle_time);
        
        // 将精细搜索时间输出到日志文件
        if (perf_log.is_open()) {
            perf_log << "[局部精细搜索总结]" << std::endl;
            perf_log << "  总处理时间: " << std::fixed << std::setprecision(2) << fine_search_time << " ms" << std::endl;
            perf_log << "  局部粒子数量: " << total_particles << std::endl;
            perf_log << "  平均每粒子时间: " << std::fixed << std::setprecision(2) << avg_particle_time << " ms" << std::endl;
            perf_log << "  最佳评分: " << std::fixed << std::setprecision(6) << best_fine_score << std::endl;
            perf_log << "  时间点: " << std::put_time(std::localtime(&now_time_t), "%H:%M:%S") << std::endl;
            perf_log << "========================================" << std::endl;
        }
        
        // 更新最终的最佳位姿
        if (best_fine_score > 0.0) {
            // 从最佳位姿矩阵中提取位置和角度
            float yaw = atan2(best_fine_pose(1, 0), best_fine_pose(0, 0)) * 180.0 / M_PI;
            if (yaw < 0) yaw += 360.0;
            
            // 更新全局最佳位姿
            MaxRobotPose = best_fine_pose;
            MaxScore = best_fine_score;
            
            RCLCPP_INFO(this->get_logger(), "最终最佳位姿: x=%.2f, y=%.2f, 角度=%d度, 评分=%.6f",
                MaxRobotPose(0,3), MaxRobotPose(1,3), static_cast<int>(yaw), MaxScore);
        } else {
            // 如果精细搜索没有找到更好的位姿，则使用粗搜索的结果
            Eigen::Vector3f tempinitialExtTrans;
            tempinitialExtTrans << best_sparse_position[0], best_sparse_position[1], 0;
            setInitialPose(static_cast<int>(best_sparse_angle), tempinitialExtTrans);
            MaxRobotPose = initialPose;
            MaxScore = best_sparse_score;
            
            RCLCPP_INFO(this->get_logger(), "使用粗搜索结果作为最终位姿: x=%.2f, y=%.2f, 角度=%d度, 评分=%.6f",
                MaxRobotPose(0,3), MaxRobotPose(1,3), static_cast<int>(best_sparse_angle), MaxScore);
        }
        
        // 计算全局定位总耗时（粗搜索+精细搜索）
        double rescue_total_time = coarse_search_time + fine_search_time;
        RCLCPP_INFO(this->get_logger(), "全局定位总耗时: %.2f ms", rescue_total_time);
        
        // 将全局定位总时间输出到日志文件
        if (perf_log.is_open()) {
            // 添加空行以区分总结部分
            perf_log << std::endl;
            perf_log << "----------------------------------------" << std::endl;
            perf_log << "全局定位总结" << std::endl;
            perf_log << "----------------------------------------" << std::endl;
            perf_log << "原始粒子数量: " << corridorGuess.size() << std::endl;
            perf_log << "稀疏粒子数量: " << sparse_particles.size() << std::endl;
            perf_log << "局部粒子数量: " << local_particles.size() << std::endl;
            perf_log << "粗搜索时间: " << std::fixed << std::setprecision(2) << coarse_search_time << " ms (" << std::fixed << std::setprecision(1) << (coarse_search_time / rescue_total_time) * 100.0 << "%)" << std::endl;
            perf_log << "精细搜索时间: " << std::fixed << std::setprecision(2) << fine_search_time << " ms (" << std::fixed << std::setprecision(1) << (fine_search_time / rescue_total_time) * 100.0 << "%)" << std::endl;
            perf_log << "总处理时间: " << std::fixed << std::setprecision(2) << rescue_total_time << " ms" << std::endl;
            perf_log << "最佳评分: " << std::fixed << std::setprecision(6) << MaxScore << std::endl;
            perf_log << "最佳位置: x=" << MaxRobotPose(0,3) << ", y=" << MaxRobotPose(1,3) << ", 角度=" << static_cast<int>(std::fmod(atan2(MaxRobotPose(1,0), MaxRobotPose(0,0)) * 180.0 / M_PI, 360.0)) << std::endl;
            perf_log << "执行时间: " << std::put_time(std::localtime(&now_time_t), "%Y-%m-%d %H:%M:%S") << std::endl;
            perf_log << "----------------------------------------" << std::endl;
            
            // 关闭日志文件
            perf_log.close();
            RCLCPP_INFO(this->get_logger(), "性能日志文件已关闭");
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

        // 发布全局定位结果的marker
        visualization_msgs::msg::Marker global_loc_marker;
        global_loc_marker.header.frame_id = "map";
        global_loc_marker.header.stamp = this->now();
        global_loc_marker.ns = "global_localization";
        global_loc_marker.id = 0;
        global_loc_marker.type = visualization_msgs::msg::Marker::SPHERE;
        global_loc_marker.action = visualization_msgs::msg::Marker::ADD;
        
        // 设置位置
        global_loc_marker.pose.position.x = MaxRobotPose(0,3);
        global_loc_marker.pose.position.y = MaxRobotPose(1,3);
        global_loc_marker.pose.position.z = 0.0;
        global_loc_marker.pose.orientation.x = 0.0;
        global_loc_marker.pose.orientation.y = 0.0;
        global_loc_marker.pose.orientation.z = 0.0;
        global_loc_marker.pose.orientation.w = 1.0;
        
        // 设置大小
        global_loc_marker.scale.x = 1.0;
        global_loc_marker.scale.y = 1.0;
        global_loc_marker.scale.z = 1.0;
        
        // 设置颜色（蓝色）
        global_loc_marker.color.r = 0.0;
        global_loc_marker.color.g = 0.0;
        global_loc_marker.color.b = 1.0;
        global_loc_marker.color.a = 1.0;  // 不透明
        
        // 设置持续时间（秒）
        global_loc_marker.lifetime = rclcpp::Duration(0, 0);  // 0表示永久存在
        
        // 发布标记
        pubGlobalLocMarker->publish(global_loc_marker);
        RCLCPP_INFO(this->get_logger(), "已发布全局定位结果marker: x=%.2f, y=%.2f", MaxRobotPose(0,3), MaxRobotPose(1,3));


        
        // 生成并保存可视化图像
        if (visualization_enabled_ && corridorGuess.size() > 0) {
            // 提取WiFi定位位置（使用第一个粒子作为初始WiFi定位结果）
            std::vector<float> wifi_position = {corridorGuess[0][0], corridorGuess[0][1]};
            
            // 提取全局定位最终结果位置
            std::vector<float> final_position = {MaxRobotPose(0,3), MaxRobotPose(1,3)};
            
            // 构建输出文件路径
            std::ostringstream tsViz;
            tsViz.precision(2);
            tsViz << std::fixed << rclcpp::Time(mapHeader.stamp).seconds();
            std::string vizFilename = "/home/jay/AGLoc_ws/figs_localization/localization_result_" + tsViz.str() + ".png";
            
            // 保存可视化图像
            saveVisualizationImage(wifi_position, final_position, vizFilename);
            RCLCPP_INFO(this->get_logger(), "已保存定位结果可视化图像到: %s", vizFilename.c_str());
        }

        // 重置MaxScore为一个较低的值，确保下次能够更新
        MaxScore = 0.0;
}

/**
 * Draw the point cloud map on the image with enhanced visualization
 */
void CloudInitializer::drawMapOnImage(cv::Mat& image, float scale, float offset_x, float offset_y) {
    // Directly draw all points with increased size for better visibility
    for (size_t i = 0; i < map_pc->points.size(); i++) {
        const pcl::PointXYZI& point = map_pc->points[i];
        
        // Skip invalid points
        if (!isValidPoint(point)) {
            continue;
        }
        
        // Calculate image coordinates
        int img_x = static_cast<int>((point.x * scale) + offset_x);
        int img_y = static_cast<int>((point.y * scale) + offset_y);
        
        // Check if coordinates are within image boundaries
        if (img_x >= 0 && img_x < image.cols && img_y >= 0 && img_y < image.rows) {
            // Draw point with increased size (3 pixels) for better visibility
            cv::circle(image, cv::Point(img_x, img_y), 3, cv::Scalar(50, 50, 50), -1);
        }
    }
}

/**
 * Generate and save visualization image with map, WiFi position, and global localization results
 */
void CloudInitializer::saveVisualizationImage(const std::vector<float>& wifi_position, 
                                           const std::vector<float>& final_position,
                                           const std::string& output_path) {
    // Check point cloud size
    if (map_pc->points.empty()) {
        RCLCPP_WARN(this->get_logger(), "Map point cloud is empty, cannot generate visualization image");
        return;
    }
    
    // Calculate map's bounding box
    float min_x = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float min_y = std::numeric_limits<float>::max();
    float max_y = std::numeric_limits<float>::lowest();
    
    for (const auto& point : map_pc->points) {
        if (!isValidPoint(point)) continue;
        
        min_x = std::min(min_x, point.x);
        max_x = std::max(max_x, point.x);
        min_y = std::min(min_y, point.y);
        max_y = std::max(max_y, point.y);
    }
    
    // Add margin to boundaries
    float margin = 5.0f;
    min_x -= margin;
    max_x += margin;
    min_y -= margin;
    max_y += margin;
    
    // Calculate image size and scale
    float map_width = max_x - min_x;
    float map_height = max_y - min_y;
    
    // Set image width to 1200 pixels, height calculated proportionally
    const int img_width = 1200;
    const float scale = img_width / map_width;
    const int img_height = static_cast<int>(map_height * scale) + 100;  // Extra space for title and legend
    
    // Create white background image
    cv::Mat visualization(img_height, img_width, CV_8UC3, cv::Scalar(255, 255, 255));
    
    // Calculate map to image coordinate offset
    float offset_x = -min_x * scale;
    float offset_y = -min_y * scale + 50;  // Top padding for title
    
    // Draw the map with enhanced visualization
    drawMapOnImage(visualization, scale, offset_x, offset_y);
    
    // Draw WiFi localization result (blue dot)
    int wifi_x = static_cast<int>((wifi_position[0] * scale) + offset_x);
    int wifi_y = static_cast<int>((wifi_position[1] * scale) + offset_y);
    
    if (wifi_x >= 0 && wifi_x < visualization.cols && wifi_y >= 0 && wifi_y < visualization.rows) {
        cv::circle(visualization, cv::Point(wifi_x, wifi_y), 10, cv::Scalar(255, 0, 0), -1);
        cv::putText(visualization, "WiFi Position", cv::Point(wifi_x + 15, wifi_y), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 0, 0), 2);
    }
    
    // Draw global localization result (red dot)
    int final_x = static_cast<int>((final_position[0] * scale) + offset_x);
    int final_y = static_cast<int>((final_position[1] * scale) + offset_y);
    
    if (final_x >= 0 && final_x < visualization.cols && final_y >= 0 && final_y < visualization.rows) {
        cv::circle(visualization, cv::Point(final_x, final_y), 10, cv::Scalar(0, 0, 255), -1);
        cv::putText(visualization, "Global Position", cv::Point(final_x + 15, final_y), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
    }
    
    // Draw image title
    std::ostringstream title;
    title << "Localization Result Visualization - " << rclcpp::Time(mapHeader.stamp).seconds() << " sec";
    cv::putText(visualization, title.str(), cv::Point(10, 30), 
               cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 0), 2);
    
    // Draw legend
    cv::rectangle(visualization, cv::Point(10, img_height - 70), cv::Point(300, img_height - 10), cv::Scalar(255, 255, 255), -1);
    cv::rectangle(visualization, cv::Point(10, img_height - 70), cv::Point(300, img_height - 10), cv::Scalar(0, 0, 0), 1);
    
    cv::circle(visualization, cv::Point(30, img_height - 50), 10, cv::Scalar(255, 0, 0), -1);
    cv::putText(visualization, "WiFi Localization", cv::Point(50, img_height - 45), 
               cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 0), 2);
    
    cv::circle(visualization, cv::Point(30, img_height - 25), 10, cv::Scalar(0, 0, 255), -1);
    cv::putText(visualization, "Global Localization", cv::Point(50, img_height - 20), 
               cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 0), 2);
    
    // Draw visualization timestamp and scale info
    std::ostringstream scaleInfo;
    scaleInfo << "Scale: 1 pixel = " << std::fixed << std::setprecision(3) << (1.0/scale) << " meters";
    cv::putText(visualization, scaleInfo.str(), cv::Point(img_width - 400, img_height - 20), 
               cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 1);
    
    // Save the image
    cv::imwrite(output_path, visualization);
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
                                   double & /* minDist */,
                                   bool& /* findIntersection */) {
    double min_error = 0;
    // 移除未使用的变量
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

                // 移除未使用的变量赋值
                // mapLength 和 PCLength 已计算但不需要存储
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
    if (inside_index < 0 || static_cast<size_t>(inside_index) >= AG_index.area_index.size()) {
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
    
    RCLCPP_DEBUG(get_logger(), "AG_index range: start=%ld, end=%ld", 
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
/**
 * @file cloudInitializer_multithread.cpp
 * @author AGLoc优化
 * @brief 多线程版本的rescueRobot方法实现
 * @date 2025-03-10
 */

#include "localization_using_area_graph/cloudInitializer.hpp"

/**
 * @brief 多线程版本的rescueRobot方法
 * 
 * 使用线程池并行处理不同角度和粒子的组合，
 * 显著提高定位算法的效率。
 */
void CloudInitializer::rescueRobotMultiThread() {
    // 检查是否已经完成
    if(isRescueFinished) {
        RCLCPP_WARN(this->get_logger(), "已经完成救援，不需要重新执行");
        return;
    }
    
    // 检查是否准备好猜测
    if(!bGuessReady) {
        RCLCPP_WARN(this->get_logger(), "还没有准备好猜测位置，无法执行救援");
        return;
    }
    
    // 重置最大分数
    MaxScore = 0.0;
    
    if(pause_iter) {
        RCLCPP_INFO(this->get_logger(), "一次救援任务开始!");
    }
    
    // 增加救援次数计数
    rescueTimes++;
    
    // 创建结果文件
    RCLCPP_INFO(this->get_logger(), "----------- 猜测已就绪，开始救援 -----------------");

    // 准备输出文件
    // std::ostringstream ts;
    // ts.precision(2);
    // ts << std::fixed << rclcpp::Time(mapHeader.stamp).seconds();
    // std::string filename = "/home/jay/AGLoc_ws/frameResult" + 
    //                       ts.str() + "rescueRoom.txt";
                          
    // {
    //     std::lock_guard<std::mutex> lock(file_mutex);
    //     rescueRoomStream.open(filename, std::ofstream::out | std::ofstream::app);
    //     rescueRoomStream.setf(std::ios::fixed);
    //     rescueRoomStream.precision(2);
        
    //     if (!rescueRoomStream.good()) {
    //         RCLCPP_ERROR(this->get_logger(), "打开文件出错");
    //         return;
    //     }
    // }

    auto startC = std::chrono::high_resolution_clock::now();
    
    // 如果没有IMU初始化信息，才会进行角度遍历
    if (!initialization_imu) {
        size_t try_time = static_cast<size_t>(360/rescue_angle_interval);
        auto organizedCloudInDS = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        
        // 下采样点云
        pcl::VoxelGrid<pcl::PointXYZI> downSizeFurthestRing;
        downSizeFurthestRing.setLeafSize(scoreDownsampleRate, 
                                        scoreDownsampleRate, 
                                        scoreDownsampleRate);
        downSizeFurthestRing.setInputCloud(furthestRing);
        downSizeFurthestRing.filter(*organizedCloudInDS);
        
        RCLCPP_INFO(this->get_logger(), "下采样点云大小 = %lu", 
                    organizedCloudInDS->points.size());

        // 创建线程池
        // 直接将线程数限制为8个，减少线程管理开销
        unsigned int num_threads = 16u;
        // 如果硬件核心数少于8个，则使用硬件支持的线程数
        num_threads = std::min(num_threads, std::thread::hardware_concurrency());
        // 至少使用2个线程
        num_threads = std::max(num_threads, 2u);
        
        RCLCPP_INFO(this->get_logger(), "使用 %u 个线程并行评估位姿", num_threads);
        agloc::ThreadPool pool(num_threads);
        
        // 任务队列
        std::vector<std::future<void>> futures;
        size_t total_tasks = try_time; // 修改为只统计角度数量，因为每个线程处理一个角度
        futures.reserve(total_tasks);
        
        // 提交任务到线程池 - 修改为每个线程处理一个角度的所有粒子
        for(size_t i = 0; i < try_time; i++) {
            // 将每个角度作为一个任务提交到线程池
            futures.emplace_back(
                pool.enqueue([this, i, &organizedCloudInDS]() {
                    // 获取当前角度
                    int current_angle = static_cast<int>(std::fmod(initialYawAngle + i * rescue_angle_interval, 360.0));
                    RCLCPP_INFO(this->get_logger(), "线程开始处理角度 %d", current_angle);
                    
                    // 用于跟踪线程内部的最佳位姿和分数
                    double thread_max_score = 0.0;
                    Eigen::Matrix4f thread_max_pose = Eigen::Matrix4f::Identity();
                    bool thread_has_better_pose = false;
                    
                    // 处理此角度下的所有粒子
                    for(size_t j = 0; j < corridorGuess.size(); j++) {
                        auto startTime = this->now();
                        
                        // 为每个粒子评估创建独立的临时变量
                        double local_insideScore = 0;
                        double local_outsideScore = 0;
                        double local_insideTotalRange = 0;
                        double local_outsideTotalScore = 0;
                        double local_turkeyScore = 0;
                        int local_numofInsidePoints = 0;
                        int local_numofOutsidePoints = 0;
                        
                        // 创建线程本地的点云对象
                        auto local_transformed_pc = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
                        local_transformed_pc->points.resize(organizedCloudInDS->points.size());
                        
                        // 设置初始位姿
                        Eigen::Vector3f tempinitialExtTrans;
                        tempinitialExtTrans << corridorGuess[j][0], corridorGuess[j][1], 0;
                        
                        // 创建线程本地的机器人位姿矩阵
                        Eigen::Matrix4f local_robotPose = Eigen::Matrix4f::Identity();
                        
                        // 线程安全地设置初始位姿
                        this->setInitialPoseThreadSafe(
                            current_angle,
                            tempinitialExtTrans,
                            local_robotPose);
                        
                        // 变换点云
                        pcl::transformPointCloud(*organizedCloudInDS, *local_transformed_pc, local_robotPose);
                        
                        // 增加批处理大小，减少锁竞争 - 每处理100个粒子才发布一次
                        #ifdef DEBUG_PUBLISH
                        if(j % 100 == 0) {
                            std::lock_guard<std::mutex> lock(publish_mutex);
                            auto this_guess_stamped = geometry_msgs::msg::PointStamped();
                            this_guess_stamped.header.frame_id = "map";
                            this_guess_stamped.point.x = corridorGuess[j][0];
                            this_guess_stamped.point.y = corridorGuess[j][1];
                            this_guess_stamped.point.z = 0;
                            pubRobotGuessMarker->publish(this_guess_stamped);
                        }
                        #endif
                        
                        // 执行ICP或计算最近点
                        if(bInitializationWithICP) {
                            // 线程安全的ICP初始化
                            this->initializationICPThreadSafe(
                                corridorGuess[j][2], local_robotPose, local_transformed_pc,
                                local_insideScore, local_outsideScore,
                                local_numofInsidePoints, local_numofOutsidePoints,
                                local_turkeyScore);
                                
                            // 增加批处理大小，减少锁竞争 - 每处理100个粒子才发布一次点云
                            #ifdef DEBUG_PUBLISH
                            if(j % 100 == 0) {
                                std::lock_guard<std::mutex> lock(publish_mutex);
                                sensor_msgs::msg::PointCloud2 outMsg;
                                pcl::toROSMsg(*local_transformed_pc, outMsg);
                                outMsg.header = mapHeader;
                                pubTransformedPC->publish(outMsg);
                            }
                            #endif
                        } else {
                            // 线程安全的最近点计算
                            this->calClosestMapPointThreadSafe(
                                corridorGuess[j][2], local_robotPose, local_transformed_pc,
                                local_insideScore, local_outsideScore,
                                local_numofInsidePoints, local_numofOutsidePoints,
                                local_turkeyScore);
                        }
                        
                        // 计算结果角度
                        double result_angle = 0;
                        if(bInitializationWithICP) {
                            Eigen::Matrix4d temp = local_robotPose.cast<double>();
                            Eigen::Matrix3d temp_ = temp.topLeftCorner(3,3);
                            Eigen::Quaterniond quaternion(temp_);
                            Eigen::Vector3d eulerAngle = quaternion.matrix().eulerAngles(1,2,0);
                            result_angle = eulerAngle[1]/M_PI*180;
                        } else {
                            result_angle = current_angle;
                        }
                        
                        // 增加批处理大小，减少文件锁争用 - 每处理100个粒子才写入一次
                        if(j % 100 == 0 || j == corridorGuess.size() - 1) {
                            this->writeResultToFile(
                                result_angle, j, local_robotPose,
                                local_numofInsidePoints, local_insideScore,
                                local_numofOutsidePoints, local_outsideScore,
                                local_insideTotalRange, local_outsideTotalScore,
                                local_turkeyScore);
                        }
                        
                        // 线程内部先更新本地最佳分数，减少全局锁争用
                        double current_score = 1.0/(local_insideScore + local_outsideScore);
                        if(thread_max_score < current_score) {
                            thread_max_score = current_score;
                            thread_max_pose = local_robotPose;
                            thread_has_better_pose = true;
                            
                            // 记录线程内部找到更好位姿
                            RCLCPP_INFO(this->get_logger(), 
                                "线程 (角度=%d) 找到更好位姿: x=%.2f, y=%.2f, 评分=%.10f",
                                current_angle, local_robotPose(0,3), local_robotPose(1,3), current_score);
                        }
                        
                        #ifdef DETAILED_TIMING
                        if(j % 100 == 0) { // 每100个粒子记录一次时间
                            auto endTime = this->now();
                            RCLCPP_INFO(this->get_logger(), 
                                        "角度 %d, 完成粒子 %zu/%zu, 运行时间: %f ms", 
                                        current_angle, j, corridorGuess.size(),
                                        (endTime - startTime).seconds() * 1000);
                        }
                        #endif
                    }
                    
                    // 处理完所有粒子后，更新全局最佳位姿
                    if(thread_has_better_pose) {
                        std::lock_guard<std::mutex> lock(score_mutex);
                        if(MaxScore < thread_max_score) {
                            MaxScore = thread_max_score;
                            MaxRobotPose = thread_max_pose;
                            
                            #ifdef DEBUG_PUBLISH
                            // 发布当前最佳位姿
                            auto pose_max_stamped = geometry_msgs::msg::PointStamped();
                            pose_max_stamped.header.frame_id = "map";
                            pose_max_stamped.point.x = thread_max_pose(0,3);
                            pose_max_stamped.point.y = thread_max_pose(1,3);
                            pose_max_stamped.point.z = 0;
                            
                            // 发布消息
                            {
                                std::lock_guard<std::mutex> pub_lock(publish_mutex);
                                pubCurrentMaxRobotPose->publish(pose_max_stamped);
                            }
                            #endif
                            
                            RCLCPP_INFO(this->get_logger(), 
                                "当前全局最佳猜测: x=%.2f, y=%.2f, yaw=%d, 评分=%.10f",
                                thread_max_pose(0,3), thread_max_pose(1,3), 
                                current_angle, thread_max_score);
                        }
                    }
                    
                    RCLCPP_INFO(this->get_logger(), "线程完成角度 %d 的所有粒子评估", current_angle);
                })
            );
        }
        
        // 减少进度报告频率，只在关键点报告
        size_t completed = 0;
        size_t last_reported = 0;
        // 减少报告频率，从25%改为50%
        size_t report_interval = total_tasks / 2; // 每50%报告一次
        if(report_interval == 0) report_interval = 1;
        
        // 等待所有任务完成
        for(size_t i = 0; i < futures.size(); i++) {
            futures[i].wait();
            completed++;
            
            // 减少进度报告频率
            if(completed - last_reported >= report_interval) {
                last_reported = completed;
                RCLCPP_INFO(this->get_logger(), "完成进度: %.1f%% (%zu/%zu)",
                          (100.0 * completed) / total_tasks, completed, total_tasks);
            }
        }
        
        // 完成后发布最终的最佳位姿
        {
            std::lock_guard<std::mutex> lock(publish_mutex);
            auto pose_max_stamped = geometry_msgs::msg::PointStamped();
            pose_max_stamped.header.frame_id = "map";
            pose_max_stamped.point.x = MaxRobotPose(0,3);
            pose_max_stamped.point.y = MaxRobotPose(1,3);
            pose_max_stamped.point.z = 0;
            pubCurrentMaxRobotPose->publish(pose_max_stamped);
            
            // 如果需要，可以在这里发布最终的变换点云
            if(MaxScore > 0) {
                auto final_transformed_pc = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
                pcl::transformPointCloud(*organizedCloudInDS, *final_transformed_pc, MaxRobotPose);
                sensor_msgs::msg::PointCloud2 outMsg;
                pcl::toROSMsg(*final_transformed_pc, outMsg);
                outMsg.header = mapHeader;
                pubTransformedPC->publish(outMsg);
            }
        }

        RCLCPP_INFO(this->get_logger(), "----------------------救援机器人完成!!!--------------------------");
        bRescueRobot = false;
        isRescueFinished = true;
    }
    else {
        RCLCPP_INFO(this->get_logger(), "使用IMU初始化，跳过多线程救援");
        // 这里可以实现initialization_imu=true的情况下的代码，与原始rescueRobot保持一致即可
    }
    
    // 计算运行时间
    auto endC = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsedC = endC - startC;
    rescueRunTime = elapsedC.count();
    
    // 关闭输出文件
    {
        std::lock_guard<std::mutex> lock(file_mutex);
        if(rescueRoomStream.is_open()) {
            rescueRoomStream.close();
        }
    }
    
    RCLCPP_INFO(this->get_logger(), 
               "救援机器人运行时间 = %.3f秒", 
               rescueRunTime);
               
    // 发布结束信号
    geometry_msgs::msg::Pose doneSig;
    pubDONEsignal->publish(doneSig);
}

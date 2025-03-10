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
    std::ostringstream ts;
    ts.precision(2);
    ts << std::fixed << rclcpp::Time(mapHeader.stamp).seconds();
    std::string filename = "/home/jay/AGLoc_ws/frameResult" + 
                          ts.str() + "rescueRoom.txt";
                          
    {
        std::lock_guard<std::mutex> lock(file_mutex);
        rescueRoomStream.open(filename, std::ofstream::out | std::ofstream::app);
        rescueRoomStream.setf(std::ios::fixed);
        rescueRoomStream.precision(2);
        
        if (!rescueRoomStream.good()) {
            RCLCPP_ERROR(this->get_logger(), "打开文件出错");
            return;
        }
    }

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
        // 获取硬件支持的线程数，并限制最大线程数为16
        unsigned int num_threads = std::min(std::thread::hardware_concurrency(), 16u);
        // 至少使用2个线程
        num_threads = std::max(num_threads, 2u);
        
        RCLCPP_INFO(this->get_logger(), "使用 %u 个线程并行评估位姿", num_threads);
        agloc::ThreadPool pool(num_threads);
        
        // 任务队列
        std::vector<std::future<void>> futures;
        size_t total_tasks = try_time * corridorGuess.size();
        futures.reserve(total_tasks);
        
        // 提交所有任务到线程池
        for(size_t i = 0; i < try_time; i++) {
            for(size_t j = 0; j < corridorGuess.size(); j++) {
                // 将每个(i,j)组合作为一个任务提交到线程池
                futures.emplace_back(
                    pool.enqueue([this, i, j, &organizedCloudInDS]() {
                        auto startTime = this->now();
                        
                        // 为每个线程创建独立的临时变量
                        double local_insideScore = 0;
                        double local_outsideScore = 0;
                        double local_insideTotalRange = 0;
                        double local_outsideTotalScore = 0;
                        double local_turkeyScore = 0;
                        int local_numofInsidePoints = 0;
                        int local_numofOutsidePoints = 0;
                        double local_accumulateAngle = 0;
                        double local_averDistancePairedPoints = 0;
                        
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
                            static_cast<int>(std::fmod(initialYawAngle + i * rescue_angle_interval, 360.0)),
                            tempinitialExtTrans,
                            local_robotPose);
                        
                        // 变换点云
                        pcl::transformPointCloud(*organizedCloudInDS, *local_transformed_pc, local_robotPose);
                        
                        // 移除每次迭代的消息发布，减少锁竞争
                        // 只在调试模式下发布，通过编译宏控制
                        #ifdef DEBUG_PUBLISH
                        {
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
                                
                            // 移除每次ICP后的点云发布
                            #ifdef DEBUG_PUBLISH
                            {
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
                            result_angle = static_cast<int>(std::fmod(initialYawAngle + i * rescue_angle_interval, 360.0));
                        }
                        
                        // 写入结果文件
                        this->writeResultToFile(
                            result_angle, j, local_robotPose,
                            local_numofInsidePoints, local_insideScore,
                            local_numofOutsidePoints, local_outsideScore,
                            local_insideTotalRange, local_outsideTotalScore,
                            local_turkeyScore);
                        
                        // 使用原子操作更新最佳位姿，减少锁的持有时间
                        {
                            double current_score = 1.0/(local_insideScore + local_outsideScore);
                            bool update_needed = false;
                            
                            // 使用短暂的锁检查是否需要更新
                            {
                                std::lock_guard<std::mutex> lock(score_mutex);
                                if(MaxScore < current_score) {
                                    MaxScore = current_score;
                                    MaxRobotPose = local_robotPose;
                                    update_needed = true;
                                }
                            }
                            
                            // 只有在真正需要更新时才发布消息和记录日志
                            // 这部分代码在锁外执行，减少锁的持有时间
                            if(update_needed) {
                                #ifdef DEBUG_PUBLISH
                                // 发布当前最佳位姿
                                auto pose_max_stamped = geometry_msgs::msg::PointStamped();
                                pose_max_stamped.header.frame_id = "map";
                                pose_max_stamped.point.x = local_robotPose(0,3);
                                pose_max_stamped.point.y = local_robotPose(1,3);
                                pose_max_stamped.point.z = 0;
                                
                                // 使用单独的锁发布消息
                                {
                                    std::lock_guard<std::mutex> lock(publish_mutex);
                                    pubCurrentMaxRobotPose->publish(pose_max_stamped);
                                }
                                #endif
                                
                                // 减少日志输出频率，只记录显著改进
                                static double last_reported_score = 0.0;
                                if(current_score > last_reported_score * 1.05) { // 只有提高5%以上才报告
                                    last_reported_score = current_score;
                                    RCLCPP_INFO(this->get_logger(), 
                                        "当前最佳猜测: x=%.2f, y=%.2f, yaw=%d, 评分=%.4f",
                                        local_robotPose(0,3), local_robotPose(1,3), 
                                        static_cast<int>(std::fmod(initialYawAngle + i * rescue_angle_interval, 360.0)),
                                        current_score);
                                }
                            }
                        }
                        
                        // 移除每次迭代的时间记录，减少日志输出
                        // 可以考虑使用计数器，只记录每N次迭代的时间
                        #ifdef DETAILED_TIMING
                        auto endTime = this->now();
                        RCLCPP_DEBUG(this->get_logger(), 
                                    "单个猜测运行时间: %f ms", 
                                    (endTime - startTime).seconds() * 1000);
                        #endif
                    })
                );
            }
        }
        
        // 减少进度报告频率，只在关键点报告
        size_t completed = 0;
        size_t last_reported = 0;
        // 减少报告频率，从10%改为25%
        size_t report_interval = total_tasks / 4; // 每25%报告一次
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

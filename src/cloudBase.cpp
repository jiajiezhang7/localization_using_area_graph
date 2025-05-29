/**
 * @file cloudBase.cpp
 * @author Jiajie Zhang (ROS2 port)
 *         Fujing Xie (original ROS1 implementation)
 *         Sören Schwertfeger (original ROS1 implementation)
 * @brief Implementation of cloudBase class for Area Graph-based LiDAR localization
 *        ROS2 implementation of the core AGLoc functionality
 * @version 0.1
 * @date 2024-11-09
 * 
 * @details Implements the following key functionalities:
 *          - Point cloud data processing and organization
 *          - Clutter removal and point cloud subsampling
 *          - Area Graph intersection calculations
 *          - Point cloud registration and pose estimation
 *          - Memory management for point cloud data structures
 *          - ROS2 node handling and message processing
 * 
 * @note This implementation corresponds to the methods described in:
 *       "Robust Lifelong Indoor LiDAR Localization using the Area Graph"
 *       IEEE Robotics and Automation Letters, 2023
 * 
 * @dependencies
 *        - ROS2 core libraries
 *        - PCL library for point cloud processing
 *        - Area Graph data structures
 *        - TF2 for coordinate transformations
 * 
 * @copyright Copyright (c) 2024, ShanghaiTech University
 *            All rights reserved.
 * 
 */
#include "localization_using_area_graph/cloudBase.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <opencv2/opencv.hpp>

// 静态成员变量的定义
std::mutex CloudBase::agIndexMutex;
bool CloudBase::AGindexReceived = false;
area_graph_data_parser::msg::AGindex CloudBase::AG_index;

void CloudBase::saveTUMTraj(geometry_msgs::msg::PoseStamped & pose_stamped) {
    robotPoseTum << pose_stamped.header.stamp.sec + pose_stamped.header.stamp.nanosec * 1e-9 
                 << " " << pose_stamped.pose.position.x 
                 << " " << pose_stamped.pose.position.y 
                 << " " << pose_stamped.pose.position.z 
                 << " " << pose_stamped.pose.orientation.x 
                 << " " << pose_stamped.pose.orientation.y 
                 << " " << pose_stamped.pose.orientation.z 
                 << " " << pose_stamped.pose.orientation.w << std::endl;
}



// robotPose 是机器人的当前位置姿态（4x4矩阵），从中提取出机器人的x、y、z坐标位置。
// map_pc 是一个点云，它存储了区域边界的顶点信息。每个点的intensity值用来标记边界的特征（比如是否是区域的结束点）。
// 代码的核心逻辑是：
    // 从机器人位置发射一条射线（通过创建一个远处的点robotInfinity）
    // 检查这条射线与区域边界的交点数量（throughTimes）
    // 如果交点数为奇数，说明点在区域内；如果为偶数，说明点在区域外

// input: robotPose, Area的起始点
bool CloudBase::areaInsideChecking(const Eigen::Matrix4f& robotPose, int areaStartIndex) {
    // 创建机器人当前位置点
    pcl::PointXYZI robot;
    robot.x = robotPose(0,3);
    robot.y = robotPose(1,3);
    robot.z = robotPose(2,3);
    robot.intensity = 0;

    // 创建一个远处的点,用于射线检测
    pcl::PointXYZI robotInfinity;
    robotInfinity.x = robotPose(0,3) + 5132;
    robotInfinity.y = robotPose(1,3) + 2345;
    robotInfinity.z = robotPose(2,3);
    robotInfinity.intensity = 0;

    // 记录射线穿过边界的次数
    int throughTimes = 0;

    // 创建可视化标记
    auto line_strip = std::make_unique<visualization_msgs::msg::Marker>();
    line_strip->type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_strip->header = mapHeader;
    line_strip->color.b = 1.0;
    line_strip->color.a = 1.0;
    line_strip->scale.x = 0.1;
    line_strip->id = 1;
    line_strip->action = visualization_msgs::msg::Marker::ADD;

    // 检查该区域所有边界线段
    for(int i = areaStartIndex; i < areaStartIndex + 1000000; i++) {
        // 到达区域末尾时退出
        if((int)map_pc->points[i].intensity % 3 == 2) {
            // intensity % 3 == 2 --> Area的结束点
            break;
        }
        bool b_inray;
        inRayGeneral(map_pc->points[i], map_pc->points[(i+1)%mapSize], robot, robotInfinity, b_inray);
        if(b_inray) {
            throughTimes++;
        }
    }

    // 如果射线穿过边界次数为奇数,说明点在区域内部
    if(throughTimes % 2 == 1) {
        // 可视化射线
        geometry_msgs::msg::Point p;
        p.x = robot.x;
        p.y = robot.y;
        p.z = robot.z;

        geometry_msgs::msg::Point p_;
        p_.x = robotInfinity.x;
        p_.y = robotInfinity.y;
        p_.z = robotInfinity.z;

        line_strip->points.push_back(p);
        line_strip->points.push_back(p_);
        pubinfinity->publish(*line_strip);
        RCLCPP_INFO(this->get_logger(), "------------------throughTimes: %d ---------------------------", throughTimes);
        return true;
    } else {
        return false;
    }
}

// 初始化CloudBase类
CloudBase::CloudBase(const std::string& node_name)
    : ParamServer(node_name) {
    {
        std::lock_guard<std::mutex> lock(agIndexMutex);
        AGindexReceived = false;
    }
    initializeVariables();
    initializePublishers();
    initializeSubscribers();
    allocateMemory();


    RCLCPP_INFO(this->get_logger(), "CloudBase initialized successfully");
}

// 接受到AGindex后的回调处理
void CloudBase::AGindexCB(const area_graph_data_parser::msg::AGindex::SharedPtr msg) {
    AG_index = *msg;
    {
        std::lock_guard<std::mutex> lock(agIndexMutex);
        AGindexReceived = true;
    }
    // RCLCPP_INFO(get_logger(), 
    //         "Successfully Received AG_index with %zu areas", 
    //         AG_index.area_index.size());
    
    // 添加详细日志
    // RCLCPP_DEBUG(get_logger(), "First few area indices:");
    // for(size_t i = 0; i < std::min(size_t(3), AG_index.area_index.size()); i++) {
    //     RCLCPP_DEBUG(get_logger(), "Area[%zu]: id=%d", i, AG_index.area_index[i]);
    // }
}


// 真实被使用的map来自area_graph_data_parser的mapPC_AG
// AGLoc系统中，地图数据的流向：  topology_publisher -> /mapPC_AG -> CloudBase::mapAGCB() -> pubMapPC(它被定位流程真正用到)+ pubAGMapTransformedPC
/**
 * @brief 处理来自Area Graph的地图点云数据的回调函数
 * @details 该函数负责:
 *          1. 接收和转换Area Graph地图点云
 *          2. 对地图进行坐标变换
 *          3. 计算地图中心点
 *          4. 初始化地图相关参数
 * @param laserCloudMsg 输入的地图点云消息(ROS PointCloud2格式)
 */
void CloudBase::mapAGCB(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg) {
    // 添加调试信息
    // RCLCPP_DEBUG(this->get_logger(), "Receiving map from AG");

    // 获取当前地图点云的大小
    mapSize = map_pc->points.size();
    
    // 检查条件
    if(!isAGIndexReceived() || mapInit) {
        RCLCPP_DEBUG_ONCE(this->get_logger(), "AGindexReceived: %d, mapInit: %d", 
                    isAGIndexReceived(), mapInit);
        return; 
    }


    // 设置地图坐标系为"map"
    std_msgs::msg::Header tempHeader = laserCloudMsg->header;
    // 设置发布的可视化topic的地图坐标系为"map"，这样才能检验map_pc的正确性
    tempHeader.frame_id = "map";

    // 创建PCL点云对象用于存储原始和变换后的地图
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserAGMap(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserAGMapTansformed(new pcl::PointCloud<pcl::PointXYZI>());

    // 将ROS消息转换为PCL点云格式
    try {
        pcl::fromROSMsg(*laserCloudMsg, *laserAGMap);
        RCLCPP_INFO(this->get_logger(), "---------------PCL conversion successful. AGMap Points: %zu-----------------", 
                    laserAGMap->points.size());

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "PCL conversion failed: %s", e.what());
        return;
    }

    // 从TF中获取map到AGmap的变换关系，而不是从参数中获取
    Eigen::Matrix4f mapPose = Eigen::Matrix4f::Identity();
    try {
        // 查询最新的map到AGmap的变换关系
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped = tf_buffer_->lookupTransform("map", "AGmap", tf2::TimePointZero);
        
        // 从TransformStamped中提取变换信息
        Eigen::Vector3f translation;
        translation.x() = transformStamped.transform.translation.x;
        translation.y() = transformStamped.transform.translation.y;
        translation.z() = transformStamped.transform.translation.z;
        
        Eigen::Quaternionf rotation(
            transformStamped.transform.rotation.w,
            transformStamped.transform.rotation.x,
            transformStamped.transform.rotation.y,
            transformStamped.transform.rotation.z
        );
        
        // 构建变换矩阵
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.translation() = translation;
        transform.rotate(rotation);
        mapPose = transform.matrix();
        
        RCLCPP_INFO(this->get_logger(), "Map->AGmap transform retrieved from TF: [%f, %f, %f]", 
                   translation.x(), translation.y(), translation.z());
    } catch (const tf2::TransformException& ex) {
        RCLCPP_ERROR(this->get_logger(), "Could not transform map to AGmap: %s", ex.what());
        RCLCPP_WARN(this->get_logger(), "Falling back to default parameters for map transformation");
        
        // 如果TF失败，回退到参数中的值（保持兼容性）
        Eigen::Affine3f transform_initial = Eigen::Affine3f::Identity();
        transform_initial.translation() << mapExtTrans[0], mapExtTrans[1], mapExtTrans[2];
        transform_initial.rotate(Eigen::AngleAxisf(mapYawAngle/180.0*M_PI, Eigen::Vector3f::UnitZ()));
        mapPose = transform_initial.matrix();
    }

    // 对点云进行坐标变换
    try {
        pcl::transformPointCloud(*laserAGMap, *laserAGMapTansformed, mapPose);
        RCLCPP_INFO(this->get_logger(), "-----------AGmap Points transformation successful------------");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Transform failed: %s", e.what());
        return;
    }

    // 发布变换后的点云
    sensor_msgs::msg::PointCloud2 outMsg;
    pcl::toROSMsg(*laserAGMapTansformed, outMsg);
    outMsg.header = tempHeader;
    // 给particle_filter使用
    pubAGMapTransformedPC->publish(outMsg);

    // 初始化地图中心计算相关变量
    double mapCenterWeight = 0;
    mapCenterInitialization.setZero();
    mapReceivedTimes++;
    
    mapHistogram.clear();

    // 如果地图未初始化，进行初始化处理
    if(!mapInit) {
        map_pc->clear();
        mapSize = laserAGMapTansformed->points.size(); 
        map_pc->points.resize(mapSize);

        // 遍历所有点，进行地图初始化
        for (int i = 0; i < mapSize; i++) {
            // 创建并设置点的属性
            pcl::PointXYZI thisPoint;
            thisPoint.x = laserAGMapTansformed->points[i].x;  
            thisPoint.y = laserAGMapTansformed->points[i].y;
            thisPoint.z = laserAGMapTansformed->points[i].z;
            thisPoint.intensity = laserAGMapTansformed->points[i].intensity;

            // 验证点的intensity值是否有效
            int intensityMod3 = static_cast<int>(thisPoint.intensity) % 3;
            if(intensityMod3 < 0 || intensityMod3 > 2) {
                RCLCPP_WARN(this->get_logger(), 
                            "Invalid intensity value at point %d: %.3f (mod 3 = %d)",
                            i, thisPoint.intensity, intensityMod3);
            }

            map_pc->points[i] = thisPoint;
            mapHistogram.push_back(0);

            // 计算相邻点的中点和距离
            double middile_x = (map_pc->points[i].x - map_pc->points[(i-1+mapSize)%mapSize].x)/2;
            double middile_y = (map_pc->points[i].y - map_pc->points[(i-1+mapSize)%mapSize].y)/2;
            double length = std::sqrt(
                std::pow(map_pc->points[i].x - map_pc->points[(i-1+mapSize)%mapSize].x, 2) +
                std::pow(map_pc->points[i].y - map_pc->points[(i-1+mapSize)%mapSize].y, 2));
                
            // 累加权重和加权坐标
            mapCenterWeight += length;
            mapCenterInitialization(0) += middile_x * length;
            mapCenterInitialization(1) += middile_y * length;
        }
        
        // 计算地图中心点的加权平均坐标
        mapCenterInitialization = mapCenterInitialization / mapCenterWeight;

        // 输出地图中心点信息并设置初始化标志
        RCLCPP_INFO(this->get_logger(), "Map center = [%f, %f]", 
                    mapCenterInitialization(0), mapCenterInitialization(1));
        mapInit = true;
        RCLCPP_INFO(this->get_logger(), "AG Map initialized success, this is the %d map.", 
                    mapReceivedTimes);
    }

    // 发布处理后的地图点云
    sensor_msgs::msg::PointCloud2 outMsgMap;
    pcl::toROSMsg(*map_pc, outMsgMap);
    outMsgMap.header = tempHeader;
    pubMapPC->publish(outMsgMap);

    // 创建可视化图像 - map_visualization
    // const int img_size = 800;
    // const int margin = 50;
    // cv::Mat visualization = cv::Mat::zeros(img_size + 2*margin, img_size + 2*margin, CV_8UC3);
    
    // // 计算点云的边界框
    // float min_x = std::numeric_limits<float>::max();
    // float max_x = std::numeric_limits<float>::lowest();
    // float min_y = std::numeric_limits<float>::max();
    // float max_y = std::numeric_limits<float>::lowest();
    
    // for (const auto& point : map_pc->points) {
    //     min_x = std::min(min_x, point.x);
    //     max_x = std::max(max_x, point.x);
    //     min_y = std::min(min_y, point.y);
    //     max_y = std::max(max_y, point.y);
    // }
    
    // // 计算缩放因子
    // float scale = img_size / std::max(max_x - min_x, max_y - min_y);
    
    // // 绘制地图点云（白色）
    // for (size_t i = 0; i < map_pc->points.size(); i++) {
    //     int x = static_cast<int>((map_pc->points[i].x - min_x) * scale) + margin;
    //     int y = static_cast<int>((map_pc->points[i].y - min_y) * scale) + margin;
    //     cv::circle(visualization, cv::Point(x, y), 1, cv::Scalar(255, 255, 255), -1);
        
    // }
    
    // // 绘制地图中心点（绿色）
    // int center_x = static_cast<int>((mapCenterInitialization(0) - min_x) * scale) + margin;
    // int center_y = static_cast<int>((mapCenterInitialization(1) - min_y) * scale) + margin;
    // cv::circle(visualization, cv::Point(center_x, center_y), 5, cv::Scalar(0, 255, 0), -1);
    
    // // 绘制初始位置（红色）
    // int init_x = static_cast<int>((initialExtTrans[0] - min_x) * scale) + margin;
    // int init_y = static_cast<int>((initialExtTrans[1] - min_y) * scale) + margin;
    // cv::circle(visualization, cv::Point(init_x, init_y), 5, cv::Scalar(0, 0, 255), -1);
    
    // // 绘制初始方向箭头
    // float arrow_length = 20.0; // 箭头长度（像素）
    // float init_angle = initialYawAngle * M_PI / 180.0; // 转换为弧度
    // int arrow_x = init_x + static_cast<int>(arrow_length * cos(init_angle));
    // int arrow_y = init_y + static_cast<int>(arrow_length * sin(init_angle));
    // cv::arrowedLine(visualization, cv::Point(init_x, init_y), 
    //                 cv::Point(arrow_x, arrow_y),
    //                 cv::Scalar(0, 0, 255), 2, cv::LINE_AA, 0, 0.3);
    
    // // 添加图例
    // cv::putText(visualization, "Map Center", cv::Point(margin, img_size + margin * 1.5), 
    //             cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
    // cv::putText(visualization, "Initial Pose", cv::Point(margin + 150, img_size + margin * 1.5), 
    //             cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
    
    // // 保存图像
    // cv::imwrite("/home/jay/AGLoc_ws/src/localization_using_area_graph/maps/map_visualization.png", visualization);
    // RCLCPP_INFO(this->get_logger(), "Map visualization saved to /home/jay/AGLoc_ws/src/localization_using_area_graph/maps/map_visualization.png");
}


// 函数作用：将原始的无序激光点云数据组织成有序的格式（按行列排列）， 过滤掉无效和不需要的点（如地面点、太远或太近的点）
// 输入： LaserCloudIn
// 输出 --- 函数计算后的结果存储在：
    // 1. organizedCloudIn64 -- 完整的64线点云
    // 2. organizedCloudIn -- 筛选后的有序点云(根据运行模式不同而存储不同内容：降采样点云或最远点)
void CloudBase::organizePointcloud() {
    // 添加参数验证
    if (N_SCAN <= 0 || Horizon_SCAN <= 0) {
        RCLCPP_ERROR(get_logger(), "Invalid scan parameters: N_SCAN=%d, Horizon_SCAN=%d", 
                     N_SCAN, Horizon_SCAN);
        return;
    }

    if (!laserCloudIn || laserCloudIn->empty()) {
        RCLCPP_ERROR(get_logger(), "Empty input point cloud");
        return;
    }
    // 为点云分配内存前进行大小检查
    if (transformed_pc && transformed_pc->width > 0) {
        size_t cloud_size = transformed_pc->width * transformed_pc->height;
        if (cloud_size == 0) {
            RCLCPP_ERROR(get_logger(), "Invalid cloud size: width=%d, height=%d",
                        transformed_pc->width, transformed_pc->height);
            return;
        }
    }
    try {
        // 3. 初始化数据结构
        // RCLCPP_DEBUG(get_logger(), "Resizing point clouds for organization...");
        organizedCloudIn64->resize(64 * Horizon_SCAN);
        
        // 正确初始化 furthestRing
        furthestRing->clear();
        furthestRing->width = Horizon_SCAN;   // 设置宽度
        furthestRing->height = 1;             // 设置高度为1（无序点云）
        furthestRing->points.resize(Horizon_SCAN);
        furthestRing->is_dense = false;       // 可能包含无效点

        // 2. 同样初始化transformedFurthestRing
        transformedFurthestRing->clear();
        transformedFurthestRing->width = Horizon_SCAN;
        transformedFurthestRing->height = 1;
        transformedFurthestRing->is_dense = false;
        transformedFurthestRing->points.resize(Horizon_SCAN);

        // 3. 添加验证
        if (furthestRing->width != Horizon_SCAN) {
            RCLCPP_ERROR(get_logger(), 
            "FurthestRing initialization failed: width=%zu, expected=%d",
            furthestRing->width, Horizon_SCAN);
            return;
        
        }
        if(bFurthestRingTracking) {
            N_SCAN = 64;
            // RCLCPP_DEBUG(get_logger(), "FurthestRingTracking enabled, N_SCAN set to 64");
        }
        
        int cloudSize = laserCloudIn->points.size();

        furthestRing->points.resize(Horizon_SCAN, pcl::PointXYZI());
        transformedFurthestRing->points.resize(Horizon_SCAN, pcl::PointXYZI());

        // 4. 处理每个点
        int validPoints = 0;
        int filteredPoints = 0;
        
        for (int i = 0; i < cloudSize; ++i) {
            pcl::PointXYZI thisPoint;
            thisPoint.x = laserCloudIn->points[i].x;
            thisPoint.y = laserCloudIn->points[i].y;
            thisPoint.z = laserCloudIn->points[i].z;

            // 5. 计算点的距离信息
            float range_xy = std::sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y);
            thisPoint.intensity = range_xy;
            float range = std::sqrt(range_xy * range_xy + thisPoint.z * thisPoint.z);

            // 6. 获取行索引并验证
            // 这里就是访问ring的地方
            int rowIdn = laserCloudIn->points[i].ring;
            if (rowIdn < 0 || rowIdn >= N_SCAN) {
                RCLCPP_ERROR(get_logger(), 
                    "Invalid ring index %d at point %d [x:%.2f, y:%.2f, z:%.2f]",
                    rowIdn, i, thisPoint.x, thisPoint.y, thisPoint.z);
                filteredPoints++;
                continue;
            }

            // 7. 应用降采样
            if (rowIdn % downsampleRate != 0) {
                filteredPoints++;
                continue;
            }

            // 8. 计算列索引
            float horizonAngle = std::atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;
            static float ang_res_x = 360.0/float(Horizon_SCAN);
            int columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;
            
            // 9. 验证列索引
            if (columnIdn >= Horizon_SCAN) {
                columnIdn -= Horizon_SCAN;
            }
            if (columnIdn < 0 || columnIdn >= Horizon_SCAN) {
                RCLCPP_DEBUG(get_logger(), 
                    "Point %d filtered: column index %d out of range", i, columnIdn);
                filteredPoints++;
                continue;
            }


            // 11. 处理点的存储
            int index = columnIdn + rowIdn * Horizon_SCAN;
            if (range < lidarMinRange || range > lidarMaxRange ||
                thisPoint.z < groundThred || thisPoint.z > ceilingThred) {
                // 清零点并存储
                thisPoint.x = thisPoint.y = thisPoint.z = thisPoint.intensity = 0;
                
                if(!initialized && bRescueRobot) {
                    organizedCloudIn->points[index%Horizon_SCAN] = thisPoint;
                } else {
                    organizedCloudIn->points[index] = thisPoint;
                    organizedCloudIn64->points[index] = thisPoint;
                }
                filteredPoints++;
                continue;
            }

            // 12. 存储有效点
            if((!initialized && bRescueRobot)) {
                organizedCloudIn->points[index%Horizon_SCAN] = thisPoint;
            } else {
                organizedCloudIn->points[index] = thisPoint;
                organizedCloudIn64->points[index] = thisPoint;
            }
            validPoints++;

            // 13. 更新最远点
            if(range_xy > furthestRing->points[columnIdn].intensity) {
                furthestRing->points[columnIdn] = thisPoint;
                furthestRing->points[columnIdn].intensity = range_xy;
            }
        }

        // 14. 记录处理结果
        // RCLCPP_INFO(get_logger(), 
        //     "-------------Point cloud organized: %d valid points, %d filtered points--------------",
        //     validPoints, filteredPoints);

        // rescue without initialization
        if(!initialized && bRescueRobot) {

            if (furthestRing->width == 0) {
                RCLCPP_ERROR(get_logger(), "FurthestRing width is zero before assignment");
                return;
            }

            *organizedCloudIn = *furthestRing; 
            RCLCPP_DEBUG(get_logger(), "Rescue mode: Using furthestRing as organizedCloudIn");
        }
        if(bFurthestRingTracking) {
            if (furthestRing->width == 0 || furthestRing->points.empty()) {
                RCLCPP_ERROR(get_logger(), 
                    "Invalid furthestRing: width=%zu, points=%zu",
                    furthestRing->width, furthestRing->points.size());
                return;
            }
            *organizedCloudIn = *furthestRing; 
            N_SCAN = 1;
            RCLCPP_DEBUG_ONCE(get_logger(), "FurthestRingTracking: Reset N_SCAN to 1");
        }


    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Error in organizePointcloud: %s", e.what());
        throw;
    }
}



void CloudBase::verticalRadiusFilter() {
    for (int i = 0; i < Horizon_SCAN; i++) {
        double average_radius = 0;
        int number = 0;
        
        for(int j = N_SCAN-1; j > 0; j--) {
            bool initialized = false;
            double radius = std::sqrt(
                std::pow(UsefulPoints1->points[i + j * Horizon_SCAN].x, 2) +
                std::pow(UsefulPoints1->points[i + j * Horizon_SCAN].y, 2));
                
            if(radius > 0.1 && !initialized) {
                average_radius += radius;
                number++;
                initialized = true;
            }
            
            if(number == 3) {
                initialized = true;
                number = 0;
                average_radius = average_radius/5;
                continue;
            }
            
            if(initialized) {
                if(std::abs(radius - average_radius) > radiusDisthred) {
                    UsefulPoints1->points[i + j * Horizon_SCAN].x = 0;
                    UsefulPoints1->points[i + j * Horizon_SCAN].y = 0;
                    UsefulPoints2->points[i + j * Horizon_SCAN].x = 0;
                    UsefulPoints2->points[i + j * Horizon_SCAN].y = 0;
                }
            }
        }
    }
}

void CloudBase::setInitialPose(double initialYawAngle, Eigen::Vector3f initialExtTrans) {
    robotPose.setZero();
    Eigen::Affine3f transform_initial = Eigen::Affine3f::Identity();
    transform_initial.translation() << initialExtTrans[0], initialExtTrans[1], initialExtTrans[2];
    
    
    transform_initial.rotate(
        Eigen::AngleAxisf(initialYawAngle/180.0*M_PI, Eigen::Vector3f::UnitZ())
    );
    // 仅PoseTracking模式下，直接将yaml中的参数转换为初始位姿矩阵
    robotPose = transform_initial.matrix();
    
    // RCLCPP_DEBUG(this->get_logger(), "Initial robot pose set with yaw angle: %f", initialYawAngle);
}

void CloudBase::pubPclCloud(
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub,
    std_msgs::msg::Header* cloudHeader) {
    
    if (!cloud || !pub || !cloudHeader) {
        RCLCPP_WARN(this->get_logger(), "Invalid inputs to pubPclCloud");
        return;
    }

    sensor_msgs::msg::PointCloud2 tempCloud;
    pcl::toROSMsg(*cloud, tempCloud);
    tempCloud.header = *cloudHeader;
    pub->publish(tempCloud);
}

void CloudBase::setEveryFrame() {
    bPCA = false;
    accumulateAngle = 0;
    averDistancePairedPoints = 0;
    onlyOneDirection = false;
    
    // mapHistogram has same size as map, calculate every frame of lidar data
    if (!mapHistogram.empty()) {
        std::fill(mapHistogram.begin(), mapHistogram.end(), 0);
    }
    numTotalHistogram = 0;
    
    // RCLCPP_DEBUG(this->get_logger(), "Frame parameters reset");
}


// 实用工具函数添加
void CloudBase::allocateMemory() {
    // 添加前置检查
    if (N_SCAN <= 0 || Horizon_SCAN <= 0) {
        RCLCPP_ERROR(get_logger(), "Invalid N_SCAN(%d) or Horizon_SCAN(%d)", 
                     N_SCAN, Horizon_SCAN);
        return;
    }
    // Initialize point cloud pointers
    laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
    organizedCloudIn.reset(new pcl::PointCloud<pcl::PointXYZI>());
    organizedCloudIn64.reset(new pcl::PointCloud<pcl::PointXYZI>());
    furthestRing.reset(new pcl::PointCloud<pcl::PointXYZI>());
    transformedFurthestRing.reset(new pcl::PointCloud<pcl::PointXYZI>());
    transformed_pc.reset(new pcl::PointCloud<pcl::PointXYZI>());
    map_pc.reset(new pcl::PointCloud<pcl::PointXYZI>());
    mapCorridorEnlarge_pc.reset(new pcl::PointCloud<pcl::PointXYZI>());
    ringMapP1.reset(new pcl::PointCloud<pcl::PointXYZI>());
    ringMapP2.reset(new pcl::PointCloud<pcl::PointXYZI>());
    intersectionOnMap.reset(new pcl::PointCloud<pcl::PointXYZI>());
    UsefulPoints1.reset(new pcl::PointCloud<pcl::PointXYZI>());
    UsefulPoints2.reset(new pcl::PointCloud<pcl::PointXYZI>());
    laserUppestRing.reset(new pcl::PointCloud<pcl::PointXYZI>());
    potentialCeilingPoints.reset(new pcl::PointCloud<pcl::PointXYZI>());
    insidePC.reset(new pcl::PointCloud<pcl::PointXYZI>());
    outsidePC.reset(new pcl::PointCloud<pcl::PointXYZI>());

    // Pre-allocate vectors with default sizes where appropriate
    mapHistogram.reserve(1000);  // 预估大小，可根据实际情况调整
    weightsTurkey.reserve(1000);
    Vec_pcx.reserve(1000);
    Vec_pcy.reserve(1000);
    Vec_pedalx.reserve(1000);
    Vec_pedaly.reserve(1000);

    RCLCPP_INFO(this->get_logger(), "Memory allocated for point clouds and vectors");
}

void CloudBase::resetParameters() {
    {
        std::lock_guard<std::mutex> lock(agIndexMutex);
        AGindexReceived = false;
    }
    // Clear or resize point clouds
    laserCloudIn->clear();
    organizedCloudIn->clear();
    furthestRing->clear();
    transformed_pc->clear();
    UsefulPoints1->clear();
    UsefulPoints2->clear();
    
    // Reset vectors
    usefulIndex.clear();
    outsideAreaIndexRecord.clear();
    outsideAreaLastRingIndexRecord.clear();
    
    // Reset counters and flags
    numIcpPoints = 0;
    weightSumTurkey = 0;
    weightSumCauchy = 0;
    
    // Reset other parameters
    accumulateAngle = 0;
    averDistancePairedPoints = 0;
    
    RCLCPP_DEBUG(this->get_logger(), "Parameters reset");
}

bool CloudBase::isAGIndexReceived() const {
    std::lock_guard<std::mutex> lock(agIndexMutex);
    return AGindexReceived;
}
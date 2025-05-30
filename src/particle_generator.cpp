/**
 * @file particle_generator.cpp
 * @brief 核心功能：为全局定位生成初始粒子猜测
 * @author Jiajie Zhang
 * @date 2024-11-09
 */

// 关键点：
    // - 根据WiFi定位的粗略位置生成候选粒子
    // - 对每个粒子检查是否在Area Graph的有效区域内
    // - 发布 "/particles_for_init" 话题

#include "localization_using_area_graph/particle_generator.hpp"

// 初始化部分
ParticleGenerator::ParticleGenerator()
    : Node("particle_generator"),
      received_wifi_location_(false),  // 新增标记变量
      gen_(rd_()),
      dist_(0.0, 1.0)
{
    initializeParameters();
    initializePublishersSubscribers();

    RCLCPP_INFO(this->get_logger(), "ParticleGenerator initialized");
}

// 参数配置
void ParticleGenerator::initializeParameters()
{
    // Declare and get parameters
    this->declare_parameter("particle_generator_step", 2.0);  // 粒子采样步长
    this->declare_parameter("particle_generator_radius", 5.0);  //搜索半径
    this->declare_parameter("bRescueRobot", false);
    this->declare_parameter("use_room_info", true);  // 是否使用room信息来过滤粒子
    this->declare_parameter("boundary_filter_threshold", 0.5);  // 边界过滤阈值（米）
    this->declare_parameter("root_long", 0.0);
    this->declare_parameter("root_lat", 0.0);
    this->declare_parameter("pointCloudTopic", "/hesai/pandar");

    // 声明地图变换参数
    this->declare_parameter("mapExtrinsicTrans", std::vector<double>{0.0, 0.0, 0.0});
    this->declare_parameter("mapYawAngle", 0.0);

    // 读取参数值
    step_ = this->get_parameter("particle_generator_step").as_double();
    radius_ = this->get_parameter("particle_generator_radius").as_double();
    bRescueRobot_ = this->get_parameter("bRescueRobot").as_bool();
    use_room_info_ = this->get_parameter("use_room_info").as_bool();
    boundary_filter_threshold_ = this->get_parameter("boundary_filter_threshold").as_double();
    root_longitude_ = this->get_parameter("root_long").as_double();
    root_latitude_ = this->get_parameter("root_lat").as_double();
    pointCloudTopic_ = this->get_parameter("pointCloudTopic").as_string();

    // 读取地图变换参数
    auto trans_vec = this->get_parameter("mapExtrinsicTrans").as_double_array();
    if (trans_vec.size() >= 3) {
        map_extrinsic_trans_ = {trans_vec[0], trans_vec[1], trans_vec[2]};
    }
    map_yaw_angle_ = this->get_parameter("mapYawAngle").as_double() * M_PI / 180.0;  // 转换为弧度

    RCLCPP_INFO(this->get_logger(), "Loaded parameters - step: %.2f, radius: %.2f, boundary_filter: %.2f",
                step_, radius_, boundary_filter_threshold_);
    RCLCPP_INFO(this->get_logger(), "Map transform - trans: [%.1f, %.1f, %.1f], yaw: %.1f deg",
                map_extrinsic_trans_[0], map_extrinsic_trans_[1], map_extrinsic_trans_[2],
                map_yaw_angle_ * 180.0 / M_PI);
}

// 发布者订阅者初始化
void ParticleGenerator::initializePublishersSubscribers()
{
    // 创建QoS配置，使用可靠传输以确保消息不会丢失
    auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10))
        .reliable()
        .transient_local();  // 持久订阅，可以接收到连接前的消息

    // 普通消息的QoS
    auto best_effort_qos = rclcpp::QoS(rclcpp::KeepLast(10));

    // 发布用于初始化的采样粒子
    particle_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/particles_for_init", reliable_qos);

    // 发布用于可视化的粒子
    viz_particle_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/particles_for_viz", reliable_qos);

    // 发布WiFi中心点标记
    wifi_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/wifi_center_marker", reliable_qos);

    // 发布房间位置标记
    room_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/room_marker", reliable_qos);

    // 订阅1: LiDAR 点云话题
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        pointCloudTopic_, best_effort_qos,
        std::bind(&ParticleGenerator::lidarCallback, this, std::placeholders::_1));

    // 订阅2: osmAG地图中的node点云（经由mapAGCB处理，其坐标数值已被变换到map坐标系下）
    agmap_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/pubAGMapTransformedPC", best_effort_qos,
        std::bind(&ParticleGenerator::agmapCallback, this, std::placeholders::_1));

    // 订阅3: WiFi定位结果 - 使用与robot_loc兼容的QoS设置
    // 注意：不使用transient_local，因为它可能与发布者不兼容
    auto wifi_qos = rclcpp::QoS(rclcpp::KeepLast(10))
        .reliable()
        .durability_volatile();  // 使用volatile而非transient_local

    wifi_sub_ = this->create_subscription<rss::msg::WifiLocation>(
        "/WifiLocation", wifi_qos,
        std::bind(&ParticleGenerator::wifiCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "创建了 /WifiLocation 话题的订阅，使用可靠消息策略");
}

Eigen::Vector3d CoordinateTransform(const GeoCoordinate& init, const GeoCoordinate& cur) {
    std::array<double, 2> reference{init.latitude, init.longitude};
    std::array<double, 2> current{cur.latitude, cur.longitude};
    std::array<double, 2> cur_xy;

    // 使用WGS84转换，不考虑旋转
    cur_xy = wgs84::toCartesian(reference, current);

    // 简单处理高度差，如果没有高度信息就默认为0
    double z = (cur.altitude - init.altitude);

    return {cur_xy[0], cur_xy[1], z};
}



void ParticleGenerator::wifiCallback(const rss::msg::WifiLocation::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "接收到WiFi定位消息: [%f, %f]",
                msg->longitude, msg->latitude);

    std::lock_guard<std::mutex> lock(wifi_mutex_);
    latest_wifi_location_ = msg;
    received_wifi_location_ = true;  // 标记已收到WiFi定位结果

    // 立即尝试生成粒子，如果已接收激光雷达数据
    static bool printed_waiting = false;
    if (!latest_lidar_received_) {
        if (!printed_waiting) {
            RCLCPP_INFO(this->get_logger(), "收到WiFi定位消息，但激光雷达数据尚未就绪，等待中...");
            printed_waiting = true;
        }
    }
}

void ParticleGenerator::lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // 检查是否已收到WiFi定位结果
    {
        std::lock_guard<std::mutex> lock(wifi_mutex_);
        // 如果已经发布过粒子，直接返回
        if (particles_published_) {
            return;
        }
        if (!received_wifi_location_) {
            static int wait_count = 0;
            if (wait_count % 10 == 0) { // 每10次打印一次日志，减少日志冗余
                RCLCPP_INFO(this->get_logger(), "等待WiFi定位结果...已等待%d次", wait_count);
            }
            wait_count++;
            return;  // 如果还没有收到WiFi定位结果，直接返回
        }
        // 收到了WiFi定位结果，标记激光雷达数据已接收
        latest_lidar_received_ = true;
        RCLCPP_INFO(this->get_logger(), "收到WiFi定位和激光雷达数据，开始生成粒子");
    }

    std::array<double, 2> wifi_center;

    // 获取最新的WiFi定位结果
    {
        std::lock_guard<std::mutex> lock(wifi_mutex_);
        GeoCoordinate root_coord;
        root_coord.longitude = root_longitude_;
        root_coord.latitude = root_latitude_;
        root_coord.altitude = 0.0;  // 高度设为0

        GeoCoordinate wifi_coord;
        wifi_coord.longitude = latest_wifi_location_->longitude;
        wifi_coord.latitude = latest_wifi_location_->latitude;
        wifi_coord.altitude = latest_wifi_location_->altitude;
        // TODO, 以下三个信息都还没有被用过
        wifi_coord.room_longitude = latest_wifi_location_->room_long;
        wifi_coord.room_latitude = latest_wifi_location_->room_lat;
        wifi_coord.floor = latest_wifi_location_->floor;

        // 如果启用了room信息，计算room坐标所在的Area ID
        RCLCPP_DEBUG(this->get_logger(), "当前use_room_info_参数值: %s", use_room_info_ ? "true" : "false");

        if (use_room_info_) {
            GeoCoordinate room_coord;
            room_coord.longitude = latest_wifi_location_->room_long;
            room_coord.latitude = latest_wifi_location_->room_lat;
            room_coord.altitude = latest_wifi_location_->altitude;

            RCLCPP_DEBUG(this->get_logger(), "Room经纬度信息 - 经度: %.6f, 纬度: %.6f",
                         room_coord.longitude, room_coord.latitude);

            // 将room的经纬度转换为局部坐标
            Eigen::Vector3d room_local = CoordinateTransform(root_coord, room_coord);

            // 应用AGmap到map的变换
            // 1. 旋转变换
            double cos_yaw = std::cos(map_yaw_angle_);
            double sin_yaw = std::sin(map_yaw_angle_);
            double x_rotated = room_local.x() * cos_yaw - room_local.y() * sin_yaw;
            double y_rotated = room_local.x() * sin_yaw + room_local.y() * cos_yaw;

            // 2. 平移变换
            Eigen::Vector2d room_point(
                x_rotated + map_extrinsic_trans_[0],
                y_rotated + map_extrinsic_trans_[1]
            );

            RCLCPP_DEBUG(this->get_logger(), "Room局部坐标(变换前) - x: %.2f, y: %.2f",
                         room_local.x(), room_local.y());
            RCLCPP_DEBUG(this->get_logger(), "Room局部坐标(变换后) - x: %.2f, y: %.2f",
                         room_point.x(), room_point.y());

            // 判断点在哪个Area内
            room_area_id_ = -1;
            RCLCPP_DEBUG(this->get_logger(), "AGmaps_大小: %zu", AGmaps_.size());

            for (size_t i = 0; i < AGmaps_.size(); i++) {
                bool intersects = checkIntersection(room_point, AGmaps_[i]);
                RCLCPP_DEBUG(this->get_logger(), "检查Area %zu: %s", i, intersects ? "在区域内" : "不在区域内");

                if (intersects) {
                    room_area_id_ = i;
                    RCLCPP_DEBUG(this->get_logger(), "Room坐标对应的Area ID: %zu", room_area_id_);
                    break;
                }
            }

            if (room_area_id_ == INVALID_AREA_ID) {
                RCLCPP_WARN(this->get_logger(), "未找到Room坐标所在的Area");
            } else {
                // 发布房间位置标记
                publishRoomMarker(msg->header.stamp, room_point.x(), room_point.y());
            }
        }

        // 转换为AGmap局部坐标
        Eigen::Vector3d local_pos = CoordinateTransform(root_coord, wifi_coord);

        // 应用AGmap到map的变换
        // 1. 旋转变换
        double cos_yaw = std::cos(map_yaw_angle_);
        double sin_yaw = std::sin(map_yaw_angle_);
        double x_rotated = local_pos.x() * cos_yaw - local_pos.y() * sin_yaw;
        double y_rotated = local_pos.x() * sin_yaw + local_pos.y() * cos_yaw;

        // 2. 平移变换
        wifi_center = {
            x_rotated + map_extrinsic_trans_[0],
            y_rotated + map_extrinsic_trans_[1]
        };
        // 只在首次收到WiFi定位结果时打印信息
        static bool first_wifi_location = true;
        if (first_wifi_location) {
            RCLCPP_INFO_ONCE(this->get_logger(), "变换后的WiFi center: [%.2f, %.2f]", wifi_center[0], wifi_center[1]);
            first_wifi_location = false;
        }

        // 创建并发布WiFi中心点标记
        publishWifiCenterMarker(msg->header.stamp, wifi_center[0], wifi_center[1]);
    }

    // 添加噪声
    wifi_center[0] += 0.25 * dist_(gen_);
    wifi_center[1] += 0.25 * dist_(gen_);

    // 生成并发布粒子
    generateParticles(msg->header.stamp, wifi_center);
}

void ParticleGenerator::generateParticles(const rclcpp::Time& stamp,
                                        const std::array<double, 2>& wifi_center)
{
    // 创建PCL点云对象
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

    int total_candidates = 0;
    int filtered_by_boundary = 0;
    int filtered_by_room = 0;
    int generated_particles = 0;

    for (double x = wifi_center[0] - radius_; x <= wifi_center[0] + radius_; x += 1.0/step_) {
        for (double y = wifi_center[1] - radius_; y <= wifi_center[1] + radius_; y += 1.0/step_) {
            Eigen::Vector2d point(x, y);

            // 1. 检查是否在搜索圆内
            if ((point - Eigen::Vector2d(wifi_center[0], wifi_center[1])).norm() > radius_) {
                continue;
            }

            total_candidates++;

            // 2. 检查每个粒子是否在Area Graph的有效区域内
            for (size_t i = 0; i < AGmaps_.size(); i++) {
                if (checkIntersection(point, AGmaps_[i])) {
                    // 3. 如果启用了room信息且找到了room所在的Area，只生成该Area内的粒子
                    if (use_room_info_ && room_area_id_ != INVALID_AREA_ID && i != room_area_id_) {
                        RCLCPP_DEBUG(this->get_logger(), "跳过Area %zu的粒子，因为不是Room所在的Area(%zu)", i, room_area_id_);
                        filtered_by_room++;
                        continue;
                    }

                    // 4. 边界过滤：检查粒子是否过于接近Area边界
                    if (isNearAreaBoundary(point, AGmaps_[i], boundary_filter_threshold_)) {
                        double distance = distanceToPolygonBoundary(point, AGmaps_[i]);
                        RCLCPP_DEBUG(this->get_logger(), "过滤边界粒子: Area %zu, 位置(%.2f, %.2f), 距离边界%.3fm < 阈值%.3fm",
                                    i, x, y, distance, boundary_filter_threshold_);
                        filtered_by_boundary++;
                        continue;
                    }

                    // 5. 通过所有过滤条件，生成粒子
                    double distance = distanceToPolygonBoundary(point, AGmaps_[i]);
                    RCLCPP_DEBUG(this->get_logger(), "生成粒子: Area %zu, 位置(%.2f, %.2f), 距离边界%.3fm",
                                i, x, y, distance);

                    pcl::PointXYZI p;
                    p.x = x;
                    p.y = y;
                    p.z = i;  // Area索引
                    p.intensity = 1.0;
                    cloud->points.push_back(p);
                    generated_particles++;
                }
            }
        }
    }

    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;

    // 创建一个用于可视化的点云副本，将z值设为0
    pcl::PointCloud<pcl::PointXYZI>::Ptr viz_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    viz_cloud->points.reserve(cloud->points.size());

    for (const auto& point : cloud->points) {
        pcl::PointXYZI viz_point = point;
        viz_point.z = 0.0;  // 可视化时z值设为0
        viz_cloud->points.push_back(viz_point);
    }

    viz_cloud->width = viz_cloud->points.size();
    viz_cloud->height = 1;
    viz_cloud->is_dense = true;

    // 转换为ROS消息并发布原始粒子（保持不变）
    sensor_msgs::msg::PointCloud2 particles_msg;
    pcl::toROSMsg(*cloud, particles_msg);
    particles_msg.header.stamp = stamp;
    particles_msg.header.frame_id = "map";

    // Publish original particles
    particle_pub_->publish(particles_msg);

    // 转换为ROS消息并发布可视化粒子
    sensor_msgs::msg::PointCloud2 viz_particles_msg;
    pcl::toROSMsg(*viz_cloud, viz_particles_msg);
    viz_particles_msg.header.stamp = stamp;
    viz_particles_msg.header.frame_id = "map";

    // Publish visualization particles
    viz_particle_pub_->publish(viz_particles_msg);

    // 标记粒子已发布
    particles_published_ = true;

    // 输出过滤统计信息
    RCLCPP_INFO(this->get_logger(),
        "粒子生成完成 - 候选位置: %d, 边界过滤: %d, 房间过滤: %d, 最终生成: %d",
        total_candidates, filtered_by_boundary, filtered_by_room, generated_particles);
    RCLCPP_INFO(this->get_logger(),
        "边界过滤阈值: %.2fm, 过滤率: %.1f%%",
        boundary_filter_threshold_,
        total_candidates > 0 ? (filtered_by_boundary * 100.0 / total_candidates) : 0.0);
    RCLCPP_INFO(this->get_logger(), "粒子已生成并发布，后续不再更新粒子");
}

// 它所订阅的是已经转换到map坐标系下的Area Graph的 node point
void ParticleGenerator::agmapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // Convert point cloud to PCL format
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *cloud);

    // Process AG map points
    std::vector<Eigen::Vector2d> area;
    AGmaps_.clear();

    for (const auto& point : cloud->points) {
        // intensity % 3 = 0 --- Area的起始点
        if (static_cast<int>(point.intensity) % 3 == 0) {
            area.clear();
            area.emplace_back(point.x, point.y);
        // intensity % 3 = 1 --- Area的中间点
        } else if (static_cast<int>(point.intensity) % 3 == 1) {
            area.emplace_back(point.x, point.y);
        // intensity % 3 = 2 --- Area的结束点
        } else if (static_cast<int>(point.intensity) % 3 == 2) {
            area.emplace_back(point.x, point.y);
            AGmaps_.push_back(area);
        }
    }
}

// 发布WiFi中心点标记
void ParticleGenerator::publishWifiCenterMarker(const rclcpp::Time& stamp, double x, double y) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = stamp;
    marker.ns = "wifi_center";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // 设置位置
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // 设置大小（直径为0.5米的球）
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    // 设置颜色（绿色）
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;  // 不透明

    // 设置持续时间（秒）
    marker.lifetime = rclcpp::Duration(0, 0);  // 0表示永久存在

    // 发布标记
    wifi_marker_pub_->publish(marker);
}

bool ParticleGenerator::checkIntersection(const Eigen::Vector2d& point,
                                        const std::vector<Eigen::Vector2d>& polygon) {
    if (polygon.size() < 3) return false;

    int intersections = 0;
    size_t j = polygon.size() - 1;

    // 使用射线投射算法
    for (size_t i = 0; i < polygon.size(); i++) {
        // 检查点是否在多边形边界上
        if ((polygon[i][1] > point[1]) != (polygon[j][1] > point[1])) {
            double slope = (polygon[j][0] - polygon[i][0]) * (point[1] - polygon[i][1]) /
                         (polygon[j][1] - polygon[i][1]) + polygon[i][0];

            if (point[0] < slope)
                intersections++;
        }
        j = i;
    }

    // 奇数个交点表示点在多边形内部
    return (intersections % 2) == 1;
}

// 发布房间位置标记
void ParticleGenerator::publishRoomMarker(const rclcpp::Time& stamp, double x, double y)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = stamp;
    marker.ns = "room_position";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;  // 黄色
    marker.color.a = 1.0;

    marker.lifetime = rclcpp::Duration(0, 0);  // 0表示永久存在

    room_marker_pub_->publish(marker);
}

// 检查点是否过于接近Area边界
bool ParticleGenerator::isNearAreaBoundary(const Eigen::Vector2d& point,
                                         const std::vector<Eigen::Vector2d>& polygon,
                                         double threshold) {
    double distance = distanceToPolygonBoundary(point, polygon);
    return distance < threshold;
}

// 计算点到多边形边界的最短距离
double ParticleGenerator::distanceToPolygonBoundary(const Eigen::Vector2d& point,
                                                   const std::vector<Eigen::Vector2d>& polygon) {
    if (polygon.size() < 3) return std::numeric_limits<double>::max();

    double min_distance = std::numeric_limits<double>::max();

    // 遍历多边形的每条边
    for (size_t i = 0; i < polygon.size(); i++) {
        size_t j = (i + 1) % polygon.size();

        const Eigen::Vector2d& p1 = polygon[i];
        const Eigen::Vector2d& p2 = polygon[j];

        // 计算点到线段的距离
        Eigen::Vector2d line_vec = p2 - p1;
        Eigen::Vector2d point_vec = point - p1;

        // 计算投影参数t
        double line_length_sq = line_vec.squaredNorm();
        if (line_length_sq < 1e-10) {
            // 线段退化为点
            min_distance = std::min(min_distance, (point - p1).norm());
            continue;
        }

        double t = point_vec.dot(line_vec) / line_length_sq;

        Eigen::Vector2d closest_point;
        if (t < 0.0) {
            // 最近点是线段起点
            closest_point = p1;
        } else if (t > 1.0) {
            // 最近点是线段终点
            closest_point = p2;
        } else {
            // 最近点在线段上
            closest_point = p1 + t * line_vec;
        }

        double distance = (point - closest_point).norm();
        min_distance = std::min(min_distance, distance);
    }

    return min_distance;
}
#include "localization_using_area_graph/agloc_localizer_node.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <memory>
#include <string>
#include <utility>
#include <chrono>
#include "nav2_util/node_utils.hpp"

namespace localization_using_area_graph
{

AGLocLocalizerNode::AGLocLocalizerNode(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("agloc_localizer", "", options),
  initialized_(false)
{
  RCLCPP_INFO(get_logger(), "创建AGLoc定位器");
}

AGLocLocalizerNode::~AGLocLocalizerNode()
{
  RCLCPP_INFO(get_logger(), "销毁AGLoc定位器");
}

nav2_util::CallbackReturn AGLocLocalizerNode::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "配置AGLoc定位器");
  
  // 获取参数
  global_frame_id_ = declare_parameter<std::string>("global_frame_id", "map");
  odom_frame_id_ = declare_parameter<std::string>("odom_frame_id", "odom");
  base_frame_id_ = declare_parameter<std::string>("base_frame_id", "base_link");
  tf_broadcast_ = declare_parameter<bool>("tf_broadcast", true);
  
  // 为nav2_util::LifecycleNode基类的bond连接设置超时参数
  this->declare_parameter("bond_timeout", 4.0);
  
  // 创建 TF 组件
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(shared_from_this());

  // 创建发布器
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "agloc_pose", rclcpp::QoS(1).reliable());
  particle_cloud_pub_ = create_publisher<nav2_msgs::msg::ParticleCloud>(
    "particle_cloud", rclcpp::QoS(1).reliable());
    
  // 创建初始位姿转发发布器
  initial_pose_forward_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/initialpose_agloc", rclcpp::QoS(1).reliable());

  // 创建初始位姿订阅者
  initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", rclcpp::QoS(1).reliable(),
    std::bind(&AGLocLocalizerNode::initialPoseCallback, this, std::placeholders::_1));

  // 适配器模式：监听现有cloud_handler节点的位姿输出
  // 根据AGLoc系统的实际输出话题订阅位姿信息
  agloc_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/cloud_handler/pose", rclcpp::QoS(1).reliable(),
    std::bind(&AGLocLocalizerNode::agloc_pose_callback, this, std::placeholders::_1));

  // 创建定时器，定期发布位姿和TF (10Hz)
  publish_timer_ = create_wall_timer(
    std::chrono::milliseconds(100), 
    std::bind(&AGLocLocalizerNode::publish_timer_callback, this));

  // 使用nav2_util::LifecycleNode的bond连接管理机制
  // 这会自动创建bond连接，并与lifecycle_manager正确通信
  double bond_timeout = this->get_parameter("bond_timeout").as_double();
  RCLCPP_INFO(get_logger(), "设置bond超时时间: %.2f秒", bond_timeout);
  
  // 注意：不需要手动创建bond连接，在on_activate中会自动调用createBond()

  RCLCPP_INFO(get_logger(), "AGLoc定位器适配器已初始化，监听cloud_handler节点的位姿输出");
  RCLCPP_INFO(get_logger(), "订阅话题: /cloud_handler/pose");
  initialized_ = true;
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn AGLocLocalizerNode::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "激活AGLoc定位器");
  
  if (!initialized_) {
    RCLCPP_ERROR(get_logger(), "无法激活，未初始化");
    return nav2_util::CallbackReturn::FAILURE;
  }
  
  // 激活发布器
  pose_pub_->on_activate();
  particle_cloud_pub_->on_activate();
  initial_pose_forward_pub_->on_activate();
  
  // 使用基类的createBond()方法创建bond连接
  createBond();
  
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn AGLocLocalizerNode::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "停用AGLoc定位器");
  
  if (!initialized_) {
    return nav2_util::CallbackReturn::SUCCESS;
  }
  
  // 停用发布器
  pose_pub_->on_deactivate();
  particle_cloud_pub_->on_deactivate();
  initial_pose_forward_pub_->on_deactivate();
  
  // 使用基类的destroyBond()方法断开bond连接
  destroyBond();
  
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn AGLocLocalizerNode::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "清理AGLoc定位器");
  
  if (!initialized_) {
    return nav2_util::CallbackReturn::SUCCESS;
  }
  
  // 清理资源
  pose_pub_.reset();
  particle_cloud_pub_.reset();
  initial_pose_sub_.reset();
  agloc_pose_sub_.reset();
  initial_pose_forward_pub_.reset();
  
  // 清理TF相关资源
  tf_buffer_.reset();
  tf_listener_.reset();
  tf_broadcaster_.reset();
  
  // 使用基类的destroyBond()方法清理bond连接
  destroyBond();
  
  initialized_ = false;
  has_pose_ = false;
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn AGLocLocalizerNode::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "关闭AGLoc定位器");
  return nav2_util::CallbackReturn::SUCCESS;
}

geometry_msgs::msg::PoseWithCovarianceStamped AGLocLocalizerNode::getPoseEstimate()
{
  // 返回最新的位姿估计
  return current_pose_;
}

// AGLoc点云处理节点位姿输出的回调函数
void AGLocLocalizerNode::agloc_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (!initialized_) {
    return;
  }
  
  // 将map->雷达的位姿转换为map->base_link的位姿
  // 首先，从PoseStamped创建tf2变换
  tf2::Transform T_map_lidar;
  tf2::fromMsg(msg->pose, T_map_lidar);
  
  // 定义雷达到base_link的静态变换
  // 根据提供的参数：x=0.34058, y=0, z=0.3465, qz=0.70710678, qw=0.70710678
  tf2::Transform T_lidar_base;
  tf2::Vector3 translation(-0.34058, 0.0, -0.3465); // 注意这里取反，因为我们需要从雷达到base_link
  tf2::Quaternion rotation(0.0, 0.0, -0.70710678, 0.70710678); // 同样取反  0.0, 0.0, -0.70710678, 0.70710678
  rotation.normalize();
  T_lidar_base.setOrigin(translation);
  T_lidar_base.setRotation(rotation);
  
  // 计算map->base_link的变换
  // 从AGLoc系统中得到的robotPose事实上是map->hesai_lidar的关系
  tf2::Transform T_map_base = T_map_lidar * T_lidar_base;
  
  // 转换为PoseWithCovariance格式
  geometry_msgs::msg::PoseWithCovarianceStamped pose_with_cov;
  pose_with_cov.header = msg->header;
  tf2::toMsg(T_map_base, pose_with_cov.pose.pose);
  
  // 设置协方差（使用固定值）
  for (int i = 0; i < 36; i++) {
    pose_with_cov.pose.covariance[i] = 0.0;
  }
  // 位置协方差
  pose_with_cov.pose.covariance[0] = 0.05;  // x
  pose_with_cov.pose.covariance[7] = 0.05;   // y
  pose_with_cov.pose.covariance[14] = 0.05;  // z
  // 旋转协方差
  pose_with_cov.pose.covariance[21] = 0.1;  // roll
  pose_with_cov.pose.covariance[28] = 0.1;  // pitch
  pose_with_cov.pose.covariance[35] = 0.1;  // yaw
  
  // 保存当前位姿
  current_pose_ = pose_with_cov;
  has_pose_ = true;
  
  RCLCPP_DEBUG(get_logger(), "收到AGLoc的位姿更新(转换到base_link): x=%.2f, y=%.2f", 
               pose_with_cov.pose.pose.position.x, pose_with_cov.pose.pose.position.y);
}

// 定时器回调函数，发布位姿和TF
void AGLocLocalizerNode::publish_timer_callback()
{
  if (!initialized_) {
    return;
  }
  
  // 即使还没有收到位姿更新，也尝试发布初始化的map->odom变换
  // 这样可以确保在启动时TF树不会断开
  publishPoseEstimate();
  
  // 只有收到位姿更新后才发布粒子云
  // TODO 这段代码不应该存在
  if (has_pose_) {
    publishParticleCloud();
  }
}

// AGLoc使用自己的初始位姿设置逻辑，不需要外部initialpose

void AGLocLocalizerNode::publishPoseEstimate()
{
  if (!initialized_) {
    return;
  }
  
  // 更新时间戳
  auto current_time = this->now();
  
  // 只有当有位姿数据时才发布位姿
  if (has_pose_) {
    current_pose_.header.stamp = current_time;
    pose_pub_->publish(current_pose_);
  }
  
  // 如果不需要发布TF，直接返回
  if (!tf_broadcast_) {
    return;
  }
  
  // 获取odom到base_link的变换
  geometry_msgs::msg::TransformStamped odom_to_base;
  try {
    // 使用超时等待变换可用
    // 这里设置100ms的超时，并使用当前时间而非零时间
    auto timeout = tf2::durationFromSec(0.1);
    auto current_time = this->now();
    
    // 尝试获取最新的变换
    odom_to_base = tf_buffer_->lookupTransform(
      odom_frame_id_, base_frame_id_, current_time, timeout);
      
    RCLCPP_DEBUG(get_logger(), "成功获取odom->base_link变换");
  } catch (tf2::TransformException & e) {
    // 如果失败，尝试获取最新可用的变换
    try {
      RCLCPP_DEBUG(get_logger(), "尝试获取最新的odom->base_link变换");
      odom_to_base = tf_buffer_->lookupTransform(
        odom_frame_id_, base_frame_id_, tf2::TimePointZero, tf2::durationFromSec(0.1));
      RCLCPP_DEBUG(get_logger(), "成功获取odom->base_link变换");
    } catch (tf2::TransformException & e2) {
      RCLCPP_ERROR(get_logger(), "获取odom到base变换失败: %s", e2.what());
      
      // 如果仍然失败，使用单位变换作为默认值
      odom_to_base.header.stamp = this->now();
      odom_to_base.header.frame_id = odom_frame_id_;
      odom_to_base.child_frame_id = base_frame_id_;
      odom_to_base.transform.translation.x = 0.0;
      odom_to_base.transform.translation.y = 0.0;
      odom_to_base.transform.translation.z = 0.0;
      odom_to_base.transform.rotation.x = 0.0;
      odom_to_base.transform.rotation.y = 0.0;
      odom_to_base.transform.rotation.z = 0.0;
      odom_to_base.transform.rotation.w = 1.0;
      
      RCLCPP_WARN(get_logger(), "使用默认的odom->base_link变换");
    }
  }
  
  // 计算map到odom的变换
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = current_time;
  tf_msg.header.frame_id = global_frame_id_;
  tf_msg.child_frame_id = odom_frame_id_;
  
  // T_m_o = T_m_b * T_b_o^(-1)
  tf2::Transform T_m_b;
  tf2::Transform T_b_o;
  
  if (has_pose_) {
    // 如果已有位姿数据，从current_pose_设置map到base的变换
    tf2::Vector3 t_m_b(current_pose_.pose.pose.position.x, 
                     current_pose_.pose.pose.position.y,
                     current_pose_.pose.pose.position.z);
                     
    tf2::Quaternion q_m_b;
    tf2::fromMsg(current_pose_.pose.pose.orientation, q_m_b);
    
    T_m_b.setOrigin(t_m_b);
    T_m_b.setRotation(q_m_b);
  } else {
    // 如果还没有位姿数据，使用单位矩阵作为初始值
    // 即map和base_link初始重合
    T_m_b.setIdentity();
  }
  
  // 设置base到odom的变换
  tf2::fromMsg(odom_to_base.transform, T_b_o);
  
  // 计算map到odom的变换
  tf2::Transform T_m_o = T_m_b * T_b_o.inverse();
  
  // 转换为消息格式
  tf2::toMsg(T_m_o, tf_msg.transform);
  
  // 发布TF
  tf_broadcaster_->sendTransform(tf_msg);
  
  RCLCPP_INFO_ONCE(get_logger(), "已发布TF变换: map->odom (使用%s位姿)", 
               has_pose_ ? "实际" : "初始化");
}

void AGLocLocalizerNode::publishParticleCloud()
{
  if (!initialized_) {
    return;
  }
  
  // 简单实现，发布一个空的粒子云
  // 如果AGLoc支持粒子表示，可以从CloudHandler获取
  nav2_msgs::msg::ParticleCloud cloud_msg;
  cloud_msg.header.stamp = this->now();
  cloud_msg.header.frame_id = global_frame_id_;
  
  particle_cloud_pub_->publish(cloud_msg);
}

// 初始位姿回调函数
void AGLocLocalizerNode::initialPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "收到初始位姿设置请求");
  
  if (!initialized_) {
    RCLCPP_WARN(get_logger(), "AGLoc定位器未初始化，无法设置初始位姿");
    return;
  }
  
  // 设置手动初始化标志
  manual_init_ = true;
  
  // 修改消息的帧和时间戳
  geometry_msgs::msg::PoseWithCovarianceStamped forward_msg = *msg;
  forward_msg.header.stamp = this->now();
  forward_msg.header.frame_id = global_frame_id_;
  
  // 转发初始位姿到cloud_handler节点
  initial_pose_forward_pub_->publish(forward_msg);
  
  double yaw = tf2::getYaw(msg->pose.pose.orientation);
  RCLCPP_INFO(get_logger(), "已转发初始位姿到cloud_handler节点: x=%.2f, y=%.2f, yaw=%.2f", 
              msg->pose.pose.position.x, msg->pose.pose.position.y, yaw);
}

}  // namespace localization_using_area_graph

// AGLoc定位器作为独立节点启动，复用Nav2的生命周期管理机制
int main(int argc, char ** argv)
{
  // 初始化ROS
  rclcpp::init(argc, argv);
  
  // 创建节点选项，启用节点成组功能以与Nav2集成
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);
  
  // 输出启动信息
  std::cout << "启动AGLoc定位器节点 - 等待lifecycle_manager管理..." << std::endl;
  
  // 创建并运行节点
  auto node = std::make_shared<localization_using_area_graph::AGLocLocalizerNode>(options);
  rclcpp::spin(node->get_node_base_interface());
  
  // 清理并退出
  rclcpp::shutdown();
  return 0;
}

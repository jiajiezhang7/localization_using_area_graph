#include "localization_using_area_graph/agloc_localizer_node.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <memory>
#include <string>
#include <utility>
#include <chrono>

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
  
  // 创建 TF 组件
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(shared_from_this());

  // 创建发布器
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "agloc_pose", rclcpp::QoS(1).reliable());
  particle_cloud_pub_ = create_publisher<nav2_msgs::msg::ParticleCloud>(
    "particle_cloud", rclcpp::QoS(1).reliable());

  // 创建初始位姿订阅者
  initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", rclcpp::QoS(1).reliable(),
    std::bind(&AGLocLocalizerNode::initialPoseCallback, this, std::placeholders::_1));

  // 创建 CloudHandler
  try {
    cloud_handler_ = std::make_shared<CloudHandler>();
    // 这里可以进行其他必要的初始化
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "创建CloudHandler失败: %s", e.what());
    return nav2_util::CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(get_logger(), "AGLoc定位器已初始化，使用CloudHandler处理点云数据");
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
  cloud_handler_.reset();
  
  initialized_ = false;
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

void AGLocLocalizerNode::updateState()
{
  if (!initialized_) {
    return;
  }

  // 更新位姿信息并发布
  publishPoseEstimate();
  publishParticleCloud();
}

// AGLoc使用自己的初始位姿设置逻辑，不需要外部initialpose

void AGLocLocalizerNode::publishPoseEstimate()
{
  if (!initialized_) {
    return;
  }
  
  // 获取当前时间
  auto current_time = this->now();
  
  // 获取当前位姿
  Eigen::Matrix4f current_pose = cloud_handler_->getRobotPose();
  
  // 转换为ROS消息格式 - 发布agloc_pose (Navigation2需要)
  geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
  pose_msg.header.stamp = current_time;
  pose_msg.header.frame_id = global_frame_id_;
  
  // 设置位置
  pose_msg.pose.pose.position.x = current_pose(0,3);
  pose_msg.pose.pose.position.y = current_pose(1,3);
  pose_msg.pose.pose.position.z = current_pose(2,3);
  
  // 从旋转矩阵转换为四元数
  Eigen::Matrix3f rot = current_pose.block<3,3>(0,0);
  Eigen::Quaternionf q(rot);
  pose_msg.pose.pose.orientation.w = q.w();
  pose_msg.pose.pose.orientation.x = q.x();
  pose_msg.pose.pose.orientation.y = q.y();
  pose_msg.pose.pose.orientation.z = q.z();
  
  // 设置协方差（使用固定值）
  for (int i = 0; i < 36; i++) {
    pose_msg.pose.covariance[i] = 0.0;
  }
  // 位置协方差
  pose_msg.pose.covariance[0] = 0.05;  // x
  pose_msg.pose.covariance[7] = 0.05;   // y
  pose_msg.pose.covariance[14] = 0.05;  // z
  // 旋转协方差
  pose_msg.pose.covariance[21] = 0.1;  // roll
  pose_msg.pose.covariance[28] = 0.1;  // pitch
  pose_msg.pose.covariance[35] = 0.1;  // yaw
  
  // 保存当前位姿用于getPoseEstimate
  current_pose_ = pose_msg;
  
  // 发布位姿
  pose_pub_->publish(pose_msg);
  
  // 如果不需要发布TF，直接返回
  if (!tf_broadcast_) {
    return;
  }
  
  // 获取odom到base_link的变换
  geometry_msgs::msg::TransformStamped odom_to_base;
  try {
    // 尝试获取最新的变换
    odom_to_base = tf_buffer_->lookupTransform(
      odom_frame_id_, base_frame_id_, tf2::TimePointZero);
  } catch (tf2::TransformException & e) {
    RCLCPP_ERROR(get_logger(), "获取odom到base变换失败: %s", e.what());
    RCLCPP_WARN(get_logger(), "无法计算map到odom的变换，跳过TF发布");
    return;
  }
  
  // 计算map到odom的变换
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = current_time;
  tf_msg.header.frame_id = global_frame_id_;
  tf_msg.child_frame_id = odom_frame_id_;
  
  // T_m_o = T_m_b * T_b_o^(-1)
  tf2::Transform T_m_b;
  tf2::Transform T_b_o;
  
  // 设置map到base的变换
  tf2::Vector3 t_m_b(current_pose(0,3), current_pose(1,3), current_pose(2,3));
  tf2::Matrix3x3 R_m_b;
  R_m_b.setValue(
    current_pose(0,0), current_pose(0,1), current_pose(0,2),
    current_pose(1,0), current_pose(1,1), current_pose(1,2),
    current_pose(2,0), current_pose(2,1), current_pose(2,2));
  T_m_b.setBasis(R_m_b);
  T_m_b.setOrigin(t_m_b);
  
  // 设置base到odom的变换
  tf2::fromMsg(odom_to_base.transform, T_b_o);
  
  // 计算map到odom的变换
  tf2::Transform T_m_o = T_m_b * T_b_o.inverse();
  
  // 转换为消息格式
  tf2::toMsg(T_m_o, tf_msg.transform);
  
  // 发布TF
  tf_broadcaster_->sendTransform(tf_msg);
  
  RCLCPP_DEBUG(get_logger(), "已发布位姿和TF变换: map->odom");
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
  RCLCPP_INFO(get_logger(), "收到初始位姿");
  
  if (!initialized_ || !cloud_handler_) {
    RCLCPP_WARN(get_logger(), "AGLoc定位器未初始化，无法设置初始位姿");
    return;
  }
  
  // 从消息中提取位姿
  double yaw = tf2::getYaw(msg->pose.pose.orientation);
  Eigen::Vector3f position(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  
  // 调用CloudHandler的setInitialPose方法
  cloud_handler_->setInitialPose(yaw, position);
  RCLCPP_INFO(get_logger(), "已设置初始位姿: x=%.2f, y=%.2f, yaw=%.2f", 
              position.x(), position.y(), yaw);
}

}  // namespace localization_using_area_graph

// 主函数
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto options = rclcpp::NodeOptions();
  auto node = std::make_shared<localization_using_area_graph::AGLocLocalizerNode>(options);
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}

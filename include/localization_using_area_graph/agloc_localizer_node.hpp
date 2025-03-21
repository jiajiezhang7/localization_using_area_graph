#ifndef AGLOC_LOCALIZER_NODE_HPP_
#define AGLOC_LOCALIZER_NODE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "nav2_util/lifecycle_node.hpp"
#include "bondcpp/bond.hpp"
#include "nav2_msgs/msg/particle_cloud.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/utils.h"
#include "sensor_msgs/msg/point_cloud2.hpp"

// 注意：不再包含CloudHandler.hpp，我们不直接使用它
// 而是监听现有cloud_handler节点的输出

namespace localization_using_area_graph
{

/**
 * @class AGLocLocalizerNode
 * @brief 适配器节点，提供Nav2兼容的生命周期接口，连接AGLoc系统与Nav2
 *
 * 此节点不直接处理点云，而是监听现有的cloud_handler节点的输出，
 * 将其转换为Nav2所需的格式，并提供生命周期管理和标准接口。
 */
class AGLocLocalizerNode : public nav2_util::LifecycleNode
{
public:
  explicit AGLocLocalizerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~AGLocLocalizerNode();

  // 生命周期节点接口实现
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  // 定位更新接口
  geometry_msgs::msg::PoseWithCovarianceStamped getPoseEstimate();

protected:
  // AGLoc节点的位姿结果回调
  void agloc_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  
  // 用于初始定位的回调
  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    
  // 发布接口
  void publishPoseEstimate();
  void publishParticleCloud();

  // 位姿发布定时器回调
  void publish_timer_callback();

  // 发布器
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav2_msgs::msg::ParticleCloud>::SharedPtr particle_cloud_pub_;
  
  // 用于转发初始位姿的发布器
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_forward_pub_;

  // TF相关
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  tf2::Duration transform_tolerance_{std::chrono::duration_cast<tf2::Duration>(std::chrono::duration<double>(0.1))};
  bool tf_broadcast_{true};

  // 坐标系相关
  std::string global_frame_id_{""};
  std::string odom_frame_id_{""};
  std::string base_frame_id_{""};
  
  // 订阅者
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr agloc_pose_sub_;
  
  // 保存当前位姿
  geometry_msgs::msg::PoseWithCovarianceStamped current_pose_;
  bool has_pose_{false};
  
  // 发布定时器
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // 使用nav2_util::LifecycleNode基类提供的bond连接管理机制
  // 无需手动管理bond连接

  // 状态相关
  bool initialized_{false};
  bool manual_init_{false};  // 标记是否手动设置了初始位姿
};

}  // namespace localization_using_area_graph

#endif  // AGLOC_LOCALIZER_NODE_HPP_

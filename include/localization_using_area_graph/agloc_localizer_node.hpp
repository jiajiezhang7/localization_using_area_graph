#ifndef AGLOC_LOCALIZER_NODE_HPP_
#define AGLOC_LOCALIZER_NODE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "nav2_util/lifecycle_node.hpp"
#include "nav2_msgs/msg/particle_cloud.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/utils.h"

#include "localization_using_area_graph/cloudHandler.hpp"

namespace localization_using_area_graph
{

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

  void updateState();

protected:
  // 回调函数
  void initialPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    
  // 发布TF和位姿
  void publishPoseEstimate();
  void publishParticleCloud();

  // 参数和配置

  // 发布器
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav2_msgs::msg::ParticleCloud>::SharedPtr particle_cloud_pub_;

  // AGLoc组件 - 直接使用CloudHandler，它已经包含了CloudInitializer
  std::shared_ptr<CloudHandler> cloud_handler_;

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

  // 状态相关
  bool initialized_;
  geometry_msgs::msg::PoseWithCovarianceStamped current_pose_;

};

}  // namespace localization_using_area_graph

#endif  // AGLOC_LOCALIZER_NODE_HPP_

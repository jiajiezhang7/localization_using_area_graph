/**
 * @file odom_fusion.cpp
 * @author Jiajie Zhang (ROS2 port)
 * @brief 里程计与ICP融合算法实现
 * @version 0.1
 * @date 2024-12-02
 */

#include "localization_using_area_graph/odom_fusion.hpp"
#include <tf2/utils.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <cmath>

namespace agloc_fusion {

OdomFusion::OdomFusion(rclcpp::Node* node)
    : node_(node), logger_(node->get_logger()), gen_(rd_()) {

    // 从参数服务器获取参数
    odom_alpha1_ = node_->get_parameter("odom_alpha1").as_double();
    odom_alpha2_ = node_->get_parameter("odom_alpha2").as_double();
    odom_alpha3_ = node_->get_parameter("odom_alpha3").as_double();
    odom_alpha4_ = node_->get_parameter("odom_alpha4").as_double();

    icp_weight_ = node_->get_parameter("icp_weight").as_double();
    odom_weight_ = node_->get_parameter("odom_weight").as_double();
    adaptive_weight_ = node_->get_parameter("adaptive_weight").as_bool();

    enable_multi_hypothesis_ = node_->get_parameter("enable_multi_hypothesis").as_bool();
    max_hypotheses_ = node_->get_parameter("max_hypotheses").as_int();
    hypothesis_weight_threshold_ = node_->get_parameter("hypothesis_weight_threshold").as_double();
    debug_fusion_ = node_->get_parameter("debug_fusion").as_bool();

    // 初始化随机数分布
    normal_dist_ = std::normal_distribution<double>(0.0, 1.0);

    // 初始化假设集合
    if (enable_multi_hypothesis_) {
        hypotheses_.reserve(max_hypotheses_);
    }

    RCLCPP_INFO(logger_, "里程计融合模块初始化完成");
    RCLCPP_INFO(logger_, "运动噪声参数: α1=%.3f, α2=%.3f, α3=%.3f, α4=%.3f",
                odom_alpha1_, odom_alpha2_, odom_alpha3_, odom_alpha4_);
    RCLCPP_INFO(logger_, "融合权重: ICP=%.2f, Odom=%.2f, 自适应=%s",
                icp_weight_, odom_weight_, adaptive_weight_ ? "是" : "否");
}

void OdomFusion::updateOdometry(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
    // 保存上一帧数据
    previous_odom_ = latest_odom_;

    // 更新最新数据
    latest_odom_.position.x() = odom_msg->pose.pose.position.x;
    latest_odom_.position.y() = odom_msg->pose.pose.position.y;
    latest_odom_.position.z() = odom_msg->pose.pose.position.z;

    // 使用tf2_eigen进行四元数转换
    latest_odom_.orientation.x() = odom_msg->pose.pose.orientation.x;
    latest_odom_.orientation.y() = odom_msg->pose.pose.orientation.y;
    latest_odom_.orientation.z() = odom_msg->pose.pose.orientation.z;
    latest_odom_.orientation.w() = odom_msg->pose.pose.orientation.w;

    latest_odom_.linear_vel.x() = odom_msg->twist.twist.linear.x;
    latest_odom_.linear_vel.y() = odom_msg->twist.twist.linear.y;
    latest_odom_.linear_vel.z() = odom_msg->twist.twist.linear.z;

    latest_odom_.angular_vel.x() = odom_msg->twist.twist.angular.x;
    latest_odom_.angular_vel.y() = odom_msg->twist.twist.angular.y;
    latest_odom_.angular_vel.z() = odom_msg->twist.twist.angular.z;

    latest_odom_.timestamp = odom_msg->header.stamp;

    // 添加到历史记录（保持固定大小）
    odom_history_.push_back(latest_odom_);
    if (odom_history_.size() > 100) {  // 保持最近100帧
        odom_history_.pop_front();
    }

    if (debug_fusion_) {
        RCLCPP_DEBUG(logger_, "更新里程计数据: pos=[%.3f, %.3f], vel=[%.3f, %.3f]",
                     latest_odom_.position.x(), latest_odom_.position.y(),
                     latest_odom_.linear_vel.x(), latest_odom_.angular_vel.z());
    }
}

Eigen::Matrix4f OdomFusion::predictPose(const Eigen::Matrix4f& current_pose,
                                       const rclcpp::Time& target_time) {
    if (odom_history_.size() < 2) {
        if (debug_fusion_) {
            RCLCPP_WARN(logger_, "里程计历史数据不足，返回当前位姿");
        }
        return current_pose;
    }

    // 计算时间差
    double dt = (target_time - latest_odom_.timestamp).seconds();
    if (std::abs(dt) > 0.5) {  // 超过0.5秒认为数据过旧
        RCLCPP_WARN(logger_, "里程计数据过旧 (dt=%.3f), 返回当前位姿", dt);
        return current_pose;
    }

    // 基于速度进行简单预测
    Eigen::Matrix4f predicted_pose = current_pose;

    // 预测平移
    double dx = latest_odom_.linear_vel.x() * dt;
    double dy = latest_odom_.linear_vel.y() * dt;

    // 预测旋转
    double dtheta = latest_odom_.angular_vel.z() * dt;

    // 应用运动噪声（如果启用）
    if (std::abs(dx) > 1e-6 || std::abs(dy) > 1e-6 || std::abs(dtheta) > 1e-6) {
        double delta_trans = std::sqrt(dx*dx + dy*dy);
        auto [noisy_trans, noisy_rot1, noisy_rot2] = applyMotionNoise(delta_trans, dtheta, 0.0);

        // 应用噪声后的运动
        predicted_pose = applyMotionDelta(current_pose, noisy_trans, noisy_rot1, noisy_rot2);
    }

    if (debug_fusion_) {
        RCLCPP_DEBUG(logger_, "位姿预测: dt=%.3f, dx=%.3f, dy=%.3f, dtheta=%.3f",
                     dt, dx, dy, dtheta);
    }

    return predicted_pose;
}

Eigen::Matrix4f OdomFusion::fusePoses(const Eigen::Matrix4f& icp_pose,
                                     const Eigen::Matrix4f& predicted_pose,
                                     double icp_score,
                                     const rclcpp::Time& timestamp) {
    // 计算实际使用的权重
    double actual_icp_weight = icp_weight_;
    if (adaptive_weight_) {
        actual_icp_weight = computeAdaptiveWeight(icp_score);
    }
    double actual_odom_weight = 1.0 - actual_icp_weight;

    // 位置融合
    Eigen::Vector3f icp_pos = icp_pose.block<3,1>(0,3);
    Eigen::Vector3f pred_pos = predicted_pose.block<3,1>(0,3);
    Eigen::Vector3f fused_pos = actual_icp_weight * icp_pos + actual_odom_weight * pred_pos;

    // 旋转融合（使用四元数插值）
    Eigen::Matrix3f icp_rot = icp_pose.block<3,3>(0,0);
    Eigen::Matrix3f pred_rot = predicted_pose.block<3,3>(0,0);

    Eigen::Quaternionf icp_quat(icp_rot);
    Eigen::Quaternionf pred_quat(pred_rot);

    // 四元数球面线性插值
    Eigen::Quaternionf fused_quat = icp_quat.slerp(actual_odom_weight, pred_quat);

    // 构建融合后的位姿矩阵
    Eigen::Matrix4f fused_pose = Eigen::Matrix4f::Identity();
    fused_pose.block<3,3>(0,0) = fused_quat.toRotationMatrix();
    fused_pose.block<3,1>(0,3) = fused_pos;

    if (debug_fusion_) {
        double pos_diff = (icp_pos - pred_pos).norm();
        RCLCPP_DEBUG(logger_, "位姿融合: ICP权重=%.3f, 位置差=%.3f, ICP得分=%.6f",
                     actual_icp_weight, pos_diff, icp_score);
    }

    return fused_pose;
}

std::tuple<double, double, double> OdomFusion::applyMotionNoise(double delta_trans,
                                                               double delta_rot1,
                                                               double delta_rot2) {
    // 基于AMCL的运动噪声模型
    double noise_trans = odom_alpha3_ * delta_trans + odom_alpha1_ * (std::abs(delta_rot1) + std::abs(delta_rot2));
    double noise_rot1 = odom_alpha1_ * std::abs(delta_rot1) + odom_alpha2_ * delta_trans;
    double noise_rot2 = odom_alpha1_ * std::abs(delta_rot2) + odom_alpha2_ * delta_trans;

    // 添加高斯噪声
    double noisy_trans = delta_trans + normal_dist_(gen_) * noise_trans;
    double noisy_rot1 = delta_rot1 + normal_dist_(gen_) * noise_rot1;
    double noisy_rot2 = delta_rot2 + normal_dist_(gen_) * noise_rot2;

    return std::make_tuple(noisy_trans, noisy_rot1, noisy_rot2);
}

double OdomFusion::computeAdaptiveWeight(double icp_score) {
    // 基于ICP得分自适应调整权重
    // 得分越高，ICP权重越大
    double adaptive_icp_weight = icp_weight_;

    if (icp_score > 0.8) {
        adaptive_icp_weight = std::min(0.95, icp_weight_ + 0.1);
    } else if (icp_score < 0.3) {
        adaptive_icp_weight = std::max(0.5, icp_weight_ - 0.2);
    }

    return adaptive_icp_weight;
}

bool OdomFusion::isOdomValid(double timeout) const {
    if (odom_history_.empty()) {
        return false;
    }

    auto now = node_->now();
    double dt = (now - latest_odom_.timestamp).seconds();
    return dt <= timeout;
}

void OdomFusion::setFusionParams(double icp_weight, double odom_weight, bool adaptive) {
    icp_weight_ = icp_weight;
    odom_weight_ = odom_weight;
    adaptive_weight_ = adaptive;

    RCLCPP_INFO(logger_, "更新融合参数: ICP权重=%.2f, 里程计权重=%.2f, 自适应=%s",
                icp_weight_, odom_weight_, adaptive_weight_ ? "是" : "否");
}

void OdomFusion::reset() {
    odom_history_.clear();
    hypotheses_.clear();
    latest_odom_ = OdomData();
    previous_odom_ = OdomData();

    RCLCPP_INFO(logger_, "里程计融合模块已重置");
}

void OdomFusion::updateHypotheses(const Eigen::Matrix4f& new_pose,
                                 double icp_score,
                                 const rclcpp::Time& timestamp) {
    if (!enable_multi_hypothesis_) {
        return;
    }

    // 创建新假设
    PoseHypothesis new_hypothesis(new_pose, icp_score, icp_score, timestamp);

    if (hypotheses_.empty()) {
        hypotheses_.push_back(new_hypothesis);
        return;
    }

    // 更新现有假设的权重
    for (auto& hyp : hypotheses_) {
        double distance = computePoseDistance(hyp.pose, new_pose);
        // 距离越近，权重衰减越少
        hyp.weight *= std::exp(-distance * 0.5);
    }

    // 添加新假设
    hypotheses_.push_back(new_hypothesis);

    // 归一化权重
    normalizeHypotheses();

    // 重采样（移除低权重假设）
    resampleHypotheses();

    if (debug_fusion_) {
        RCLCPP_DEBUG(logger_, "更新假设: 当前假设数=%zu, 新假设权重=%.3f",
                     hypotheses_.size(), new_hypothesis.weight);
    }
}

PoseHypothesis OdomFusion::getBestHypothesis() const {
    if (hypotheses_.empty()) {
        return PoseHypothesis();
    }

    auto best_it = std::max_element(hypotheses_.begin(), hypotheses_.end(),
        [](const PoseHypothesis& a, const PoseHypothesis& b) {
            return a.weight < b.weight;
        });

    return *best_it;
}

double OdomFusion::computePoseDistance(const Eigen::Matrix4f& pose1, const Eigen::Matrix4f& pose2) {
    // 计算位置距离
    Eigen::Vector3f pos_diff = pose1.block<3,1>(0,3) - pose2.block<3,1>(0,3);
    double pos_distance = pos_diff.norm();

    // 计算角度距离
    Eigen::Matrix3f rot1 = pose1.block<3,3>(0,0);
    Eigen::Matrix3f rot2 = pose2.block<3,3>(0,0);
    Eigen::Matrix3f rot_diff = rot1.transpose() * rot2;

    // 从旋转矩阵提取角度差
    double angle_diff = std::acos(std::max(-1.0, std::min(1.0, (rot_diff.trace() - 1.0) / 2.0)));

    // 综合距离（位置权重更高）
    return pos_distance + 0.5 * angle_diff;
}

std::tuple<double, double, double> OdomFusion::computeOdomDelta(const OdomData& odom1,
                                                               const OdomData& odom2) {
    // 计算位置变化
    Eigen::Vector3d pos_diff = odom2.position - odom1.position;
    double delta_trans = pos_diff.head<2>().norm();

    // 计算角度变化
    Eigen::Quaterniond q_diff = odom1.orientation.inverse() * odom2.orientation;
    double delta_rot = 2.0 * std::atan2(q_diff.z(), q_diff.w());

    // 简化：假设只有一次旋转
    return std::make_tuple(delta_trans, delta_rot, 0.0);
}

Eigen::Matrix4f OdomFusion::applyMotionDelta(const Eigen::Matrix4f& pose,
                                            double delta_trans,
                                            double delta_rot1,
                                            double delta_rot2) {
    Eigen::Matrix4f new_pose = pose;

    // 应用第一次旋转
    Eigen::Matrix3f rot1;
    rot1 << std::cos(delta_rot1), -std::sin(delta_rot1), 0,
            std::sin(delta_rot1), std::cos(delta_rot1), 0,
            0, 0, 1;

    // 应用平移
    Eigen::Vector3f trans(delta_trans, 0, 0);

    // 应用第二次旋转
    Eigen::Matrix3f rot2;
    rot2 << std::cos(delta_rot2), -std::sin(delta_rot2), 0,
            std::sin(delta_rot2), std::cos(delta_rot2), 0,
            0, 0, 1;

    // 组合变换
    Eigen::Matrix4f delta_transform = Eigen::Matrix4f::Identity();
    delta_transform.block<3,3>(0,0) = rot2 * rot1;
    delta_transform.block<3,1>(0,3) = rot1 * trans;

    return pose * delta_transform;
}

void OdomFusion::normalizeHypotheses() {
    if (hypotheses_.empty()) return;

    double total_weight = 0.0;
    for (const auto& hyp : hypotheses_) {
        total_weight += hyp.weight;
    }

    if (total_weight > 1e-6) {
        for (auto& hyp : hypotheses_) {
            hyp.weight /= total_weight;
        }
    }
}

void OdomFusion::resampleHypotheses() {
    if (hypotheses_.size() <= max_hypotheses_) {
        return;
    }

    // 按权重排序
    std::sort(hypotheses_.begin(), hypotheses_.end(),
        [](const PoseHypothesis& a, const PoseHypothesis& b) {
            return a.weight > b.weight;
        });

    // 保留前max_hypotheses_个假设
    hypotheses_.resize(max_hypotheses_);

    // 移除权重过低的假设
    hypotheses_.erase(
        std::remove_if(hypotheses_.begin(), hypotheses_.end(),
            [this](const PoseHypothesis& hyp) {
                return hyp.weight < hypothesis_weight_threshold_;
            }),
        hypotheses_.end());
}

} // namespace agloc_fusion

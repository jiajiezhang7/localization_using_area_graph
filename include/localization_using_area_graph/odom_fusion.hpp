/**
 * @file odom_fusion.hpp
 * @author Jiajie Zhang (ROS2 port)
 * @brief 里程计与ICP融合算法，基于AMCL思想设计
 * @version 0.1
 * @date 2024-12-02
 * 
 * 该文件实现了基于AMCL算法思想的里程计与ICP融合机制，包括：
 * 1. 运动模型预测
 * 2. 位姿融合算法
 * 3. 多假设跟踪（可选）
 * 4. 自适应权重调整
 */

#pragma once
#ifndef _ODOM_FUSION_HPP_
#define _ODOM_FUSION_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Dense>
#include <random>
#include <deque>

namespace agloc_fusion {

/**
 * @brief 位姿假设结构体
 * 用于多假设跟踪，每个假设包含位姿和权重
 */
struct PoseHypothesis {
    Eigen::Matrix4f pose;           // 位姿矩阵
    double weight;                  // 假设权重
    double icp_score;              // ICP匹配得分
    rclcpp::Time timestamp;        // 时间戳
    
    PoseHypothesis() : weight(0.0), icp_score(0.0) {
        pose = Eigen::Matrix4f::Identity();
    }
    
    PoseHypothesis(const Eigen::Matrix4f& p, double w, double score, const rclcpp::Time& t)
        : pose(p), weight(w), icp_score(score), timestamp(t) {}
};

/**
 * @brief 里程计数据结构体
 */
struct OdomData {
    Eigen::Vector3d position;       // 位置 (x, y, z)
    Eigen::Quaterniond orientation; // 方向四元数
    Eigen::Vector3d linear_vel;     // 线速度
    Eigen::Vector3d angular_vel;    // 角速度
    rclcpp::Time timestamp;         // 时间戳
    
    OdomData() {
        position.setZero();
        orientation.setIdentity();
        linear_vel.setZero();
        angular_vel.setZero();
    }
};

/**
 * @brief 基于AMCL思想的里程计融合类
 * 
 * 该类实现了以下核心功能：
 * 1. 运动模型预测：基于里程计数据预测机器人位姿
 * 2. 位姿融合：将ICP结果与里程计预测进行加权融合
 * 3. 多假设跟踪：维护多个位姿假设，提高鲁棒性
 * 4. 自适应权重：根据匹配质量动态调整融合权重
 */
class OdomFusion {
public:
    /**
     * @brief 构造函数
     * @param node ROS2节点指针，用于参数获取和日志输出
     */
    explicit OdomFusion(rclcpp::Node* node);
    
    /**
     * @brief 析构函数
     */
    ~OdomFusion() = default;

    /**
     * @brief 更新里程计数据
     * @param odom_msg 里程计消息
     */
    void updateOdometry(const nav_msgs::msg::Odometry::SharedPtr odom_msg);
    
    /**
     * @brief 基于运动模型预测位姿
     * @param current_pose 当前位姿
     * @param target_time 目标时间
     * @return 预测的位姿
     */
    Eigen::Matrix4f predictPose(const Eigen::Matrix4f& current_pose, 
                               const rclcpp::Time& target_time);
    
    /**
     * @brief 融合ICP结果与里程计预测
     * @param icp_pose ICP优化后的位姿
     * @param predicted_pose 里程计预测的位姿
     * @param icp_score ICP匹配得分（用于自适应权重）
     * @param timestamp 当前时间戳
     * @return 融合后的位姿
     */
    Eigen::Matrix4f fusePoses(const Eigen::Matrix4f& icp_pose,
                             const Eigen::Matrix4f& predicted_pose,
                             double icp_score,
                             const rclcpp::Time& timestamp);
    
    /**
     * @brief 多假设跟踪更新
     * @param new_pose 新的位姿观测
     * @param icp_score ICP匹配得分
     * @param timestamp 时间戳
     */
    void updateHypotheses(const Eigen::Matrix4f& new_pose,
                         double icp_score,
                         const rclcpp::Time& timestamp);
    
    /**
     * @brief 获取最佳位姿假设
     * @return 权重最高的位姿假设
     */
    PoseHypothesis getBestHypothesis() const;
    
    /**
     * @brief 检查里程计数据是否有效
     * @param timeout 超时时间
     * @return 是否有效
     */
    bool isOdomValid(double timeout) const;
    
    /**
     * @brief 获取最新的里程计数据
     * @return 最新里程计数据
     */
    const OdomData& getLatestOdom() const { return latest_odom_; }
    
    /**
     * @brief 设置融合参数
     */
    void setFusionParams(double icp_weight, double odom_weight, bool adaptive);
    
    /**
     * @brief 重置融合状态
     */
    void reset();

private:
    // ROS2相关
    rclcpp::Node* node_;
    rclcpp::Logger logger_;
    
    // 参数
    double odom_alpha1_, odom_alpha2_, odom_alpha3_, odom_alpha4_;  // 运动噪声参数
    double icp_weight_, odom_weight_;                               // 融合权重
    bool adaptive_weight_;                                          // 自适应权重
    bool enable_multi_hypothesis_;                                  // 多假设跟踪
    int max_hypotheses_;                                           // 最大假设数
    double hypothesis_weight_threshold_;                           // 假设权重阈值
    bool debug_fusion_;                                            // 调试模式
    
    // 数据存储
    OdomData latest_odom_;                                         // 最新里程计数据
    OdomData previous_odom_;                                       // 上一帧里程计数据
    std::deque<OdomData> odom_history_;                           // 里程计历史数据
    std::vector<PoseHypothesis> hypotheses_;                      // 位姿假设集合
    
    // 随机数生成器（用于运动噪声）
    std::random_device rd_;
    std::mt19937 gen_;
    std::normal_distribution<double> normal_dist_;
    
    // 内部方法
    
    /**
     * @brief 应用运动模型噪声
     * @param delta_trans 平移变化量
     * @param delta_rot1 第一次旋转变化量
     * @param delta_rot2 第二次旋转变化量
     * @return 添加噪声后的运动增量
     */
    std::tuple<double, double, double> applyMotionNoise(double delta_trans, 
                                                       double delta_rot1, 
                                                       double delta_rot2);
    
    /**
     * @brief 计算自适应权重
     * @param icp_score ICP匹配得分
     * @return 调整后的ICP权重
     */
    double computeAdaptiveWeight(double icp_score);
    
    /**
     * @brief 计算两个位姿之间的距离
     * @param pose1 位姿1
     * @param pose2 位姿2
     * @return 位姿距离
     */
    double computePoseDistance(const Eigen::Matrix4f& pose1, const Eigen::Matrix4f& pose2);
    
    /**
     * @brief 从里程计数据计算运动增量
     * @param odom1 起始里程计数据
     * @param odom2 结束里程计数据
     * @return 运动增量 (delta_trans, delta_rot1, delta_rot2)
     */
    std::tuple<double, double, double> computeOdomDelta(const OdomData& odom1, 
                                                       const OdomData& odom2);
    
    /**
     * @brief 应用运动增量到位姿
     * @param pose 当前位姿
     * @param delta_trans 平移增量
     * @param delta_rot1 第一次旋转增量
     * @param delta_rot2 第二次旋转增量
     * @return 更新后的位姿
     */
    Eigen::Matrix4f applyMotionDelta(const Eigen::Matrix4f& pose,
                                    double delta_trans,
                                    double delta_rot1,
                                    double delta_rot2);
    
    /**
     * @brief 归一化假设权重
     */
    void normalizeHypotheses();
    
    /**
     * @brief 重采样假设（移除低权重假设）
     */
    void resampleHypotheses();
};

} // namespace agloc_fusion

#endif // _ODOM_FUSION_HPP_

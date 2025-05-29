/**
 * @file cloudInitializer_pose_evaluation.cpp
 * @author AGLoc优化
 * @brief 位姿评估相关方法实现
 * @date 2025-03-15
 */

#include "localization_using_area_graph/cloudInitializer.hpp"

/**
 * @brief 使用ICP评估位姿，不修改全局状态
 *
 * @param initialPose 初始位姿估计
 * @param areaId 区域ID
 * @param inputCloud 输入点云
 * @return std::pair<Eigen::Matrix4f, double> 优化后的位姿和评分
 */
std::pair<Eigen::Matrix4f, double> CloudInitializer::evaluatePoseWithICP(
    const Eigen::Matrix4f& initialPose,
    int areaId,
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud) {

    // 创建局部变量，避免修改全局状态
    Eigen::Matrix4f localRobotPose = initialPose;
    double localInsideScore = 0.0;
    double localOutsideScore = 0.0;
    int localNumofInsidePoints = 0;
    int localNumofOutsidePoints = 0;
    double localTurkeyScore = 0.0;

    // 创建局部点云
    auto localTransformedPC = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    pcl::transformPointCloud(*inputCloud, *localTransformedPC, localRobotPose);

    // 创建局部点云对象
    auto localInsidePC = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    auto localOutsidePC = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    auto localUsefulPoints1 = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    auto localUsefulPoints2 = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

    localUsefulPoints1->points.resize(Horizon_SCAN, pcl::PointXYZI());
    localUsefulPoints2->points.resize(Horizon_SCAN, pcl::PointXYZI());

    // 局部变量
    std::vector<int> localUsefulIndex;
    std::vector<double> localWeightsTurkey;
    double localWeightSumTurkey = 0.0;
    int localNumIcpPoints = 0;

    // 中心点向量
    Eigen::Vector3f localPCCenter = Eigen::Vector3f::Zero();
    Eigen::Vector3f localMapCenter = Eigen::Vector3f::Zero();

    // 向量存储
    std::vector<double> localVec_pcx;
    std::vector<double> localVec_pcy;
    std::vector<double> localVec_pedalx;
    std::vector<double> localVec_pedaly;

    // 调用线程安全版本的计算最近点方法
    this->calClosestMapPointThreadSafe(
        areaId,
        localRobotPose,
        localTransformedPC,
        localInsideScore,
        localOutsideScore,
        localNumofInsidePoints,
        localNumofOutsidePoints,
        localTurkeyScore);

    // ICP迭代
    int totalIteration = icp_init_iteration;
    Eigen::Matrix4f localRobotPoseGuess = localRobotPose;

    for (int iteration = 0; iteration < totalIteration; iteration++) {
        // 重置评分变量
        double localAverDistancePairedPoints = 0;
        double localInsideTotalRange = 0;
        double localOutsideTotalScore = 0;

        localPCCenter.setZero();
        localMapCenter.setZero();
        localWeightSumTurkey = 0.0;
        localNumIcpPoints = 0;

        // 准备迭代所需的结构
        localUsefulIndex.clear();
        localWeightsTurkey.clear();
        localVec_pcx.clear();
        localVec_pcy.clear();
        localVec_pedalx.clear();
        localVec_pedaly.clear();

        // 局部变量计算最近点匹配
        std::vector<int> localNumofIntersection;
        std::vector<double> localInRayDis;
        std::vector<double> localInRayRange;
        std::vector<double> localMatchWithOutside;

        size_t transformed_pc_size = localTransformedPC->points.size();
        localNumofIntersection.resize(transformed_pc_size, 0);
        localInRayDis.resize(transformed_pc_size, 0);
        localInRayRange.resize(transformed_pc_size, 0);
        localMatchWithOutside.resize(transformed_pc_size, 0);

        localInsidePC->clear();
        localInsidePC->points.resize(transformed_pc_size);
        localOutsidePC->clear();
        localOutsidePC->points.resize(transformed_pc_size);

        // 重新计算最近点
        this->calClosestMapPointThreadSafe(
            areaId,
            localRobotPose,
            localTransformedPC,
            localInsideScore,
            localOutsideScore,
            localNumofInsidePoints,
            localNumofOutsidePoints,
            localTurkeyScore);

        // 计算中心点
        if (use_weight) {
            localPCCenter = localPCCenter / localWeightSumTurkey;
            localMapCenter = localMapCenter / localWeightSumTurkey;
        } else if (localNumIcpPoints > 0) {
            localPCCenter = localPCCenter / localNumIcpPoints;
            localMapCenter = localMapCenter / localNumIcpPoints;
        }

        // 计算旋转矩阵
        Eigen::Matrix2d W;
        W.setZero();

        for (int i = 0; i < localNumIcpPoints; i++) {
            Eigen::Vector2d PCVec;
            Eigen::Vector2d MapVec;

            int idx = localUsefulIndex[i];
            if (localUsefulPoints1->points[idx].x != 0 ||
                localUsefulPoints1->points[idx].y != 0) {

                if (use_weight) {
                    PCVec << localUsefulPoints1->points[idx].x,
                            localUsefulPoints1->points[idx].y;
                    MapVec << localUsefulPoints2->points[idx].x,
                            localUsefulPoints2->points[idx].y;
                    W += localWeightsTurkey[i] * MapVec * PCVec.transpose();
                } else {
                    PCVec << localUsefulPoints1->points[idx].x - localPCCenter(0),
                            localUsefulPoints1->points[idx].y - localPCCenter(1);
                    MapVec << localUsefulPoints2->points[idx].x - localMapCenter(0),
                            localUsefulPoints2->points[idx].y - localMapCenter(1);
                    W += MapVec * PCVec.transpose();
                }
            }
        }

        // 确保类型一致，将float类型转换为double类型
        Eigen::Vector2d localPCCenterDouble = localPCCenter.head<2>().cast<double>();
        Eigen::Vector2d localMapCenterDouble = localMapCenter.head<2>().cast<double>();

        if (use_weight) {
            W = 1/localWeightSumTurkey * W - localMapCenterDouble * localPCCenterDouble.transpose();
        }

        // SVD分解计算旋转矩阵
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix2d U = svd.matrixU();
        Eigen::Matrix2d V = svd.matrixV();
        Eigen::Matrix2d rotationMatrix = U * V.transpose();

        Eigen::Vector2d translation = localMapCenterDouble - rotationMatrix * localPCCenterDouble;

        // 更新局部机器人位姿
        Eigen::Matrix4f localRobotPoseOldInv = localRobotPose.inverse();
        localRobotPose(0,3) += translation(0);
        localRobotPose(1,3) += translation(1);
        localRobotPose(3,3) = 1;
        localRobotPose.topLeftCorner(2,2) = rotationMatrix.cast<float>() * localRobotPose.topLeftCorner(2,2);
        localRobotPose(2,2) = 1;

        // 变换点云
        pcl::transformPointCloud(*localTransformedPC, *localTransformedPC, localRobotPose * localRobotPoseOldInv);

        // 检查是否收敛
        if (translation.norm() < icp_stop_translation_thred) {
            break;
        }

        // 检查移动距离是否过大
        if (sqrt(pow(localRobotPoseGuess(0,3) - localRobotPose(0,3), 2) +
                pow(localRobotPoseGuess(1,3) - localRobotPose(1,3), 2)) > 5) {
            break;
        }
    }

    // 最终评分计算
    double finalScore = 0.0;

    // 重新计算最近点以获取最终评分
    this->calClosestMapPointThreadSafe(
        areaId,
        localRobotPose,
        localTransformedPC,
        localInsideScore,
        localOutsideScore,
        localNumofInsidePoints,
        localNumofOutsidePoints,
        localTurkeyScore);

    // 计算外部和内部评分
    if (localNumofInsidePoints > 0) {
        localInsideScore = localInsideScore / localNumofInsidePoints;
    } else {
        localInsideScore = 999999;
    }

    if (localNumofOutsidePoints > 0) {
        localOutsideScore = localOutsideScore / localNumofOutsidePoints;
    } else {
        localOutsideScore = 999999;
    }

    // 计算最终评分 - 强化边缘位姿检测和惩罚
    long double base_score = 1.0L / (static_cast<long double>(localInsideScore) + static_cast<long double>(localOutsideScore));

    // 强化边缘位姿惩罚机制
    long double edge_penalty = 1.0L;
    bool is_edge_pose = false;

    if (localNumofInsidePoints > 0 && localNumofOutsidePoints > 0) {
        // 计算内外点比例，理想情况下应该有更多内部点
        long double inside_ratio = static_cast<long double>(localNumofInsidePoints) /
                                 (static_cast<long double>(localNumofInsidePoints) + static_cast<long double>(localNumofOutsidePoints));

        // 强化边缘检测：提高阈值到0.8，使用指数惩罚
        if (inside_ratio < 0.8L) {
            is_edge_pose = true;
            // 使用指数惩罚而非线性惩罚，更严厉
            long double ratio_penalty = std::pow(inside_ratio / 0.8L, 2.0L);
            edge_penalty *= ratio_penalty;

            // 对于极端边缘位姿（内部点比例<0.5），应用额外严厉惩罚
            if (inside_ratio < 0.5L) {
                edge_penalty *= 0.1L; // 额外90%惩罚
            }
        }

        // 检查几何一致性：如果外部点过多，可能是错误位姿
        if (localNumofOutsidePoints > localNumofInsidePoints * 1.5) {
            is_edge_pose = true;
            edge_penalty *= 0.3L; // 70%惩罚
        }

        // Turkey评分一致性检查：如果Turkey评分过低，说明匹配质量差
        if (localTurkeyScore > 0) {
            long double turkey_ratio = localTurkeyScore / (static_cast<long double>(localNumofInsidePoints) + static_cast<long double>(localNumofOutsidePoints));
            if (turkey_ratio < 2.0L) { // Turkey评分平均值过低
                is_edge_pose = true;
                edge_penalty *= 0.5L; // 50%惩罚
            }

            // 只有在Turkey评分足够高时才给予加分
            if (turkey_ratio > 5.0L && !is_edge_pose) {
                long double turkey_bonus = std::min(1.1L, 1.0L + turkey_ratio / 100.0L);
                edge_penalty *= turkey_bonus;
            }
        }

        // 距离一致性检查：如果平均距离过大，说明匹配质量差
        long double avg_inside_score = static_cast<long double>(localInsideScore) / static_cast<long double>(localNumofInsidePoints);
        long double avg_outside_score = static_cast<long double>(localOutsideScore) / static_cast<long double>(localNumofOutsidePoints);

        if (avg_inside_score > 1.0L || avg_outside_score > 1.5L) {
            is_edge_pose = true;
            edge_penalty *= 0.4L; // 60%惩罚
        }
    }

    // 如果没有内部点，这肯定是错误位姿
    if (localNumofInsidePoints == 0) {
        is_edge_pose = true;
        edge_penalty = 0.01L; // 99%惩罚
    }

    finalScore = static_cast<double>(base_score * edge_penalty);

    // 记录评估结果
    RCLCPP_INFO(this->get_logger(),
        "ICP评估位姿: x=%.2f, y=%.2f, 评分=%.10f (基础=%.10f, 边缘惩罚=%.4f, 内部点=%d, 外部点=%d, 边缘位姿=%s)",
        localRobotPose(0,3), localRobotPose(1,3), finalScore,
        static_cast<double>(base_score), static_cast<double>(edge_penalty),
        localNumofInsidePoints, localNumofOutsidePoints, is_edge_pose ? "是" : "否");

    return std::make_pair(localRobotPose, finalScore);
}

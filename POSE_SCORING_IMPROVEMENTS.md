# 全局定位位姿评分精度改进

## 问题描述

在全局定位过程中，当采样的粒子过于接近通道边缘时，最终选择的位姿可能错误地位于通道边缘，这与预期的评分原理不符。主要问题包括：

1. **数值精度不足**：当多个位姿评分非常接近时，`double`类型的精度可能不足以区分最优解
2. **角度搜索精度有限**：粗粒度12度、精细5度的步长可能错过最优角度
3. **边缘位姿处理不当**：评分机制无法准确识别和惩罚通道边缘的不良位姿

## 改进方案

### 1. 提升角度搜索精度

**粗粒度搜索改进：**
- 角度步长：从12度减小到6度
- 搜索范围：保持360度全覆盖
- 影响：提高粗搜索阶段的角度精度，减少遗漏最优角度的可能性

**精细搜索改进：**
- 角度步长：从5度减小到3度
- 搜索范围：在最佳角度附近±30度（从±30度扩展到±30度，但步长更精细）
- 影响：在精细搜索阶段获得更高的角度精度

### 2. 提升数值精度

**评分计算改进：**
- 使用`long double`进行中间计算
- 最终转换为`double`返回
- 公式：`current_score = static_cast<double>(base_score * edge_penalty)`

**代码变更：**
```cpp
// 原始代码
double current_score = 1.0/(insideScore + outsideScore);

// 改进后代码
long double base_score = 1.0L/(static_cast<long double>(insideScore) + static_cast<long double>(outsideScore));
double current_score = static_cast<double>(base_score * edge_penalty);
```

### 3. 边缘位姿惩罚机制

**内外点比例分析：**
- 计算内部点与总点数的比例
- 理想情况下，机器人应位于通道内部，内部点比例应较高
- 当内部点比例低于70%时，应用线性惩罚

**Turkey评分加权：**
- 利用现有的Turkey权重评分
- 为稳定的Turkey评分提供额外加分
- 最大加分不超过20%

**惩罚公式：**
```cpp
long double edge_penalty = 1.0L;
if (numofInsidePoints > 0 && numofOutsidePoints > 0) {
    long double inside_ratio = static_cast<long double>(numofInsidePoints) / 
                             (static_cast<long double>(numofInsidePoints) + static_cast<long double>(numofOutsidePoints));
    
    // 内部点比例惩罚
    if (inside_ratio < 0.7L) {
        edge_penalty = inside_ratio / 0.7L;
    }
    
    // Turkey评分加权
    if (turkeyScore > 0) {
        long double turkey_bonus = std::min(1.2L, 1.0L + turkeyScore / 1000.0L);
        edge_penalty *= turkey_bonus;
    }
}
```

## 实施细节

### 修改的文件

1. **localization_using_area_graph/src/cloudInitializer.cpp**
   - 粗粒度搜索角度步长：12度 → 6度
   - 精细搜索角度步长：5度 → 3度
   - 评分计算：添加数值精度提升和边缘惩罚

2. **localization_using_area_graph/src/cloudInitializer_pose_evaluation.cpp**
   - ICP评估中的评分计算：同样应用数值精度提升和边缘惩罚

### 调试信息增强

添加了详细的调试输出，包括：
- 基础评分值
- 边缘惩罚因子
- 内部点和外部点数量
- 内外点比例

示例输出：
```
精细搜索更新最佳位姿: x=10.50, y=5.20, 角度=45度, 评分=0.12345678 (基础=0.11111111, 边缘惩罚=1.111, 内部点=150, 外部点=50)
```

## 预期效果

1. **提高定位精度**：更精细的角度搜索能够找到更准确的位姿
2. **减少边缘误判**：边缘惩罚机制能够有效识别和避免通道边缘的不良位姿
3. **增强数值稳定性**：高精度计算减少因数值误差导致的错误选择
4. **改善对称结构处理**：结合现有的双峰候选机制，更好地处理对称通道结构

## 测试建议

1. **对比测试**：在相同场景下对比改进前后的定位结果
2. **边缘场景测试**：特别测试机器人接近通道边缘时的定位表现
3. **角度精度测试**：验证精细角度搜索是否能找到更准确的朝向
4. **性能影响评估**：评估角度步长减小对计算时间的影响

## 参数调优

如需进一步调优，可考虑以下参数：

- **内部点比例阈值**：当前为0.7，可根据实际场景调整
- **Turkey评分权重**：当前最大加权1.2倍，可根据需要调整
- **角度搜索范围**：精细搜索的±30度范围可根据需要调整

## 注意事项

1. 角度步长减小会增加计算量，需要在精度和性能之间平衡
2. 边缘惩罚参数需要根据实际地图特征进行调优
3. 建议在实际环境中充分测试后再部署到生产环境

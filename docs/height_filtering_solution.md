# 跨楼层动态加载兼容的高度预过滤解决方案

## 概述

本解决方案旨在修复多楼层 AGMap 环境下的区域判断错误问题，与您正在实现的跨楼层动态加载功能完全兼容。

## 问题分析

### 根本原因
- 当前区域判断算法使用2D射线相交法，完全忽略Z轴（高度）信息
- 在多楼层环境下，垂直重叠区域在XY平面投影相同但高度不同
- 导致机器人同时被判断为位于多个区域内，触发"机器人位置在多个区域内!"错误

### 核心缺陷
1. `inRayGeneral` 函数只进行2D平面计算
2. `calIntersection` 函数忽略Z坐标参与
3. 缺少高度约束验证机制
4. 区域遍历时未考虑楼层分离

## 解决方案设计

### 1. 楼层感知机制
- 利用现有的 `mapAGCB()` 中的TF监听逻辑
- 从 `map->AGmap` 的TF变换中提取Z偏移量
- 实时监测楼层变化（Z值变化超过阈值时）
- 自动计算当前楼层编号

### 2. 高度预过滤策略
- 在执行2D射线相交前，先进行高度兼容性检查
- 计算区域的高度范围，与机器人高度进行匹配
- 只对高度兼容的区域执行射线相交判断
- 保持原有2D算法的高效性

### 3. 兼容性设计
- 保持现有 `mapAGCB()` 中的TF监听逻辑不变
- 通过参数开关控制功能启用/禁用
- 向后兼容，不影响单楼层环境的使用
- 楼层切换时自动重置相关状态

## 实现细节

### 核心函数

#### `areaInsideCheckingWithHeight()`
```cpp
bool CloudBase::areaInsideCheckingWithHeight(const Eigen::Matrix4f& robotPose, int areaStartIndex) {
    if (!enable_height_filtering) {
        return areaInsideChecking(robotPose, areaStartIndex);
    }
    
    double robotHeight = robotPose(2,3);
    
    // 高度兼容性检查
    if (!isAreaHeightCompatible(robotHeight, areaStartIndex)) {
        return false;
    }
    
    // 执行原始的2D射线相交检查
    return areaInsideChecking(robotPose, areaStartIndex);
}
```

#### `isAreaHeightCompatible()`
```cpp
bool CloudBase::isAreaHeightCompatible(double robotHeight, int areaStartIndex) {
    // 计算区域高度范围
    double minAreaHeight = std::numeric_limits<double>::max();
    double maxAreaHeight = std::numeric_limits<double>::lowest();
    
    for(int i = areaStartIndex; i < areaStartIndex + 1000000; i++) {
        if((int)map_pc->points[i].intensity % 3 == 2) break;
        
        double pointHeight = map_pc->points[i].z;
        minAreaHeight = std::min(minAreaHeight, pointHeight);
        maxAreaHeight = std::max(maxAreaHeight, pointHeight);
    }
    
    // 检查机器人高度是否在区域高度范围内
    return (robotHeight >= minAreaHeight - height_tolerance) && 
           (robotHeight <= maxAreaHeight + height_tolerance);
}
```

#### `updateFloorState()`
```cpp
void CloudBase::updateFloorState(double tf_z_offset) {
    current_floor_z_offset_ = tf_z_offset;
    
    if (detectFloorChange(tf_z_offset)) {
        floor_changed_ = true;
        current_floor_number_ = calculateFloorNumber(tf_z_offset);
        resetFloorState();
    }
    
    last_floor_z_offset_ = tf_z_offset;
}
```

### 参数配置

```yaml
# 跨楼层高度过滤参数
enable_height_filtering: true          # 启用高度过滤
floor_height: 8.0                      # 单层楼高度（米）
height_tolerance: 1.0                  # 高度容差（米）
floor_change_threshold: 4.0            # 楼层切换检测阈值（米）
enable_floor_detection: true           # 启用楼层自动检测
debug_height_filtering: false          # 调试信息开关
```

## 集成方式

### 1. 在 `mapAGCB()` 中集成楼层状态更新
```cpp
// 在TF变换获取成功后添加
if (enable_height_filtering && enable_floor_detection) {
    updateFloorState(translation.z());
}
```

### 2. 在 `gettingInsideWhichArea()` 中使用高度过滤
```cpp
if (enable_height_filtering) {
    binside = areaInsideCheckingWithHeight(robotPose, i);
} else {
    binside = areaInsideChecking(robotPose, i);
}
```

## 优势特点

1. **完全兼容**：与现有跨楼层动态加载功能无缝集成
2. **高效性能**：高度预过滤减少不必要的2D计算
3. **参数化配置**：支持灵活的参数调整
4. **向后兼容**：不影响单楼层环境的使用
5. **调试友好**：提供详细的调试信息输出

## 使用指南

### 单楼层环境
```yaml
enable_height_filtering: false
```

### 多楼层环境
```yaml
enable_height_filtering: true
floor_height: 8.0                      # 根据实际楼层高度调整
height_tolerance: 1.0                  # 根据地图精度调整
```

### 调试模式
```yaml
debug_height_filtering: true           # 查看详细过滤信息
```

## 预期效果

1. **消除多区域错误**：解决"机器人位置在多个区域内!"问题
2. **提高定位精度**：减少跨楼层的错误匹配
3. **增强系统稳定性**：楼层切换时自动适应
4. **保持性能**：高度预过滤提高整体效率

## 测试建议

1. **单楼层测试**：验证向后兼容性
2. **多楼层测试**：验证高度过滤效果
3. **楼层切换测试**：验证动态加载兼容性
4. **性能测试**：对比启用前后的处理时间

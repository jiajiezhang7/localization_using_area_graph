# AGLoc系统自动rosbag录制工具

## 功能说明

这个Python脚本用于在AGLoc系统启动时自动开始录制指定的ROS话题。脚本会智能检测目标话题是否开始发布，一旦检测到就自动开始录制。

**🔥 重要特性：时间戳同步**
- 默认使用仿真时间模式，确保新录制的rosbag与播放的rosbag使用相同的时间戳
- 支持与`run_time_sync_fixed.launch.py`完美配合
- 自动检测`/clock`话题以确保时间同步正确建立

## 录制话题

- `/rss` - WiFi RSS数据
- `/RobotPath` - 机器人路径信息
- `/RobotPathLonLat` - 机器人路径地理坐标

## 使用方法

### 基本使用

```bash
# 在AGLoc workspace目录下
cd /home/jay/AGLoc_ws/src
python3 localization_using_area_graph/scripts/auto_bag_recorder.py
```

### 指定输出目录

```bash
python3 localization_using_area_graph/scripts/auto_bag_recorder.py --output-dir /path/to/recordings
```

### 限制录制时长

```bash
# 录制60秒后自动停止
python3 localization_using_area_graph/scripts/auto_bag_recorder.py --duration 60
```

### 时间同步设置

```bash
# 使用仿真时间（默认，推荐）- 与播放的rosbag时间戳一致
python3 localization_using_area_graph/scripts/auto_bag_recorder.py

# 使用系统时间（不推荐，除非有特殊需求）
python3 localization_using_area_graph/scripts/auto_bag_recorder.py --use-system-time
```

### 等待所有话题模式（推荐解决方案）

```bash
# 等待所有话题都检测到后再开始录制（推荐）
python3 localization_using_area_graph/scripts/auto_bag_recorder.py --wait-all-topics

# 或者使用快速模式（检测到新话题时自动重启录制）
python3 localization_using_area_graph/scripts/auto_bag_recorder.py
```

### 完整参数示例

```bash
python3 localization_using_area_graph/scripts/auto_bag_recorder.py \
    --output-dir ./my_recordings \
    --duration 120 \
    --wait-all-topics
    # 使用仿真时间模式，等待所有话题
```

## 使用流程

1. **启动录制工具**：
   ```bash
   python3 localization_using_area_graph/scripts/auto_bag_recorder.py
   ```

2. **启动AGLoc系统**：
   ```bash
   ros2 launch localization_using_area_graph run_time_sync_fixed.launch.py
   ```

3. **自动开始录制**：
   - 录制工具会自动检测话题的出现
   - 一旦检测到任意一个目标话题，就开始录制
   - 录制过程中会显示进度信息

4. **停止录制**：
   - 按 `Ctrl+C` 停止录制
   - 或者达到指定时长后自动停止

## 输出文件

录制的rosbag文件会以时间戳命名，例如：
```
recordings/agloc_recording_20231215_143052/
```

## 参数说明

| 参数 | 短参数 | 默认值 | 说明 |
|------|--------|--------|------|
| `--output-dir` | `-o` | `./recordings` | 录制文件输出目录 |
| `--duration` | `-d` | 无限制 | 录制时长（秒） |
| `--use-system-time` | 无 | false | 使用系统时间而非仿真时间 |
| `--wait-all-topics` | 无 | false | 🔥等待所有话题都出现再录制（推荐） |
   <!-- /home/jay/AGLoc_ws/rosbag_wifi_gt -->
   
## 注意事项

1. **时间同步**：默认使用仿真时间模式，确保与播放rosbag时间戳一致
2. **启动顺序**：建议先启动录制工具，再启动AGLoc系统  
3. **权限要求**：确保对输出目录有写权限
4. **磁盘空间**：确保有足够的磁盘空间存储录制文件
5. **话题检测**：脚本每秒检查一次话题状态
6. **时钟同步**：仿真时间模式下会等待`/clock`话题出现后再开始录制
7. **优雅退出**：使用Ctrl+C可以安全停止录制

## 故障排除

### 问题：检测不到话题
**解决方案**：
- 确认AGLoc系统已正常启动
- 检查话题名称是否正确：`ros2 topic list`
- 确认ROS2环境变量设置正确

### 问题：录制文件为空
**解决方案**：
- 确认话题确实在发布数据：`ros2 topic echo /rss`
- 检查磁盘空间是否充足
- 确认rosbag record命令可以正常执行

### 问题：时间戳不同步
**解决方案**：
- 确保使用仿真时间模式（默认）
- 检查`/clock`话题是否正常发布：`ros2 topic echo /clock`
- 确认AGLoc系统使用了`--clock`参数播放rosbag
- 避免使用`--use-system-time`参数

### 问题：录制进程异常终止
**解决方案**：
- 检查系统资源使用情况
- 确认没有其他程序占用相同文件
- 查看脚本日志输出获取详细错误信息

### 问题：等待时钟同步时间过长
**解决方案**：
- 确认AGLoc系统已正常启动并开始播放rosbag
- 检查rosbag播放是否使用了`--clock`参数
- 如果不需要时间同步，可使用`--use-system-time`参数

### 问题：录制的rosbag缺少某些话题
**解决方案**：
- **推荐**：使用`--wait-all-topics`参数等待所有话题出现后再录制
- 检查缺少的话题是否真的在发布：`ros2 topic list`
- 检查话题发布频率：`ros2 topic hz /topic_name`
- 使用便捷脚本：`./start_auto_recording.sh -w` 
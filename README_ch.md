# 基于区域图的定位系统

这是一个使用区域图进行室内定位的ROS2功能包，并且兼容Navigation2的定位模块，可以替代AMCL，从而结合osmAG与LiDAR进行定位。

它需要与osmAG Navigation Stack中的其他包结合使用：
- `AreaGraphDataParser`  （必须，用于解析osmAG地图）
- `wifi_loc` (可需，如果需要开启WiFi结合全局定位)
- `rss` （可需，WiFi定位所需的自定义消息格式）
![demo1](images/demo1.png)


## 1. 环境要求：

### ROS2 Iron
在一个步骤中安装此功能包的依赖项：
```bash
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 第三方库：ceres 及其依赖项：
```bash
sudo apt-get update
sudo apt-get install -y libgoogle-glog-dev libgflags-dev
sudo apt-get install -y libatlas-base-dev
sudo apt-get install -y libeigen3-dev
sudo apt-get install -y libsuitesparse-dev
sudo apt-get install -y libceres-dev
```

## 2. 

- 与 `area_graph_data_parser` 的关系：

    - `osmAG.xml` -> `area_graph_data_parser` -> ROS2 话题: `/mapPC_AG`, `/AG_index` -> `localization_using_area_graph`
- 与 `wifi_loc`的关系：
- `/rss` -> `wifi_loc` -> `/WiFiLocation` -> `localization_using_area_graph`


## 3. 数据准备：
 - 一个 `osmAG.xml` 地图文件 (由 `AreaGraphDataParser` 功能包解析)
 - 一个 rosbag 文件 (必须包含激光雷达点云话题，在launch中统一被重映射为`/hesai/pandar`）
    - /tf, /static_tf （run.launch.py为原生全局map坐标系下的robotPath绝对位姿，要求禁用tf，但agloc_nav2_integration.launch.py为了与Nav2结合，把定位结果转化为tf后发布，因此需要tf）
    - /rss （此为所收集的WiFi信号，如果开启WiFi定位则需要)


## 4. 编译和运行：
```bash
(由于自定义消息格式原因，以下编译 `area_graph_data_parser` 命令需要执行两次)
colcon build --symlink-install --packages-select area_graph_data_parser 
colcon build --symlink-install --packages-select localization_using_area_graph
source install/setup.bash
ros2 launch localization_using_area_graph run.launch.py
```
## 5. 输出：
 - run.launch.py输出：全局机器人路径，由 AGLoc 系统估计，作为 ROS2 话题 "/RobotPath" 发布，坐标系为 `map`。
 - agloc_nav2_integration.launch.py输出： 实时更新机器人定位，转化为tf发布，结果体现在 map->odom->base_link的tf关系中


## 运行模式

### WiFi全局定位 + 位姿跟踪
 -  将参数中的 `bRescueRobot` 设置为 `true`
 -  将 `run.launch.py`,`agloc_nav2_integration.launch.py` 中的 `use_global_localization` 设置为 `true`
 -  WiFi定位机理可选择使用真值AP位置/估计AP位置，通过launch文件参数 `use_true_ap_positions` 来控制

### 仅位姿跟踪
 -  将参数中的 `bRescueRobot` 设置为 `false`
 -  将 `run.launch.py`,`agloc_nav2_integration.launch.py` 中的 `use_global_localization` 设置为 `false`
 - 在/config/params.yaml中需要提供机器人相对于map坐标系的（需要凑出来）:
    - `initialYawAngle: yaw `
    - `initialExtrinsicTrans: [float:x, float:y, 0.0] `

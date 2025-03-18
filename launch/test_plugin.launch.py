#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    # 获取包路径
    pkg_dir = get_package_share_directory('localization_using_area_graph')
    nav2_pkg_dir = get_package_share_directory('nav2_bringup')
    
    # 声明参数
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_bag = LaunchConfiguration('use_bag')
    use_global_localization = LaunchConfiguration('use_global_localization')
    map_dir = LaunchConfiguration('map_dir')
    map_name = LaunchConfiguration('map_name')
    params_file = LaunchConfiguration('params_file')
    osm_file = LaunchConfiguration('osm_file')
    bag_file = LaunchConfiguration('bag_file')
    
    # 声明启动参数
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='使用仿真时间')
    
    declare_use_bag_cmd = DeclareLaunchArgument(
        'use_bag',
        default_value='false',
        description='是否使用ROS2 bag进行测试')
    
    declare_use_global_localization_cmd = DeclareLaunchArgument(
        'use_global_localization',
        default_value='false',
        description='是否使用全局定位')
    
    declare_map_dir_cmd = DeclareLaunchArgument(
        'map_dir',
        default_value=os.path.join(pkg_dir, 'maps'),
        description='地图文件目录')
    
    declare_map_name_cmd = DeclareLaunchArgument(
        'map_name',
        default_value='area_graph_map',
        description='地图文件名称')
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_dir, 'config', 'agloc_nav2_params.yaml'),
        description='Nav2参数文件')
    
    declare_osm_file_cmd = DeclareLaunchArgument(
        'osm_file',
        default_value='',  # 将在agloc_components.launch.py中设置默认值
        description='区域图OSM文件路径')
    
    declare_bag_file_cmd = DeclareLaunchArgument(
        'bag_file',
        default_value='',  # 将在agloc_components.launch.py中设置默认值
        description='ROS2 bag文件路径')
    
    # 启动AGLoc组件（TF、区域图解析器等）
    agloc_components_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'agloc_components.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_bag': use_bag,
            'use_global_localization': use_global_localization,
            'osm_file': osm_file,
            'bag_file': bag_file
        }.items())
    
    # 启动Nav2定位节点
    start_localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg_dir, 'launch', 'localization_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'map': os.path.join(map_dir, map_name)
        }.items())
    
    # 启动RViz2进行可视化
    rviz_config_file = os.path.join(pkg_dir, 'rviz', 'localization.rviz')
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')
    
    # 返回LaunchDescription
    ld = LaunchDescription()
    
    # 添加声明
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_bag_cmd)
    ld.add_action(declare_use_global_localization_cmd)
    ld.add_action(declare_map_dir_cmd)
    ld.add_action(declare_map_name_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_osm_file_cmd)
    ld.add_action(declare_bag_file_cmd)
    
    # 添加启动命令
    ld.add_action(agloc_components_cmd)  # 先启动AGLoc组件
    ld.add_action(start_localization_cmd) # 然后启动Nav2定位节点
    ld.add_action(start_rviz_cmd)         # 最后启动RViz2可视化
    
    return ld

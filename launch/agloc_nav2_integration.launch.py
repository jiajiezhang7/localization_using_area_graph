#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取包路径
    pkg_dir = get_package_share_directory('localization_using_area_graph')
    topology_pkg_dir = get_package_share_directory('area_graph_data_parser')
    nav2_pkg_dir = get_package_share_directory('nav2_bringup')

    # 声明参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='使用仿真时间'
    )

    use_bag_arg = DeclareLaunchArgument(
        'use_bag',
        default_value='true',
        description='是否使用ROS2 bag进行测试'
    )

    bag_file_arg = DeclareLaunchArgument(
        'bag_file',
        default_value='/home/jay/AGLoc_ws/rosbag/agile03_Mars_door_01',
        description='ROS2 bag文件路径'
    )

    osm_file_arg = DeclareLaunchArgument(
        'osm_file',
        default_value=os.path.join(topology_pkg_dir, 'data', 'fix_id', 'SIST1_F2_All_Sec.osm'),
        description='区域图OSM文件路径'
    )

    use_global_localization_arg = DeclareLaunchArgument(
        'use_global_localization',
        default_value='true',
        description='是否使用全局定位'
    )

    start_offset_arg = DeclareLaunchArgument(
        'start_offset',
        default_value='0.0',
        description='Bag播放的起始时间偏移（秒）'
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_dir, 'maps', 'elevator.yaml'),
        description='地图YAML文件路径'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_dir, 'config', 'agloc_nav2_params.yaml'),
        description='Nav2参数文件'
    )

    autostart_arg = DeclareLaunchArgument(
        'autostart', 
        default_value='true',
        description='自动启动导航栈'
    )

    use_lifecycle_mgr_arg = DeclareLaunchArgument(
        'use_lifecycle_mgr', 
        default_value='true',
        description='是否使用生命周期管理器'
    )

    use_composition_arg = DeclareLaunchArgument(
        'use_composition', 
        default_value='false',
        description='是否使用组合式启动'
    )

    use_respawn_arg = DeclareLaunchArgument(
        'use_respawn', 
        default_value='false',
        description='节点崩溃时是否重新生成'
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level', 
        default_value='info',
        description='日志级别'
    )

    # 设置use_sim_time参数
    use_sim_time_param = SetParameter(
        name='use_sim_time',
        value=LaunchConfiguration('use_sim_time')
    )

    # Robot Localization节点（WiFi定位）
    robot_loc = Node(
        package='wifi_loc',
        executable='robot_loc',
        name='robot_loc',
        parameters=[
            os.path.join(pkg_dir, 'config', 'params.yaml'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'use_true_ap_positions': False,
                'bag_path': LaunchConfiguration('bag_file'),
                'wait_for_robotpose': True,  # 等待AGLoc位姿结果并在可视化中显示
            }
        ],
        condition=IfCondition(LaunchConfiguration('use_global_localization')),
        output='screen',
    )

    # 从run.launch.py参考AGLoc需要的TF变换
    # TF变换：world -> map
    world_to_map_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='transform_world_to_map',
        arguments=[
            '--frame-id', 'world',
            '--child-frame-id', 'map'
        ]
    )

    # TF变换：map -> AGmap
    map_to_agmap_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='transform_map_to_agmap',
        arguments=[
            '--frame-id', 'map',
            '--child-frame-id', 'AGmap',
            '--x', '9.8',      
            '--y', '-34.3',      
            '--z', '-16.0',     
            '--qx', '0',
            '--qy', '0',
            '--qz', '-0.649',
            '--qw', '0.760'
        ]
    )
    
    # 静态TF变换：map -> odom（作为初始值，后续会被动态更新）
    map_to_odom_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='transform_map_to_odom',
        arguments=[
            '--frame-id', 'map',
            '--child-frame-id', 'odom',
            '--x', '0.0',      
            '--y', '0.0',      
            '--z', '0.0',     
            '--qx', '0.0',
            '--qy', '0.0',
            '--qz', '0.0',
            '--qw', '1.0'
        ]
    )

    # 区域图数据解析器
    area_graph_parser = Node(
        package='area_graph_data_parser',
        executable='main',
        name='main', 
        output='screen',
        parameters=[{
            'osm_file_path': LaunchConfiguration('osm_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # 粒子生成器（用于全局定位）
    particle_generator = Node(
        package='localization_using_area_graph',
        executable='particle_generator',
        name='particle_generator',
        parameters=[
            os.path.join(pkg_dir, 'config', 'params.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        condition=IfCondition(LaunchConfiguration('use_global_localization')),
        output='screen',
    )
    


    # 添加CloudHandler节点（AGLoc系统核心组件）- 从run.launch.py参考
    cloud_handler = Node(
        package='localization_using_area_graph',
        executable='cloud_handler',
        name='cloud_handler', 
        output='screen',
        parameters=[
            os.path.join(pkg_dir, 'config', 'params.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    # 添加AGLoc适配器节点（连接AGLoc与Nav2）
    agloc_localizer = Node(
        package='localization_using_area_graph',
        executable='agloc_localizer_node',
        name='agloc_localizer', 
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    # 播放ROS2 bag（可选）
    play_bag = ExecuteProcess(
        cmd=['ros2', 'bag', 'play',
             LaunchConfiguration('bag_file'),
             '-s', 'mcap',
             '--clock',
             '--rate', '0.5',
             '--start-offset', LaunchConfiguration('start_offset'),
             '--remap', '/lidar_points:=/hesai/pandar'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_bag'))
    )

    # 启动RViz2进行可视化
    rviz_config_file = os.path.join(pkg_dir, 'config', 'ag_loc_nav2.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    # 生命周期管理器 - 从agloc_localization_launch.py参考
    lifecycle_nodes = ['map_server', 'agloc_localizer']
    lifecycle_manager = Node(
        condition=IfCondition(LaunchConfiguration('use_lifecycle_mgr')),
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'autostart': LaunchConfiguration('autostart')},
            {'node_names': lifecycle_nodes}
        ]
    )

    # Nav2 Map Server - 从agloc_localization_launch.py参考
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'yaml_filename': LaunchConfiguration('map')},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    # 创建LaunchDescription并添加所有节点
    ld = LaunchDescription()
    
    # 添加参数声明
    ld.add_action(use_sim_time_arg)
    ld.add_action(use_sim_time_param)
    ld.add_action(use_bag_arg)
    ld.add_action(bag_file_arg)
    ld.add_action(osm_file_arg)
    ld.add_action(use_global_localization_arg)
    ld.add_action(start_offset_arg)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(params_file_arg)
    ld.add_action(autostart_arg)
    ld.add_action(use_lifecycle_mgr_arg)
    ld.add_action(use_composition_arg)
    ld.add_action(use_respawn_arg)
    ld.add_action(log_level_arg)
    
    # 添加TF变换
    ld.add_action(world_to_map_transform)
    ld.add_action(map_to_agmap_transform)
    # map->odom变换由AGLocLocalizerNode动态发布
    
    # 添加区域图解析器（必须先启动）
    ld.add_action(area_graph_parser)
    
    # 延迟启动其他组件，确保区域图解析器已完成初始化
    # 第一步：启动WiFi定位和粒子生成器
    timer_actions_1 = TimerAction(
        period=2.0,  # 2秒后启动
        actions=[
            robot_loc,
            particle_generator
        ]
    )
    ld.add_action(timer_actions_1)
    
    # 第二步：启动Nav2地图服务器和AGLoc核心组件
    timer_actions_2 = TimerAction(
        period=4.0,  # 4秒后启动
        actions=[
            map_server,
            cloud_handler
        ]
    )
    ld.add_action(timer_actions_2)
    
    # 第三步：启动AGLoc适配器节点
    timer_actions_3 = TimerAction(
        period=6.0,  # 6秒后启动
        actions=[
            agloc_localizer
        ]
    )
    ld.add_action(timer_actions_3)
    
    # 第四步：启动生命周期管理器
    timer_actions_4 = TimerAction(
        period=8.0,  # 8秒后启动
        actions=[
            lifecycle_manager
        ]
    )
    ld.add_action(timer_actions_4)
    
    # 第五步：启动RViz和bag播放
    timer_actions_5 = TimerAction(
        period=10.0,  # 10秒后启动
        actions=[
            rviz_node,
            play_bag
        ]
    )
    ld.add_action(timer_actions_5)

    return ld

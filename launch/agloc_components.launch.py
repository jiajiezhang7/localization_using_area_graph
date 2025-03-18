#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取包路径
    pkg_dir = get_package_share_directory('localization_using_area_graph')
    topology_pkg_dir = get_package_share_directory('area_graph_data_parser')

    # 声明参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='使用仿真时间'
    )

    use_bag_arg = DeclareLaunchArgument(
        'use_bag',
        default_value='false',
        description='是否使用ROS2 bag进行测试'
    )

    bag_file_arg = DeclareLaunchArgument(
        'bag_file',
        default_value='/home/jay/AGLoc_ws/rosbag/95',  # 注意不需要.db3后缀
        description='ROS2 bag文件路径'
    )

    osm_file_arg = DeclareLaunchArgument(
        'osm_file',
        default_value=os.path.join(topology_pkg_dir, 'data', 'fix_id', 'SIST1_F2_All_Sec.osm'),
        description='区域图OSM文件路径'
    )

    use_global_localization_arg = DeclareLaunchArgument(
        'use_global_localization',
        default_value='false',
        description='是否使用全局定位'
    )

    # 设置use_sim_time参数
    use_sim_time_param = SetParameter(
        name='use_sim_time',
        value=LaunchConfiguration('use_sim_time')
    )

    # 创建启动描述
    ld = LaunchDescription()

    # 添加参数声明
    ld.add_action(use_sim_time_arg)
    ld.add_action(use_sim_time_param)
    ld.add_action(use_bag_arg)
    ld.add_action(bag_file_arg)
    ld.add_action(osm_file_arg)
    ld.add_action(use_global_localization_arg)

    # TF变换：world -> map
    ld.add_action(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='transform_world_to_map',
        arguments=[
            '--frame-id', 'world',
            '--child-frame-id', 'map'
        ]
    ))

    # TF变换：map -> AGmap
    ld.add_action(Node(
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
    ))

    # TF变换：map -> PandarQT（激光雷达）
    # 事实上这只是用于肉眼标出真值，而对机器人定位无丝毫影响
    ld.add_action(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='transform_map_to_lidar',
        arguments=[
            '--frame-id', 'map',
            '--child-frame-id', 'PandarQT',
            '--x', '4.0',
            '--y', '-6.0',
            '--z', '0.0',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '1.5708'
        ]
    ))

    # TF变换：base_link -> PandarQT（这是实际的机器人需要发布的tf关系 baselink->pandarQT，但由于目前是rosbag测试，则这个tf关系会在rosbag中被记录）
    # ld.add_action(Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='transform_base_to_lidar',
    #     arguments=[
    #         '--frame-id', 'base_link',
    #         '--child-frame-id', 'velodyne',
    #         '--x', '0.0',
    #         '--y', '0.0',
    #         '--z', '0.3',
    #         '--roll', '0',
    #         '--pitch', '0',
    #         '--yaw', '0'
    #     ],
    #     condition=UnlessCondition(LaunchConfiguration('use_bag'))
    # ))

    # 区域图数据解析器
    ld.add_action(Node(
        package='area_graph_data_parser',
        executable='main',
        name='main', 
        output='screen',
        parameters=[{
            'osm_file_path': LaunchConfiguration('osm_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    ))

    # 粒子生成器（用于全局定位）
    ld.add_action(Node(
        package='localization_using_area_graph',
        executable='particle_generator',
        name='particle_generator',
        parameters=[
            os.path.join(pkg_dir, 'config', 'params.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        condition=IfCondition(LaunchConfiguration('use_global_localization')),
        output='screen',
    ))

    # 播放ROS2 bag（可选）
    ld.add_action(ExecuteProcess(
        cmd=['ros2', 'bag', 'play',
             LaunchConfiguration('bag_file'),
             '--clock',
             '--rate', '0.5'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_bag'))
    ))

    return ld

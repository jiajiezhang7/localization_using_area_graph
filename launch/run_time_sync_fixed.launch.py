from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('localization_using_area_graph')
    topology_pkg_dir = get_package_share_directory('area_graph_data_parser')

    bag_file_arg = DeclareLaunchArgument(
        'bag_file',
        default_value='/home/jay/AGLoc_ws/rosbag/95',
        description='Path to ROS2 bag file'
    )

    osm_file = os.path.join(topology_pkg_dir, 'data', 'fix_id', 'SIST1_F2_All_Sec.osm')
    params_file = os.path.join(pkg_dir, 'config', 'params.yaml')

    use_global_localization_arg = DeclareLaunchArgument(
        'use_global_localization',
        default_value='false',
        description='Whether to use global localization'
    )

    # 时间同步相关参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # 全局时间参数设置 - 确保所有节点使用相同的时间源
    use_sim_time_param = SetParameter(
        name='use_sim_time',
        value=LaunchConfiguration('use_sim_time')
    )

    # 时间同步延迟参数
    node_startup_delay_arg = DeclareLaunchArgument(
        'node_startup_delay',
        default_value='2.0',
        description='Delay for node startup to ensure proper initialization'
    )

    bag_startup_delay_arg = DeclareLaunchArgument(
        'bag_startup_delay',
        default_value='5.0',
        description='Delay before starting rosbag to ensure all nodes are ready'
    )

    start_offset_arg = DeclareLaunchArgument(
        'start_offset',
        default_value='0.0',
        description='Start playing the bag from this time offset (in seconds)'
    )

    return LaunchDescription([
        # 参数声明
        use_sim_time_arg,
        use_sim_time_param,
        use_global_localization_arg,
        bag_file_arg,
        start_offset_arg,
        node_startup_delay_arg,
        bag_startup_delay_arg,

        # 启动信息
        LogInfo(msg="启动AGLoc系统 - 时间同步优化版本"),

        # 第一阶段：立即启动的基础节点
        # TF变换发布器 - 确保时间同步
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='transform_world_to_map',
            arguments=[
                '--frame-id', 'world',
                '--child-frame-id', 'map'
            ],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),

        Node(
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
            ],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),

        # Area Graph Data Parser - 立即启动
        Node(
            package='area_graph_data_parser',
            executable='main',
            name='main',
            output='screen',
            parameters=[
                {'osm_file_path': osm_file},
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ]
        ),

        # 第二阶段：延迟启动RViz和其他可视化节点
        TimerAction(
            period=LaunchConfiguration('node_startup_delay'),
            actions=[
                LogInfo(msg="启动RViz2可视化节点"),
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', os.path.join(pkg_dir, 'config', 'localization.rviz')],
                    parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
                ),
            ]
        ),

        # 第三阶段：延迟启动主要处理节点
        TimerAction(
            period=3.0,
            actions=[
                LogInfo(msg="启动主要定位处理节点"),
                # Main AGLoc node (Cloud Handler)
                Node(
                    package='localization_using_area_graph',
                    executable='cloud_handler',
                    name='cloud_handler',
                    parameters=[
                        params_file,
                        {'use_sim_time': LaunchConfiguration('use_sim_time')}
                    ],
                    output='screen',
                ),

                # Particle Generator node
                Node(
                    package='localization_using_area_graph',
                    executable='particle_generator',
                    name='particle_generator',
                    parameters=[
                        params_file,
                        {'use_sim_time': LaunchConfiguration('use_sim_time')}
                    ],
                    condition=IfCondition(LaunchConfiguration('use_global_localization')),
                    output='screen',
                ),

                # Robot Localization node
                Node(
                    package='wifi_loc',
                    executable='robot_loc',
                    name='robot_loc',
                    parameters=[
                        params_file,
                        {
                            'use_sim_time': LaunchConfiguration('use_sim_time'),
                            'bag_path': LaunchConfiguration('bag_file'),
                            'use_true_ap_positions': False,
                            'wait_for_robotpose': False,
                        }
                    ],
                    condition=IfCondition(LaunchConfiguration('use_global_localization')),
                    output='screen',
                ),
            ]
        ),

        # 第四阶段：最后启动rosbag播放
        TimerAction(
            period=LaunchConfiguration('bag_startup_delay'),
            actions=[
                LogInfo(msg="所有节点已启动，开始播放rosbag"),
                ExecuteProcess(
                    cmd=['ros2', 'bag', 'play',
                         LaunchConfiguration('bag_file'),
                         '--clock',
                         '--remap', '/lidar_points:=/hesai/pandar',
                         '--rate', '0.5'],
                    output='screen'
                ),
            ]
        ),

    ])

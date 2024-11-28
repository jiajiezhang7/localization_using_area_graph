from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition 
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('localization_using_area_graph')
    topology_pkg_dir = get_package_share_directory('area_graph_data_parser')

    bag_file_arg = DeclareLaunchArgument(
        'bag_file',
        default_value='/home/jay/AGLoc_ws/rosbag/0524',  # 注意不需要.db3后缀
        description='Path to ROS2 bag file (without .db3 extension)'
    )

    osm_file = os.path.join(topology_pkg_dir, 'data', 'fix_id', 'SIST1_1plus2_D.osm')
    params_file = os.path.join(pkg_dir, 'config', 'params.yaml')

    use_global_localization_arg = DeclareLaunchArgument(
        'use_global_localization',
        default_value='false',
        description='Whether to use global localization'
    )

    # 添加use_sim_time参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # Set use_sim_time for all nodes
    use_sim_time_param = SetParameter(
        name='use_sim_time',
        value=LaunchConfiguration('use_sim_time')
    )

    return LaunchDescription([
        # Parameters
        use_sim_time_arg,
        use_sim_time_param,
        use_global_localization_arg,
        bag_file_arg,
        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_dir, 'config', 'localization.rviz')],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),

        # world -> map transform
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_map',
            arguments=[
                '--frame-id', 'world',
                '--child-frame-id', 'map'
                # 默认值为单位变换，因为我们不需要任何平移和旋转
            ]
        ),


        # 0524 AG camera base link transform
        # map -> AGmap transform
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_base_link',
            arguments=[
                '--frame-id', 'map',
                '--child-frame-id', 'AGmap',
                '--x', '146.0',
                '--y', '-64.0',
                '--z', '0.0'
                # 没有旋转，使用默认的单位四元数
            ]
        ),

        # PandarQT to map transform
        # TODO Jiajie 不知道pandarQT和map的坐标系设置有什么用意，但关注到数字和map->AGmap正好是相反的
        # PandarQT -> map transform
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='pandarqt_map',
            arguments=[
                '--frame-id', 'PandarQT',
                '--child-frame-id', 'map',
                '--x', '-146.0',
                '--y', '64.0',
                '--z', '0.0'
                # 没有旋转，使用默认的单位四元数
            ]
        ),

        # Area Graph Data Parser (formerly data_process)
        # TODO Jiajie 虽然在这里传了osm_file，但未确认是否源文件有没有使用这个参数（因为我之前碰到的情况是，非得改topology_publisher.cpp中的原文件路径才可以更换文件）
        Node(
            package='area_graph_data_parser',
            executable='topology_publisher',
            name='topology_publisher',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'),
                         'osm_file': osm_file}],
            output='screen',
        ),

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
            prefix=['gdb -ex run --args'],  # 添加gdb调试
        ),

        # TODO Jiajie Fujing的run.launch源文件注释掉了，估计是另外窗口播放了rosbag
        # Data Sender node (formerly sendData.py)
        Node(
            package='localization_using_area_graph',
            executable='data_sender',
            name='data_sender',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'bag_file': LaunchConfiguration('bag_file'),  # 使用launch参数
                'start_timestamp': 0.0,
                'frequency': 10,
                'storage_id': 'sqlite3'  # 添加存储格式参数
            }],
            output='screen',
        ),

        # Particle Generator node (formerly generateParticles.py)
        # 4. 位姿初始化：
            # - 方式1：particle_generator（全局定位）
            # - 方式2：params.yaml中设定固定初始位姿
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

        # TODO Fujing的run.launch源文件注释掉了，估计没有再用自己的mapLine和mapMarkers，这意味着三个txt地图文件也不重要了，
        # 而是全面转向data_process所提供的AG信息，包括可视化和mapPC_AG(这还没找到证据)
        # Map Handler node，发布话题: /mapLine, /mapMarkers，全部在/map坐标系下
        # Node(
        #     package='localization_using_area_graph',
        #     executable='map_handler',
        #     name='map_handler',
        #     parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        #     output='screen',
        # ),

        # Destination Receiver node (formerly getDestinationFromRVIZ.py)
            #  接收RViz中设置的目标点, 订阅 "/goal_pose" 话题, 将目标点保存到yaml文件
            #  这是为导航功能准备的，纯定位不需要

        # Node(
        #     package='localization_using_area_graph',
        #     executable='destination_receiver',
        #     name='destination_receiver',
        #     parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        #     output='screen',
        # ),
    ])
from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import DeclareLaunchArgument, ExecuteProcess
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
        default_value='/home/jay/AGLoc_ws/rosbag/1226/1226.mcap',  # 注意不需要.db3后缀
        description='Path to ROS2 bag file'
    )

    osm_file = os.path.join(topology_pkg_dir, 'data', 'fix_id', 'SIST1_D_F2.osm')
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

    # 添加start_offset参数声明，可以让bag从指定时间开始播放
    start_offset_arg = DeclareLaunchArgument(
        'start_offset',
        default_value='47.0',
        description='Start playing the bag from this time offset (in seconds)'
    )

    # 添加地图文件路径
    map_file = os.path.join(pkg_dir, 'maps', 'elevator.yaml')  # 请确保地图文件路径正确

    return LaunchDescription([
        # Parameters
        use_sim_time_arg,
        use_sim_time_param,
        use_global_localization_arg,
        bag_file_arg,
        start_offset_arg,
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
            name='transform_world_to_map',
            arguments=[
                '--frame-id', 'world',
                '--child-frame-id', 'map'
                # 默认值为单位变换，因为我们不需要任何平移和旋转
            ]
        ),


        # 1226+elevator AG camera base link transform
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='transform_map_to_agmap',
            arguments=[
                # 这里已经做出了坐标系的转换，因此params.yaml中的mapExtTrans不应该再在mapAGCB中被使用
                '--frame-id', 'map',
                '--child-frame-id', 'AGmap',
                '--x', '-32.5',
                '--y', '-7.0',
                '--z', '-16.0',
                '--yaw', '3.26321'
            ]
        ),
        # hesai_lidar -> AGMap, indentical, 为了让AGmap和hesai_lidar的坐标系一致，因为定位的原理需要ICP比较
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='transform_agmap_to_hesailidar',
            arguments=[
                '--frame-id', 'map',
                # 归根结底是因为Fujing的定位根本没有用到odom，因此我的修改是：禁用rosbag中的tf发布
                '--child-frame-id', 'hesai_lidar',
                '--x', '0.0',
                '--y', '0.0',
                '--z', '0.0'
            ]
        ),
        # Area Graph Data Parser
        Node(
            package='area_graph_data_parser',
            executable='main',
            name='main', 
            output='screen',
            parameters=[{
                'osm_file_path': osm_file  # 将参数传递给main节点
            }]
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
            # prefix=['gdb -ex run --args'], 
        ),

        # Play Bag File
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', '-s', 'mcap',
                 '--remap', '/tf:=/ignored_tf', '/tf_static:=/ignored_tf_static',
                 '--start-offset', LaunchConfiguration('start_offset'),
                 LaunchConfiguration('bag_file'),
                 '--clock'],
            output='screen'
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

        # 开启gridmap的用意是为了把AGmap和gridmap对齐，从而得到map->AGmap的转换矩阵
        # Map Server Node
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'yaml_filename': map_file
            }]
        ),

        # Lifecycle Manager Node
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'autostart': True},
                {'node_names': ['map_server']}
            ]
        ),

    ])
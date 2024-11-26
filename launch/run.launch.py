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
        default_value='/home/johnnylin/AGLoc_ws/bags/0524',  # 注意不需要.db3后缀
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
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', os.path.join(pkg_dir, 'config', 'areaGraphRviz.rviz')],
        #     parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        # ),

        # Static transforms
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_map',
            arguments=['0', '0', '0', '0', '0', '0', '/world', '/map'],
        ),

        # 0510 AG camera base link transform
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_base_link',
            arguments=['9.8', '-34.3', '0.0', '0', '0', '-0.649', '0.760', 'map', 'AGmap'],
        ),

        # PandarQT to map transform
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='pandarqt_map',
            arguments=['0', '0', '0', '0', '0', '0', 'PandarQT', 'map'],
        ),

        # Area Graph Data Parser (formerly data_process)
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
        ),

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

        # Map Handler node
        Node(
            package='localization_using_area_graph',
            executable='map_handler',
            name='map_handler',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen',
        ),

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
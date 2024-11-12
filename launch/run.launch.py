from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('localization_using_area_graph')
    
    # Declare parameters
    params_file = os.path.join(pkg_dir, 'config', 'params.yaml')
    
    # Create and return launch description
    return LaunchDescription([
        # Load parameters
        DeclareLaunchArgument(
            'params_file',
            default_value=params_file,
            description='Path to the ROS2 parameters file to use'
        ),
        
        # Nodes
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_dir, 'config', 'areaGraphRviz.rviz')],
        ),
        
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
            name='PandarQT_map',
            arguments=['0', '0', '0', '0', '0', '0', 'PandarQT', 'map'],
        ),
        
        # Main nodes
        Node(
            package='localization_using_area_graph',
            executable='cloud_handler',
            name='cloud_handler',
            parameters=[LaunchConfiguration('params_file')],
            output='screen',
        ),
        
        Node(
            package='area_graph_data_parser',
            executable='topology_publisher',
            name='topology_publisher',
            output='screen',
        ),
    ])
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetParameter
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('localizationUsingAreaGraph')
    
    # Declare parameters
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
            
        # Set use_sim_time parameter
        SetParameter(name='use_sim_time', value=True),

        # AMCL Send Odom TF node
        Node(
            package='localization_using_area_graph',
            executable='amcl_send_odom_tf',
            name='amcl_send_odom_tf',
            output='screen'
        ),

        # Static Transform Publishers
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_PandarQT',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_link', 'PandarQT']
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='PandarQT_scan',
            arguments=['0', '0', '0', '0', '0', '0', 'PandarQT', 'scan']
        ),

        # Save AMCL Result node
        Node(
            package='localization_using_area_graph',
            executable='save_amcl_result',
            name='saveAmclTumResult',
            output='screen'
        ),

        # Pointcloud to Laserscan node
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            remappings=[('cloud_in', '/hesai/pandar')],
            parameters=[{
                'transform_tolerance': 0.01,
                'min_height': -0.1,
                'max_height': 1.3,
                'angle_min': -3.1415926,
                'angle_max': 3.1415926,
                'angle_increment': 0.003,
                'scan_time': 0.1,
                'range_min': 0.2,
                'range_max': 100.0,
                'use_inf': True,
                'concurrency_level': 2
            }],
            output='screen'
        ),

        # Map Server node
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                # TODO 路径硬编码问题
                'yaml_filename': '/home/jay/AGLoc_ws/gmapping_map/0510gmappingAreaGraph.yaml'
            }]
        ),

        # AMCL node
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            remappings=[('scan', 'scan')],
            parameters=[{
                'initial_pose_a': 1.7,
                'use_map_topic': True,
                'odom_model_type': 'omni',
                'odom_alpha5': 0.1,
                'transform_tolerance': 0.1,
                'gui_publish_rate': 10.0,
                'laser_max_beams': 3000,
                'min_particles': 500,
                'max_particles': 2000,
                'kld_err': 0.1,
                'kld_z': 0.99,
                'odom_alpha1': 0.1,
                'odom_alpha2': 0.1,
                'odom_alpha3': 0.1,
                'odom_alpha4': 0.1,
                'laser_z_hit': 0.9,
                'laser_z_short': 0.05,
                'laser_z_max': 2.0,
                'laser_z_rand': 0.5,
                'laser_sigma_hit': 0.2,
                'laser_lambda_short': 0.1,
                'laser_model_type': 'likelihood_field',
                'laser_min_range': 0.1,
                'laser_max_range': 500.0,
                'laser_likelihood_max_dist': 2.0,
                'update_min_d': 0.2,
                'update_min_a': 0.5,
                'resample_interval': 1,
                'recovery_alpha_slow': 0.0,
                'recovery_alpha_fast': 0.0
            }]
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_dir, 'config', 'amclRviz.rviz')],
            output='screen'
        )
    ])
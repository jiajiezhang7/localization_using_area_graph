#!/usr/bin/env python3
# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import EqualsSubstitution
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.substitutions import NotEqualsSubstitution
from launch_ros.actions import LoadComposableNodes, SetParameter
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # 获取启动目录
    bringup_dir = get_package_share_directory('nav2_bringup')
    agloc_dir = get_package_share_directory('localization_using_area_graph')

    namespace = LaunchConfiguration('namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    container_name_full = (namespace, '/', container_name)
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    use_global_localization = LaunchConfiguration('use_global_localization')
    use_bag = LaunchConfiguration('use_bag')

    # 修改为包含AGLoc相关节点
    lifecycle_nodes = ['map_server', 'agloc_localizer']

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True),
        allow_substs=True)

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='顶级命名空间')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value='',
        description='地图YAML文件的完整路径')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='使用仿真时间')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(agloc_dir, 'config', 'agloc_nav2_params.yaml'),
        description='所有启动节点使用的ROS2参数文件的完整路径')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='自动启动导航栈')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='False',
        description='如果为True则使用组合式启动')

    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name', default_value='nav2_container',
        description='如果使用组合式启动，节点将加载到的容器名称')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='当节点崩溃时是否重新生成。当禁用组合时应用。')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='日志级别')
        
    declare_use_global_localization_cmd = DeclareLaunchArgument(
        'use_global_localization',
        default_value='false',
        description='是否使用全局定位')
        
    declare_use_bag_cmd = DeclareLaunchArgument(
        'use_bag',
        default_value='true',
        description='是否使用ROS2 bag进行测试')

    # 创建节点组
    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            Node(
                condition=IfCondition(EqualsSubstitution(LaunchConfiguration('map'), '')),
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                condition=IfCondition(NotEqualsSubstitution(LaunchConfiguration('map'), '')),
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params,
                            {'yaml_filename': map_yaml_file}],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            # AGLoc的组件 - 替代了原本的AMCL节点
            # 添加CloudHandler节点（AGLoc系统核心组件）
            Node(
                package='localization_using_area_graph',
                executable='cloud_handler_node',
                name='cloud_handler',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params,
                           {'use_sim_time': use_sim_time}],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            # 添加AGLoc适配器节点（连接AGLoc与Nav2）
            Node(
                package='localization_using_area_graph',
                executable='agloc_localizer_node',
                name='agloc_localizer',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            # 区域图数据解析器
            Node(
                package='area_graph_data_parser',
                executable='main',
                name='main', 
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time}
                ],
                remappings=remappings),
            # 粒子生成器（用于全局定位）
            Node(
                condition=IfCondition(LaunchConfiguration('use_global_localization')),
                package='localization_using_area_graph',
                executable='particle_generator',
                name='particle_generator',
                parameters=[
                    os.path.join(agloc_dir, 'config', 'params.yaml'),
                    {'use_sim_time': use_sim_time}
                ],
                output='screen',
                remappings=remappings),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'autostart': autostart},
                            {'node_names': lifecycle_nodes}])
        ]
    )

    # 组合式节点加载器 - 与上面的类似但使用组合式加载
    load_composable_nodes = GroupAction(
        condition=IfCondition(use_composition),
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            # 地图服务器组合节点 - 如果map参数为空
            LoadComposableNodes(
                target_container=container_name_full,
                condition=IfCondition(EqualsSubstitution(LaunchConfiguration('map'), '')),
                composable_node_descriptions=[
                    ComposableNode(
                        package='nav2_map_server',
                        plugin='nav2_map_server::MapServer',
                        name='map_server',
                        parameters=[configured_params],
                        remappings=remappings),
                ],
            ),
            # 地图服务器组合节点 - 如果map参数不为空
            LoadComposableNodes(
                target_container=container_name_full,
                condition=IfCondition(NotEqualsSubstitution(LaunchConfiguration('map'), '')),
                composable_node_descriptions=[
                    ComposableNode(
                        package='nav2_map_server',
                        plugin='nav2_map_server::MapServer',
                        name='map_server',
                        parameters=[configured_params,
                                    {'yaml_filename': map_yaml_file}],
                        remappings=remappings),
                ],
            ),
            # AGLoc相关组合节点
            LoadComposableNodes(
                target_container=container_name_full,
                composable_node_descriptions=[
                    # 这里不使用组合式加载AGLoc节点，因为AGLoc节点可能尚未适配组合式加载
                    ComposableNode(
                        package='nav2_lifecycle_manager',
                        plugin='nav2_lifecycle_manager::LifecycleManager',
                        name='lifecycle_manager_localization',
                        parameters=[{'autostart': autostart,
                                    'node_names': lifecycle_nodes}]),
                ],
            )
        ]
    )

    # 创建启动描述并填充
    ld = LaunchDescription()

    # 设置环境变量
    ld.add_action(stdout_linebuf_envvar)

    # 声明启动选项
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_use_global_localization_cmd)
    ld.add_action(declare_use_bag_cmd)

    # 添加动作以启动所有定位节点
    ld.add_action(load_nodes)
    ld.add_action(load_composable_nodes)

    return ld

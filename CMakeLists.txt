cmake_minimum_required(VERSION 3.8)
project(localization_using_area_graph)

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)           # 添加这个
find_package(tf2_msgs REQUIRED)            # 添加这个
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenMP REQUIRED)
find_package(area_graph_data_parser REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED COMPONENTS
  common
  io
  filters
  registration
  kdtree
  search
  sample_consensus
  segmentation
)
# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${pcl_conversions_INCLUDE_DIRS}
  ${pcl_ros_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Create utility library
add_library(utility
  src/utility.cpp
)

ament_target_dependencies(utility
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
  cv_bridge
  image_transport
  area_graph_data_parser
)

target_link_libraries(utility
  ${PCL_LIBRARIES}
  ${pcl_conversions_LIBRARIES}
  ${pcl_ros_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${CERES_LIBRARIES}
  OpenMP::OpenMP_CXX
)

# Create executables for all nodes
## Main AGLoc node
add_executable(cloud_handler
  src/cloudHandler.cpp
  src/cloudBase.cpp
  src/cloudInitializer.cpp
)

target_link_libraries(cloud_handler
  utility
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${CERES_LIBRARIES}
  ${message_filters_LIBRARIES}
  OpenMP::OpenMP_CXX
  tf2_geometry_msgs::tf2_geometry_msgs  # 修改这一行
)

## AMCL TF broadcaster
add_executable(amcl_odom_tf_broadcaster
  src/amcl_odom_tf_broadcaster_node.cpp
  src/amcl_odom_tf_broadcaster.cpp
)

target_link_libraries(amcl_odom_tf_broadcaster
  utility
)

## Particle Generator
add_executable(particle_generator
  src/particle_generator_node.cpp
  src/particle_generator.cpp
)

target_link_libraries(particle_generator
  utility
  ${PCL_LIBRARIES}
)

## Data Sender
add_executable(data_sender
  src/data_sender_node.cpp
  src/data_sender.cpp
)

target_link_libraries(data_sender
  utility
  rosbag2_cpp::rosbag2_cpp
  rosbag2_storage::rosbag2_storage
)

## Map Handler
add_executable(map_handler
  src/map_handler_node.cpp
  src/map_handler.cpp
)

target_link_libraries(map_handler
  utility
)

## Destination Receiver
add_executable(destination_receiver
  src/destination_receiver_node.cpp
  src/destination_receiver.cpp
)

target_link_libraries(destination_receiver
  utility
  yaml-cpp
)

# Add dependencies to all executables
foreach(target
    cloud_handler
    amcl_odom_tf_broadcaster
    particle_generator
    data_sender
    map_handler
    destination_receiver)
  ament_target_dependencies(${target}
    rclcpp
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    visualization_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs  # 添加这一行
    cv_bridge
    image_transport
    area_graph_data_parser
  )
endforeach()

# Install targets
install(
  TARGETS
    cloud_handler
    utility
    amcl_odom_tf_broadcaster
    particle_generator
    data_sender
    map_handler
    destination_receiver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install include directory
install(
  DIRECTORY include/
  DESTINATION include
)

# Install launch and config files
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
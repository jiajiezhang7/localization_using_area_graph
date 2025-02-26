cmake_minimum_required(VERSION 3.8)
project(localization_using_area_graph)

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

# 处理 Anaconda 环境中的库冲突
if(DEFINED ENV{CONDA_PREFIX})
  message(STATUS "Conda environment detected: $ENV{CONDA_PREFIX}")
  # 设置 RPATH 以优先使用系统库而不是 Conda 库
  set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
  set(CMAKE_INSTALL_RPATH "/usr/lib/x86_64-linux-gnu:/usr/lib/gcc/x86_64-linux-gnu/11")
  # 忽略 Conda 库的警告
  set(CMAKE_POLICY_DEFAULT_CMP0074 NEW)
  set(CMAKE_POLICY_DEFAULT_CMP0057 NEW)
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
find_package(tf2_eigen REQUIRED)           
find_package(tf2_msgs REQUIRED)            
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
find_package(rss REQUIRED)
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
  OpenCV  # 添加这一行
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
  tf2_geometry_msgs::tf2_geometry_msgs  
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

ament_target_dependencies(particle_generator
  rclcpp
  sensor_msgs
  pcl_conversions
  rss
)



# Add dependencies to all executables
foreach(target
    cloud_handler
    particle_generator)
    
  ament_target_dependencies(${target}
    rclcpp
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    visualization_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs  
    cv_bridge
    image_transport
    area_graph_data_parser
    OpenCV  # 添加这一行
  )
endforeach()

# Install targets
install(
  TARGETS
    cloud_handler
    utility
    particle_generator
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
install(DIRECTORY
  launch 
  config
  maps
  DESTINATION share/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
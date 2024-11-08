cmake_minimum_required(VERSION 3.8)
project(localization_using_area_graph)

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
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenMP REQUIRED)
find_package(area_graph_data_parser REQUIRED)

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
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
  ${OpenCV_LIBRARIES}
  ${CERES_LIBRARIES}
  OpenMP::OpenMP_CXX
)

# Create the main executable
add_executable(cloud_handler
  src/cloudHandler.cpp
  src/cloudBase.cpp
  src/cloudInitializer.cpp
)

# Link utility library to main executable
target_link_libraries(cloud_handler
  utility
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${CERES_LIBRARIES}
  OpenMP::OpenMP_CXX
)

ament_target_dependencies(cloud_handler
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

# Install targets
install(
  TARGETS cloud_handler utility
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
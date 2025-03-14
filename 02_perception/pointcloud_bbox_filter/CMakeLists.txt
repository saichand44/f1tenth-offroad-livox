cmake_minimum_required(VERSION 3.8)
project(pointcloud_bbox_filter)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

SET(CMAKE_CXX_FLAGS "-O2 -g ${CMAKE_CXX_FLAGS}")

# find dependencies
foreach(PACKAGE 
  ament_cmake 
  ament_cmake_python
  rclcpp
  sensor_msgs
  geometry_msgs
  pcl_conversions
  PCL
  tf2_eigen
  Eigen3
  )
  find_package(${PACKAGE} REQUIRED)
endforeach()

add_library(bbox_filter SHARED 
  src/pcl_bbox_filter.cpp)

ament_target_dependencies(bbox_filter
  rclcpp
  sensor_msgs
  geometry_msgs
  pcl_conversions
  tf2_eigen
  Eigen3
)

# Include Cpp "include" directory
include_directories(include/${PROJECT_NAME})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})

# Create Cpp executable
add_executable(bbox_filter_node src/bbox_filter_node.cpp)

target_link_libraries(bbox_filter_node
  ${PCL_LIBRARIES}
  Eigen3::Eigen
  bbox_filter
)

# Install Cpp executables
install(TARGETS
  bbox_filter_node
  DESTINATION lib/${PROJECT_NAME})

# Install Python modules

# Install Python executables

# Install additional folders

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
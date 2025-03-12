cmake_minimum_required(VERSION 3.8)
project(point_cloud_undistortion)

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
  )
  find_package(${PACKAGE} REQUIRED)
endforeach()

# Include Cpp "include" directory

# Create Cpp executable

# Install Cpp executables

# Install Python modules

# Install Python executables

# Install additional folders

# Install config files

# Install launch files

ament_package()
# Copyright(c) 2020 livoxtech limited.

cmake_minimum_required(VERSION 3.14)
project(livox_ros_driver2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

list(INSERT CMAKE_MODULE_PATH 0 "${PROJECT_SOURCE_DIR}/cmake/modules")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

# Print version info
include(cmake/version.cmake)
project(${PROJECT_NAME} VERSION ${LIVOX_ROS_DRIVER2_VERSION} LANGUAGES CXX)
message(STATUS "${PROJECT_NAME} version: ${LIVOX_ROS_DRIVER2_VERSION}")

# Add ROS Version MACRO
add_definitions(-DBUILDING_ROS2)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(PCL REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# check apr
find_package(PkgConfig)
pkg_check_modules(APR apr-1)
if (APR_FOUND)
  message(${APR_INCLUDE_DIRS})
  message(${APR_LIBRARIES})
endif (APR_FOUND)

# generate custom msg headers
set(LIVOX_INTERFACES livox_interfaces2)
rosidl_generate_interfaces(${LIVOX_INTERFACES}
  "msg/CustomPoint.msg"
  "msg/CustomMsg.msg"
  DEPENDENCIES builtin_interfaces std_msgs
  LIBRARY_NAME ${PROJECT_NAME}
)

# make sure the livox_lidar_sdk_shared library is installed
find_library(LIVOX_LIDAR_SDK_LIBRARY liblivox_lidar_sdk_shared.so /usr/local/lib REQUIRED)

# find the include directories of the SDK
find_path(LIVOX_LIDAR_SDK_INCLUDE_DIR
  NAMES "livox_lidar_api.h" "livox_lidar_def.h"
  REQUIRED)

# PCL library
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# livox ros2 driver target
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/livox_ros_driver2.cpp
  src/lddc.cpp
  src/driver_node.cpp
  src/lds.cpp
  src/lds_lidar.cpp

  src/comm/comm.cpp
  src/comm/ldq.cpp
  src/comm/semaphore.cpp
  src/comm/lidar_imu_data_queue.cpp
  src/comm/cache_index.cpp
  src/comm/pub_handler.cpp

  src/parse_cfg_file/parse_cfg_file.cpp
  src/parse_cfg_file/parse_livox_lidar_cfg.cpp

  src/call_back/lidar_common_callback.cpp
  src/call_back/livox_lidar_callback.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE ${livox_sdk_INCLUDE_DIRS})

# get include directories of custom msg headers
rosidl_get_typesupport_target(cpp_typesupport_target
  ${LIVOX_INTERFACES} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME} "${cpp_typesupport_target}")

# include file directory
target_include_directories(${PROJECT_NAME} PUBLIC
  ${PCL_INCLUDE_DIRS}
  ${APR_INCLUDE_DIRS}
  ${LIVOX_LIDAR_SDK_INCLUDE_DIR}
  ${LIVOX_INTERFACES_INCLUDE_DIRECTORIES}   # for custom msgs
  3rdparty
  src
)

# link libraries
target_link_libraries(${PROJECT_NAME}
  ${LIVOX_LIDAR_SDK_LIBRARY}
  ${cpp_typesupport_target}   # for custom msgs
  ${PPT_LIBRARY}
  ${Boost_LIBRARY}
  ${PCL_LIBRARIES}
  ${APR_LIBRARIES}
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "livox_ros::DriverNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  config
  launch_ROS2
)


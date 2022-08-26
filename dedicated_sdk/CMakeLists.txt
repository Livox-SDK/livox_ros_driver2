cmake_minimum_required(VERSION 3.0)

project(livox_sdk)

set(CMAKE_CXX_STANDARD 11)

message(STATUS "main project dir: " ${PROJECT_SOURCE_DIR})

if (CMAKE_CROSSCOMPILING)
	set(THREADS_PTHREAD_ARG
		"PLEASE_FILL_OUT-FAILED_TO_RUN"
		CACHE STRING "Result from TRY_RUN" FORCE)
endif()

if (UNIX)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
endif(UNIX)

add_subdirectory(livox_sdk/sdk_core sdk_core)
add_subdirectory(livox_sdk_vehicle/sdk_core vehicle_sdk_core)
add_subdirectory(livox_lidar_sdk/sdk_core livox_sdk_core)
add_subdirectory(livox_sdk_common)

add_subdirectory(sample/industry/industry_lidar_sample)
add_subdirectory(sample/industry/industry_lidar_upgrade)

add_subdirectory(sample/lvx/parse_lvx_file)
add_subdirectory(sample/lvx/record_lvx2_file)
add_subdirectory(sample/lvx/record_lvx2_file_industrial)
add_subdirectory(sample/lvx/obstacle_avoidance_test)

add_subdirectory(sample/vehicle/vehicle_multi_lidar_upgrade)
add_subdirectory(sample/vehicle/vehicle_lidar_upgrade)
add_subdirectory(sample/vehicle/vehicle_lidar_quick_start)
add_subdirectory(sample/vehicle/vehicle_lidar_preconfig)

add_subdirectory(sample/hap/livox_multi_lidar_upgrade)
add_subdirectory(sample/hap/livox_lidar_quick_start)
add_subdirectory(sample/hap/livox_lidar_view_quick_start)
add_subdirectory(sample/hap/livox_lidar_for_test)
add_subdirectory(sample/hap/livox_lidar_record_lvx2_file)
add_subdirectory(sample/hap/livox_lidar_parse_lvx2_file)
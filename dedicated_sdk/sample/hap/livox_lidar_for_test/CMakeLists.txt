cmake_minimum_required(VERSION 3.0)

set(DEMO_NAME livox_lidar_for_test)
add_executable(${DEMO_NAME} main.cpp)

target_link_libraries(${DEMO_NAME}
        PRIVATE
        livox_lidar_sdk_static
				)

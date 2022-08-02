cmake_minimum_required(VERSION 3.0)

set(DEMO_NAME livox_multi_lidar_upgrade)
add_executable(${DEMO_NAME} main.cpp)

target_link_libraries(${DEMO_NAME}
        PUBLIC
        ${PROJECT_NAME}_common_static
        livox_lidar_sdk_static
        )

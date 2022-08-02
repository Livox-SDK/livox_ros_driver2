cmake_minimum_required(VERSION 3.0)

set(DEMO_NAME livox_lidar_parse_lvx2_file)
add_executable(${DEMO_NAME} main.cpp)

target_link_libraries(${DEMO_NAME}
        PRIVATE
        ${PROJECT_NAME}_common_static
        livox_lidar_sdk_static
        # ${PROJECT_NAME}_vehicle_static
        # ${PROJECT_NAME}_static
        )

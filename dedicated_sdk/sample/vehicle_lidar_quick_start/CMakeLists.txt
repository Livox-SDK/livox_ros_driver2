cmake_minimum_required(VERSION 3.0)

set(DEMO_NAME lidar_quick_start)
add_executable(${DEMO_NAME} main.cpp)

target_link_libraries(${DEMO_NAME}
        PRIVATE
        ${PROJECT_NAME}_vehicle_static
        )

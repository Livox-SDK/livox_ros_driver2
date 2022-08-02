cmake_minimum_required(VERSION 3.0)

set(DEMO_NAME direct_lidar_reset)

add_executable(${DEMO_NAME} 
        parse_cfg_file.h
        parse_cfg_file.cpp
        direct_lidar_reset_main.cpp
        )

target_include_directories(${DEMO_NAME}
        PUBLIC
        rapidjson
        ../../sdk_core/include/third_party
        ../../sdk_core/include/
        )

target_link_libraries(${DEMO_NAME}
        PRIVATE
        ${PROJECT_NAME}_direct_static
        )




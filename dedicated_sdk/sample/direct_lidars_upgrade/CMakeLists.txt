cmake_minimum_required(VERSION 3.0)

set(DEMO_NAME direct_lidars_upgrade)

add_executable(${DEMO_NAME} 
        parse_cfg_file.h
        parse_cfg_file.cpp
        direct_lidars_upgrade_main.cpp
        )

target_include_directories(${DEMO_NAME}
        PUBLIC
        rapidjson
        )

target_link_libraries(${DEMO_NAME}
        PRIVATE
	${PROJECT_NAME}_common_static
        ${PROJECT_NAME}_direct_static
        ${PROJECT_NAME}_vehicle_static
        ${PROJECT_NAME}_static
        )




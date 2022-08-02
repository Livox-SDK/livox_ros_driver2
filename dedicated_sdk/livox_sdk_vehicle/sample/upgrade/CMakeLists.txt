cmake_minimum_required(VERSION 3.0)

set(DEMO_NAME lidar_upgrade)

add_executable(${DEMO_NAME} 
        livox_upgrade_tool.cpp
        livox_manager.cpp
        firmware.cpp
        upgrade.cpp
        ../../sdk_core/src/third_party/FastCRC/FastCRCsw.cpp
        )

target_link_libraries(${DEMO_NAME}
        PRIVATE
        ${PROJECT_NAME}_static
        )

target_include_directories(${DEMO_NAME}
        PUBLIC
        ../../sdk_core/include
        ../../sdk_core/include/third_party
        ../../sdk_core/src/command_handler
        ../../sdk_core/src/
        )
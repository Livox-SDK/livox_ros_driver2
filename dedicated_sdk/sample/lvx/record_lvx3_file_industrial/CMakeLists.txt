cmake_minimum_required(VERSION 3.0)

set(DEMO_NAME record_lvx3_file_industrial)
add_executable(${DEMO_NAME} main.cpp)

target_link_libraries(${DEMO_NAME}
        PRIVATE
        ${PROJECT_NAME}_common_static
        ${PROJECT_NAME}_vehicle_static
        ${PROJECT_NAME}_static
        $<$<CXX_COMPILER_ID:MSVC>:${LIVOX_SDK_COMMON_PATH}/third_party/PPT/lib/win_x64/PPT.lib>
        $<$<CXX_COMPILER_ID:Clang>:${LIVOX_SDK_COMMON_PATH}/third_party/PPT/lib/linux_x86_64_clang/libPPT.a>
        $<$<CXX_COMPILER_ID:GNU>:${LIVOX_SDK_COMMON_PATH}/third_party/PPT/lib/linux_x86_64_gnu/libPPT.a>
        )

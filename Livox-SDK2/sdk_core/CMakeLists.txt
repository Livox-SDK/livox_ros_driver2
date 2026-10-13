cmake_minimum_required(VERSION 3.0)

set(SDK_LIBRARY_STATIC livox_lidar_sdk_static)
set(SDK_LIBRARY_SHARED livox_lidar_sdk_shared)

add_library(${SDK_LIBRARY_STATIC} STATIC "")
add_library(${SDK_LIBRARY_SHARED} SHARED "")

set(LIVOX_SDK_MAJOR_VERSION "0")
set(LIVOX_SDK_MINOR_VERSION "0")
set(LIVOX_SDK_PATCH_VERSION "2")
set(LIVOX_SDK_VERSION_STRING "${LIVOX_SDK_MAJOR_VERSION}.${LIVOX_SDK_MINOR_VERSION}.${LIVOX_SDK_PATCH_VERSION}")

set(LIVOX_API_HEADER
        ../include/livox_lidar_def.h
        ../include/livox_lidar_api.h
        ../include/livox_lidar_cfg.h
        )

set_target_properties(${SDK_LIBRARY_STATIC} #${SDK_LIBRARY_SHARED} 
        PROPERTIES
        PUBLIC_HEADER "${LIVOX_API_HEADER}"
        )

if(WIN32)
  set(PLATFORM win)
else(WIN32)
  set(PLATFORM unix)
endif (WIN32)

target_compile_options(${SDK_LIBRARY_STATIC}
        PRIVATE $<$<CXX_COMPILER_ID:GNU>:-Wall>#-Wno-c++11-long-long>
        PRIVATE $<$<CXX_COMPILER_ID:AppleClang>:-Wno-unknown-pragmas -Wall -Werror -Wno-c++11-long-long>
        PRIVATE $<$<CXX_COMPILER_ID:Clang>:-Wno-unknown-pragmas -Wall -Werror -Wno-c++11-long-long>
        )

target_compile_options(${SDK_LIBRARY_SHARED}
        PRIVATE $<$<CXX_COMPILER_ID:GNU>:-Wall>#-Wno-c++11-long-long>
        PRIVATE $<$<CXX_COMPILER_ID:AppleClang>:-Wno-unknown-pragmas -Wall -Werror -Wno-c++11-long-long>
        PRIVATE $<$<CXX_COMPILER_ID:Clang>:-Wno-unknown-pragmas -Wall -Werror -Wno-c++11-long-long>
        )

set(LIVOX_PRIVATE_INCLUDE_DIR 
        ../3rdparty 
        ../3rdparty/spdlog 
        .
        )
set(LIVOX_PUBLIC_INCLUDE_DIR 
        ../include
        )

target_include_directories(
        ${SDK_LIBRARY_STATIC}
        PUBLIC
        ${LIVOX_PUBLIC_INCLUDE_DIR}
        PRIVATE
        ${LIVOX_PRIVATE_INCLUDE_DIR}
        )        
target_include_directories(
        ${SDK_LIBRARY_SHARED}
        PUBLIC
        ${LIVOX_PUBLIC_INCLUDE_DIR}
        PRIVATE
        ${LIVOX_PRIVATE_INCLUDE_DIR}
        )

set(MAIN_SOURCES
        device_manager.cpp
        livox_lidar_sdk.cpp
        params_check.cpp
        parse_cfg_file.cpp
        upgrade_manager.cpp
        )
set(BASE_SOURCES
        base/io_loop.cpp
        base/thread_base.cpp
        base/io_thread.cpp
        base/logging.cpp
        base/network/${PLATFORM}/network_util.cpp
        base/multiple_io/multiple_io_base.cpp
        base/multiple_io/multiple_io_epoll.cpp
        base/multiple_io/multiple_io_poll.cpp
        base/multiple_io/multiple_io_select.cpp
        base/multiple_io/multiple_io_kqueue.cpp
        base/wake_up/${PLATFORM}/wake_up_pipe.cpp
        )
set(COMM_SOURCES
        comm/comm_port.cpp
        comm/sdk_protocol.cpp
        comm/generate_seq.cpp
        )
set(UPGRADE_SOURCES
        upgrade_manager.cpp
        upgrade/firmware.cpp
        upgrade/livox_lidar_upgrader.cpp
        )
set(LOGGER_HANDLER_SOURCES
        logger_handler/logger_manager.cpp
        logger_handler/logger_handler.cpp
        logger_handler/file_manager.cpp
        )
set(DATA_HANDLER_SOURCES
        data_handler/data_handler.cpp
        )
set(COMMAND_HANDLER_SOURCES
        command_handler/command_impl.cpp
        command_handler/general_command_handler.cpp
        command_handler/hap_command_handler.cpp
        command_handler/mid360_command_handler.cpp
        command_handler/build_request.cpp
        command_handler/parse_lidar_state_info.cpp
        command_handler/mid360s_command_handler.cpp
        command_handler/avia2_command_handler.cpp
        command_handler/mid360l_command_handler.cpp
        )
set(DEBUG_POINT_CLOUD_HANDLER_SOURCES
        debug_point_cloud_handler/debug_point_cloud_manager.cpp
        debug_point_cloud_handler/debug_point_cloud_handler.cpp
        )

set(LIVOX_SOURCES
        ../3rdparty/FastCRC/FastCRC_tables.h
        ../3rdparty/FastCRC/FastCRCsw.cpp
        ${MAIN_SOURCES}
        ${BASE_SOURCES}
        ${COMM_SOURCES}
        ${UPGRADE_SOURCES}
        ${LOGGER_HANDLER_SOURCES}
        ${DATA_HANDLER_SOURCES}
        ${COMMAND_HANDLER_SOURCES}
        ${DEBUG_POINT_CLOUD_HANDLER_SOURCES}
        )

target_sources(${SDK_LIBRARY_STATIC}
        PRIVATE
        ${LIVOX_SOURCES}
        )
target_sources(${SDK_LIBRARY_SHARED}
        PRIVATE
        ${LIVOX_SOURCES}
        )

install(TARGETS ${SDK_LIBRARY_STATIC} ${SDK_LIBRARY_SHARED}
        PUBLIC_HEADER DESTINATION include
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib)

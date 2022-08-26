cmake_minimum_required(VERSION 3.0)

set(SDK_LIBRARY livox_lidar_sdk_static)

add_library(${SDK_LIBRARY} STATIC "")

set(LIVOX_SDK_MAJOR_VERSION "2")
set(LIVOX_SDK_MINOR_VERSION "3")
set(LIVOX_SDK_PATCH_VERSION "0")
set(LIVOX_SDK_VERSION_STRING "${LIVOX_SDK_MAJOR_VERSION}.${LIVOX_SDK_MINOR_VERSION}.${LIVOX_SDK_PATCH_VERSION}")

target_include_directories(${SDK_LIBRARY}
        PUBLIC
        include
        include/comm
        include/third_party/FastCRC
        include/third_party/spdlog
        include/third_party/cmdline
        include/third_patry/rapidjson
        PRIVATE
        src)

set_target_properties(${SDK_LIBRARY} PROPERTIES PUBLIC_HEADER "include/livox_lidar_def.h;include/livox_lidar_api.h")

if(WIN32)
  set(PLATFORM win)
else(WIN32)
  set(PLATFORM unix)
endif (WIN32)

target_compile_options(${SDK_LIBRARY}
        PRIVATE $<$<CXX_COMPILER_ID:GNU>:-Wall>#-Wno-c++11-long-long>
        PRIVATE $<$<CXX_COMPILER_ID:AppleClang>:-Wno-unknown-pragmas -Wall -Werror -Wno-c++11-long-long>
        PRIVATE $<$<CXX_COMPILER_ID:Clang>:-Wno-unknown-pragmas -Wall -Werror -Wno-c++11-long-long>)

target_sources(${SDK_LIBRARY}
        PRIVATE
        src/third_party/FastCRC/FastCRC_tables.hpp
        src/third_party/FastCRC/FastCRCsw.cpp
        src/base/io_loop.h
        src/base/io_loop.cpp
        src/base/thread_base.h
        src/base/thread_base.cpp
        src/base/io_thread.h
        src/base/io_thread.cpp
        src/base/network/network_util.h
        src/base/network/${PLATFORM}/network_util.cpp
        src/base/logging.h
        src/base/logging.cpp
        src/base/noncopyable.h

        src/livox_lidar_sdk.cpp
        src/parse_cfg_file.h
        src/parse_cfg_file.cpp
        src/params_check.h
        src/params_check.cpp
        src/device_manager.h
        src/device_manager.cpp

        src/comm/comm_port.cpp
        src/comm/sdk_protocol.h
        src/comm/sdk_protocol.cpp
        src/comm/define.h
        src/comm/generate_seq.h
        src/comm/generate_seq.cpp

        src/comm/CRC/Crc.h
        src/comm/CRC/Crc.cpp
        src/comm/CRC/Crc_Cfg.h
        src/comm/CRC/Crc_Cfg.cpp

        src/data_handler/data_handler.h
        src/data_handler/data_handler.cpp

        src/command_handler/command_handler.h
        src/command_handler/command_impl.h
        src/command_handler/command_impl.cpp
        src/command_handler/general_command_handler.h
        src/command_handler/general_command_handler.cpp
        src/command_handler/hap_command_handler.h
        src/command_handler/hap_command_handler.cpp
        src/command_handler/mid360_command_handler.h
        src/command_handler/mid360_command_handler.cpp
        src/command_handler/pa_command_handler.h
        src/command_handler/pa_command_handler.cpp

        src/command_handler/build_request.h
        src/command_handler/build_request.cpp

        src/command_handler/parse_lidar_state_info.h
        src/command_handler/parse_lidar_state_info.cpp
        
        src/base/multiple_io/multiple_io_base.h
        src/base/multiple_io/multiple_io_base.cpp
        src/base/multiple_io/multiple_io_factory.h
        src/base/multiple_io/multiple_io_epoll.h
        src/base/multiple_io/multiple_io_epoll.cpp
        src/base/multiple_io/multiple_io_poll.h
        src/base/multiple_io/multiple_io_poll.cpp
        src/base/multiple_io/multiple_io_select.h
        src/base/multiple_io/multiple_io_select.cpp
        src/base/multiple_io/multiple_io_kqueue.h
        src/base/multiple_io/multiple_io_kqueue.cpp
        src/base/wake_up/wake_up_pipe.h
        src/base/wake_up/${PLATFORM}/wake_up_pipe.cpp)

install(TARGETS ${SDK_LIBRARY}
        PUBLIC_HEADER DESTINATION include
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib)

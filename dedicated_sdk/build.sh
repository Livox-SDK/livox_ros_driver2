#!/bin/bash

readonly VERSION_ROS1="ROS1"
readonly VERSION_ROS2="ROS2"

ROS_VERSION=""

function CheckParams() {
    echo "param 1:"$1
    # Set working ROS version
    if [ "$1" = "ROS2" ]; then
        ROS_VERSION=${VERSION_ROS2}
    elif [ "$1" = "ROS1" ]; then
        ROS_VERSION=${VERSION_ROS1}
    else
        echo "Invalid Argument"
        exit
    fi
    echo "ROS version is: "$ROS_VERSION
}

# function RemoveStaticLib() {
#     if [ ${ROS_VERSION} = ${VERSION_ROS2} ]; then

#         if [ -f "/usr/local/lib/liblivox_sdk_common_static.a" ]; then
#             rm /usr/local/lib/liblivox_sdk_common_static.a
#         fi

#         if [ -f "/usr/local/lib/liblivox_sdk_direct_static.a"]; then
#             rm /usr/local/lib/liblivox_sdk_direct_static.a
#         fi
#         if [ -f "/usr/local/lib/liblivox_sdk_static.a"]; then
#             rm /usr/local/lib/liblivox_sdk_static.a
#         fi
#         if [ -f "/usr/local/lib/liblivox_sdk_vehicle_static.a"]; then
#             rm /usr/local/lib/liblivox_sdk_vehicle_static.a
#         fi
#     fi
# }


function GenerateCmakeFile() {
    # substitute the files/folders: CMakeList.txt, package.xml(s)
    if [ ${ROS_VERSION} = ${VERSION_ROS1} ]; then
        cp ./CMakeLists.txt.static ./CMakeLists.txt
        cp ./livox_sdk/sdk_core/CMakeLists.txt.static ./livox_sdk/sdk_core/CMakeLists.txt
        cp ./livox_sdk_common/CMakeLists.txt.static ./livox_sdk_common/CMakeLists.txt
        cp ./livox_sdk_direct/sdk_core/CMakeLists.txt.static ./livox_sdk_direct/sdk_core/CMakeLists.txt
        cp ./livox_sdk_vehicle/sdk_core/CMakeLists.txt.static ./livox_sdk_vehicle/sdk_core/CMakeLists.txt
        cp ./livox_lidar_sdk/sdk_core/CMakeLists.txt.static ./livox_lidar_sdk/sdk_core/CMakeLists.txt

    elif [ ${ROS_VERSION} = ${VERSION_ROS2} ]; then
        cp ./CMakeLists.txt.shared ./CMakeLists.txt
        cp ./livox_sdk/sdk_core/CMakeLists.txt.shared ./livox_sdk/sdk_core/CMakeLists.txt
        cp ./livox_sdk_common/CMakeLists.txt.shared ./livox_sdk_common/CMakeLists.txt
        cp ./livox_sdk_direct/sdk_core/CMakeLists.txt.shared ./livox_sdk_direct/sdk_core/CMakeLists.txt
        cp ./livox_sdk_vehicle/sdk_core/CMakeLists.txt.shared ./livox_sdk_vehicle/sdk_core/CMakeLists.txt
        cp ./livox_lidar_sdk/sdk_core/CMakeLists.txt.shared ./livox_lidar_sdk/sdk_core/CMakeLists.txt
    fi
}

function Complier() {
    if [ ! -d "./build" ];then
        mkdir build
    else
        echo "文件夹已经存在"
    fi

    cd ./build
    cmake .. && make -j && make install
}

cd `dirname $0`
echo "Working Path: "`pwd`

CheckParams $1
# RemoveStaticLib
GenerateCmakeFile
Complier


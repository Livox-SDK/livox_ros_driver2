#!/bin/bash

readonly VERSION_ROS1="ROS1"
readonly VERSION_ROS2="ROS2"

pushd `pwd` > /dev/null
cd `dirname $0`
echo "Working Path: "`pwd`

ROS_VERSION=""

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

# clear `build/` folder.
# TODO: Do not clear these folders, if the last build is based on the same ROS version.
rm -rf ../../build/
rm -rf ../../devel/
rm -rf ../../install/
# clear src/CMakeLists.txt if it exists.
if [ -f ../CMakeLists.txt ]; then
    rm -f ../CMakeLists.txt
fi

# exit

# substitute the files/folders: CMakeList.txt, package.xml(s)
if [ ${ROS_VERSION} = ${VERSION_ROS1} ]; then
    if [ -f CMakeLists.txt ]; then
        rm CMakeLists.txt
    fi
    if [ -f package.xml ]; then
        rm package.xml
    fi
    cp -f CMakeLists_ROS1.txt CMakeLists.txt
    cp -f package_ROS1.xml package.xml
elif [ ${ROS_VERSION} = ${VERSION_ROS2} ]; then
    if [ -f CMakeLists.txt ]; then
        rm CMakeLists.txt
    fi
    if [ -f package.xml ]; then
        rm package.xml
    fi
    cp CMakeLists_ROS2.txt CMakeLists.txt
    cp -f package_ROS2.xml package.xml
    cp -rf config_ROS2/ config/
    cp -rf launch_ROS2/ launch/
    cp ../livox_interfaces/package_ROS2.xml ../livox_interfaces/package.xml
    cp ../livox_sdk_vendor/package_ROS2.xml ../livox_sdk_vendor/package.xml
fi

# build
pushd `pwd` > /dev/null
if [ $ROS_VERSION = ${VERSION_ROS1} ]; then
    cd ../../
    catkin_make
elif [ $ROS_VERSION = ${VERSION_ROS2} ]; then
    cd ../../
    colcon build
fi
popd > /dev/null

# remove the substituted folders/files
rm CMakeLists.txt

if [ $ROS_VERSION = ${VERSION_ROS2} ]; then
    rm ../livox_interfaces/package.xml
    rm ../livox_sdk_vendor/package.xml
    rm -rf config/
    rm -rf launch/
fi


popd > /dev/null

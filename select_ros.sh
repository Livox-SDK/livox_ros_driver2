#!/bin/bash

readonly VERSION_ROS1="ROS1"
readonly VERSION_ROS2="ROS2"

cd `dirname $0`
echo "Working Path: "`pwd`

ROS_VERSION=""
ROS_HUMBLE=""

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

# substitute the files/folders: CMakeList.txt, package.xml(s)
if [ -d launch ]; then
    rm -rf launch/
fi

if [ -f package.xml ]; then
    rm package.xml
fi

if [ ${ROS_VERSION} = ${VERSION_ROS1} ]; then
    cp -f package_ROS1.xml package.xml
elif [ ${ROS_VERSION} = ${VERSION_ROS2} ]; then
    cp -f package_ROS2.xml package.xml
    cp -rf launch_ROS2/ launch/
fi

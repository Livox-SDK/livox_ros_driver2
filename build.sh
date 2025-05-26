#!/bin/bash

readonly VERSION_ROS1="ROS1"
readonly VERSION_ROS2="ROS2"

pushd `pwd` > /dev/null
cd `dirname $0`
echo "Working Path: "`pwd`

ROS_VERSION=""
ROS_DISTRO="jazzy" # Default to jazzy

# Set working ROS version
if [ -z "$1" ]; then
    # No argument provided, use default ROS2 (Jazzy)
    ROS_VERSION=${VERSION_ROS2}
    # ROS_DISTRO is already "jazzy" by default
    echo "No argument provided, defaulting to ROS2 Jazzy."
elif [ "$1" = "ROS1" ]; then
    ROS_VERSION=${VERSION_ROS1}
    ROS_DISTRO="" # Not applicable for ROS1
elif [ "$1" = "ROS2" ]; then
    # Generic ROS2 argument, use default ROS2 (Jazzy)
    ROS_VERSION=${VERSION_ROS2}
    # ROS_DISTRO is already "jazzy" by default
    echo "Generic ROS2 argument, defaulting to Jazzy."
else
    # Argument provided is assumed to be a ROS2 distribution name
    ROS_VERSION=${VERSION_ROS2}
    ROS_DISTRO="$1"
    echo "Using ROS2 distribution: $ROS_DISTRO"
fi

# Basic validation for ROS_DISTRO if ROS_VERSION is ROS2
if [ "${ROS_VERSION}" = "${VERSION_ROS2}" ]; then
    if [ -z "${ROS_DISTRO}" ]; then
        echo "Error: ROS_DISTRO is empty for ROS2 build. This should not happen."
        exit 1
    fi
    # You could add a list of known/supported distros here for stricter validation if desired
    # Example: if [[ ! " humble jazzy iron " =~ " ${ROS_DISTRO} " ]]; then ...
fi

echo "ROS version is: ${ROS_VERSION}"
if [ "${ROS_VERSION}" = "${VERSION_ROS2}" ]; then
    echo "ROS distribution is: ${ROS_DISTRO}"
fi

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
    if [ -f package.xml ]; then
        rm package.xml
    fi
    cp -f package_ROS1.xml package.xml
elif [ ${ROS_VERSION} = ${VERSION_ROS2} ]; then
    if [ -f package.xml ]; then
        rm package.xml
    fi
    cp -f package_ROS2.xml package.xml
    cp -rf launch_ROS2/ launch/
fi

# build
pushd `pwd` > /dev/null
if [ $ROS_VERSION = ${VERSION_ROS1} ]; then
    cd ../../
    catkin_make -DROS_EDITION=${VERSION_ROS1}
elif [ $ROS_VERSION = ${VERSION_ROS2} ]; then
    # cd ../../ # Modified: Keep build context in /app for sandbox environment
    colcon build --cmake-args -DROS_EDITION=${VERSION_ROS2} -DROS_DISTRO=${ROS_DISTRO}
fi
popd > /dev/null

# remove the substituted folders/files
if [ $ROS_VERSION = ${VERSION_ROS2} ]; then
    rm -rf launch/
fi

popd > /dev/null

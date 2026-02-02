#!/bin/bash

readonly VERSION_ROS1="ROS1"
readonly VERSION_ROS2="ROS2"


pushd `pwd` > /dev/null
cd `dirname $0`
echo "Working Path: "`pwd`

ROS_VERSION=""

# Set working ROS version
if [[ "$1" = "ROS1" || "$1" = "ROS2" ]]; then
	ROS_VERSION="$1"
else
    echo "Invalid Argument"
    exit
fi
echo "ROS version is: "$ROS_VERSION"/""$ROS_DISTRO"

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

CMAKE_VER="$(cmake --version | awk 'NR==1{print $3}')"

# currert_version >= 3.27 ?
if printf '%s\n' "3.27" "$CMAKE_VER" | sort -V -C; then
  SET_NEW_POLICY="-DCMAKE_POLICY_DEFAULT_CMP0144=NEW"
else
  SET_NEW_POLICY=""
fi

# build
pushd `pwd` > /dev/null
if [ $ROS_VERSION = ${VERSION_ROS1} ]; then
    cd ../../
    catkin_make -DROS_EDITION=${VERSION_ROS1} ${SET_NEW_POLICY}
elif [ $ROS_VERSION = ${VERSION_ROS2} ]; then
    cd ../../
	colcon build --cmake-args -DROS_EDITION=${VERSION_ROS2} -DROS_DISTRO="${ROS_DISTRO}" ${SET_NEW_POLICY} 
fi
popd > /dev/null

# remove the substituted folders/files
if [ $ROS_VERSION = ${VERSION_ROS2} ]; then
    rm -rf launch/
fi

popd > /dev/null 

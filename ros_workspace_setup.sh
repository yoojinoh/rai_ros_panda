#!/bin/bash
ROS_VERSION=$1
SCRIPTPATH="$( cd "$(dirname "$0")" ; pwd -P )"
PROJPATH=$(dirname $SCRIPTPATH)

echo $SCRIPTPATH

mkdir -p $SCRIPTPATH/catkin_ws/src

cd $SCRIPTPATH/catkin_ws/
source /opt/ros/$ROS_VERSION/setup.bash

catkin_make

source $SCRIPTPATH/catkin_ws/devel/setup.bash

# clone repo
cd $SCRIPTPATH/catkin_ws/src
git clone https://github.com/yoojinoh/rai_ros_panda.git


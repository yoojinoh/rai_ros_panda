## (Method 1) Minimum installation
You need ROS installed by default
1. Create a project folder.
```
mkdir practical_robotics && cd practical_robotics
```
2. Create a bash script with the following and save it as **ros_workspace_setup.sh**
The following script will do:
* create catkin workspace
* create non_catkin workspace, clone and compile robotics-course repository
* download franka_description folder
* clone this repository
* catkin_make catkin workspace
```
#!/bin/bash
ROS_VERSION=$1
SCRIPTPATH="$( cd "$(dirname "$0")" ; pwd -P )"
PROJPATH=$(dirname $SCRIPTPATH)

echo $SCRIPTPATH

mkdir -p $SCRIPTPATH/catkin_ws/src

mkdir $SCRIPTPATH/catkin_ws/non_catkin
cd $SCRIPTPATH/catkin_ws/non_catkin
touch CATKIN_IGNORE

git clone --recursive https://github.com/humans-to-robots-motion/robotics-course.git
cd robotics-course
mkdir build && cd build
cmake ..
make -j $(command nproc)
echo "Successfully compiled robotics-course!!"

cd $SCRIPTPATH/catkin_ws/
source /opt/ros/$ROS_VERSION/setup.bash

catkin_make
source $SCRIPTPATH/catkin_ws/devel/setup.bash

# clone repo
cd $SCRIPTPATH/catkin_ws/src
git clone https://github.com/yoojinoh/rai_ros_panda.git

cd $SCRIPTPATH/catkin_ws/src
# install subversion for downloading subfolder in git repo
sudo apt install subversion -y

# download only franka_description from repo
svn export https://github.com/frankaemika/franka_ros.git/trunk/franka_description

cd $SCRIPTPATH/catkin_ws/
catkin_make

source $SCRIPTPATH/catkin_ws/devel/setup.bash
echo "Sourced Catkin workspace!!!"

```
3. Run the bash script. Put in your ros version as argument!! (Ubuntu 18.04-melodic/20.04-noetic) This will create a ROS catkin workspace and clone the repository 
```
bash ros_workspace_setup.sh melodic
```


## (Method 2) Full installation including Panda Robot

1. Install [Franka Control Interface (FCI)](https://frankaemika.github.io/docs/installation_linux.html#installation-on-linux) until [Building the ROS packages](https://frankaemika.github.io/docs/installation_linux.html#building-the-ros-packages)

2. When building the ROS package with catkin_make ..., you might encounter a build error regarding **metapackages**. Go to your franka_ros repository directory and create a CATKIN_IGNORE file.
```
cd /path_to_your_franka_ros_repo
cd franka_ros
touch CATKIN_IGNORE
```
3. Rebuild the ROS workspace

## Usage
1. Always be sure to source your catkin workspace devel/setup.bash in your terminal
```
cd /path/to/your/catkin_ws
source devel/setup.bash
```
2. Launch Rviz visualizer
```
roslaunch rai_ros_panda panda_rviz.launch
```
3. Run example script
```
rosrun rai_ros_panda example.py
```


# Create your own file
The rai code that you will be running should be a .py file rather than .ipynb file. Convert it into a python file
```
jupyter nbconvert --to script 'my-notebook.ipynb'
```
1. Run Rviz using the following command
```
roslaunch rai_ros_panda panda_rviz.launch
```
2. Use the PandaJointPublisher in your python file. Be sure to add your **robotics-course/build** folder to the python path.

Example python code
```
#!/usr/bin/env python

import sys, time
import numpy as np
import rospy  # for this you have to source your devel/setup.bash
rospy.init_node('publish_robot_states')


# sys.path.append('/path_to_your_robotics-course/build')
import libry as ry
from panda_joint_publisher import PandaJointPublisher

RealWorld = ry.Config()
RealWorld.addFile("../../scenarios/challenge.g")
S = RealWorld.simulation(ry.SimulatorEngine.bullet, True)
S.addSensor("camera")

C = ry.Config()
C.addFile('../../scenarios/pandasTable.g')

# initialize joint publisher after initializing rai worlds
JointPub = PandaJointPublisher(ConfigWorld=C,RealWorld=RealWorld)

for t in range(300):
    time.sleep(0.01)

    if t%10 == 0:
       JointPub.publish()
        
    S.step([], tau, ry.ControlMode.none)

```

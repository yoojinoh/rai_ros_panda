## (Method 1) Minimum installation

1. Download the [franka_description](https://github.com/frankaemika/franka_ros/tree/develop/franka_description) folder into your ros src workspace. You the easiest way is to git clone the whole repository and delete everything else except **franka_description**

2. Run ros_workspace_setup.sh in your desired directory. This will create a ROS catkin workspace and clone the repository 
```
bash ros_workspace_directory.sh
```
OR manually

2. Clone repository

3. compile catkin workspace
```
git clone https://github.com/yoojinoh/rai_ros_panda.git
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
The rai code that you will be running should be a .py file rather than .ipynb file. Convert it into a python file
```
jupyter nbconvert --to script 'my-notebook.ipynb'
```
1. Run Rviz using the following command
```
roslaunch rai_ros_panda panda_rviz.launch
```
2. Use the PandaJointPublisher in your python file. 
Example python code
```
import time.sleep
import numpy as np
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

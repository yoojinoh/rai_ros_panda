import os, sys, time

### import Rai 
ROBOTICS_PATH=os.path.dirname(os.path.abspath(__file__))+'/../../../non_catkin/robotics-course'
sys.path.append(ROBOTICS_PATH+'/build')
import libry as ry
import cv2 as cv
import numpy as np

### import rospy and inititate node ###
import rospy  # for this you have to source your devel/setup.bash
rospy.init_node('publish_robot_states')
from panda_joint_publisher import PandaJointPublisher
########################################

#Let's edit the real world before we create the simulation
RealWorld = ry.Config()
RealWorld.addFile(ROBOTICS_PATH+"/scenarios/challenge.g")
D = RealWorld.view()

#change some colors
RealWorld.getFrame("obj0").setColor([0,1,0])
RealWorld.getFrame("obj1").setColor([1,0,0])
RealWorld.getFrame("obj2").setColor([1,1,0])
RealWorld.getFrame("obj3").setColor([1,0,1])
RealWorld.getFrame("obj4").setColor([0,1,1])

#you can also change the shape & size
RealWorld.getFrame("obj0").setColor([1.,0,0])
RealWorld.getFrame("obj0").setShape(ry.ST.sphere, [.03])
#RealWorld.getFrame("obj0").setShape(ry.ST.ssBox, [.05, .05, .2, .01])
RealWorld.getFrame("obj0").setPosition([0., .2, 2.])

#remove some objects
for o in range(5,30):
    name = "obj%i" % o
    print("deleting", name)
    RealWorld.delFrame(name)

# instantiate the simulation
S = RealWorld.simulation(ry.SimulatorEngine.bullet, True)
S.addSensor("camera")

# create your model world
C = ry.Config()
C.addFile(ROBOTICS_PATH+'/scenarios/pandasTable.g')
cameraFrame = C.frame("camera")
D2 = C.view()

### initialize joint publisher after initializing rai worlds ###
JointPub = PandaJointPublisher(ConfigWorld=C,RealWorld=RealWorld)
########################################

#the focal length
f = 0.895
f = f * 360.
fxfypxpy = [f, f, 320., 180.]

points = []
tau = .01

for t in range(300):
    time.sleep(0.01)

    #grab sensor readings from the simulation
    q = S.get_q()
    if t%10 == 0:
        [rgb, depth] = S.getImageAndDepth()  #we don't need images with 100Hz, rendering is slow
        points = S.depthData2pointCloud(depth, fxfypxpy)
        cameraFrame.setPointCloud(points, rgb)

        ### publish robot joint states ###
        JointPub.publish()    
        ##################################

        if len(rgb)>0: cv.imshow('OPENCV - rgb', rgb)
        if len(depth)>0: cv.imshow('OPENCV - depth', 0.5* depth)

        if cv.waitKey(1) & 0xFF == ord('q'):
            break
        
    S.step([], tau, ry.ControlMode.none)

cv.destroyAllWindows()

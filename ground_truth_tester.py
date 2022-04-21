import pybullet as p
import pybullet_data as pd
import numpy as np
import open3d as o3d
from matplotlib import pyplot as plt
import math


client = p.connect(p.GUI)
p.setGravity(0, 0, -10)

# Extend object search path
p.setAdditionalSearchPath(pd.getDataPath()) 

planeId = p.loadURDF("plane.urdf")
pandaUid = p.loadURDF("franka_panda/panda.urdf", useFixedBase = True)
trayUID = p.loadURDF("tray/traybox.urdf", basePosition=[0.65, 0, 0])
plateUID = p.loadURDF("data/dinnerware/plate.urdf", basePosition = [0.7, 0, 0.01])
cup1UID = p.loadURDF("data/dinnerware/cup/cup_small.urdf", basePosition = [0.85, 0.1, 0.01])
cup2UID = p.loadURDF("data/dinnerware/cup/cup_small.urdf", basePosition = [0.55, -0.1, 0.02])
duck = p.loadURDF("data/duck/duck_vhacd.urdf", basePosition = [0.5,0,0.02], baseOrientation = [0,10,0,0])

p.setTimeStep=1/240
p.setJointMotorControl2(pandaUid,0,p.POSITION_CONTROL,0)
p.setJointMotorControl2(pandaUid,1,p.POSITION_CONTROL,math.pi/4.+.3)
p.setJointMotorControl2(pandaUid,2,p.POSITION_CONTROL,0)
p.setJointMotorControl2(pandaUid,3,p.POSITION_CONTROL,-math.pi/2.+.15)
p.setJointMotorControl2(pandaUid,4,p.POSITION_CONTROL,0)
p.setJointMotorControl2(pandaUid,5,p.POSITION_CONTROL,3*math.pi/4)
p.setJointMotorControl2(pandaUid,6,p.POSITION_CONTROL,-math.pi/4.)
p.setJointMotorControl2(pandaUid,9,p.POSITION_CONTROL,0.08)
p.setJointMotorControl2(pandaUid,10,p.POSITION_CONTROL,0.08)

c = 0

while True:
    p.stepSimulation()
    p.performCollisionDetection(client)
    c = p.getContactPoints(bodyA = pandaUid, physicsClientId = client)
    if(c):
        break


print(len(c))
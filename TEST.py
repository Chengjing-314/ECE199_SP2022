import pybullet as p
import pybullet_data as pd
import numpy as np
from matplotlib import pyplot as plt

p.connect(p.GUI)
p.setGravity(0, 0, -10)

# Extend object search path
p.setAdditionalSearchPath(pd.getDataPath()) 

print(pd.getDataPath())


planeId = p.loadURDF("plane.urdf")
pandaUID = p.loadURDF("franka_panda/panda.urdf", useFixedBase = True)
trayUID = p.loadURDF("tray/traybox.urdf", basePosition=[0.65, 0, 0])
plateUID = p.loadURDF("data/dinnerware/plate.urdf", basePosition = [0.7, 0, 0.01])
cup1UID = p.loadURDF("data/dinnerware/cup/cup_small.urdf", basePosition = [0.85, 0.1, 0.01])
cup2UID = p.loadURDF("data/dinnerware/cup/cup_small.urdf", basePosition = [0.55, -0.1, 0.02])



viewMatrix = p.computeViewMatrix(
    cameraEyePosition=[0, 1, 2],
    cameraTargetPosition=[0, 0, 0],
    cameraUpVector=[0, 0, 1])

projectionMatrix = p.computeProjectionMatrixFOV(
    fov=60.0,
    aspect=1.0,
    nearVal=0.1,
    farVal=5.1)

width, height, rgbImg, depthImg, segImg = p.getCameraImage( # img are numpy array
    width=1024, 
    height=1024,
    viewMatrix=viewMatrix,
    projectionMatrix=projectionMatrix)


plt.imshow(rgbImg, interpolation='nearest')
plt.show()

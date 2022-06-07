import pybullet as p
import pybullet_data as pd
from torch import angle
from scipy.spatial.transform import Rotation
import numpy as np
import open3d as o3d
import math
from matplotlib import pyplot as plt
from Pipeline_Util import * 
from test_util import *
from tqdm import tqdm


def load_scene():
    planeId = p.loadURDF("plane.urdf")
    # pandaUID = p.loadURDF("franka_panda/panda.urdf", useFixedBase = True)
    trayUID = p.loadURDF("tray/traybox.urdf", basePosition=[0.65, 0, 0])
    plateUID = p.loadURDF("data/dinnerware/plate.urdf", basePosition = [0.7, 0, 0.01])
    cup1UID = p.loadURDF("data/dinnerware/cup/cup_small.urdf", basePosition = [0.85, 0.1, 0.01])
    cup2UID = p.loadURDF("data/dinnerware/cup/cup_small.urdf", basePosition = [0.55, -0.1, 0.02])
    duck = p.loadURDF("data/duck/duck_vhacd.urdf", basePosition = [0.5,0,0.02], baseOrientation = [0,10,0,0])


client = p.connect(p.GUI)

p.setAdditionalSearchPath(pd.getDataPath()) 


fpos = np.load('experiment_data/fpos.npy')
fneg = np.load('experiment_data/fneg.npy')
pandaUid = p.loadURDF("franka_panda/panda.urdf", useFixedBase = True)



ids = 1

visualID = p.createVisualShape(shapeType=p.GEOM_MESH, fileName="mesh.obj")
collisionID = p.createCollisionShape(shapeType=p.GEOM_MESH,fileName="mesh.obj")
mID = p.createMultiBody(baseCollisionShapeIndex=collisionID,
                  baseVisualShapeIndex=visualID)

b, pts = set_joints_and_collision_status(pandaUid, fpos[ids], client)
print(pts)

# load_scene()
# print(set_joints_and_collision_status(pandaUid, fpos[ids], client))

while True:
    set_joints_and_collision_status(pandaUid, fpos[ids], client)

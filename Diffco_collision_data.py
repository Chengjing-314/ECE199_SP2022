import pybullet as p
import pybullet_data as pd
from scipy.spatial.transform import Rotation
import numpy as np
import open3d as o3d
import math
from matplotlib import pyplot as plt
from Pipeline_Util import * 



client = p.connect(p.GUI)
p.setGravity(0, 0, -10)

# Extend object search path
p.setAdditionalSearchPath(pd.getDataPath()) 


# planeId = p.loadURDF("plane.urdf")
# pandaUID = p.loadURDF("franka_panda/panda.urdf", useFixedBase = True)
trayUID = p.loadURDF("tray/traybox.urdf", basePosition=[0.65, 0, 0])
plateUID = p.loadURDF("data/dinnerware/plate.urdf", basePosition = [0.7, 0, 0.01])
cup1UID = p.loadURDF("data/dinnerware/cup/cup_small.urdf", basePosition = [0.85, 0.1, 0.01])
cup2UID = p.loadURDF("data/dinnerware/cup/cup_small.urdf", basePosition = [0.55, -0.1, 0.02])
duck = p.loadURDF("data/duck/duck_vhacd.urdf", basePosition = [0.5,0,0.02], baseOrientation = [0,10,0,0])

    

viewMatrix = get_view_matrix([0, 1, 3], 
                             [0, 0, 0], 
                             [0, 0, 1])

projectionMatrix = get_projection_matrix()

_, _ , rgbImg, depthImg, _ = get_image(viewMatrix, projectionMatrix)

#p.disconnect() # optional, save resource

intrin = get_intrin()

extrin = get_extrin(viewMatrix)

cam = get_camera(extrin)

depthImg = true_z_from_depth_buffer(depthImg)

rgbd = buffer_to_rgbd(rgbImg, depthImg)

pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, 
                                                     cam.intrinsic, 
                                                     cam.extrinsic)
mesh = pcd_to_mesh(pcd)

p.resetSimulation()

pandaUid = p.loadURDF("franka_panda/panda.urdf", useFixedBase = True)

meshID = place_mesh(mesh)

angles = [0, math.pi/4.+.3, 0, -math.pi/2.+.15, 0, 3*math.pi/4.,-math.pi/4., 0.08, 0.08]

print(set_joints_and_collision_status(pandaUid, angles, client))
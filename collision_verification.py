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



client = p.connect(p.DIRECT)
p.setGravity(0, 0, -10)

# Extend object search path
p.setAdditionalSearchPath(pd.getDataPath()) 

def load_scene():
    planeId = p.loadURDF("plane.urdf")
    # pandaUID = p.loadURDF("franka_panda/panda.urdf", useFixedBase = True)
    trayUID = p.loadURDF("tray/traybox.urdf", basePosition=[0.65, 0, 0])
    plateUID = p.loadURDF("data/dinnerware/plate.urdf", basePosition = [0.7, 0, 0.01])
    cup1UID = p.loadURDF("data/dinnerware/cup/cup_small.urdf", basePosition = [0.85, 0.1, 0.01])
    cup2UID = p.loadURDF("data/dinnerware/cup/cup_small.urdf", basePosition = [0.55, -0.1, 0.02])
    duck = p.loadURDF("data/duck/duck_vhacd.urdf", basePosition = [0.5,0,0.02], baseOrientation = [0,10,0,0])
    

load_scene()

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

iteration = 1000

angles = generate_panda_angles(iteration)

mesh_sim = np.array([])


for i in range(len(angles)):
    pandaUid = p.loadURDF("franka_panda/panda.urdf", useFixedBase = True)
    meshID = place_mesh(mesh)
    mesh_sim  = np.append(mesh_sim, set_joints_and_collision_status(pandaUid, angles[i], client))
    p.resetSimulation()


ground_truth = np.array([])

for i in range(len(angles)):
    pandaUid = p.loadURDF("franka_panda/panda.urdf", useFixedBase = True)
    load_scene()
    ground_truth = np.append(ground_truth, set_joints_and_collision_status(pandaUid, angles[i], client))
    p.resetSimulation()


acc = (np.sum(mesh_sim == ground_truth) / len(ground_truth)) * 100

tpos = 0
fpos = 0
fneg = 0 

fneg_pose = []
fpos_pose = []

print(len(ground_truth), len(mesh_sim))

for i in range(len(mesh_sim)):
    if mesh_sim[i] == True and ground_truth[i] == True:
        tpos += 1
    if mesh_sim[i] == False and ground_truth[i] == True:
        fneg += 1
        fneg_pose.append(angles[i])
    if mesh_sim[i] == True and ground_truth[i] == False:
        fpos += 1
        fpos_pose.append(angles[i])

recall = tpos / (tpos + fneg)

np.save('experiment_data/fneg.npy', np.array(fneg_pose))
np.save('experiment_data/fpos.npy', np.array(fpos_pose))

precision = tpos / (tpos + fpos)

print("accuracy: ", acc, "precision: ", precision, "recall: ", recall)
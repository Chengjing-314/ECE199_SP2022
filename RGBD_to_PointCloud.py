import pybullet as p
import pybullet_data as pd
import numpy as np
import open3d as o3d
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
duck = p.loadURDF("data/duck/duck_vhacd.urdf", basePosition = [0.5,0.0,0], baseOrientation = [0,0,1,0])


IMG_LEN = 1024


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
    width=IMG_LEN, 
    height=IMG_LEN,
    viewMatrix=viewMatrix,
    projectionMatrix=projectionMatrix)



f = 1 / 2 * np.tan(30) 

#FIXME
intrin = np.array([[f, 0, IMG_LEN/2 + 0.5], # -1
              [0, f, IMG_LEN/2 + 0.5],
              [0, 0, 1]])

# intrin = np.array([[f, 0, 0], # -1
#               [0, f, 0],
#               [0, 0, 1]])

extrin = np.array(list(viewMatrix)).reshape((4,4))

rgbImg = rgbImg[:,:,:3]

intrinsic = o3d.camera.PinholeCameraIntrinsic()
intrinsic.intrinsic_matrix = intrin
cam = o3d.camera.PinholeCameraParameters()
cam.intrinsic = intrinsic
cam.extrinsic = extrin


depthImg = depthImg[:, :, None]


depth_as_img = o3d.geometry.Image((depthImg * 1000).astype(np.uint16))

rgbd_as_img = o3d.geometry.Image((rgbImg).astype(np.uint8))

rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(rgbd_as_img, depth_as_img, convert_rgb_to_intensity=False)

pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, cam.intrinsic, cam.extrinsic)

# Flip it, otherwise the pointcloud will be upside down
pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
o3d.visualization.draw_geometries([pcd])


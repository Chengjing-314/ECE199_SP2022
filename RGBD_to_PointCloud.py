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
duck = p.loadURDF("data/duck/duck_vhacd.urdf", basePosition = [0.5,0,0.02], baseOrientation = [0,10,0,0])


IMG_LEN = 1024
FOV = 60
NEAR = 0.1
FAR = 5.1


def to_homog(points):
    N = points.shape[1]
    D = points.shape[0]
    One = np.ones((1,N))
    points_homog = np.vstack([points,One])
    return points_homog

def from_homog(points_homog):
    N = points_homog.shape[1]
    D = points_homog.shape[0]
    points_homog = points_homog / points_homog[D-1,:]
    points = np.delete(points_homog,D-1,0)
    return points

viewMatrix = p.computeViewMatrix(
    cameraEyePosition=[0, 1, 2],
    cameraTargetPosition=[0, 0, 0],
    cameraUpVector=[0, 0, 1])

projectionMatrix = p.computeProjectionMatrixFOV(
    fov=FOV,
    aspect=1.0,
    nearVal=NEAR,
    farVal=FAR)

width, height, rgbImg, depthImg, segImg = p.getCameraImage( # img are numpy array
    width=IMG_LEN, 
    height=IMG_LEN,
    viewMatrix=viewMatrix,
    projectionMatrix=projectionMatrix)



f = (IMG_LEN // 2) * 1 / (np.tan(np.deg2rad(FOV)/2))



#FIXME
intrin = np.array([[f, 0, (IMG_LEN-1) / 2], # -1
                   [0, f, (IMG_LEN-1) / 2],
                   [0, 0, 1]])

# intrin = np.array([[f, 0, 0], # -1
#               [0, f, 0],
#               [0, 0, 1]])

extrin = np.array(list(viewMatrix)).reshape((4,4))

rgbImg = rgbImg[:,:,:3]

intrinsic = o3d.camera.PinholeCameraIntrinsic(IMG_LEN, IMG_LEN, f, f, (IMG_LEN-1) / 2, (IMG_LEN-1) / 2)
cam = o3d.camera.PinholeCameraParameters()
cam.intrinsic = intrinsic
cam.extrinsic = extrin

# depth_tiny = far * near / (far - (far - near) * depth_buffer_tiny)

depthImg = depthImg[:, :, None]

depthIMG = 2 * depthImg - 1
depthImg = 2 * FAR * NEAR / (FAR + NEAR - (FAR-NEAR) * depthImg)


# depthImg = depthImg * (FAR - NEAR) + NEAR



depth_as_img = o3d.geometry.Image((depthImg * 1000).astype(np.uint16))

rgbd_as_img = o3d.geometry.Image((rgbImg).astype(np.uint8))

rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(rgbd_as_img, depth_as_img, convert_rgb_to_intensity=False, depth_trunc = 1000)

pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, cam.intrinsic, cam.extrinsic)

# Flip it, otherwise the pointcloud will be upside down
pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]) #TODO:should i transform
# o3d.visualization.draw_geometries([pcd])



pc_points = np.asarray(pcd.points)
pc_points = np.swapaxes(pc_points,0,1)
pc_points = to_homog(pc_points)

identity = np.identity(3)
identity = np.hstack([identity, np.array([[0],[0],[0]])])

camera_matrix = intrin @ identity @ extrin

image = from_homog(camera_matrix @ pc_points)


color = np.asarray(pcd.colors)
color = np.swapaxes(color,0,1)


recover = np.zeros((IMG_LEN, IMG_LEN, 3))

print(f'here: {image}')

image = np.floor(image)

for i in range(image.shape[1]):
    x, y = image[:,i]
    x, y = int(x), int(y)
    recover[x][y] = color[:,i]
    
recover = np.rot90(recover,3)

plt.subplot(1,2,1)
plt.imshow(recover)
plt.subplot(1,2,2)
plt.imshow(rgbImg)
plt.show()
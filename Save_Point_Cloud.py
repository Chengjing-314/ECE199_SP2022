import pybullet as p
import pybullet_data as pd
import numpy as np
import open3d as o3d
from matplotlib import pyplot as plt
from PIL import Image

p.connect(p.GUI)
p.setGravity(0, 0, -10)

# Extend object search path
p.setAdditionalSearchPath(pd.getDataPath()) 


planeId = p.loadURDF("plane.urdf")
# pandaUID = p.loadURDF("franka_panda/panda.urdf", useFixedBase = True)
trayUID = p.loadURDF("tray/traybox.urdf", basePosition=[0.65, 0, 0])
plateUID = p.loadURDF("data/dinnerware/plate.urdf", basePosition = [0.7, 0, 0.01])
cup1UID = p.loadURDF("data/dinnerware/cup/cup_small.urdf", basePosition = [0.85, 0.1, 0.01])
cup2UID = p.loadURDF("data/dinnerware/cup/cup_small.urdf", basePosition = [0.55, -0.1, 0.02])
duck = p.loadURDF("data/duck/duck_vhacd.urdf", basePosition = [0.5,0,0.02], baseOrientation = [0,10,0,0])


IMG_LEN = 1024
FOV = 60
NEAR = 0.1
FAR = 5.1
f = (IMG_LEN // 2) * 1 / (np.tan(np.deg2rad(FOV)/2)) # Focal length


def to_homog(points):
    """
    Transform points from cartesian coordinates to homogenous coordinates.

    Args:
        points (numpy ndarray): 3 * n  numpy array.

    Returns:
        homogenous transform of the coordinates, 4 * n numpy array.
    """
    N = points.shape[1]
    D = points.shape[0]
    One = np.ones((1,N))
    points_homog = np.vstack([points,One])
    return points_homog

def from_homog(points_homog):
    """
    Transform points from homogenous coordinates to cartesion coordinates.

    Args:
        points_homog (numpy ndarray): 3 * n numpy array.

    Returns:
        cartesion transform of the coordinates, 2 * n numpy array.
    """
    N = points_homog.shape[1]
    D = points_homog.shape[0]
    points_homog = points_homog / points_homog[D-1,:]
    points = np.delete(points_homog,D-1,0)
    return points


#p.computeViewMatrix(cameraEyePosition=[0, 1, 2],
      #                         cameraTargetPosition=[0, 0, 0],
     #                          cameraUpVector=[0, 0, 1])

def get_view_matrix(eye_pos, target_pos, camera_up_vec):
    """
    Calculate camera viewMatrix(extrinsic matrix)

    Args:
        eye_pos (list):  The position of the camera in world frame.
        target_pos (list): The position of the target. 
        camera_up_vec (list): The up direction of the camera.

    Returns:
       
    """
    return p.computeViewMatrix(cameraEyePosition=eye_pos,
                               cameraTargetPosition=target_pos,
                               cameraUpVector=camera_up_vec)


def get_projection_matrix(fov=FOV, aspect = 1.0, nearVal = NEAR, farVal = FAR):
    return  p.computeProjectionMatrixFOV(fov=fov,
                                         aspect=aspect,
                                         nearVal=nearVal,
                                         farVal=farVal)


def get_image(viewMatrix, projectionMatrix, width = IMG_LEN, height = IMG_LEN):
    """
    Get the image from the pybullet synthetic camera. 
    Args:
        viewMatrix: Camera view matrix(extrinsic matirx) from get_view_matrix.
        projectionMatrix: Camera projection matrix(OpenGL projection matrix) from get_projection_matrix.
        width: image width, Defaults to IMG_LEN.
        height: image height. Defaults to IMG_LEN.

    Returns:
        width, height, RGB_image, Depth_image, Segmentation_image
    """
    width, height, RGB_img, Depth_img, segmentation_img = p.getCameraImage(
                                                                    width=width, 
                                                                  height=height,
                                                          viewMatrix=viewMatrix,
                                              projectionMatrix=projectionMatrix)
    
    RGB_img = RGB_img[:,:,:3] # Dropped Alha Channel
    Depth_img = Depth_img[:,:,None] # Add thrid axis
    return width, height, RGB_img, Depth_img, segmentation_img 


def get_intrin(): 
    """
    Calculate the intrinsic matrix of camera.

    Returns:
         a numpy array of the intrisinc camera
    """
    return np.array([[f, 0, (IMG_LEN-1) / 2], 
                     [0, f, (IMG_LEN-1) / 2],
                     [0, 0, 1]])
    
def get_extrin(viewMatrix):
    """
    Return the extrinsic matrix of the camera

    Args:
        viewMatrix (tuple): viewMatrix from get_view_matrix

    Returns:
        numpy array of the extrinsic matrix. 
    """
    return np.array(list(viewMatrix)).reshape((4,4)).T

def get_camera(extrin):
    """
    Return a camera object from extrinsic matrix from get_extrin for point cloud
    generation
    
    Args:
        extrin (numpy array): 4 x 4 extrinsic matrix from get_extrin.

    Returns:
        camera object for open3d cloud generation. 
    """
    intrinsic = o3d.camera.PinholeCameraIntrinsic(IMG_LEN, IMG_LEN, f, f, 
                                                  (IMG_LEN-1) / 2, 
                                                  (IMG_LEN-1) / 2)
    cam = o3d.camera.PinholeCameraParameters()
    cam.intrinsic = intrinsic
    cam.extrinsic = extrin
    return cam

def true_z_from_depth_buffer(depthImg, far = FAR, near = NEAR):
    """
    function will take in a depth buffer from depth camera in NDC coordinate and
    convert it in to true z value. 
    
    Args:
        depthImg (numpy array): real depth in world frame
    """
    depthImg = 2 * depthImg - 1
    depthImg = 2 * far * near / (far + near - (far-near) * depthImg)
    
    return depthImg
    

def buffer_to_rgbd(rgbImg, depthImg):
    """
    function will convert two numpy array to o3d rgbd image for point cloud 
    generation

    Args:
        rgbImg (_type_): _description_
        depthImg (_type_): _description_

    Returns:
        _type_: _description_
    """
    
    depth_as_img = o3d.geometry.Image((depthImg * 1000).astype(np.uint16))

    rgbd_as_img = o3d.geometry.Image((rgbImg).astype(np.uint8))

    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(rgbd_as_img, 
                                                              depth_as_img, 
                                                 convert_rgb_to_intensity=False, 
                                                             depth_trunc = 1000)
    
    return rgbd
    

viewMatrix = get_view_matrix([0, 1, 2], 
                             [0, 0, 0], 
                             [0, 0, 1])

projectionMatrix = get_projection_matrix()

_, _ , rgbImg, depthImg, _ = get_image(viewMatrix, projectionMatrix)

intrin = get_intrin()

extrin = get_extrin(viewMatrix)

cam = get_camera(extrin)

depthImg = true_z_from_depth_buffer(depthImg)

rgbd = buffer_to_rgbd(rgbImg, depthImg)

pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, 
                                                     cam.intrinsic, 
                                                     cam.extrinsic)


o3d.io.write_point_cloud("pcd.ply", pcd)




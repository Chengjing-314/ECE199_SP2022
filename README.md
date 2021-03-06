# ECE199 SP2022


## Update
This repo contain codes that use pybullet to obtain rgbd camera and tools to convert it to Open3d point cloud. 

## Usable Objects
***Must Include***
```python
import pybullet_data as pd
pybullet.setAdditionalSearchPath(pd.getDataPath())
```
### URDF
```python
pybullet.loadURDF(PATH)
```
|No.|Object Name|Load URDF Path|
|----|---|----|
|1|Table|"/data/table/table.urdf"|
|2|Tray|"/data/tray/traybox.urdf"|
|3|Racecar|"/data/racecar/racecar.urdf"|
|4|Plate(Bowl?)|"/data/dinnerware/plate.urdf"|
|5|Cup|"/data/dinnerware/cup/cup_small.urdf"|
|6|Lego(very small)|"/data/lego/lego.urdf"|
|7|Small Sphere|"/data/sphere_small.urdf"|
|8|Duck|"/data/duck_vhacd.urdf"|
|9|Star War R2D2|"/data/r2d2.urdf"|

### Soft Body
```python
pybullet.loadSoftBody(OBJ_PATH,simFileName=VTK_PATH)
```
|No.|Object Name|Load Soft Body Path|
|----|---|----|
|1|Banana|"/data/banana.obj","/data/banana.vtk"|

## Verification Steps on Point cloud. 

1. Get view matrix and projection matrix, feed in through the get_image function.
2. Save the RGB and Depth array and get intrinsic and extrinsic matrix.
3. Project point cloud.
4. Use camera matrix to get the original image. 


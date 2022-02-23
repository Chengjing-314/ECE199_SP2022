# ECE199 SP2022

## -Usable Objects
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

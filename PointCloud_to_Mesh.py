import numpy as np
import open3d as o3d
from matplotlib import pyplot as plt
from PIL import Image



alpha = 0.05

pcd = o3d.io.read_point_cloud('./pcd.ply')

pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]) 

pcd.estimate_normals()

downpcd = pcd.voxel_down_sample(voxel_size = 0.009)

print(pcd)
print(downpcd)

# o3d.visualization.draw_geometries([downpcd])

### ball pivot ####
radii = [0.00025, 0.005, 0.01, 0.02, 0.04]
mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
    downpcd, o3d.utility.DoubleVector(radii))

####  alpha  ####
# alpha = 0.03
# mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)

#### poisson ####
# with o3d.utility.VerbosityContextManager(
#         o3d.utility.VerbosityLevel.Debug) as cm:
#     mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
#         downpcd, depth=20)

# mesh = mesh.filter_smooth_taubin(8)

# mesh.paint_uniform_color([1, 0.706, 0])

print(mesh)

o3d.visualization.draw_geometries([mesh])
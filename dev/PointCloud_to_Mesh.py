import numpy as np
import open3d as o3d
from matplotlib import pyplot as plt
from PIL import Image
import fcl
import time

start_time = time.time()


alpha = 0.05

pcd = o3d.io.read_point_cloud('./pcd.ply')

# pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]) 

pcd.estimate_normals()

# b = pcd.compute_nearest_neighbor_distance()
# b = np.array(b)
# print(b.mean())

downpcd = pcd.voxel_down_sample(voxel_size = 0.095)

# print(pcd)
# print(downpcd)

# o3d.visualization.draw_geometries([downpcd])

def round(num):
        working = str(num-int(num))
        for i, e in enumerate(working[2:]):
            if e != '0':
                return int(num) + float(working[:i+3])

b = downpcd.compute_nearest_neighbor_distance()
b = np.array(b)
c = round(b.mean())
print(round(b.mean()))

#TODO: find the average distance between point cloud and their closest point. 
#TODO: Segmentation_image. 
#TODO: Collision checking
#TODO: Understand the distance 



#### ball pivot ####
# radii = [0.045, 0.0125, 0.025, 0.05, 0.1, 0.2, 0.4, 0.8] # 0.05
radii = [0.075, 0.07, 0.035, 0.14, 0.28, 0.56]
mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
    downpcd, o3d.utility.DoubleVector(radii))

print(mesh)

#o3d.visualization.draw_geometries([mesh])



#o3d.io.write_triangle_mesh('./mesh.obj', mesh)
# tri = np.asarray(mesh.triangles)
# vert = np.asarray(mesh.vertices)



# m = fcl.BVHModel()
# m.beginModel(len(vert), len(tri))
# m.addSubModel(vert,tri)
# m.endModel()

print("--- %s seconds ---" % (time.time() - start_time))

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




# put mesh back into pybullet 
# 
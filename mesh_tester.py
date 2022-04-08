import pybullet as p
import pybullet_data as pd
import numpy as np


p.connect(p.GUI)
p.setGravity(0, 0, -10)
p.setAdditionalSearchPath(pd.getDataPath()) 


# planeId = p.loadURDF("plane.urdf")
pandaUID = p.loadURDF("franka_panda/panda.urdf", useFixedBase = True)
visualID = p.createVisualShape(shapeType=p.GEOM_MESH, fileName="mesh.obj")
collisionID = p.createCollisionShape(shapeType=p.GEOM_MESH,fileName="mesh.obj")
p.createMultiBody(baseCollisionShapeIndex=collisionID,
                  baseVisualShapeIndex=visualID)#,
                #   basePosition = [0, 2, 6])

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

while True:
    continue
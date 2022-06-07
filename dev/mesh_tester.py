import pybullet as p
import pybullet_data as pd
import numpy as np
import math


p.setGravity(0, 0, -10)
p.setAdditionalSearchPath(pd.getDataPath()) 


client = p.connect(p.GUI)
# planeId = p.loadURDF("plane.urdf")
pandaUid = p.loadURDF("franka_panda/panda.urdf", useFixedBase = True)
visualID = p.createVisualShape(shapeType=p.GEOM_MESH, fileName="mesh.obj")
collisionID = p.createCollisionShape(shapeType=p.GEOM_MESH,fileName="mesh.obj")
p.createMultiBody(baseCollisionShapeIndex=collisionID,
                  baseVisualShapeIndex=visualID)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)


p.setTimeStep=1/240
p.resetJointState(pandaUid,0,p.POSITION_CONTROL,0)
p.resetJointState(pandaUid,1,p.POSITION_CONTROL,math.pi/4.+.3)
p.resetJointState(pandaUid,2,p.POSITION_CONTROL,0)
p.resetJointState(pandaUid,3,p.POSITION_CONTROL,-math.pi/2.+.15)
p.resetJointState(pandaUid,4,p.POSITION_CONTROL,0)
p.resetJointState(pandaUid,5,p.POSITION_CONTROL,3*math.pi/4)
p.resetJointState(pandaUid,6,p.POSITION_CONTROL,-math.pi/4.)
p.resetJointState(pandaUid,9,p.POSITION_CONTROL,0.08)
p.resetJointState(pandaUid,10,p.POSITION_CONTROL,0.08)

c = 0

p.performCollisionDetection(client)
c = p.getContactPoints(bodyA = pandaUid, physicsClientId = client)


pts = np.array([m[5] for m in c])

print(len(pts),pts)


# gts = np.array([(0.7686808727562892, 0.003917806569749038, 0.03951751450995039), (0.7673650446168004, -0.01328388589395121, 0.038664704364934833), (0.8052859038894746, -4.469782687428661e-05, 0.07828891000127755), 
# (0.8043447919432735, -6.777830027429317e-05, 0.07841453670299192)])

# red = []

# for pt in pts:
#     min_v = 10
#     for gt in gts:
#         nd = np.sum(np.square(pt-gt))
#         if nd < min_v:
#             min_v = nd
#     red.append(min_v)
    
# print(red)

# [(0.7686808727562892, 0.003917806569749038, 0.03951751450995039), (0.7673650446168004, -0.01328388589395121, 0.038664704364934833), (0.8052859038894746, -4.469782687428661e-05, 0.07828891000127755), 
# (0.8043447919432735, -6.777830027429317e-05, 0.07841453670299192)]
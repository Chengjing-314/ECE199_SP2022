import pybullet as p
import pybullet_data as pd
import numpy as np
import math


client = p.connect(p.GUI)
p.setGravity(0, 0, -10)
p.setAdditionalSearchPath(pd.getDataPath()) 


# planeId = p.loadURDF("plane.urdf")
pandaUid = p.loadURDF("franka_panda/panda.urdf", useFixedBase = True)
visualID = p.createVisualShape(shapeType=p.GEOM_MESH, fileName="mesh.obj")
collisionID = p.createCollisionShape(shapeType=p.GEOM_MESH,fileName="mesh.obj")
p.createMultiBody(baseCollisionShapeIndex=collisionID,
                  baseVisualShapeIndex=visualID)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)


# p.setTimeStep=1/240
# p.setJointMotorControl2(pandaUid,0,p.POSITION_CONTROL,0)
# p.setJointMotorControl2(pandaUid,1,p.POSITION_CONTROL,math.pi/4.+.3)
# p.setJointMotorControl2(pandaUid,2,p.POSITION_CONTROL,0)
# p.setJointMotorControl2(pandaUid,3,p.POSITION_CONTROL,-math.pi/2.+.15)
# p.setJointMotorControl2(pandaUid,4,p.POSITION_CONTROL,0)
# p.setJointMotorControl2(pandaUid,5,p.POSITION_CONTROL,3*math.pi/4)
# p.setJointMotorControl2(pandaUid,6,p.POSITION_CONTROL,-math.pi/4.)
# p.setJointMotorControl2(pandaUid,9,p.POSITION_CONTROL,0.08)
# p.setJointMotorControl2(pandaUid,10,p.POSITION_CONTROL,0.08)

c = 0

while True:
    p.stepSimulation()
    p.performCollisionDetection(client)
    c = p.getContactPoints(bodyA = pandaUid, physicsClientId = client)
    if(c):
        break


print(len(c))
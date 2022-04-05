import pybullet as p
import pybullet_data as pd
import numpy as np


p.connect(p.GUI)
p.setGravity(0, 0, -10)
p.setAdditionalSearchPath(pd.getDataPath()) 

# 12 joints, 0 - 6 joints, 9 - 10 clove position. 
pandaUID = p.loadURDF("franka_panda/panda.urdf", useFixedBase = True)

p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING)



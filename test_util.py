import numpy as np


def generate_panda_angles(num_angle):
    res = []
    for _ in range(num_angle):
        angles = np.zeros(9)
        for i in range(len(angles)):
            if i == 0 or i == 2 or i == 4 or i == 6:
                angles[i] =  np.random.uniform(-2.8973, 2.8973)
            elif i == 1:
                angles[i] =  np.random.uniform(-1.7628, 1.7628)
            elif i  == 3:
                angles[i] = np.random.uniform(-0.0698, -3.0718)
            elif i == 5:
                angles[i] = np.random.uniform(-0.0175, 3.7525)
            elif i == 7 or i == 8:
                angles[i] = np.random.uniform(0.02, 0.08)
        res.append(angles)
    return np.array(res)
        


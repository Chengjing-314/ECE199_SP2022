import numpy as np


def generate_panda_angles(num_angle):
    res = []
    for _ in range(num_angle):
        anlges = np.zeros(9)
        lim1 = [0,2,4]
        lim2 = [7,8]
        for i in range(len(anlges)):
            if i in lim1:
                anlges[i] = np.random.uniform(0, 1)
            elif i in lim2:
                anlges[i] = np.random.uniform(0, 1)
            else:
                anlges[i] = np.random.uniform(0, 2 * np.pi)
        res.append(anlges)
    return np.array(res)
        


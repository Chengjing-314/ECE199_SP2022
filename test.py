# shift = 2
# a = ord('a')

# for i in range(39):
#     if a + shift > ord('z'):
#         a = ord('a') + a + shift - ord('z') - 1
#     else:
#         a += shift
#     print(chr(a))

import numpy as np


a = np.array([1,2])

b = np.array([-3, 4])

a += -1 *b if b > 0 else b

print(a)

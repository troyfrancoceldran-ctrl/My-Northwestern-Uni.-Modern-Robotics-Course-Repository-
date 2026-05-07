import numpy as np 
from scipy.linalg import logm, expm
import math as m 


r_1 = np.array([
    [0, 0, 1]
])

r_2 = np.array([
    [0, 9, 0]
])

r_cross = np.cross(r_1, r_2)
print(r_cross)


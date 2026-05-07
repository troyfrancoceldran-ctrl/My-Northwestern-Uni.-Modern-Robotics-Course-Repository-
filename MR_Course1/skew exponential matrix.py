import numpy as np
from scipy.linalg import logm, expm
import math as mt 

# --- Method 1: The hardcoded matrix from the previous step ---
R_1 = np.array([
    [0,  1,  0],
    [0,  0,  1],
    [1,  0,  0],
])

R_2 = np.array([
    [1,  0,  0],
    [0,  0,  1],
    [0,  -1,  0],
])

# --- Method 2: Calculating it directly from the exponential coordinates ---
# The exponential coordinates (omega_hat * theta)
w = np.array([1, 2, 0.5])
theta = np.linalg.norm(w)
omega_hat = w / theta

# Form the 3x3 skew-symmetric matrix [w]
w_bracket = np.array([
    [0, 0.5,  -1],
    [-0.5, 0,  2],
    [1, -2,  0],
])

R_rod = np.array([
    [0, 0, 1],
    [-1, 0, 0],
    [0, -1, 0],
])

# Use scipy's expm to calculate the matrix exponential (e^[w])
R_res = logm(R_rod)

print("Exact Computed Matrix (rounded to 4 decimal places):")
print(np.round(R_res, 4))




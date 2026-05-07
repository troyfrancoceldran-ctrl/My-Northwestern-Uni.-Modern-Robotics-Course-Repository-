import numpy as np
from scipy.linalg import logm, expm
import math as mt 

# =============================================================================
# --- Part 1: Transposition and Transformation Matrices
# =============================================================================

# 1. Define the Rotation Matrices (R_sa and R_sb)
R_sa = np.array([
    [0, -1,  0],  # x_a(x), y_a(x), z_a(x)
    [0,  0, -1],  # x_a(y), y_a(y), z_a(y)
    [1,  0,  0]   # x_a(z), y_a(z), z_a(z)
])

R_sb = np.array([
    [1,  0,  0],  
    [0,  0,  1],  
    [0, -1,  0]   
])

# 2. Define the Translation Vectors (p_sa and p_sb)
p_sa = np.array([
    [0],
    [0],
    [1]
])

p_sb = np.array([
    [0],
    [2],
    [0]
])

p_b = np.array([
    [1],
    [2],
    [3],
    [1]
])

# 3. Assemble the Homogeneous Transformation Matrices (T_sa and T_sb)
top_half_a = np.hstack((R_sa, p_sa))
top_half_b = np.hstack((R_sb, p_sb))

bottom_row = np.array([[0, 0, 0, 1]])

T_sa = np.vstack((top_half_a, bottom_row))
T_sb = np.vstack((top_half_b, bottom_row))

# 4. Calculate the Inverses
# We use .astype(int) to clean up the floating point artifacts from np.linalg.inv
T_sa_inv = np.linalg.inv(T_sa).astype(int)
T_sb_inv = np.linalg.inv(T_sb).astype(int)

print("Constructed Transformation Matrix T_sa:")
print(T_sa)
print("\nConstructed Transformation Matrix T_sb:")
print(T_sb)
print("-" * 50)

print("Constructed Transformation Matrix T_sa^-1:")
print(T_sa_inv)
print("\nConstructed Transformation Matrix T_sb^-1:")
print(T_sb_inv)
print("-" * 50)

T_1 = T_sb @ T_sa
print("T_1 is:", T_1)

p_s_homo = T_sb @ p_b

p_s = p_s_homo[0:3]

print("Point in {s} coordinates (p_s):")
print(p_s.astype(int))


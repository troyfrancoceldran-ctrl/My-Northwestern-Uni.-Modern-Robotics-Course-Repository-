import numpy as np 
from scipy.linalg import logm, expm
import modern_robotics as mr
from modern_robotics.core import VecTose3

# =============================================================================
# UTILITY FUNCTIONS
# =============================================================================

def VecToso3(omg):
    """Returns the 3x3 skew-symmetric matrix of a 3-vector"""
    return np.array([
        [0,       -omg[2],  omg[1]],
        [omg[2],   0,      -omg[0]],
        [-omg[1],  omg[0],  0     ]
    ])

def Adjoint(T):
    """Returns the 6x6 adjoint matrix of T"""
    R = T[0:3, 0:3]
    p = T[0:3, 3]
    p_bracket = VecToso3(p)
    
    adj = np.zeros((6, 6))
    adj[0:3, 0:3] = R
    adj[3:6, 3:6] = R
    adj[3:6, 0:3] = p_bracket @ R
    return adj

# =============================================================================
# PART 1: TRANSFORMATION MATRICES & POINT CONVERSIONS
# =============================================================================

# 1. Define Basic Components
R_sa = np.array([[0, -1,  0], 
                [0,  0, -1], 
                [1,  0,  0]])

R_sb = np.array([[1,  0,  0], 
                [0,  0,  1], 
                [0, -1,  0]])

p_sa = np.array([[0], [0], [1]])
p_sb = np.array([[0], [2], [0]])

# 2. Assemble T_sa and T_sb
bottom_row = np.array([[0, 0, 0, 1]])
T_sa = np.vstack((np.hstack((R_sa, p_sa)), bottom_row))
T_sb = np.vstack((np.hstack((R_sb, p_sb)), bottom_row))

# 3. Calculate Inverses and Operative Matrices
T_as = np.linalg.inv(T_sa)
T_sa_inv = T_as  # Explicit definition for printing
T_bs = np.linalg.inv(T_sb)
T_1 = T_as @ T_sb  # T_sa transformed by T_sb in fixed frame {s}

# 4. Point Conversion (pb to ps)
p_b_homo = np.array([1, 2, 3, 1])
p_s_homo = T_sb @ p_b_homo

# --- Print Part 1 ---
print("--- Part 1: Transformation Matrices ---")
print(f"T_sa:\n{T_sa}\n")
print(f"T_sb:\n{T_sb}\n")
print(f"T_as:\n{T_as}\n")
print(f"T_bs:\n{T_bs}\n")
print(f"T_1:\n{T_1}")
print(f"\nPoint p in {{s}} coordinates:\n{p_s_homo[:3].astype(int)}")
print("-" * 50)

# =============================================================================
# PART 2: TWISTS, LOGARITHMS, AND EXPONENTIALS
# =============================================================================

# 1. Twist Adjoint Calculation
Vs = np.array([3, 2, 1, -1, -2, -3])
Va = Adjoint(T_as) @ Vs

# 2. Matrix Logarithm of T_sa
se3_log_matrix = np.real(logm(T_sa))
theta = np.arccos(np.clip((np.trace(R_sa) - 1) / 2.0, -1.0, 1.0))

# 3. Matrix Exponential of S_theta
S_theta_vec = np.array([0, 1, 2, 3, 0, 0])
omg_th = S_theta_vec[0:3]
v_th = S_theta_vec[3:6]

se3_mat_exp = np.zeros((4, 4))
se3_mat_exp[0:3, 0:3] = VecToso3(omg_th)
se3_mat_exp[0:3, 3] = v_th
T_from_exp = expm(se3_mat_exp)

# --- Print Part 2 ---
print("\n--- Part 2: Twist & Exponential Math ---")
print(f"Va representation:\n{np.round(Va).astype(int)}")
print(f"\nRotation angle theta: {theta:.4f} radians")
print(f"\nMatrix Log [S]theta of T_sa:\n{np.round(se3_log_matrix, 4)}")
print(f"\nMatrix Exponential from S_theta:\n{np.round(T_from_exp, 4)}")
print("-" * 50)

# =============================================================================
# PART 3: FORCE AND MOMENTS
# =============================================================================

vector_b2 = np.array([[1], [0], [0]])
vector_b1 = np.array([[2], [1], [0]])

fs = R_sb @ vector_b1
rotated_moment = R_sb @ vector_b2
cross_prod = np.cross(p_sb.flatten(), fs.flatten())
ms = rotated_moment.flatten() + cross_prod

F_s = np.concatenate((ms, fs.flatten()))

# --- Print Part 3 ---
print("\n--- Part 3: Force and Moments ---")
print(f"Force in {{s}} (fs):\n{fs.flatten().astype(int)}")
print(f"\nMoment in {{s}} (ms):\n{ms.astype(int)}")
print(f"\nFull Wrench F_s:\n{F_s.astype(int)}")
print("-" * 50)

# =============================================================================
# PART 4: EXTRA OPERATIONS
# =============================================================================
print("\n--- Part 4: Extra Operations ---")

# 1. TransInv Function
T = np.array([
    [0, -1, 0, 3],
    [1,  0, 0, 0],
    [0,  0, 1, 1],
    [0,  0, 0, 1]
])
Trans_inv = mr.TransInv(T)
print(f"Inverse of T:\n{Trans_inv}\n")

# 2. find se(3)
V = np.array([1, 0, 0, 0, 2, 3])
se3_matrix = VecTose3(V)
print(f"se(3) matrix of V:\n{se3_matrix}\n")

# 3. ScrewToAxis function
def ScrewToAxis(q, s, h):
    q = np.array(q)
    s = np.array(s)
    return np.r_[s, np.cross(q, s) + (s * h)]

s_hat = np.array([1, 0, 0])
p = np.array([0, 0, 2])
h = 1
axis = ScrewToAxis(p, s_hat, h)
S_int = axis.astype(int).tolist()
print(f"Screw Axis:\n{S_int}")
print("-" * 50)

# =============================================================================
# PART 5: MatrixLog6 and MatrixExp6
# =============================================================================
print("\n--- Part 5: MatrixLog6 and MatrixExp6 ---")

S_theta = np.array([
    [0,      -1.5708,  0,  2.3562],
    [1.5708,  0,       0, -2.3562],
    [0,       0,       0,  1     ],
    [0,       0,       0,  0     ],
])

se3exp = mr.MatrixExp6(S_theta)
print(f"The SE(3) matrix representation of the twist is:\n{np.around(se3exp, 4)}\n")

T_exp = np.array([
    [0, -1, 0, 3],
    [1,  0, 0, 0],
    [0,  0, 1, 1],
    [0,  0, 0, 1],
])

se3log = mr.MatrixLog6(T_exp)
print(f"The se(3) matrix representation of the twist is:\n{np.around(se3log, 4)}")
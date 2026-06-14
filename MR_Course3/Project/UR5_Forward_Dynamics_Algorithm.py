"""
===========================================================
UR5 Robot Arm Forward Dynamics Simulation
===========================================================
Author      : Troy Celdran
Description : Simulates the free-fall motion of a UR5 6-DOF
            industrial robot arm under gravity using the
            Forward Dynamics algorithm from the Modern
            Robotics library. Zero torques are applied to
            all joints, and the robot starts from rest.
            Joint angles at each timestep are recorded and
            exported as a .csv file compatible with the
            CoppeliaSim UR5 csv animation scene.
Usage       : Run this script directly. Two .csv files will
            be automatically generated:
                - simulation1.csv  (home config, 3 seconds)
                - simulation2.csv  (joint 2 = -1 rad, 5 seconds)
Dependencies: numpy, modern_robotics
===========================================================
"""
#----NECESSARY LIBRARIES-----#
import math as mt
import numpy as np 
import modern_robotics as mr 

# ================= FUNCTION BLOCKS ========================
# 1.1 Define function for all the UR5 Robot Arm parameters.
def define_ur5_parameters():
    return Mlist, Glist, Slist

# 1.2 Define function for the UR5 Forward Dynamics simulation.
def UR5_dynamics_Algo(thetalist0, dthetalist0, taumat, g, Ftipmat, Mlist, Glist, Slist, dt, intRes):
    return mr.ForwardDynamicsTrajectory(thetalist0, dthetalist0, taumat, g, Ftipmat, Mlist, Glist, Slist, dt, intRes)

# 1.3 Define function for .csv file export
def export_csv(thetamat, filename):
    np.savetxt(filename, thetamat, delimiter = ',')


# ================= TEST SCRIPT AND EXECUTION BLOCKS =======================
# --- Link frame transformation matrices (home configuration) ---
M01 = np.array([[1, 0, 0, 0], 
                [0, 1, 0, 0], 
                [0, 0, 1, 0.089159], 
                [0, 0, 0, 1]])

M12 = np.array([[0, 0, 1, 0.28], 
                [0, 1, 0, 0.13585], 
                [-1, 0, 0, 0], 
                [0, 0, 0, 1]])

M23 = np.array([[1, 0, 0, 0], 
                [0, 1, 0, -0.1197], 
                [0, 0, 1, 0.395], 
                [0, 0, 0, 1]])

M34 = np.array([[0, 0, 1, 0], 
                [0, 1, 0, 0], 
                [-1, 0, 0, 0.14225], 
                [0, 0, 0, 1]])

M45 = np.array([[1, 0, 0, 0], 
                [0, 1, 0, 0.093], 
                [0, 0, 1, 0],
                [0, 0, 0, 1]])

M56 = np.array([[1, 0, 0, 0], 
                [0, 1, 0, 0], 
                [0, 0, 1, 0.09465], 
                [0, 0, 0, 1]])

M67 = np.array([[1, 0, 0, 0], 
                [0, 0, 1, 0.0823], 
                [0, -1, 0, 0], 
                [0, 0, 0, 1]])

# --- Spatial inertia matrices for each link ---
G1 = np.diag([0.010267495893, 0.010267495893, 0.00666, 3.7, 3.7, 3.7])
G2 = np.diag([0.22689067591, 0.22689067591, 0.0151074, 8.393, 8.393, 8.393])
G3 = np.diag([0.049443313556, 0.049443313556, 0.004095, 2.275, 2.275, 2.275])
G4 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
G5 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
G6 = np.diag([0.0171364731454, 0.0171364731454, 0.033822, 0.1879, 0.1879, 0.1879])

# --- Grouped parameter lists for the MR library ---
Glist = [G1, G2, G3, G4, G5, G6]
Mlist = [M01, M12, M23, M34, M45, M56, M67]

# --- Screw axes of each joint in the space frame (columns = joints) ---
Slist = np.array([[0,         0,         0,         0,        0,        0],
                [0,         1,         1,         1,        0,        1],
                [1,         0,         0,         0,       -1,        0],
                [0, -0.089159, -0.089159, -0.089159, -0.10915, 0.005491],
                [0,         0,         0,         0,  0.81725,        0],
                [0,         0,     0.425,   0.81725,        0,  0.81725]])

# --- Calling UR5 dynamics functions and defining shared external parameters ---
gravity = np.array([0, 0, -9.81])
dt = 0.01
intRes = 1
dthetalist0 = np.zeros(6)  # robot starts from rest, float by default

# --- Scenario 1: Home configuration, 3 seconds ---
thetalist0 = np.zeros(6)
total_time = 3
N_steps = int(total_time / dt)
taumat = np.zeros((N_steps, 6))
Ftipmat = np.zeros((N_steps, 6))
thetamat, dthetamat = UR5_dynamics_Algo(thetalist0, dthetalist0, taumat, gravity, Ftipmat, Mlist, Glist, Slist, dt, intRes)
export_csv(thetamat, "simulation1.csv")

# --- Scenario 2: Joint 2 at -1 radian, 5 seconds ---
thetalist0 = np.array([0, -1, 0, 0, 0, 0], dtype=float)
total_time = 5
N_steps = int(total_time / dt)
taumat = np.zeros((N_steps, 6))
Ftipmat = np.zeros((N_steps, 6))
thetamat, dthetamat = UR5_dynamics_Algo(thetalist0, dthetalist0, taumat, gravity, Ftipmat, Mlist, Glist, Slist, dt, intRes)
export_csv(thetamat, "simulation2.csv")




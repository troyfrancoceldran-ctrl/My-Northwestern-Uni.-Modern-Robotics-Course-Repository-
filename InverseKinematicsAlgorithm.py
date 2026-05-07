"""A SIMULATION PROJECT FOR THE PARTIAL COMPLETION COURSE FOR NORTHWESTERN UNIVERSITY'S MODERN ROBOTICS COURSE

DEVELOPED BY: TROY CELD
DATE: 04/15/2025
TITLE: UR5 ROBOT ARM INVERSE KINEMATICS ALGORITHM 
"""

# Import Libraries
import numpy as np
import modern_robotics as mr
import csv  

#==========================================
#PART I. INVERSE KINEMATICS ALGORITHM
#==========================================
#The function
def IKinBodyIterates(B_list, M, T, angles, e_omg, e_v):
    """Computes inverse kinematics in the body frame and logs iterates to CSV."""
    
    # Initialize variables and tracking list
    angle_list = np.array(angles, dtype=float)
    i = 0
    max_ite = 20 
    joint_vectors = []
    
    # Calculate initial end-effector configuration and error twist
    T_initial = mr.FKinBody(M, B_list, angle_list)
    V_b = mr.se3ToVec(mr.MatrixLog6(mr.TransInv(T_initial) @ T))

    err_w, err_v = np.linalg.norm(V_b[:3]), np.linalg.norm(V_b[3:])
    
    # The actual Newton-Raphson ALgorithm
    while (err_w > e_omg or err_v > e_v) and i < max_ite:
        
        # Store current joint angles for CSV export
        joint_vectors.append(angle_list.tolist())
        
        print(f"Iteration {i}:")
        print(f"Joint Vector: {angle_list}")
        print(f"SE(3) End-Effector Configuration:\n{T_initial}")
        print(f"Twist V_b: {V_b}")
        print(f"Angular Magnitude: {err_w}")
        print(f"Linear Magnitude: {err_v}")
        print("-" * 40)
        
        # Compute Body Jacobian and update joint angles
        Jb = mr.JacobianBody(B_list, angle_list)
        angle_list += np.linalg.pinv(Jb) @ V_b
        
        # Recalculate configuration and error for the next iteration
        T_initial = mr.FKinBody(M, B_list, angle_list)
        V_b = mr.se3ToVec(mr.MatrixLog6(mr.TransInv(T_initial) @ T))
        err_w, err_v = np.linalg.norm(V_b[:3]), np.linalg.norm(V_b[3:])
        
        i += 1
        
    # Log the final converged (or max iterations) state
    joint_vectors.append(angle_list.tolist())
    
    print(f"Iteration {i}:")
    print(f"Joint Vector: {angle_list}")
    print(f"SE(3) End-Effector Configuration:\n{T_initial}")
    print(f"Twist V_b: {V_b}")
    print(f"Angular Magnitude: {err_w}")
    print(f"Linear Magnitude: {err_v}")
    print("True Roots found!" if (err_w <= e_omg and err_v <= e_v) else "Max iterations reached.")
    
    # Export joint angle history to a CSV file
    with open("iterates.csv", mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerows(joint_vectors)
        
    success = (err_w <= e_omg and err_v <= e_v)
    return angle_list, success

#==========================================
#PART II. TEST SCRIPT/ FROM EXAMPLE 4.5
#THIS IS FOR THE UR5 ROBOT ARM VALUES
#==========================================

if __name__ == "__main__":
    
    # 1. UR5 Constants(these are mm converted to m)
    W1, W2 = 0.109, 0.082
    L1, L2 = 0.425, 0.392
    H1, H2 = 0.089, 0.095

    # 2. Home Configuration (M) based from UR5 Constants
    M = np.array([[-1.0,  0.0,  0.0,  0.817],
                [ 0.0,  0.0,  1.0,  0.191],
                [ 0.0,  1.0,  0.0, -0.006],
                [ 0.0,  0.0,  0.0,  1.0  ]])

    # 3. Screw Axes in the Body Frame (B_list) 
    B1 = [0,  1,  0,  W1 + W2,     0,         L1 + L2]
    B2 = [0,  0,  1,  H2,         -L1 - L2,   0      ]
    B3 = [0,  0,  1,  H2,         -L2,        0      ]
    B4 = [0,  0,  1,  H2,          0,         0      ]
    B5 = [0, -1,  0, -W2,          0,         0      ]
    B6 = [0,  0,  1,  0,           0,         0      ]
    
    # B_list must be a 6xn matrix, where each column is a screw axis
    B_list = np.array([B1, B2, B3, B4, B5, B6]).T

    # 4. Desired End-Effector Configuration (T_sd)
    T_sd = np.array([[ 0.0,  1.0,  0.0, -0.5],
                    [ 0.0,  0.0, -1.0,  0.1],
                    [-1.0,  0.0,  0.0,  0.1],
                    [ 0.0,  0.0,  0.0,  1.0]])

    # 5. Tolerances
    e_omg, e_v = 0.001, 0.0001

    # 6. Initial Guess(Angles of guesses)
    initial_guess = [np.pi/2, -0.5, 0.5, -0.5, 0.0, 0.0]

    # 7. Call the function
    print("Initializing Newton-Raphson Inverse Kinematics...\n")
    final_angles, success = IKinBodyIterates(B_list, M, T_sd, initial_guess, e_omg, e_v)
    
    print("\n--- FINAL RESULTS ---")
    print(f"Final Joint Angles: {final_angles}")
    print(f"Successfully Converged: {success}")
    #If it converges, it should substitute to SE(3) and equal to T_sd
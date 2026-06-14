This project simulates the free-fall motion of a UR5 6-DOF industrial robot arm under gravity using the Forward Dynamics algorithm from the Modern Robotics Python library.

The simulation assumes zero motor torques, zero friction, and zero end-effector wrench — meaning the robot simply falls from a specified initial configuration under the influence of gravity (g = 9.81 m/s² in the −z direction), starting from rest. 

The script automatically generates two CSV files: simulation1.csv, which simulates the robot falling from the home configuration (all joints at zero) for 3 seconds, and simulation2.csv, which simulates the robot falling from a configuration where joint 2 is at −1 radian for 5 seconds.

Both simulations run at 100 integration steps per second (dt = 0.01s). The CSV files are formatted with six comma-separated joint angles per row, one row per timestep, and are compatible with the CoppeliaSim UR5 CSV animation scene (Scene 2).

To run the simulation, execute UR5_Forward_Dynamics_Algorithm.py directly — both CSV files will be generated automatically in the same directory. Dependencies required are numpy and 
modern_robotics.

— Troy Celdran
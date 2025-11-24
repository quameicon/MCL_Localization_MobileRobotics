MCL_Localization_MobileRobotics
===============================

This project implements Monte Carlo Localization (Particle Filter) for a mobile robot in Webots.

Files:
------
- controllers/MCLocalization/MCLocalization.py : Main Python controller
- worlds/MCLocalization.wbt                  : Webots world file with landmarks and robot
- results/                                   : Folder where CSV logs and plots will be saved

Requirements:
-------------
- Webots 2025 or later
- Python 3.0 and later
- pip install numpy matplotlib

Setup:
------
1. Open Webots and load the 'MCLocalization.wbt' world.
2. Set the robot controller to 'MCLocalization'.
3. Ensure the robot has the following devices enabled in Webots:
   - GPS
   - Inertial Unit (imu)
   - LiDAR
   - Left and Right Wheel Encoders

Running:
--------
1. Adjust parameters in `MCLocalization.py`:
   - NUM_PARTICLES : Number of particles in the filter
   - SAVE_RESULTS  : True/False to save CSV/plots
   - SNAP_EVERY    : How often to save intermediate snapshots
2. Press Play in Webots to start the simulation.
3. Observe particles converging to the robot’s true pose.
4. CSV and plot files will be saved to the 'results/' directory if SAVE_RESULTS=True.

Outputs:
--------
- Real-time particle filter visualization.
- Console logs for each step including:
  - GPS, IMU yaw
  - Odometry
  - LiDAR min/max
  - Predicted pose
  - RMSE
- CSV file with true vs estimated pose for analysis.
- Final snapshot plot of particles and estimated pose.

Notes:
------
- The particle filter updates using wheel odometry and range measurements to landmarks.
- Particles are resampled systematically to avoid depletion.
- Increasing NUM_PARTICLES improves accuracy but slows simulation.
- RMSE can be used to evaluate localization performance.

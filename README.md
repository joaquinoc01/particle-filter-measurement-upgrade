# Particle Filter Measurement Upgrade

## Overview

This project implements a 2D particle filter for robot localization using a range sensor measurement model. It is based on the exercise from Chapter 6 ("Measurement") of *Probabilistic Robotics* by Thrun, Burgard, and Fox (1999). The filter uses ray casting against a map with walls to simulate sensor measurements and updates particle weights accordingly.

## Features

- **Particle filter localization** in 2D with motion and measurement updates.
- **Ray casting measurement model** to simulate expected sensor ranges for each particle.
- **Sensor noise modeled with Gaussian distributions.**
- **Motion model with noisy rotations and translations.**
- **Adaptive resampling** based on effective sample size (Neff).
- **Angle normalization** to keep orientation within [0, 2π].
- **Logging** of robot and estimated trajectories for analysis.
- **Visualization script** (`plot_trayectory.py`) to plot trajectories.

## Upgrades From Initial Measurement Upgrade

- Increased the number of simulated rays from 5 to 19, covering a 180° field of view, improving sensor information and localization accuracy.
- Added realistic Gaussian noise to sensor measurements.
- Implemented adaptive resampling triggered when Neff falls below half the number of particles to prevent particle depletion.
- Improved weight calculation by using log probabilities for numerical stability.
- Included variance/spread output to monitor particle distribution over time.

## How to Build and Run

1. Create a build directory and navigate into it:

       mkdir build
       cd build

2. Run CMake to configure the build:

       cmake ..

3. Compile the project:

       make

4. Run the particle filter executable:

       ./particle_filter_measurement_upgrade

5. Return to the root directory:

       cd ..

6. (Optional) Activate your Python virtual environment (if using Ubuntu or similar):

       source venv/bin/activate

7. Run the visualization script to see the results:

       python plot_trayectory.py

## Adding a Trajectory Plot (PNG)

To visualize the particle filter results:

- The executable generates a CSV log file `trajectory_log.csv` containing the robot's true position and the filter's estimated position.
- Use the provided `plot_trayectory.py` Python script to plot the trajectories.
- The script outputs a PNG file (e.g., `trajectory_plot.png`) showing the robot’s path and estimated path for easy comparison.

## Screenshot

![Particle Filter Trajectory](https://github.com/user-attachments/assets/8c6fcae9-d288-4d01-a6de-26a00d1b3ff1)

## Reference

- S. Thrun, W. Burgard, and D. Fox, *Probabilistic Robotics*, MIT Press, 1999.

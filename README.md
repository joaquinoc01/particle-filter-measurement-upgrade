# Particle Filter with Measurement Upgrade

This project implements a Particle Filter for robot localization in a 2D environment, with a focus on integrating sensor measurement updates using raycasting-based simulated range readings.

---

## Features

- **2D Map Representation**: A square environment with defined walls and landmarks.
- **Robot Model**: Simulates noisy motion (forward movement and rotation) and noisy range measurements to the environment.
- **Particle Filter**: 
  - Initialization of particles around the robot’s initial pose.
  - Motion update with noise applied to rotation and translation.
  - Measurement update based on raycasting simulated sensor readings.
  - Weight calculation using a Gaussian sensor model.
  - Resampling based on effective sample size (Neff) to avoid particle degeneracy.
- **Pose Estimation**: Uses a weighted mean of particles’ poses (including proper handling of orientation).

---

## Upgrades from Initial Measurement Upgrade

- Increased number of simulated rays from 5 to 19 to better capture the sensor’s field of view and improve localization accuracy.
- Expanded the field of view for raycasting to 180 degrees, evenly distributing rays across this range for more comprehensive environment sensing.
- Added Gaussian noise to sensor measurements to model realistic sensor behavior.
- Implemented effective sample size (Neff) based adaptive resampling to maintain particle diversity and filter robustness.
- Enhanced pose estimation by computing a weighted average over particle orientations using sine and cosine components for better angular averaging.
- Added detailed logging of robot and estimated poses to CSV for trajectory analysis.

---

## Visualization

![Map and Particle Filter Visualization](https://github.com/user-attachments/assets/f7600577-82e5-483a-8fd1-947b36043dec)

---

## References

- Thrun, S., Burgard, W., & Fox, D. (2005). *Probabilistic Robotics*. MIT Press.

# Visual Monte Carlo Localization (Visual-MCL)
## Overview
### Goal
To accurately estimate the robot’s 2D pose $(x, y, \theta)$ in a known map using **Monte Carlo Localization (MCL)**.

MCL algorithm uses a **particle filter** to represent a probability distribution over the robot’s pose. Each particle is a hypothesis and is updated over time based on the following steps.

### MCL Algorithm
1. **Initialization**: randomly scatter particles across the known map each with an equal weight.
2. **Motion Update**: apply the robot’s control inputs (e.g., odometry or velocity commands) to all particles (optional: add Gaussian noise to simulate uncertainty).
3. **Sensor Update**: for each particles, predict the expected tag pose given the known tag location, then compare it to the observed tag pose.
4. **Resampling**: normalize particle weights and resample a new set of particles (proportional to their weights).

> [!NOTE]
> This project will use **AprilTag** detections as a sensor model, enabling the robot to localize only with image data.

## Requirements
* ROS Noetic
* C++
* OpenCV
* AprilTag

## Build & Run

## Project Structure
```
visual-mcl/
├── CMakeLists.txt
├── package.xml
├── launch/
│   └── 
├── src/
│   └── 
├── include/
│   └── 
```

## Acknowledgements
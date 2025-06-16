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
> This project will use KITTI dataset.

## Requirements
* ROS Noetic
* C++
* OpenCV

## Build & Run
### Docker
```
# Build a docker image
docker build -t <named-image> .

# Create a docker container (-it for interactive)
docker run -it --name <named-container> \
    --workdir /home/dev/catkin_ws \
     <named-image>

# Start the container (-i for interactive)
docker start -i named-container
```

### Build Catkin Workspace
```
# Step 1: create the catkin workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# Step 2: clone the repo into the src folder
git clone https://github.com/heonjik/visual-mcl
cd ~/catkin_ws

# Step 4: build the workspace
catkin_make

# Step 5: source the setup file
source devel/setup.bash
```

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
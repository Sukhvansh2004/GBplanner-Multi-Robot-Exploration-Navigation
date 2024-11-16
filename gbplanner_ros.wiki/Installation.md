This page will provide the instructions to setup a workspace to run gbplanner in simulation.  

These instructions assume that ROS desktop-full of the appropriate ROS distro is installed.

## Install necessary libraries:

**For Ubuntu 18.04 and ROS Melodic:**
```bash
sudo apt install python-catkin-tools \
libgoogle-glog-dev \
ros-melodic-joy \
ros-melodic-twist-mux \
ros-melodic-interactive-marker-twist-server
```
**For Ubuntu 20.04 and ROS Noetic:**
```bash
sudo apt install python3-catkin-tools \
libgoogle-glog-dev \
ros-noetic-joy \
ros-noetic-twist-mux \
ros-noetic-interactive-marker-twist-server
```

## Clone and install necessary packages:
Create the workspace:
```bash
mkdir -p gbplanner2_ws/src/exploration
cd gbplanner2_ws/src/exploration
```
Clone the planner
```bash
git clone git@github.com:ntnu-arl/gbplanner_ros.git
```

Clone and update the required packages:
```bash
cd <path/to/gbplanner2_ws>
wstool init
wstool merge ./src/exploration/gbplanner_ros/packages_ssh.rosinstall
wstool update
```

### Build:
```bash
catkin config -DCMAKE_BUILD_TYPE=Release
catkin build
```
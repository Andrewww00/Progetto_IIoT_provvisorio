# IIoT Project
This project aims to control a simulated drone and make it react to external factors such as the presence of obstacles along its path.
First, the drone is sent on a mission with GPS coordinates. When it encounters an obstacle, it switches to obstacle avoidance mode. After successfully avoiding the obstacle, the drone continues towards the mission point.

## Prerequisites
* Ubuntu 22.04
* ROS2 Humble
* PX4 Autopilot
* MAVROS
* 
## Setup
### Install PX4 Autopilot
To [Install PX4](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html#simulation-and-nuttx-pixhawk-targets) run this code 
```
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
```

Run this script in a bash shell to install everything

```
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
```
Reboot or log out.

### Install ROS2 Humble
To install ROS2 Humble: [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

### Install Dependencies

Install Python dependencies with this code

```
pip3 install --user -U empy pyros-genmsg setuptools
```

And:

```
pip3 install kconfiglib
pip install --user jsonschema
pip install --user jinja2
```

### Install MAVROS
To install MAVROS run:
```
sudo apt-get install ros-humble-mavros ros-humble-mavros-extras
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod +x install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh
```

### Install Gazebo Classic
To install Gazebo Classic run this:
```
sudo apt remove gz-garden
sudo apt install aptitude
sudo aptitude install gazebo libgazebo11 libgazebo-dev
```

### Create Folder
First of all we need to create a folder in the home directory:
```
mkdir -p ~/ros_ws
```
inside the new folder clone this repo:
```
git clone "https://github.com/Andrewww00/ros2_ws.git"
```


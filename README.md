# ign_robot
A robot example like Robocon-robot for test

```bash
sudo apt install -y \
    ros-galactic-ros2-control \
    ros-galactic-ign-ros2-control \
    ros-galactic-ros-ign \
    ros-galactic-xacro
```

## Build

```bash
source /opt/ros/galactic/setup.bash
mkdir ros2_ws/src -p
cd ros2_ws/src
git clone git@github.com:NITKK-ROS-Team-Dev/ign_robot.git
cd ../
colcon build
```

## run

```bash
source ros2_ws/install/setup.bash
ros2 launch ign_robot omni.launch.py
```

# Unitree GO2 ros2_simulation

This repository includes packages for Unitree GO2, quadrupedal robot

## Prerequisties
* Ubuntu 22.04
* ros2-humble

## Installation
* Installation of ros2-humblee: [https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html](), and source the ros2 package in the terminal.
```bash
source /opt/ros/humble/setup.bash
```

* Install dependencies
```bash
sudo apt-get update

rosdep install --from-paths src --ignore-src -r -y

```
* Install program
```bash
cd ros2_ws/

colcon build --symlink-install

source install/setup.bash
```

## Usage of go2 simulation and control
To connect the GO2 in Gazebo
* Simulation in Gazebo
```bash
## terminal 1
ros2 launch go2_gazebo spawn_go2.launch.py world_file_name:=cone.world
## terminal 2: stand up!
ros2 run go2_control simple_publisher_demo.py
```
<!-- <p align="center">
  <img src="https://github.com/TharitSinsunthorn/noppakorn-test/blob/develop/go2_gazebo.png" alt="go2's Gazebo">
</p> -->


## Packages description 
### go2_description
- URDF and mesh files for go2.
- ros2 _control and controllers configurations
- RVIZ configuration.
- Launch file for visualizing the go2 model in RVIZ

```bash 
# For default model
ros2 launch go2_description go2_visualize.launch.py

```

### go2_gazebo
- Model and world files for Gazebo simulation. 
- Launch file to spawn robot in Gazebo


<!-- ### go2_custom_interfaces
- custom **msg** and **srv** for ros2 communication in go2 -->

### go2_control
- Control scripts 

<!-- ### go2_camera
- Package for the future work of implementation of depth camera on the go2. -->

### go2_teleop
- Joy stick conteller configuration
- Launch file and script setting for go2 teleoperation.


## Author
Tharit Sinsunthorn

## Contributing

Pull requests are welcome. For major changes, please open an issue first
to discuss what you would like to change.

Please make sure to update tests as appropriate.
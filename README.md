# FIRA Autonomous Cars Simulator
FIRA Autonomous Cars Simulator for FIRARoboWorldCup Competition.
The instructions necessary to get started is available below.

### Changelog
## 1.0.4
  - Fixed and updated for ROS Noetic (Special Thanks to [Sina Moghimi](https://github.com/sinamoghimi73) for the update)
## 1.0.2
  - Removed ground plane from race track world
  - Modified Race Track mesh
  - Changed initial position of the car to be on the track
  - Added an example_pkg package to demonstrate a basic usage of topics and driving the vehicle
  - Modified the ambient color of the scene in the race track
  - Added sky and clouds to the race track

### Tested Minimum Local Hardware Requirements
CPU: Intel® Core™ i5-5257U CPU @ 2.70GHz <br/>
GPU: Intel® Iris 6100 <br/>
RAM: 8 GB

### Software Requirements
Ubuntu 20.04 and ROS Noetic used exclusively. Other versions are not officially supported.
Prior to installing our software make sure to have ROS and Catkin tools installed: http://wiki.ros.org/noetic/Installation/Ubuntu
```zsh
sudo apt-get install python3-catkin-tools python3-catkin-pkg python3-osrf-pycommon

sudo apt install ros-noetic-can-msgs ros-noetic-velocity-controllers ros-noetic-velodyne-pointcloud ros-noetic-teleop-twist-keyboard ros-noetic-gazebo-ros ros-noetic-gazebo-ros-control ros-noetic-gazebo-ros-pkgs ros-noetic-hector-gazebo-plugins ros-noetic-hector-gazebo-worlds ros-noetic-hector-gazebo ros-noetic-ros-controllers
```

### Installation
```zsh
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
mkdir src
cd src

git clone https://github.com/Fira-Autonomous-Cars/FIRA-Autonomous-Cars-Simulator.git

catkin_init_workspace
cd ..
catkin_make

echo 'source ~/catkin_ws/devel/setup.zsh' >> ~/.zshrc
source ~/.zshrc

roslaunch avisengine_environment update_world.launch #Important : This command will update every static paths in this package. Make sure to run this, otherwise the gazebo simulator won't run correctly.
```
The installation process is done.


### Running the simulator
After successfully finishing the installation process, everything is ready.
Launching the following command will start the gazebo simulator in the race track.
```bash
roslaunch avisengine_environment track_race_simple.launch 
```
You can get the front camera image from /catvehicle/camera_front/image_raw_front/compressed topic and can send control commands using a Twist message to /catvehicle/cmd_vel.

Get a list of topics by running the following command:
```bash
rostopic list
```
The car can also be driven manually by launching the following command:
```bash
roslaunch catvehicle_tests cmdvel_unsafetest.launch 
```
use the keys in your keyboard to drive the car manually: <br/>
i : Move Forward <br/>
, : Move Backward <br/>
<br/>
u : Turn Left and Forward <br/>
o : Turn Right and Forward <br/>
<br/>
m : Turn Left and Backward <br/>
. : Turn Right and Backward <br/>
<br/>
k : Center and stop <br/>
q, z : Increase or decrease speed <br/>

### Usage of the example package
There is an example package in [/example_pkg](https://github.com/Fira-Autonomous-Cars/FIRA-Autonomous-Cars-Simulator/tree/main/example_pkg) that demonstrates a basic usage of the simulator in python. 
[/example_pkg/src/drive.py](https://github.com/Fira-Autonomous-Cars/FIRA-Autonomous-Cars-Simulator/blob/main/example_pkg/src/drive.py) is used to drive the vehicle and [/example_pkg/src/imageReceive.py](https://github.com/Fira-Autonomous-Cars/FIRA-Autonomous-Cars-Simulator/blob/main/example_pkg/src/imageReceive.py) is used to receive image from the vehicle. 
The drive node uses Twist message to control the vehicle through the /catvehicle/cmd_vel topic.


### Customizing the Track
![Screenshot](https://drive.google.com/uc?export=view&id=1oJH5bNRIKqogS_7FeN3fB1J4HJtgAz7f) |
------------ |
Race track mesh in blender |


You can customize the race track by modifying the 3D file in [avisengine_environment/meshes/mesh_road.blend](https://github.com/Fira-Autonomous-Cars/FIRA-Autonomous-Cars-Simulator/blob/main/avisengine_environment/meshes/mesh_road.blend) using [Blender](https://www.blender.org/) which is a free and **open-source 3D computer graphics software**.
The instructions on how to modify this file is written in the blender file. 

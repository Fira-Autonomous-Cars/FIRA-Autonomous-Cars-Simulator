
# FIRA Autonomous Cars Simulator
FIRA Autonomous Cars Simulator for FIRA RoboWorld Cup Competition.
The instructions necessary to get started are available below.

# Changelog
## 1.1.2
- Dropped the dependency from the `gazebo_apriltag` submodule.
- Added `model.config` to `gazebo_apriltag`.

## 1.1.1
- Removed the excess model_updater.py

## 1.1.0
- Added new urban track
- Added new AprilTags models system for gazebo
- Added new Sign models system for gazebo
- Added urban track blender file
- Added models paths
- Modified urban track launch file
- Dropped the requirement to run model_updater.py in the setup
- Improved worlds lighting
- Improved the tags and sign system in terms of usability and ease of use

## 1.0.4
  - Fixed and updated for ROS Noetic (Special Thanks to [Sina Moghimi](https://github.com/sinamoghimi73) for the update)
## 1.0.2
  - Removed ground plane from race track world
  - Modified Race Track mesh
  - Changed the initial position of the car to be on the track
  - Added an example_pkg package to demonstrate basic usage of topics and driving the vehicle
  - Modified the ambient color of the scene on the race track
  - Added sky and clouds to the race track

## Tested Minimum Local Hardware Requirements
CPU: Intel® Core™ i5-5257U CPU @ 2.70GHz <br/>
GPU: Intel® Iris 6100 <br/>
RAM: 8 GB

## Software Requirements
Ubuntu 20.04 and ROS Noetic are used exclusively. Other versions are not officially supported.
Prior to installing our software make sure to have ROS and Catkin tools installed: http://wiki.ros.org/noetic/Installation/Ubuntu
```zsh
sudo apt-get install python3-catkin-tools python3-catkin-pkg python3-osrf-pycommon

sudo apt install ros-noetic-can-msgs ros-noetic-velocity-controllers ros-noetic-velodyne-pointcloud ros-noetic-teleop-twist-keyboard ros-noetic-gazebo-ros ros-noetic-gazebo-ros-control ros-noetic-gazebo-ros-pkgs ros-noetic-hector-gazebo-plugins ros-noetic-hector-gazebo-worlds ros-noetic-hector-gazebo ros-noetic-ros-controllers
```

## Installation
```zsh
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

git clone https://github.com/Fira-Autonomous-Cars/FIRA-Autonomous-Cars-Simulator.git

catkin_init_workspace
cd ..
catkin_make

echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

# For ZSH users
echo 'source ~/catkin_ws/devel/setup.zsh' >> ~/.zshrc
source ~/.zshrc

```
The installation process is done.

## Usage
After successfully finishing the installation process, everything is ready to run.

### Race Track
Launching the following command will start the gazebo simulator in the race track.
```bash
roslaunch avisengine_environment track_race_simple.launch 
```

### Urban Track
Launching the following command will start the gazebo simulator in the urban track.
```bash
roslaunch avisengine_environment track_urban_simple.launch 
```

### Topics
You can get the front camera image from /catvehicle/camera_front/image_raw_front/compressed topic and can send control commands using a Twist message to /catvehicle/cmd_vel.

Get a list of topics by running the following command:
```bash
rostopic list
```
### Manual Control Mode
The car can also be driven manually by launching the following command:
```bash
roslaunch catvehicle_tests cmdvel_unsafetest.launch 
```
Use your keyboard to drive the car manually: <br/>
| Key |  Action |
|--|--|
|i  |Move Forward  |
|,  | Move Backward |
|u  | Turn Left and Forward |
|o  | Turn Right and Forward |
|m  | Turn Left and Backward |
|.   |  Turn Right and Backward |
|k  | Center and stop |
|q, z  |  Increase or decrease speed  |



## Usage of the example package
The example package in [/example_pkg](https://github.com/Fira-Autonomous-Cars/FIRA-Autonomous-Cars-Simulator/tree/main/example_pkg)  demonstrates a basic usage of the simulator written in python. 
[/example_pkg/src/drive.py](https://github.com/Fira-Autonomous-Cars/FIRA-Autonomous-Cars-Simulator/blob/main/example_pkg/src/drive.py) is used to drive the vehicle and [/example_pkg/src/imageReceive.py](https://github.com/Fira-Autonomous-Cars/FIRA-Autonomous-Cars-Simulator/blob/main/example_pkg/src/imageReceive.py) is used to receive image from the vehicle. 
The drive node uses the Twist message to control the vehicle through the /catvehicle/cmd_vel topic.
You can easily plug your code and use the basic topics in the simulator.

## Customizing the Tracks

<table>
<tr>
<td align="center">
  <img width="441" src="https://github.com/Fira-Autonomous-Cars/FIRA-Autonomous-Cars-Simulator/blob/main/images/screenshot_race_track_blender.png?raw=true">
  <p> 
  <small>
  Race Track Model
  </small>
  </p>
</td>
<td align="center">
  <img width="441" src="https://github.com/Fira-Autonomous-Cars/FIRA-Autonomous-Cars-Simulator/blob/main/images/screenshot_urban_track_blender.png?raw=true">
  <p> 
  <small>
  Urban Track Model
  </small>
  </p>
</td>
</tr>
</table>


**Blender files locations**
  
| Name | Path  |
|--|--|
| Race Track Model | [avisengine_environment/meshes/mesh_road.blend](https://github.com/Fira-Autonomous-Cars/FIRA-Autonomous-Cars-Simulator/blob/main/avisengine_environment/meshes/mesh_road.blend) |
| Urban Track Model| [avisengine_environment/meshes/mesh_road_urban.blend](https://github.com/Fira-Autonomous-Cars/FIRA-Autonomous-Cars-Simulator/blob/main/avisengine_environment/meshes/mesh_road_urban.blend)


You can customize both the race track and urban track (base) by modifying the 3D files using [Blender](https://www.blender.org/) which is a free and **open-source 3D computer graphics software**.
The instructions on how to modify this file are written in the blender file. 

### Modify Urban Signs and Tags
Modifying the signs and AprilTags in the urban track can be done by modifying the urban world file, or by just simply adding other tags from the gazebo model library, since all the signs and tags are also added to the library and are ready to use. 

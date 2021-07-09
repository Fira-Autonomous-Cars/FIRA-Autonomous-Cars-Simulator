# FIRA Autonomous Cars Simulator
FIRA Autonomous Cars Simulator for FIRARoboWorldCup Competition.
The instructions necessary for getting started is available below.

### Tested Minimum Local Hardware Requirements
CPU: Intel® Core™ i5-5257U CPU @ 2.70GHz <br/>
GPU: Intel® Iris 6100 <br/>
RAM: 8 GB

### Software Requirements
Ubuntu 16.04 and ROS Kinetic used exclusively. Other versions are not officially supported.
Prior to installing our software make sure to have ROS and Catkin tools installed: http://wiki.ros.org/kinetic/Installation/Ubuntu
```bash
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo apt-get install python-catkin-tools
sudo apt install python-wstool
sudo apt install ros-kinetic-ros-control 
sudo apt-get install ros-kinetic-gazebo-ros-control ros-kinetic-position-controllers ros-kinetic-ros-controllers ros-kinetic-velodyne ros-kinetic-velodyne-driver 
pip install catkin_pkg
```

### Installation
```bash
# Setup catkin workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin init
# Add workspace to bashrc.
echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc
cd src
git clone https://github.com/Fira-Autonomous-Cars/FIRA-Autonomous-Cars-Simulator.git
cd ..
catkin_make
source ~/.bashrc
```
The installation process is done.

### Running the simulator
After successfully finishing the installation process, everything is ready.
Launching the following command will start the gazebo simulator in the race track.
```bash
roslaunch avisengine_environment track_race_simple.launch 
```
You can get the front camera image from /catvehicle/camera_front/image_raw_front/compressed topic and can send steering and velocity through their topics.
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

### Customizing the Track
You can customize the race track by modifying the 3D file in [avisengine_environment/meshes/mesh_road.blend](https://github.com/Fira-Autonomous-Cars/FIRA-Autonomous-Cars-Simulator/blob/main/avisengine_environment/meshes/mesh_road.blend) using [Blender](https://www.blender.org/) which is a free and **open-source 3D computer graphics software**.
The instructions on how to modify this file is written in the blender file. 

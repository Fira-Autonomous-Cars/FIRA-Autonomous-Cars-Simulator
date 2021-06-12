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



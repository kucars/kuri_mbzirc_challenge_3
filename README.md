# KURI - MBZIRC Challenge 3

Challenge 3 related tasks implementation

In order run the simulation environment the following Packages are needed: 

- ROS
- RotorS rotor_simulator    
- mavros     
- Firmware     
- mbzirc simulation environment    
- Challenge 3 specific environment    

===============
## ROS

 ROS Indigo desktop full, additional ROS packages, catkin-tools, and wstool are needed to run the simulation, follow the following basic steps for an initial setup:

### Installation Instructions
-------------------------

 ```
 $ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
 $ wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
 $ sudo apt-get update
 $ sudo apt-get install ros-indigo-desktop-full ros-indigo-joy ros-indigo-octomap-ros python-wstool python-catkin-tools
 $ sudo rosdep init
 $ rosdep update
 $ source /opt/ros/indigo/setup.bash
 ```
More information about how to install ROS can be found at: http://wiki.ros.org/indigo/Installation

===============
## Rotor simulator
RotorS is a MAV gazebo simulator.
This packages also contains some example controllers, basic worlds, a joystick interface, and example launch files.

### Installation Instructions
-------------------------

 1. If you don't have ROS workspace yet you can do so by:

 ```
 $ mkdir -p ~/catkin_ws/src
 $ cd ~/catkin_ws/src
 $ catkin_init_workspace  # initialize your catkin workspace
 $ wstool init
 ```
 > **Note** for setups with multiple workspaces please refer to the official documentation at http://docs.ros.org/independent/api/rosinstall/html/ by replacing `rosws` by `wstool`.
 
 2. rotors_simulator requires extra dependencies, get the simulator and additional dependencies

 ```
 $ cd ~/catkin_ws/src
 $ git clone https://github.com/kuri-kustar/rotors_simulator 
 $ git clone https://github.com/ethz-asl/mav_comm.git
 $ git clone https://github.com/ethz-asl/glog_catkin.git
 $ git clone https://github.com/ethz-asl/catkin_simple.git 
 ```
 3. Build your workspace with `python_catkin_tools` (therefore you need `python_catkin_tools`)

   ```
   $ cd ~/catkin_ws/
   $ catkin init  # If you haven't done this before.
   $ catkin build
   ```

 4. Add sourcing to your `.bashrc` file

   ```
   $ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
   $ source ~/.bashrc
   ```

====== 
## Mavros 

This package provides communication driver for various autopilots with MAVLink communication protocol. 

### Installation Instructions
-------------------------
1. Get the mavros packages 
 ```
$ sudo apt-get install ros-indigo-mav* 
```

=======
## Firmware

### Installation Instructions
-------------------------
This package contains the PX4 Flight Core that simulates the autopilot. Clone the following package: 
 ```
 $ cd ~/catkin_ws/src
 $ git clone https://github.com/PX4/Firmware.git
 $ cd ~/catkin_ws/
 $ catkin_make --cmake-args -DCONFIG=ros_sitl_simple
 ```
 
========= 
## MBZIRC Simulation Environment  

kuri_mbzirc_sim is a package that simulates the environment of MBZIRC challenges 
 
### Installation Instructions
-------------------------
 This package contains the 3 arenas used for the three challenges. First you need to clone the package: 
  ```
 $ cd ~/catkin_ws/src
 $ git clone https://github.com/kuri-kustar/kuri_mbzirc_sim.git
 ```
Basic Usage
-----------

Launch the simulator for main arena 

```
$ roslaunch kuri_mbzirc_sim mbzirc_arena.launch
```
========= 

## MBZIRC Challenge 3 

kuri_mbzirc_challenge_3 is a package that launchs three drones in the start position it contains the related tasks for this challenge. 

### Installation Instructions
-------------------------
Clone the package: 
  ```
 $ cd ~/catkin_ws/src
 $ git clone https://github.com/kuri-kustar/kuri_mbzirc_challenge_3.git
 ```
 
 Basic Usage
-----------

Launch the simulation environment with three drones:

```
$ roslaunch kuri_mbzirc_challenge_3 task.launch
```

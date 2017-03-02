# KURI - MBZIRC Challenge 3

[![Build Status](https://travis-ci.org/kuri-kustar/kuri_mbzirc_challenge_3.svg?branch=master)](https://travis-ci.org/kuri-kustar/kuri_mbzirc_challenge_3) [![Join the chat at https://gitter.im/kuri-kustar/kuri_mbzirc_challenge_3](https://badges.gitter.im/kuri-kustar/kuri_mbzirc_challenge_3.svg)](https://gitter.im/kuri-kustar/kuri_mbzirc_challenge_3?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

Challenge 3 related tasks implementation

In order run the simulation environment the following Packages are needed: 

- ROS
- RotorS rotor_simulator    
- mavros     
- Firmware     
- mbzirc simulation environment    
- Challenge 3 specific environment    

# Installing using rosinstall
```
cd <catkin_ws>
$ wstool init src
$ wstool set -t src kuri_mbzirc_challenge_3 https://github.com/kuri-kustar/kuri_mbzirc_challenge_3.git --git
$ wstool merge -t src https://raw.githubusercontent.com/kuri-kustar/kuri_mbzirc_challenge_3/master/mbzirc_challenge3.rosinstall
$ wstool update -t src
$ rosdep install -y -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO
$ catkin_make --cmake-args -DCONFIG=ros_sitl_simple
```

 Basic Usage
-----------

Launch the simulation environment with three drones:

```
$ roslaunch kuri_mbzirc_challenge_3 task.launch
```

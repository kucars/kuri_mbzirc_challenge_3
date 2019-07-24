# KURI - MBZIRC Challenge 3

[![Build Status](https://travis-ci.org/kucars/kuri_mbzirc_challenge_3.svg?branch=master)](https://travis-ci.org/kucars/kuri_mbzirc_challenge_3) [![Join the chat at https://gitter.im/kucars/kuri_mbzirc_challenge_3](https://badges.gitter.im/kucars/kuri_mbzirc_challenge_3.svg)](https://gitter.im/kucars/kuri_mbzirc_challenge_3?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

Challenge 3 related tasks implementation

In order run the simulation environment the following Packages are needed: 

- ROS
- mavros     
- Firmware     
- sitl_gazebo
- kuri mbzirc simulation environment    
- kuri mbzirc msgs
- Challenge 3 specific environment

# Installing using rosinstall
```
cd <catkin_ws>
$ wstool init src
$ wstool set -t src kuri_mbzirc_challenge_3 https://github.com/kucars/kuri_mbzirc_challenge_3.git --git
$ wstool merge -t src https://raw.githubusercontent.com/kucars/kuri_mbzirc_challenge_3/master/mbzirc_challenge3.rosinstall
$ wstool update -t src
$ rosdep install -y -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO
$ catkin_make --cmake-args -DCONFIG=posix_sitl_default
$ cd <catkin_ws>/src/Firmware
$ source Tools/setup_gazebo.bash $(pwd) $(pwd)/build_posix_sitl_default
$ export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
$ export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
```

 Basic Usage
-----------

Launch the simulation environment with three drones:

```
$ roslaunch kuri_mbzirc_challenge_3 task.launch
```

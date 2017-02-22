#!/bin/bash

rosservice call /uav_2/mavros/cmd/arming "value: true"
# sleep 5
rosservice call /uav_2/mavros/cmd/takeoff "{min_pitch: 0.0, yaw: 0.0, latitude: 47.3977418, longitude: 8.5455939, altitude: 10.0}"

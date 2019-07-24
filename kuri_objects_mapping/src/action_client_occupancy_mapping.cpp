/*
 * Copyright (C) 2015, Lentin Joseph and Qbotics Labs Inc.

 * Email id : qboticslabs@gmail.com

 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.


* This code will subscriber integer values from demo_topic_publisher

*/

#include "ros/ros.h"
#include <iostream>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <queue>
#include <sstream>
#include <string>
#include <time.h>
#include <algorithm> 
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/NavSatFix.h>
#include <visualization_msgs/Marker.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "kuri_msgs/MappingAction.h"

typedef kuri_msgs::MappingAction   NodeAction;
int main (int argc, char **argv)
{
  ros::init(argc, argv, "Mapping_action_client");


  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<NodeAction> ac_("create_map", true);

  ROS_INFO("Waiting for action server to start.");

  // wait for the action server to start
  ac_.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");

  // send a goal to the action
  kuri_msgs::MappingGoal goal;
  //goal.count = atoi(argv[1]);
 
  //ROS_INFO("Sending Goal [%d] and Preempt time of [%d]",goal.count, atoi(argv[2]));
  ac_.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = ac_.waitForResult(ros::Duration(20000));
  //Preempting task
  ac_.cancelGoal();

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac_.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
    //Preempting the process
   ac_.cancelGoal();

  }
  else
    ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}
/***************************************************************************
 *   Copyright (C) 2006 - 2016 by                                          *
 *      Tarek Taha, KURI  <tataha@tarektaha.com>                           *
 *      Randa Almadhoun   <randa.almadhoun@kustar.ac.ae>                   *
 *      Ahmed AlDhanhani  <ahmed.aldhanhani@kustar.ac.ae>                                                                   *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Steet, Fifth Floor, Boston, MA  02111-1307, USA.          *
 ***************************************************************************/

#include "ros/ros.h"
#include <ros/package.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include <eigen_conversions/eigen_msg.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

#include <actionlib/server/simple_action_server.h>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
//msg headers.
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <iostream>

//pcl
#include <pcl_ros/point_cloud.h>
#include <pcl/common/common_headers.h>
#include <ctime>

#include "kuri_msgs/Object.h"
#include "kuri_msgs/Objects.h"

#include "kuri_msgs/PickObjectAction.h"
#include "AerialManipulationControl.h"

class AerialManipulationAction
{
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, nav_msgs::Odometry> MySyncPolicy;
public:
  AerialManipulationAction(std::string name) :
    actionServer(nh_, name, false),
    actionName(name)
  {
    //register the goal and feeback callbacks
    actionServer.registerGoalCallback(boost::bind(&AerialManipulationAction::goalCB, this));
    actionServer.registerPreemptCallback(boost::bind(&AerialManipulationAction::preemptCB, this));
    actionServer.start();
    ROS_INFO("Action name %s" , actionName.c_str()) ;
  }

  ~AerialManipulationAction(void)
  {
  }

  void goalCB()
  {
    // accept the new goal
    goal = actionServer.acceptNewGoal()->object_2_pick;
    std::cout << "goal recived" << std::endl << std::flush ;
    result.success = aerialManipulationController.waitforResults(goal) ;
    std::cout << "Object 2 created " << result.success << std::endl << std::flush ;
    actionServer.setSucceeded(result);
  }

  void preemptCB()
  {
    ROS_INFO("%s: Preempted", actionName.c_str());
    // set the action state to preempted
    actionServer.setPreempted();
  }

protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<kuri_msgs::PickObjectAction> actionServer;
  std::string actionName;
  kuri_msgs::Object goal;
  kuri_msgs::PickObjectFeedback feedback;
  kuri_msgs::PickObjectResult   result;
  AerialManipulationControl     aerialManipulationController;

#define Ground_Z 0.0
  //test
  tf::TransformBroadcaster br;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "aerialAction");
  AerialManipulationAction aerialManipulationObject(ros::this_node::getName());
  ros::spin();
  return 0;
}


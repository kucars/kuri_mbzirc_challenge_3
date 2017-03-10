/***************************************************************************
 *   Copyright (C) 2006 - 2016 by                                          *
 *      Tarek Taha, KURI  <tataha@tarektaha.com>                           *
 *      Randa Almadhoun   <randa.almadhoun@kustar.ac.ae>                   *
 *      Ahmed AlDhanhani  <ahmed.aldhanhani@kustar.ac.ae>                  *
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

#include <AerialManipulationControl.h>
#include <string>

ObjectPicker::ObjectPicker(const ros::NodeHandle &_nh, const ros::NodeHandle &_nhPrivate):
  nh(_nh),
  nhPrivate(_nhPrivate)
{
  velPub           = nh.advertise <geometry_msgs ::TwistStamped >("/mavros/setpoint_velocity/cmd_vel", 1);
  goalSub          = nh.subscribe("/visptracker_pose_tunnel", 1000, &ObjectPicker::goalCallback,this);
  compassSub       = nh.subscribe ("/mavros/global_position/compass_hdg", 1, &ObjectPicker::headingCallback,this);

  nh.param("kpx", kpx, 0.05);
  nh.param("kix", kix, 0.0);
  nh.param("kdx", kdx, 0.05);
  nh.param("kpy", kpy, 0.05);
  nh.param("kiy", kiy, 0.0);
  nh.param("kdy", kdy, 0.05);
  nh.param("kpz", kpz, 0.05);
  nh.param("kiz", kiz, 0.0);
  nh.param("kdz", kdz, 0.05);
  nh.param("kpw", kpw, 0.05);
  nh.param("kiw", kiw, 0.0);
  nh.param("kdw", kdw, 0.05);

  nh.param("tolerance_2_goal", tolerance_2_goal, 0.2);

  goalPose.pose.position.x = 0;
  goalPose.pose.position.y = 0;
  goalPose.pose.position.z = 0;
  stopTwist.twist.linear.x  = 0;
  stopTwist.twist.linear.y  = 0;
  stopTwist.twist.linear.z  = 0;
  stopTwist.twist.angular.z = 0;

  ros::Rate loopRate(10);
  while (ros::ok())
  {
    // Failsafe: if we don't get tracking info for more than 500ms, then stop in place
    if(ros::Time::now() - goalLastReceived > ros::Duration(0.5))
    {
      stopTwist.header.stamp = ros::Time::now();
      //velPub.publish(stopTwist);
    }
    else
    {
      velPub.publish(twist);
    }
    ros:: spinOnce ();
    loopRate.sleep();
  }
}

void ObjectPicker::mavrosStateCallback(const mavros_msgs::State::ConstPtr& msg)
{
  currentState = *msg;
}

void ObjectPicker::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  goalLastReceived = ros::Time::now();
  ROS_INFO("TEST"); // this should be done before sending it to this component
  goalPose = *msg;
  real.x= 0 ; //msg ->latitude;
  real.y= 0 ; //msg ->longitude;
  real.z= 0 ; //msg ->altitude;
  w = yaw ;

  if (firstDataFlag == false )
  {
    errorX =  goalPose.pose.position.x - real.x;
    errorY =  goalPose.pose.position.y - real.y;
    errorZ =  goalPose.pose.position.z - real.z ;
    errorW =  w;
    prevErrorX = errorX;
    prevErrorY = errorY;
    prevErrorZ = errorZ;
    prevErrorW = errorW;
    firstDataFlag = true;
  }
  else
  {

    errorX =  goalPose.pose.position.x - real.x;
    errorY =  goalPose.pose.position.y - real.y;
    errorZ =  goalPose.pose.position.z - real.z ;
    errorW =  0;
    pX = kpx * errorX;
    pY = kpy * errorY;
    pZ = kpz * errorZ;
    pW = kpw * errorW;

    iX += kix * errorX;
    iY += kiy * errorY;
    iZ += kiz * errorZ;
    iW += kiw * errorW;

    dX = kdx * (errorX - prevErrorX);
    dY = kdy * (errorY - prevErrorY);
    dZ = kdz * (errorZ - prevErrorZ);
    dW = kdw * (errorW - prevErrorW);

    prevErrorX = errorX;
    prevErrorY = errorY;
    prevErrorZ = errorZ;
    prevErrorW = errorW;

    // PID conroller
    aX = pX     + iX + dX  ;
    aY = pY     + iY + dY  ;
    aZ = pZ      +iZ + dZ  ;
    aW = 10 * pW +iW + dW  ;

    // filling velocity commands
    twist.twist.linear.x = aX;
    twist.twist.linear.y =  aY;
    twist.twist.linear.z = aZ;
    twist.twist.angular.z = aW;
    twist.header.stamp = ros::Time::now();

    ROS_INFO("Error X: %0.2f \n", errorX);
    ROS_INFO("Error Y: %0.2f \n", errorY);
    ROS_INFO("Error Z: %0.2f \n", errorZ);
    ROS_INFO("derivative X: %0.2f \n", dX);
    ROS_INFO("derivative Y: %0.2f \n", dY);
    ROS_INFO("derivative Z: %0.2f \n", dZ);
    ROS_INFO("derivative W: %0.2f \n", dZ);
    ROS_INFO("W: %0.2f \n", w);
    ROS_INFO("Action X: %0.2f \n", aX);
    ROS_INFO("Action Y: %0.2f \n", aY);
    ROS_INFO("Action Z: %0.2f \n", aZ);
    ROS_INFO("Action W: %0.2f \n", aW);

    // publishing this data to be recorded in a bag file
    /*
    pidMsg.dronePoseX = real.x;                       pidMsg.dronePoseY = real.y;                     pidMsg.dronePoseZ = real.z;
    pidMsg.goalPoseX  = goalPose.pose.position.x ;    pidMsg.goalPoseY  = goalPose.pose.position.y;   pidMsg.goalPoseZ  = goalPose.pose.position.z;
    pidMsg.positionErrorX = errorX;                   pidMsg.positionErrorY = errorY;                 pidMsg.positionErrorZ = errorZ;     pidMsg.positionErrorW = errorW;
    pidMsg.PX = pX;                                   pidMsg.PY = pY;                                 pidMsg.PZ = pZ;                     pidMsg.PW = pW;
    pidMsg.IX = iX;                                   pidMsg.IY = iY;                                 pidMsg.IZ = iZ;                     pidMsg.IW = iW;
    pidMsg.DX = dX;                                   pidMsg.DY = dY;                                 pidMsg.DZ = dZ;                     pidMsg.DW = dW;
    pidMsg.PIDX =aX;                                  pidMsg.PIDY = aY;                               pidMsg.PIDZ= aZ;                    pidMsg.PIDW= aW;
    pidMsg.header.stamp = ros::Time::now();
    pidMsg.header.seq = counter++;
    */
    if ((fabs(errorX) < tolerance_2_goal) && (fabs(errorY) < tolerance_2_goal) && (fabs(errorZ) < tolerance_2_goal))
    {
      twist.twist.linear.x = 0;
      twist.twist.linear.y = 0;
      twist.twist.linear.z = 0;
      twist.twist.angular.z = 0;
    }
  }
}

void ObjectPicker::headingCallback(const std_msgs::Float64::ConstPtr& msg)
{
  yaw = msg->data * 3.14159265359 / 180.0 ;
}

bool AerialManipulationControl::waitforResults(kuri_msgs::Object goal)
{
  std::cout << "goal.color   "<< goal.color << std::endl << std::flush ;
  std::cout << "Constructor of class 2 " << std::endl << std::flush ;
  //publish pointcloud msgs:
  std::string topic2 = nh_.resolveName("/uav_1/mavros/setpoint_velocity/cmd_vel");
  velocity_pub_      = nh_.advertise<geometry_msgs::TwistStamped>(topic2, 1);

  img_sub_         = new message_filters::Subscriber<sensor_msgs::Image>(nh_, "/uav_1/downward_cam/camera/image", 10);
  camera_info_sub_ = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh_, "/uav_1/downward_cam/camera/camera_info", 10);
  uav_odom_sub_    = new message_filters::Subscriber<nav_msgs::Odometry>(nh_, "/uav_1/mavros/local_position/odom", 10);

  //initialize base_link to camera optical link
  BaseToCamera.setOrigin(tf::Vector3(0.0, 0.0, -0.2));
  BaseToCamera.setRotation(tf::Quaternion(0.707, -0.707, 0.000, -0.000));
  std::cout << " Start the action server " << std::endl << std::flush ;
  #define Ground_Z 0.0
  //test
  tf::TransformBroadcaster br;
  ros::Rate loopRate(10);
  while (ros::ok())
  {
    ros::spinOnce();
    loopRate.sleep();
    if (this->flag_2)
    {
      return true ;
      std::cout << " return treu" << std::endl << std::flush ;
    }
    else
    {
      std::cout << "Continue" << std::endl << std::flush ;

      continue ;
    }
  }
}

AerialManipulationControl::AerialManipulationControl()
{
}

AerialManipulationControl::~AerialManipulationControl()
{
}

/***************************************************************************
 *   Copyright (C) 2016 - 2017 by                                          *
 *      Tarek Taha, KURI  <tataha@tarektaha.com>                           *
 *      Randa Almadhoun   <randa.almadhoun@kustar.ac.ae>                   *
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
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int64.h>
#include <nav_msgs/Path.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include "geo.h"
#include <actionlib/client/simple_action_client.h>
#include "kuri_msgs/TrackingAction.h"
#include "kuri_msgs/TrackingActionFeedback.h"
#include "kuri_msgs/TrackingActionGoal.h"
#include "kuri_msgs/TrackingActionResult.h"
#include "kuri_msgs/TrackingGoal.h"
#include "kuri_msgs/TrackingResult.h"
#include "kuri_object_tracking/Object2Track.h"
#include "kuri_object_tracking/Object2TrackRequest.h"
#include "kuri_object_tracking/Object2TrackResponse.h"
#include "kuri_msgs/Object.h"


enum SOLO_STATES
{
  INITIATION,
  EXPLORING,
  PICKING,
  WAITING_FOR_DROP,
  DROPPING
};

geometry_msgs::PoseStamped localPoseH; //with respect to home position
geometry_msgs::PoseStamped localPoseR; //with respect to a defined reference (ex: zurich, or one of the arena corners)
sensor_msgs::NavSatFix globalPose;
ros::Time objectLastTracked;
bool firstDataFlag = true;
int count          = 0 ;
double tolerance   = 0.3;
double errorX      = 0;
double errorY      = 0;
double errorZ      = 0;
double errorW      = 0;
double w           = 0;
double yaw         = 0;
float prevErrorX;
float prevErrorY;
float prevErrorZ;
float prevErrorW;
double tolerance2Goal = 0;
bool homePoseFlag       = false;
bool transformDoneFlag  = false;
std_msgs::Bool finishedFlag;
float pX;
float pY;
float pZ;
float pW;
float iX;
float iY;
float iZ;
float iW;
float dX;
float dY;
float dZ;
float dW;
float aX;
float aY;
float aZ;
float aW;
int stateNum;
double kp;
double ki;
double kd;
double kpx;
double kix;
double kdx;
double kpy;
double kiy;
double kdy;
double kpz;
double kiz;
double kdz;
double home_lat;
double home_lon;
geometry_msgs ::TwistStamped twist;

std::vector<sensor_msgs::NavSatFix>     globalWaypoints;
geometry_msgs::PoseArray                newLocalWaypoints;
geometry_msgs::PoseStamped              goalPose;
geometry_msgs::PoseStamped              idlePose;
geometry_msgs::PoseStamped              dropZonePose;
nav_msgs::Path                          path;
mavros_msgs::State UAVState;
int currentState = INITIATION;
geometry_msgs ::Point real;

void mavrosStateCallback(const mavros_msgs::State::ConstPtr& msg)
{
  UAVState = *msg;
}

void readWaypointsFromFile()
{
  if(homePoseFlag)
  {
    count = 0;
    double wpt_lat_ref,wpt_lon_ref;
    ros::param::param("~ref_lat", wpt_lat_ref, 47.3977419);
    ros::param::param("~ref_lon", wpt_lon_ref, 8.5455938);

    std::string str1 = ros::package::getPath("kuri_system_coordinator")+"/config/exploration_waypoints_50.txt";
    const char * filename1 = str1.c_str();

    assert(filename1 != NULL);
    filename1 = strdup(filename1);
    FILE *file1 = fopen(filename1, "r");
    if (!file1)
    {
      std::cout<<"\nCan not open File";
      fclose(file1);
    }
    double locationx,locationy,locationz,qy;
    geometry_msgs::PoseStamped pose;
    std::cout<<" The local waypoints map reference: "<<(double)wpt_lat_ref<<" "<<(double)wpt_lon_ref<<std::endl;
    double lat,lon;
    float alt;

    while (!feof(file1))
    {
      map_projection_global_init(wpt_lat_ref, wpt_lon_ref,1);
      fscanf(file1,"%lf %lf %lf %lf\n",&locationx,&locationy,&locationz,&qy);
      pose.pose.position.x    = locationx;
      pose.pose.position.y    = locationy;
      pose.pose.position.z    = locationz;
      pose.pose.orientation.x = qy;
      path.poses.push_back(pose);

      globallocalconverter_toglobal(locationy,locationx,locationz,&lat,&lon,&alt);
      globalPose.latitude = lat;
      globalPose.longitude = lon;
      globalPose.altitude = alt*-1;
      globalWaypoints.push_back(globalPose);
      //convert to local with repsect to the uav home position
      map_projection_global_init(home_lat, home_lon,1);
      printf(" uav home position: x %f y %f \n",home_lat,home_lon);
      float p_x,p_y,p_z;
      globallocalconverter_tolocal(globalPose.latitude,globalPose.longitude, -1*globalPose.altitude,&p_y,&p_x,&p_z);
      geometry_msgs::Pose pt;
      pt.position.x = p_x;
      pt.position.y = p_y;
      pt.position.z = p_z;
      printf(" new local pose: x %f y %f z %f \n",p_x,p_y,p_z);
      newLocalWaypoints.poses.push_back(pt);
    }
    fclose(file1);
    goalPose.pose.position.x = newLocalWaypoints.poses[count].position.x;
    goalPose.pose.position.y = newLocalWaypoints.poses[count].position.y;
    goalPose.pose.position.z = newLocalWaypoints.poses[count].position.z;
    std::cout<<"**************** transformation Done ***************** "<<std::endl;
    transformDoneFlag = true;
  }
}

void localPoseCb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  localPoseH.pose.position.x= msg->pose.position.x;
  localPoseH.pose.position.y= msg->pose.position.y;
  localPoseH.pose.position.z= msg->pose.position.z;

  if(currentState == EXPLORING)
  {
    errorX =  goalPose.pose.position.x - localPoseH.pose.position.x;
    errorY =  goalPose.pose.position.y - localPoseH.pose.position.y;
    errorZ =  goalPose.pose.position.z - localPoseH.pose.position.z;
    double dist = sqrt(errorX*errorX + errorY*errorY + errorZ*errorZ);
    //std::cout<<"WayPoint["<<count<<"] Dist:"<<dist<<" Goal Pose X:"<<goalPose.pose.position.x<<" y:"<<goalPose.pose.position.y<<" z:"<<goalPose.pose.position.z<< " uav local pose x: "<<localPoseH.pose.position.x<<" y: "<< localPoseH.pose.position.y<<" z: "<< localPoseH.pose.position.z<<std::endl;
    if(dist < tolerance)
    {
      count++;
      if(count<newLocalWaypoints.poses.size())
      {
        std::cout<<"New WayPoint["<<count<<"] Dist:"<<dist<<" Goal Pose X:"<<goalPose.pose.position.x<<" y:"<<goalPose.pose.position.y<<" z:"<<goalPose.pose.position.z<< " uav local pose x: "<<localPoseH.pose.position.x<<" y: "<< localPoseH.pose.position.y<<" z: "<< localPoseH.pose.position.z<<std::endl;
        //std::cout<<"new waypoints x: "<<newLocalWaypoints.poses[count].position.x<<" y: "<< newLocalWaypoints.poses[count].position.y<<" z: "<< newLocalWaypoints.poses[count].position.z<<std::endl;
        // ROS_INFO("UAV %i : Sending a New uav local WayPoint(x,y,z):(%g,%g,%g)",uav_id,newLocalWaypoints.poses[count].position.x,newLocalWaypoints.poses[count].position.y,newLocalWaypoints.poses[count].position.z);
        // ROS_INFO("UAV %i : Sending a New uav global WayPoint(x,y,z):(%g,%g,%g)",uav_id,globalWaypoints[count].latitude,globalWaypoints[count].longitude,globalWaypoints[count].altitude);
        goalPose.pose.position.x = newLocalWaypoints.poses[count].position.x;
        goalPose.pose.position.y = newLocalWaypoints.poses[count].position.y;
        goalPose.pose.position.z = newLocalWaypoints.poses[count].position.z;

      }
      if(count>=newLocalWaypoints.poses.size())
      {
        transformDoneFlag=false;
        idlePose.pose.position.x = newLocalWaypoints.poses[count-1].position.x;
        idlePose.pose.position.y = newLocalWaypoints.poses[count-1].position.y;
        idlePose.pose.position.z = newLocalWaypoints.poses[count-1].position.z;
        currentState = PICKING;
        //std::cout<<"uav_state: PICKING "<<currentState<<std::endl;
        ROS_INFO("uav_state: PICKING ");
        count=0;
        newLocalWaypoints.poses.erase(newLocalWaypoints.poses.begin(),newLocalWaypoints.poses.end());
        globalWaypoints.erase(globalWaypoints.begin(), globalWaypoints.end());
      }
    }
  }
}

void trackedObjectCb(const kuri_msgs::ObjectConstPtr &msg)
{
    objectLastTracked = ros::Time::now();
    //ROS_INFO("Got object %d in sight", msg->object_id);
    if(currentState == EXPLORING)
    {
      currentState = PICKING;
      ROS_INFO("TRANSITIONING FROM EXPLORING:>> PICKING STATE");
      twist.twist.linear.x = 0;
      twist.twist.linear.y = 0;
      twist.twist.linear.z = 0;
      twist.twist.angular.z = 0;
    }

    //ROS_INFO("Got object %d in sight", msg->object_id);

    if(currentState == WAITING_FOR_DROP)
    {
        if(localPoseH.pose.position.z < 4){
            twist.twist.linear.x = 0;
            twist.twist.linear.y = 0;
            twist.twist.linear.z = 1;
        }
    }

    if(currentState == PICKING)
    {
        ROS_INFO("PICKING object %d", msg->object_id);
    real.x= 0 ; //msg ->latitude;
    real.y= 0 ; //msg ->longitude;
    real.z= 0 ; //msg ->altitude;
    w = yaw ;

    errorX =  msg->pose.pose.position.x - real.x;
    errorY =  msg->pose.pose.position.y - real.y;
    errorZ =  msg->pose.pose.position.z - real.z ;
    errorW =  w;

    if (firstDataFlag )
    {
      prevErrorX = errorX;
      prevErrorY = errorY;
      prevErrorZ = errorZ;
      prevErrorW = errorW;
      firstDataFlag = false;
      twist.twist.linear.x = 0;
      twist.twist.linear.y = 0;
      twist.twist.linear.z = 0;
      twist.twist.angular.z = 0;
    }
    else
    {
      errorX =  msg->pose.pose.position.x;
      errorY =  msg->pose.pose.position.y;
      errorZ =  msg->pose.pose.position.z;

      float mul = 10.0, error = 0.05;//will be different for camera if calibrated

/*
      errorX *= mul;
      errorY *= mul;
      errorZ *= mul;
*/
      errorW =  0;
      pX = kpx * errorX;
      pY = kpy * errorY;
      pZ = kpz * errorZ;
      pW = kp * errorW;

      iX += kix * errorX;
      iY += kiy * errorY;
      iZ += kiz * errorZ;
      iW += ki * errorW;

      dX = kdx * (errorX - prevErrorX);
      dY = kdy * (errorY - prevErrorY);
      dZ = kdz * (errorZ - prevErrorZ);
      dW = kd * (errorW - prevErrorW);

      prevErrorX = errorX;
      prevErrorY = errorY;
      prevErrorZ = errorZ;
      prevErrorW = errorW;

      // PID conroller
      aX = pX     + iX + dX  ;
      aY = pY     + iY + dY  ;
      aZ = pZ      +iZ + dZ  ;
      aW = 10 * pW +iW + dW  ;

      if(fabs(errorX) > error || fabs(errorY) > error)
      {
            mul = 20;
                   aZ = 0;
      }

      if(localPoseH.pose.position.z > 4.0 && fabs(errorX) < error || fabs(errorY) < error){
            aZ = 0.2;
      }

      aX *= -mul;
      aY *= -mul;


      // filling velocity commands
      twist.twist.linear.x = aY;
      twist.twist.linear.y =  aX; //X and Y are flipped in the simulator, or camera rotated
      twist.twist.linear.z = aZ*-20;
      twist.twist.angular.z = aW;
      twist.header.stamp = ros::Time::now();





/*
      ROS_INFO("Error X: %0.2f \n", errorX);
      ROS_INFO("Error Y: %0.2f \n", errorY);
      ROS_INFO("Error Z: %0.2f \n", errorZ);
      //ROS_INFO("derivative X: %0.2f \n", dX);
      //ROS_INFO("derivative Y: %0.2f \n", dY);
      //ROS_INFO("derivative Z: %0.2f \n", dZ);
      //ROS_INFO("derivative W: %0.2f \n", dZ);
      //ROS_INFO("W: %0.2f \n", w);*/
      ROS_INFO("Action X: %0.2f \n", aX);
      ROS_INFO("Action Y: %0.2f \n", aY);
      ROS_INFO("Action Z: %0.2f \n", aZ);
      ROS_INFO("Action W: %0.2f \n", aW);
      double dist = sqrt(errorX*errorX+ errorY*errorY + errorZ*errorZ);
      if (dist <  tolerance2Goal)
      {
        twist.twist.linear.x = 0;
        twist.twist.linear.y = 0;
        twist.twist.linear.z = 0;
        twist.twist.angular.z = 0;
      }
      //ROS_INFO("localPoseH.pose.position.z W: %0.2f \n", localPoseH.pose.position.z);
      if(localPoseH.pose.position.z < 1.0)//assume object picked?
      {
          currentState = WAITING_FOR_DROP;
      }
    }
    }
}

void globalPoseCb(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  double lat,lon,alt;
  lat=msg->latitude;
  lon=msg->longitude;
  alt=msg->altitude;
  if(!homePoseFlag)
  {
    std::cout<<"Setting GPS home pose\n";
    if(lat != 0 && lon != 0)
    {
      std::cout<<"  - GPS home pose set\n";
      home_lat = lat;
      home_lon = lon;
      homePoseFlag=true;
    }
  }
  if(homePoseFlag)
  {
    geometry_msgs::Point pt;
    pt.x = home_lat;
    pt.y = home_lon;
  }
}

void headingCallback(const std_msgs::Float64::ConstPtr& msg)
{
  yaw = msg->data * 3.14159265359 / 180.0 ;
}

int main(int argc , char **argv)
{
  ros::init(argc , argv , "playing_solo");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate( "~" );
  ros::Rate loopRate(20);
  int UAVId = 2;
  std::cout<<"starting"<<std::endl;
  std::cout<<"UAVId: "<<UAVId<<std::endl;

  //publishers and subscribers
  ros::Subscriber statePub = nh.subscribe<mavros_msgs::State>
      ("/uav_"+boost::lexical_cast<std::string>(UAVId)+"/mavros/state", 10, mavrosStateCallback);
  ros::Publisher localPosePub = nh.advertise<geometry_msgs::PoseStamped>
      ("/uav_"+boost::lexical_cast<std::string>(UAVId)+"/mavros/setpoint_position/local", 10);
  ros::ServiceClient armingClient = nh.serviceClient<mavros_msgs::CommandBool>
      ("/uav_"+boost::lexical_cast<std::string>(UAVId)+"/mavros/cmd/arming");
  ros::ServiceClient setModeClient = nh.serviceClient<mavros_msgs::SetMode>
      ("/uav_"+boost::lexical_cast<std::string>(UAVId)+"/mavros/set_mode");
  ros::Subscriber currentPose = nh.subscribe
      ("/uav_"+boost::lexical_cast<std::string>(UAVId)+"/mavros/local_position/pose", 10, localPoseCb);
  ros::Subscriber globalPoseSub = nh.subscribe
      ("/uav_"+boost::lexical_cast<std::string>(UAVId)+"/mavros/global_position/global", 10, globalPoseCb);
  ros::Subscriber trackedObjSub = nh.subscribe
      ("/uav_"+boost::lexical_cast<std::string>(UAVId)+"/tracked_object/object", 10, trackedObjectCb);
  //ros::Subscriber globalPoseSub;
  ros::Publisher velPub = nh.advertise<geometry_msgs ::TwistStamped>
      ("/uav_"+boost::lexical_cast<std::string>(UAVId)+"/mavros/setpoint_velocity/cmd_vel", 10);
  ros::Subscriber compassSub = nh.subscribe
      ("/uav_"+boost::lexical_cast<std::string>(UAVId)+"/mavros/global_position/compass_hdg", 10, headingCallback);

  nh.param("kp", kp, 0.05);
  nh.param("ki", ki, 0.0);
  nh.param("kd", kd, 0.05);
  nh.param("kpx", kpx, 0.05);
  nh.param("kix", kix, 0.0);
  nh.param("kdx", kdx, 0.05);
  nh.param("kpy", kpy, 0.05);
  nh.param("kiy", kiy, 0.0);
  nh.param("kdy", kdy, 0.05);
  nh.param("kp", kpz, 0.05);
  nh.param("ki", kiz, 0.0);
  nh.param("kd", kdz, 0.05);
  nh.param("tolerance_2_goal", tolerance2Goal, 0.2);

  goalPose.pose.position.x = 0;
  goalPose.pose.position.y = 0;
  goalPose.pose.position.z = 0;
  geometry_msgs ::TwistStamped stopTwist;
  stopTwist.twist.linear.x  = 0;
  stopTwist.twist.linear.y  = 0;
  stopTwist.twist.linear.z  = 0;
  stopTwist.twist.angular.z = 0;

  ros::ServiceClient objectsTrackingServiceClient = nh.serviceClient<kuri_object_tracking::Object2Track>("trackObjectService");
  kuri_object_tracking::Object2Track objSRV;

  bool objectTrackingInitiated = false;
  // wait for FCU connection
  while(ros::ok() && UAVState.connected)
  {
    ros::spinOnce();
    loopRate.sleep();
  }

  idlePose.pose.position.x = 0;
  idlePose.pose.position.y = 0;
  idlePose.pose.position.z = 7;

  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";

  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  //TODO:
  // - Check FCU are correct
  // - Perform System Checks
  // - Offboard launch
  // - Follow the waypoints
  // - Find objects (object tracker)
  // - Once found Pick
  // - Hover at X altitude
  // - Ask permission to drop
  // -   if granted go, otherwise, wait
  // - Go again and continue

  ros::Time lastRequest     = ros::Time::now();
  ros::Time statusUpdate    = ros::Time::now();
  ros::Time waitingForOFF   = ros::Time::now();

  while(ros::ok())
  {
    switch(currentState)
    {
    case INITIATION:
      /* This code overrides the RC mode and is dangerous when performing tests: re-use in real flight tests*/
      if( UAVState.mode != "OFFBOARD" && (ros::Time::now() - lastRequest > ros::Duration(1.0)))
      {
        if( setModeClient.call(offb_set_mode) && offb_set_mode.response.success)
        {
          ROS_INFO("Offboard enabled");
        }
        else
        {
          ROS_ERROR("Failed Enabling offboard");
        }
        lastRequest = ros::Time::now();
      }
      if( UAVState.mode != "OFFBOARD")
      {
        if(ros::Time::now() - waitingForOFF > ros::Duration(0.5))
        {
          std::cout<<"USE RC to set the system in offboard mode => Current Mode is: "<<UAVState.mode<<"\n"; fflush(stdout);
          waitingForOFF = ros::Time::now();
        }
      }
      else
      {
        if( !UAVState.armed && (ros::Time::now() - lastRequest > ros::Duration(1.0)))
        {
          if( armingClient.call(arm_cmd) && arm_cmd.response.success)
          {
            ROS_INFO("ARMING Command send through mavros, check messages related to safety switch");
            currentState = EXPLORING;
            ROS_INFO("TRANSITIONING FROM INITIATION STATE:>> EXPLORING STATE");
            readWaypointsFromFile();
          }
          else
          {
            ROS_INFO("Sending Arming message FAILED!");
          }
          lastRequest = ros::Time::now();
        }
      }
      localPosePub.publish(idlePose);
      break;
    case EXPLORING:
      //once the transform to local in terms of the home position is done
      if(transformDoneFlag)
      {
        if(!objectTrackingInitiated)
        {
          /*
          goal.uav_id = 2;
          objectsTrackingClient.sendGoal(goal,&objectTrackingDoneCallBack, &objectTrackingActiveCallback, &objectTrackingFeedbackCallback);
          */
          objSRV.request.color = "all";
          if(objectsTrackingServiceClient.call(objSRV))
          {
            ROS_INFO("Object Tracking: Call successfull");
          }
          else
          {
            ROS_ERROR("Object Tracking: Failed to call service");
          }
          objectTrackingInitiated = true;
        }
        localPosePub.publish(goalPose);
      }
      else
      {
        localPosePub.publish(idlePose);
      }
    case PICKING:
      // Failsafe: if we don't get tracking info for more than 500ms, then stop in place, or go back to exploring?
      if(ros::Time::now() - objectLastTracked > ros::Duration(0.5))
      {
        stopTwist.header.stamp = ros::Time::now();
        velPub.publish(stopTwist);
        ROS_INFO("TRANSITIONING FROM INITIATION PICKING:>> EXPLORING STATE");
        currentState = EXPLORING;
      }
      else
      {
        velPub.publish(twist);
      }
      break;
    case WAITING_FOR_DROP:
        //hold position?
    case DROPPING://Insert dropzone GPS location, or nearby, and can use camera to detect drop zone.
        dropZonePose.pose.position.x = -19.0;
        dropZonePose.pose.position.y = 0.0;
        dropZonePose.pose.position.z = 7;
        localPosePub.publish(dropZonePose);
    default:
      ROS_ERROR("Unknown State");
    }
    if(ros::Time::now() - statusUpdate > ros::Duration(1.0))
    {
      std::cout<<"Current Mode is: "<<UAVState.mode<<"\n"; fflush(stdout);
      statusUpdate = ros::Time::now();
    }
    ros::spinOnce();
    loopRate.sleep();
  }

}

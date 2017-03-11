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

int count        = 0 ;
double tolerance = 0.3;
double errorX    = 0;
double errorY    = 0;
double errorZ    = 0;


bool homePoseFlag       = false;
bool transformDoneFlag  = false;
std_msgs::Bool finishedFlag;

int stateNum;

double home_lat;
double home_lon;

std::vector<sensor_msgs::NavSatFix>     globalWaypoints;
geometry_msgs::PoseArray                newLocalWaypoints;
geometry_msgs::PoseStamped              goalPose;
geometry_msgs::PoseStamped              idlePose;
nav_msgs::Path                          path;
mavros_msgs::State UAVState;
int currentState = INITIATION;

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
        std::cout<<"uav_state: PICKING "<<currentState<<std::endl;
        count=0;
        newLocalWaypoints.poses.erase(newLocalWaypoints.poses.begin(),newLocalWaypoints.poses.end());
        globalWaypoints.erase(globalWaypoints.begin(), globalWaypoints.end());
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

// Called once when the goal completes
void objectTrackingDoneCallBack(const actionlib::SimpleClientGoalState& state, const kuri_msgs::TrackingResultConstPtr& result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
  ROS_INFO("Answer: %lu", result->tracked_objects.objects.size());
}

// Called once when the goal becomes active
void objectTrackingActiveCallback()
{
  ROS_INFO("Goal just went active");
}

// Called every time feedback is received for the goal
void objectTrackingFeedbackCallback(const kuri_msgs::TrackingFeedbackConstPtr& feedback)
{
  ROS_INFO("Got Feedback of length %lu", feedback->new_objects.objects.size());
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
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
      ("/uav_"+boost::lexical_cast<std::string>(UAVId)+"/mavros/state", 10, mavrosStateCallback);
  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
      ("/uav_"+boost::lexical_cast<std::string>(UAVId)+"/mavros/setpoint_position/local", 10);
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
      ("/uav_"+boost::lexical_cast<std::string>(UAVId)+"/mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
      ("/uav_"+boost::lexical_cast<std::string>(UAVId)+"/mavros/set_mode");
  ros::Subscriber current_pose = nh.subscribe
      ("/uav_"+boost::lexical_cast<std::string>(UAVId)+"/mavros/local_position/pose", 1000, localPoseCb);
  ros::Subscriber global_pose = nh.subscribe
      ("/uav_"+boost::lexical_cast<std::string>(UAVId)+"/mavros/global_position/global", 1000, globalPoseCb);


  // Action Clients
  ros::ServiceClient objectsTrackingServiceClient = nh.serviceClient<kuri_object_tracking::Object2Track>("trackObjectService");
  /*
  typedef actionlib::SimpleActionClient<kuri_msgs::TrackingAction> ObjectsTrackingClient;
  ObjectsTrackingClient objectsTrackingClient("TrackingAction", true);
  ROS_INFO("Waiting for Object Tracking Action Server to start.");
  objectsTrackingClient.waitForServer();
  ROS_INFO("Action server started, sending goal.");
  kuri_msgs::TrackingGoal goal;
  */
  bool objectTrackingInitiated = false;
  // wait for FCU connection
  while(ros::ok() && UAVState.connected)
  {
    ros::spinOnce();
    loopRate.sleep();
  }

  idlePose.pose.position.x = 0;
  idlePose.pose.position.y = 0;
  idlePose.pose.position.z = 10;

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
      if( UAVState.mode != "OFFBOARD" && (ros::Time::now() - lastRequest > ros::Duration(5.0)))
      {
        if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.success)
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
        if( !UAVState.armed && (ros::Time::now() - lastRequest > ros::Duration(5.0)))
        {
          if( arming_client.call(arm_cmd) && arm_cmd.response.success)
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
      local_pos_pub.publish(idlePose);
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
          kuri_object_tracking::Object2Track objSRV;
          objSRV.request.color = "all";
          if(objectsTrackingServiceClient.call(objSRV))
          {
            ROS_INFO("Call successfull");
          }
          else
          {
            ROS_ERROR("Failed to call service");
          }
          objectTrackingInitiated = true;
        }
        local_pos_pub.publish(goalPose);
      }
      else
      {
        local_pos_pub.publish(idlePose);
      }
      break;
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

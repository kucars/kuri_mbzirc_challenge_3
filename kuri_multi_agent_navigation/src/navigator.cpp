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

#include "sspp/pathplanner.h"
#include "ros/ros.h"
#include <ros/package.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <geometry_msgs/Pose.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include "sspp/distance_heuristic.h"
#include "sspp/rviz_drawing_tools.h"
#include "rviz_visual_tools/rviz_visual_tools.h"
#include "geometry_msgs/PoseStamped.h"

#include <kuri_msgs/Tasks.h>
#include <kuri_msgs/NavTask.h>
#include <kuri_msgs/NavTasks.h>
#include <kuri_msgs/FollowPathAction.h>
#include <std_msgs/Float32.h>
#include <actionlib/server/simple_action_server.h>
#include <nav_msgs/Path.h>
#include <navigator.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/NavSatFix.h>
#include <geo.h>
using namespace SSPP;



Navigator::Navigator(int uav_id)
{
    currentPoseSub = nh.subscribe("/uav_"+boost::lexical_cast<std::string>(uav_id)+"/mavros/local_position/pose", 1000, &Navigator::localPoseCallback, this);
    currentGlobalPoseSub = nh.subscribe("/uav_"+boost::lexical_cast<std::string>(uav_id)+"/mavros/global_position/global", 1000, &Navigator::globalPoseCallback, this);
    flagPub = nh.advertise<geometry_msgs::Point> ("/uav_"+boost::lexical_cast<std::string>(uav_id)+"/flag", 10);
    homeGlobalPub = nh.advertise<geometry_msgs::Point> ("/uav_"+boost::lexical_cast<std::string>(uav_id)+"/global_home_position", 10);
    // posePub        = nh.advertise<geometry_msgs::PoseStamped>("/uav_"+boost::lexical_cast<std::string>(uav_id)+"/waypoint", 10);
    stateSub = nh.subscribe<mavros_msgs::State> ("/uav_"+boost::lexical_cast<std::string>(uav_id)+"/mavros/state", 10, &Navigator::stateCallback, this);
    posePub = nh.advertise<geometry_msgs::PoseStamped> ("/uav_"+boost::lexical_cast<std::string>(uav_id)+"/mavros/setpoint_position/local", 10);
    armingClient = nh.serviceClient<mavros_msgs::CommandBool> ("/uav_"+boost::lexical_cast<std::string>(uav_id)+"/mavros/cmd/arming");
    setModeClient = nh.serviceClient<mavros_msgs::SetMode> ("/uav_"+boost::lexical_cast<std::string>(uav_id)+"/mavros/set_mode");

    visualTools.reset(new rviz_visual_tools::RvizVisualTools("map","/path_following_visualisation"));
    visualTools->deleteAllMarkers();
    visualTools->setLifetime(0.2);

    // Make these adjustable
    dist      = 0;
    count     = 0 ;
    tolerance = 0.3;
    errorX    = 0;
    errorY    = 0;
    errorZ    = 0;
    uav_i     = uav_id;
    //flag to indicate that the path following finished or not
    flag.x    = 0;
    flag.y    = 0;
    flag.z    = 0;

    homePoseFlag = false;
    transformFlag = false;
}


void Navigator::stateCallback(const mavros_msgs::State::ConstPtr& msg){
    currentState = *msg;
}

void Navigator::localPoseCallback(const geometry_msgs :: PoseStamped :: ConstPtr& msg)
{
    real.x=msg ->pose.position.x;
    real.y=msg ->pose.position.y;
    real.z=msg ->pose.position.z;

    errorX =  goalPose.pose.position.x - real.x;
    errorY =  goalPose.pose.position.y - real.y;
    errorZ =  goalPose.pose.position.z - real.z;

    if ((fabs(errorX) < tolerance) && (fabs(errorY) < tolerance) && (fabs(errorZ) < tolerance))
    {
        count++;
        if(count<newLocalWaypoints.poses.size())
        {
            ROS_INFO("UAV %i : Sending a New main local WayPoint(x,y,z):(%g,%g,%g)",uav_i,waypoints.poses[count].position.x,waypoints.poses[count].position.y,waypoints.poses[count].position.z);
            ROS_INFO("UAV %i : Sending a New uav local WayPoint(x,y,z):(%g,%g,%g)",uav_i,newLocalWaypoints.poses[count].position.x,newLocalWaypoints.poses[count].position.y,newLocalWaypoints.poses[count].position.z);
            ROS_INFO("UAV %i : Sending a New uav global WayPoint(x,y,z):(%g,%g,%g)",uav_i,globalWaypoints.poses[count].position.x,globalWaypoints.poses[count].position.y,globalWaypoints.poses[count].position.z);

            goalPose.pose.position.x = newLocalWaypoints.poses[count].position.x;
            goalPose.pose.position.y = newLocalWaypoints.poses[count].position.y;
            goalPose.pose.position.z = newLocalWaypoints.poses[count].position.z;

            if(count != waypoints.poses.size()-1)
            {
                linePoint.x = waypoints.poses[count].position.x;
                linePoint.y = waypoints.poses[count].position.y;
                linePoint.z = waypoints.poses[count].position.z;
                pathSegments.push_back(linePoint);

                linePoint.x = waypoints.poses[count+1].position.x;
                linePoint.y = waypoints.poses[count+1].position.y;
                linePoint.z = waypoints.poses[count+1].position.z;
                pathSegments.push_back(linePoint);

                visualTools->publishArrow(waypoints.poses[count],rviz_visual_tools::YELLOW, rviz_visual_tools::LARGE,0.3);
                visualTools->publishPath(pathSegments, rviz_visual_tools::RED, rviz_visual_tools::LARGE,"path_trail");

                dist = dist+ Dist(waypoints.poses[count],waypoints.poses[count+1]);
            }


        }

    }

}


void Navigator::globalPoseCallback(const sensor_msgs::NavSatFix:: ConstPtr& msg)
{
    double lat,lon,alt;
    lat=msg->latitude;
    lon=msg->longitude;
    alt=msg->altitude;

    if(!homePoseFlag)
    {
        if(lat != 0 && lon != 0)
        {
            lat_ref = lat;
            lon_ref = lon;
            homePoseFlag=true;
        }

    }

    if(homePoseFlag)
    {
        geometry_msgs::Point pt;
        pt.x = lat_ref;
        pt.y = lon_ref;
        homeGlobalPub.publish(pt);
    }

}


void Navigator::navigate(actionlib::SimpleActionServer<kuri_msgs::FollowPathAction> *actionServer,
                         kuri_msgs::FollowPathFeedback feedback,
                         kuri_msgs::FollowPathResult   result,
                         nav_msgs::Path path,
                         nav_msgs::Path pathTrail)
{
    ros::Rate loopRate(20);
    count = 0;
    int wayPointIndex = 0;

    //TODO::we should return something
    if(path.poses.size()<1)
    {
        ROS_INFO("%s: Aborted", actionName.c_str());
        result.success = false;
        actionServer->setAborted(result);
        return;
    }

    double wpt_lat_ref,wpt_lon_ref;
    ros::param::param("~ref_lat", wpt_lat_ref, 47.3977419);
    ros::param::param("~ref_lon", wpt_lon_ref, 8.5455938);

    std::cout<<" The local waypoints map reference: "<<(double)wpt_lat_ref<<" "<<(double)wpt_lon_ref<<std::endl;
    map_projection_global_init(wpt_lat_ref, wpt_lon_ref,1);

    for(int i=0;i<=(path.poses.size()-1);i++)
    {
        waypoints.poses.push_back(path.poses[i].pose);

        double lat;
        double lon;
        float alt;

        globallocalconverter_toglobal(path.poses[i].pose.position.y,path.poses[i].pose.position.x,path.poses[i].pose.position.z,&lat,&lon,&alt);
        globalPose.position.x = lat;
        globalPose.position.y = lon;
        globalPose.position.z = alt*-1;
        globalWaypoints.poses.push_back(globalPose);
    }



    offbSetMode.request.custom_mode = "OFFBOARD";
    armCmd.request.value = true;

    ros::Time lastRequest = ros::Time::now();
    ros::Time statusUpdate = ros::Time::now();

    while(ros::ok())
    {
        //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
        //PROBLEM:
        // using offboard mode and arming the drone requires publishing a position, once we stop
        // publishing a position the drone disarms and failsafe mode executes (doing landing)!!!
        // which means once the path is followed successfully, we stop publishing positions
        //so the drone lands (disarms), how to keep it armed ??!!!!!!!!

        // TEMP SOLUTION:
        //used another node (uavs_initiator.cpp) instead of the below commented lines to keep node working
        // and I used a flag that gets set and published when the path following succeed
        //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

        //        if( currentState.mode != "OFFBOARD" && (ros::Time::now() - lastRequest > ros::Duration(5.0)))
        //        {
        //            if( setModeClient.call(offbSetMode) && offbSetMode.response.success)
        //            {
        //                ROS_INFO("Offboard enabled");
        //            }
        //            lastRequest = ros::Time::now();
        //        }
        //        else
        //        {
        //            if( !currentState.armed && (ros::Time::now() - lastRequest > ros::Duration(5.0)))
        //            {
        //                if( armingClient.call(armCmd) && armCmd.response.success)
        //                {
        //                    ROS_INFO("Vehicle armed");
        //                }
        //                lastRequest = ros::Time::now();
        //            }
        //            if(ros::Time::now() - statusUpdate > ros::Duration(1.0))
        //            {
        //                std::cout<<"Current Mode is: "<<currentState.mode<<"\n"; fflush(stdout);
        //                statusUpdate = ros::Time::now();
        //            }
        //        }


        // check that preempt has not been requested by the client
        if (actionServer->isPreemptRequested() || !ros::ok())
        {
            ROS_INFO("%s: Preempted", actionName.c_str());
            actionServer->setPreempted();
            result.success = false;
            break;
        }
        //std::cout<<"uav "<<uav_i<<" count : "<<count<<" path: "<<path.poses.size()-1<<std::endl;
        if(count>(path.poses.size()-1))
        {
            ROS_INFO("%s: Succeeded", actionName.c_str());
            result.success = true;
            actionServer->setSucceeded(result);
            std::cout<<"path Trail points size: "<<pathTrail.poses.size()<<std::endl;
            std::cout<<"path points size: "<<path.poses.size()<<std::endl;
            flag.x = goalPose.pose.position.x;
            flag.y = goalPose.pose.position.y;
            flag.z = goalPose.pose.position.z;

            //Publish the flag 10 times just to make sure the other node listens to the messages
            for(int i = 0 ; i<10; i++)
                flagPub.publish(flag);
            //Problem : FCU Failsaif mode turns on when the code  break
            transformFlag = false;
            break;
        }

        //should be done once to transform the global waypoints to local in terms of the home position
        if(homePoseFlag && !transformFlag)
        {
            map_projection_global_init(lat_ref, lon_ref,1);
            for(int j = 0 ; j<=globalWaypoints.poses.size()-1;j++)
            {
                float p_x,p_y,p_z;
                globallocalconverter_tolocal(globalWaypoints.poses[j].position.x,globalWaypoints.poses[j].position.y, -1*globalWaypoints.poses[j].position.z,&p_y,&p_x,&p_z);
                geometry_msgs::Pose p;
                p.position.x = p_x;
                p.position.y = p_y;
                p.position.z = p_z;

                newLocalWaypoints.poses.push_back(p);
            }

            goalPose.pose.position.x = newLocalWaypoints.poses[count].position.x;
            goalPose.pose.position.y = newLocalWaypoints.poses[count].position.y;
            goalPose.pose.position.z = newLocalWaypoints.poses[count].position.z;
            flagOnce=true;
            transformFlag = true;
        }

        if(transformFlag)
        {
            /*if(pathTrail.poses.size()!=0 && (pathTrail.poses[wayPointIndex-1].pose.position.x != goalPose.pose.position.x ||
                                             pathTrail.poses[wayPointIndex-1].pose.position.y != goalPose.pose.position.y ||
                                             pathTrail.poses[wayPointIndex-1].pose.position.z != goalPose.pose.position.z) )
            {
                pathTrail.poses.push_back(path.poses[wayPointIndex++]);
            }
            else if(pathTrail.poses.size()==0)
            {
                pathTrail.poses.push_back(path.poses[wayPointIndex++]);
            }*/

            posePub.publish(goalPose);
            if(flagOnce)
            {
                flag.x = goalPose.pose.position.x;
                flag.y = goalPose.pose.position.y;
                flag.z = goalPose.pose.position.z;
                flagOnce=false;
            }else
            {
                flag.x=0;
                flag.y=0;
                flag.z=0;
            }

            flagPub.publish(flag);
            ROS_INFO("UAV %i :Published Pose x:%f y:%f z:%f",uav_i,path.poses[count].pose.position.x,path.poses[count].pose.position.y,path.poses[count].pose.position.z);
        }
        ros::spinOnce();
        loopRate.sleep();
    }


}

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
using namespace SSPP;

class Navigation_action_server
{
public:

    Navigation_action_server(std::string name) :
        actionServer(nh, name, false),
        actionName(name)
    {
        //register the goal and feeback callbacks
        actionServer.registerGoalCallback(boost::bind(&Navigation_action_server::goalCB, this));
        actionServer.registerPreemptCallback(boost::bind(&Navigation_action_server::preemptCB, this));
         ROS_INFO("set subscriber");
        currentPoseSub = nh.subscribe("/uav_1/mavros/local_position/pose", 1, &Navigation_action_server::startPositionCallback, this);
        posePub        = nh.advertise<geometry_msgs::PoseStamped>("/uav_1/mavros/setpoint_position/local", 10);
        visualTools.reset(new rviz_visual_tools::RvizVisualTools("map","/path_following_visualisation"));
        visualTools->deleteAllMarkers();
        visualTools->setLifetime(0.2);
        actionServer.start();
        ROS_INFO("Action Server Ready for Orders: %s", actionName.c_str());
        // Make these adjustable
        dist=0;
        threshold2Dist=0.5;
        uav1_navigator = new Navigator(1);
        uav2_navigator = new Navigator(2);
        uav3_navigator = new Navigator(3);
        // uav1_navigator->initiator();
        // uav2_navigator->initiator();
        // uav3_navigator->initiator();
        // std::cout<<"************************* Three drones are armed now ****************************"<<std::endl;


    }

    ~Navigation_action_server(void)
    {
    }

    void startPositionCallback(const geometry_msgs::PoseStamped& msg)
    {
        currentPose = msg.pose;
    }

    void goalCB()
    {
        ROS_INFO("Recieved a Goal");
        progressCount = 0;
        // accept the new goal
        goal = actionServer.acceptNewGoal()->navigation_task;
        ROS_INFO("Following for uav_%d",goal.task.uav_id);
        path = goal.path;
        if(goal.task.uav_id==1){
//        uav1_navigator->initiator();
        uav1_navigator->navigate(&actionServer,
                          feedback,
                           result,
                           path,
                           pathTrail);
        }else if(goal.task.uav_id==2){
//            uav2_navigator->initiator();
            uav2_navigator->navigate(&actionServer,
                              feedback,
                               result,
                               path,
                               pathTrail);
        }else if(goal.task.uav_id==3){
//            uav3_navigator->initiator();
            uav3_navigator->navigate(&actionServer,
                              feedback,
                               result,
                               path,
                               pathTrail);
        }else{
            ROS_INFO("uav_id is incorrect");
        }
    }

    void preemptCB()
    {
        ROS_INFO("%s: Preempted", actionName.c_str());
        // set the action state to preempted
        actionServer.setPreempted();
    }

//    void navigate()
//    {
//        ros::Rate loopRate(1);
//        //TODO::we should return something
//        if(path.poses.size()<1)
//        {
//            ROS_INFO("%s: Aborted", actionName.c_str());
//            result.success = false;
//            actionServer.setAborted(result);
//            return;
//        }
//        int wayPointIndex = 0;
//        pathTrail.poses.push_back(path.poses[wayPointIndex]);
//        while(ros::ok())
//        {
//            ros::spinOnce();
//            ROS_INFO("Current Pose x:%f y:%f z:%f",currentPose.position.x,currentPose.position.y,currentPose.position.z);
//            // check that preempt has not been requested by the client
//            if (actionServer.isPreemptRequested() || !ros::ok())
//            {
//              ROS_INFO("%s: Preempted", actionName.c_str());
//              actionServer.setPreempted();
//              result.success = false;
//              break;
//            }
//            for(int i=0;i<(pathTrail.poses.size()-1);i++)
//            {
//                linePoint.x = pathTrail.poses[i].pose.position.x;
//                linePoint.y = pathTrail.poses[i].pose.position.y;
//                linePoint.z = pathTrail.poses[i].pose.position.z;
//                robotPose.poses.push_back(pathTrail.poses[i].pose);
//                pathSegments.push_back(linePoint);

//                linePoint.x = pathTrail.poses[i+1].pose.position.x;
//                linePoint.y = pathTrail.poses[i+1].pose.position.y;
//                linePoint.z = pathTrail.poses[i+1].pose.position.z;
//                robotPose.poses.push_back(pathTrail.poses[i+1].pose);
//                pathSegments.push_back(linePoint);

//                dist = dist+ Dist(pathTrail.poses[i].pose,pathTrail.poses[i+1].pose);
//            }
//            // Redraw
//            visualTools->resetMarkerCounts();
//            for(int i=0;i<robotPose.poses.size();i++)
//            {
//                visualTools->publishArrow(robotPose.poses[i],rviz_visual_tools::YELLOW, rviz_visual_tools::LARGE,0.3);
//            }
//            if(pathSegments.size()>0)
//                visualTools->publishPath(pathSegments, rviz_visual_tools::RED, rviz_visual_tools::LARGE,"path_trail");

//            if(Dist(path.poses[wayPointIndex].pose,currentPose) < threshold2Dist)
//            {
//                // Last waypoint reached ?
//                if(++wayPointIndex>(path.poses.size()-1))
//                {
//                    ROS_INFO("%s: Succeeded", actionName.c_str());
//                    result.success = true;
//                    actionServer.setSucceeded(result);
//                    break;
//                }
//                ROS_INFO("Sending a New WayPoint(x,y,z):(%g,%g,%g)",path.poses[wayPointIndex].pose.position.x,path.poses[wayPointIndex].pose.position.y,path.poses[wayPointIndex].pose.position.z);
//                pathTrail.poses.push_back(path.poses[wayPointIndex]);
//            }
//            posePub.publish(path.poses[wayPointIndex]) ;
//            ros::spinOnce();
//            loopRate.sleep();
//        }
//    }

protected:

    ros::NodeHandle nh;
    actionlib::SimpleActionServer<kuri_msgs::FollowPathAction> actionServer;
    ros::Publisher  posePub;
    ros::Subscriber currentPoseSub;
    std::string actionName;
    float progressCount;
    kuri_msgs::NavTask goal;
    kuri_msgs::FollowPathFeedback feedback;
    kuri_msgs::FollowPathResult   result;
    kuri_msgs::NavTasks navTasks;
    ros::Subscriber sub;
    geometry_msgs::Pose currentPose;
    nav_msgs::Path path,pathTrail;
    std::vector<geometry_msgs::Point> pathSegments;
    geometry_msgs::PoseArray robotPose;
    geometry_msgs::Point linePoint;
    double dist,threshold2Dist;
    rviz_visual_tools::RvizVisualToolsPtr visualTools;
    Navigator* uav1_navigator;
    Navigator* uav2_navigator;
    Navigator* uav3_navigator;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "navigation_action_server");
    Navigation_action_server navigation_action_server(ros::this_node::getName());
    ros::spin();
    return 0;
}

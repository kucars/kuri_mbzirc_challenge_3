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
#ifndef NAVIGATOR_H_
#define NAVIGATOR_H_

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
//#include <mavros/mavros.h>
#include "geometry_msgs/PoseStamped.h"

#include <kuri_msgs/Tasks.h>
#include <kuri_msgs/NavTask.h>
#include <kuri_msgs/NavTasks.h>

#include <actionlib/server/simple_action_server.h>
#include <kuri_msgs/GeneratePathsAction.h>
#include <kuri_msgs/PickObjectAction.h>
#include <kuri_msgs/DropObjectAction.h>
#include "nav_msgs/Path.h"
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geo.h>
#include <sensor_msgs/NavSatFix.h>

namespace SSPP
{

class Navigator
{
public:
    ros::NodeHandle nh;
    ros::Publisher  posePub;
    ros::Publisher  flagPub;
    ros::Publisher homeGlobalPub;
    ros::Subscriber currentPoseSub;
    ros::Subscriber currentGlobalPoseSub;
    ros::Subscriber stateSub;
    ros::ServiceClient armingClient;
    ros::ServiceClient setModeClient;

    mavros_msgs::SetMode offbSetMode;
    mavros_msgs::CommandBool armCmd;
    mavros_msgs::State currentState;

    std::string actionName;
    float progressCount;
    kuri_msgs::NavTask goal;
    kuri_msgs::NavTasks navTasks;
    geometry_msgs::Pose currentPose;

    std::vector<geometry_msgs::Point> pathSegments;
    geometry_msgs::PoseArray robotPose;
    geometry_msgs::Point linePoint;
    double dist;
    rviz_visual_tools::RvizVisualToolsPtr visualTools;

    geometry_msgs::Pose endPose;
    geometry_msgs::Pose target_coord;
    kuri_msgs::Tasks tasks;

    geometry_msgs::Point        real;
    geometry_msgs::Pose         globalPose;
    geometry_msgs::PoseArray    waypoints;
    geometry_msgs::PoseArray    globalWaypoints;
    geometry_msgs::PoseArray    newLocalWaypoints;
    geometry_msgs::PoseStamped  goalPose;

    int count;
    float tolerance;
    float errorX;
    float errorY;
    float errorZ;
    int uav_i;
    double lat_ref;
    double lon_ref;
    bool homePoseFlag;
    bool transformFlag;
    bool flagOnce;
    geometry_msgs::Point flag;

    void navTasksCallback(const kuri_msgs::Tasks newtasks);
    void stateCallback(const mavros_msgs::State::ConstPtr& msg);
    void localPoseCallback(const geometry_msgs :: PoseStamped :: ConstPtr& msg);
    void globalPoseCallback(const sensor_msgs::NavSatFix:: ConstPtr& msg);
    //nav_msgs::Path navigate(const kuri_msgs::Tasks newtasks);
    Navigator(int uav_id);
    void navigate(actionlib::SimpleActionServer<kuri_msgs::FollowPathAction> *actionServer,
                  kuri_msgs::FollowPathFeedback feedback,
                  kuri_msgs::FollowPathResult   result,
                  nav_msgs::Path path,
                  nav_msgs::Path pathTrail);
    //~Navigator(void);

};

}

#endif /*NAVIGATOR_H_*/

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

#include "pathplanner.h"
#include "ros/ros.h"
#include <ros/package.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <geometry_msgs/Pose.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include "distance_heuristic.h"
#include "rviz_drawing_tools.h"
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


namespace SSPP
{

class Navigator
{
public:
    ros::NodeHandle nh;
    ros::Publisher  posePub;
    ros::Subscriber currentPoseSub;
    std::string actionName;
    float progressCount;
    kuri_msgs::NavTask goal;
    kuri_msgs::NavTasks navTasks;
    ros::Subscriber sub;
    geometry_msgs::Pose currentPose;

    std::vector<geometry_msgs::Point> pathSegments;
    geometry_msgs::PoseArray robotPose;
    geometry_msgs::Point linePoint;
    double dist,threshold2Dist;
    rviz_visual_tools::RvizVisualToolsPtr visualTools;

	geometry_msgs::Pose endPose;
	geometry_msgs::Pose target_coord;
	kuri_msgs::Tasks tasks;
    void startPositionCallback(const geometry_msgs::PoseStamped& msg);
    void navTasksCallback(const kuri_msgs::Tasks newtasks);
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

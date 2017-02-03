/***************************************************************************
 *   Copyright (C) 2006 - 2016 by                                          *
 *      Tarek Taha, KURI  <tataha@tarektaha.com>                           *
 *      Randa Almadhoun, KURI  <randa.almadhoun@gmail.com>                 *
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

#ifndef PATH_GENERATOR_H
#define PATH_GENERATOR_H

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

#include <component_test/occlusion_culling_gpu.h>
#include <component_test/occlusion_culling.h>
#include "sspp/coverage_path_planning_heuristic.h"
#include "sspp/rviz_drawing_tools.h"

class ExpWaypointsGenerator
{
public:
    geometry_msgs::Pose uav1_currentPose;
    geometry_msgs::Pose uav2_currentPose;
    geometry_msgs::Pose uav3_currentPose;
    ros::NodeHandle nh;

    ros::Publisher sensorPoseSSPub;
    ros::Publisher robotPoseSSPub;
    ros::Publisher connectionsPub;

    ExpWaypointsGenerator();
    void uav1_startPositionCallback(const geometry_msgs::PoseStamped& msg);
    void uav2_startPositionCallback(const geometry_msgs::PoseStamped& msg);
    void uav3_startPositionCallback(const geometry_msgs::PoseStamped& msg);
    kuri_msgs::NavTask generateExpWaypoints(const int uav_id);
};
#endif // PATH_GENERATOR_H

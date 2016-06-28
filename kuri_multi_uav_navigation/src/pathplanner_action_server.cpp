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
#include "geometry_msgs/PoseStamped.h"

#include <kuri_msgs/Tasks.h>
#include <kuri_msgs/NavTask.h>
#include <kuri_msgs/NavTasks.h>
#include <kuri_msgs/GeneratePathsAction.h>
#include <std_msgs/Float32.h>
#include <actionlib/server/simple_action_server.h>
#include "pathgenerator.h"
#include "nav_msgs/Path.h"

using namespace SSPP;

class Path_planner_action_server
{
public:

    Path_planner_action_server(std::string name) :
        actionServer(nh, name, false),
        actionName(name)
    {
        //register the goal and feeback callbacks
        ROS_INFO("Registering callbacks for action %s", actionName.c_str());
        actionServer.registerGoalCallback(boost::bind(&Path_planner_action_server::goalCB, this));
        actionServer.registerPreemptCallback(boost::bind(&Path_planner_action_server::preemptCB, this));
        ros::Subscriber currentPoseSub = nh.subscribe("/uav_1/mavros/local_position/pose", 1, &Path_planner_action_server::startPositionCallback, this);
        pathGenerator = new PathGenerator();
        ROS_INFO("Starting server for action %s", actionName.c_str());
        actionServer.start();
    }

    ~Path_planner_action_server(void)
    {
    }

    void startPositionCallback(const geometry_msgs::PoseStamped& msg)
    {
        currentPose = msg.pose;
    }

    void goalCB()
    {
        progressCount = 0;
        // accept the new goal
        ROS_INFO("Accepting Goal for action %s", actionName.c_str());
        goal = actionServer.acceptNewGoal()->tasks;
        ROS_INFO("started path planning");
        kuri_msgs::NavTasks nav_tasks = pathGenerator->generatePaths(goal);
//		kuri_msgs::NavTask nav_task;
//        nav_task.path = path;
//		kuri_msgs::NavTasks nav_tasks;
//		nav_tasks.nav_tasks.push_back(nav_task);
		result.nav_tasks = nav_tasks;
		ROS_INFO("%s: Succeeded", actionName.c_str());
                //// set the action state to succeeded
                actionServer.setSucceeded(result);
    }

    void preemptCB()
    {
        ROS_INFO("%s: Preempted", actionName.c_str());
        // set the action state to preempted
        actionServer.setPreempted();
    }


protected:

    ros::NodeHandle nh;
    actionlib::SimpleActionServer<kuri_msgs::GeneratePathsAction> actionServer;
    std::string actionName;
    float progressCount;
    kuri_msgs::Tasks goal;
    kuri_msgs::GeneratePathsFeedback feedback;
    kuri_msgs::GeneratePathsResult   result;
    kuri_msgs::NavTasks navTasks;
    ros::Subscriber sub;
    geometry_msgs::Pose currentPose;
    PathGenerator *pathGenerator;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_planner_action_server");
    Path_planner_action_server path_planner_action_server(ros::this_node::getName());
    ros::spin();
    return 0;
}

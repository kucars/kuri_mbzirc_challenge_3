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
#include <kuri_msgs/GeneratePathsAction.h>
#include <std_msgs/Float32.h>
#include <actionlib/server/simple_action_server.h>
#include "exploration_waypoints.h"
#include "nav_msgs/Path.h"
#include "kuri_msgs/GenerateExplorationWaypointsAction.h"

using namespace SSPP;

class Exploration_waypoints_action_server
{
public:

    Exploration_waypoints_action_server(std::string name) :
        actionServer(nh, name, false),
        actionName(name)
    {
        //register the goal and feeback callbacks
        ROS_INFO("Registering callbacks for action %s", actionName.c_str());
        actionServer.registerGoalCallback(boost::bind(&Exploration_waypoints_action_server::goalCB, this));
        actionServer.registerPreemptCallback(boost::bind(&Exploration_waypoints_action_server::preemptCB, this));
        expPathGenerator = new ExpWaypointsGenerator();
        ROS_INFO("Starting server for action %s", actionName.c_str());
        actionServer.start();
    }

    ~Exploration_waypoints_action_server(void)
    {

    }


    void goalCB()
    {
        progressCount = 0;
        // accept the new goal
        ROS_INFO("Accepting Goal for action %s", actionName.c_str());
        goal = actionServer.acceptNewGoal()->uav_id;
        ROS_INFO("started coverage path planning");
        kuri_msgs::NavTask nav_task = expPathGenerator->generateExpWaypoints(goal);
        result.expPath = nav_task;
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
    actionlib::SimpleActionServer<kuri_msgs::GenerateExplorationWaypointsAction> actionServer;
    std::string actionName;
    float progressCount;
    //kuri_msgs::Task goal;
    int goal;
    kuri_msgs::GenerateExplorationWaypointsFeedback feedback;
    kuri_msgs::GenerateExplorationWaypointsResult  result;
    kuri_msgs::NavTasks navTasks;
    ExpWaypointsGenerator *expPathGenerator;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "exploration_waypoints_action_server");
    Exploration_waypoints_action_server exploration_waypoints_action_server(ros::this_node::getName());
    ros::spin();
    return 0;
}

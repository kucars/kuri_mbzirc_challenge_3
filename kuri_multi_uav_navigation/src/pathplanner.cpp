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


using namespace SSPP;

class PathPlanningAction
{
public:

    PathPlanningAction(std::string name) :
        actionServer(nh, name, false),
        actionName(name)
    {
        //register the goal and feeback callbacks
        actionServer.registerGoalCallback(boost::bind(&PathPlanningAction::goalCB, this));
        actionServer.registerPreemptCallback(boost::bind(&PathPlanningAction::preemptCB, this));
        ros::Subscriber currentPoseSub = nh.subscribe("/uav_1/mavros/local_position/pose", 1, &PathPlanningAction::startPositionCallback, this);
        //subscribe to the data topic of interest
        sub = nh.subscribe("/random_number", 1, &PathPlanningAction::analysisCB, this);
        actionServer.start();
    }

    ~PathPlanningAction(void)
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
        goal = actionServer.acceptNewGoal()->tasks;
    }

    void preemptCB()
    {
        ROS_INFO("%s: Preempted", actionName.c_str());
        // set the action state to preempted
        actionServer.setPreempted();
    }

    void analysisCB(const std_msgs::Float32::ConstPtr& msg)
    {
        // make sure that the action hasn't been canceled
        if (!actionServer.isActive())
            return;

        progressCount++;
        feedback.planning_progress = progressCount;
        actionServer.publishFeedback(feedback);

        if(progressCount > 50)
        {
            result.nav_tasks = navTasks;

            if( 1 < 5.0)
            {
                ROS_INFO("%s: Aborted", actionName.c_str());
                //set the action state to aborted
                actionServer.setAborted(result);
            }
            else
            {
                ROS_INFO("%s: Succeeded", actionName.c_str());
                // set the action state to succeeded
                actionServer.setSucceeded(result);
            }
        }
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
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_planning");
    PathPlanningAction planGeneration(ros::this_node::getName());
    ros::spin();
    return 0;
}

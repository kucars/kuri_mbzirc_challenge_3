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

#include <kuri_msgs/NavTask.h>
#include <kuri_msgs/NavTasks.h>

using namespace SSPP;

geometry_msgs::Pose currentPose;
geometry_msgs::Pose endPose;
geometry_msgs::Pose target_coord;
float temp_x;
float temp_y;
kuri_msgs::NavTasks tasks;

void startPositionCallback(const geometry_msgs::PoseStamped& msg)
{
    currentPose = msg.pose;
}

void navTasksCallback(const kuri_msgs::NavTasks newtasks)
{
    std::cout<<"Received a new Task\n";
    tasks = newtasks;
}

int main(int argc, char ** argv)
{
    ROS_INFO("started");
    ros::init(argc, argv, "muti_uav_navigation");
    ros::NodeHandle nh;
    ros::Subscriber currentPoseSub = nh.subscribe("/uav_1/mavros/local_position/pose", 1, startPositionCallback);
    ros::Subscriber taskSub        = nh.subscribe("kuri_msgs/NavTasks", 1, navTasksCallback);
    ros::Publisher  posePub        = nh.advertise<geometry_msgs::PoseStamped>("/uav_1/mavros/setpoint_position/local", 10);
    rviz_visual_tools::RvizVisualToolsPtr visualTools;
    visualTools.reset(new rviz_visual_tools::RvizVisualTools("map","/sspp_visualisation"));
    visualTools->deleteAllMarkers();
    visualTools->setLifetime(0.2);

    QTime timer;
    geometry_msgs::Pose gridStartPose;
    geometry_msgs::Vector3 gridSize;
    gridStartPose.position.x = -50;
    gridStartPose.position.y = -30;
    gridStartPose.position.z = 0;
    // Dhanhani: including start to drop zone the arena is 140 x 60
    gridSize.x = 120;
    gridSize.y = 60;
    gridSize.z = 30;

    PathPlanner * pathPlanner;
    //    Pose start(0.0, 0.0, 0, DTOR(0.0));
    Pose end(19.0, 7.0, 2, DTOR(0.0));

    double robotH = 0.9, robotW = 0.5, narrowestPath = 0.987; //is not changed
    //Dhanhani: multiplied both of the next variables by 10
    double distanceToGoal = 1, regGridConRad = 7;

    QPointF robotCenter(-0.3f, 0.0f);
    Robot *robot = new Robot("Robot", robotH, robotW, narrowestPath, robotCenter);

    // Every how many iterations to display the tree
    int progressDisplayFrequency = 1;
    pathPlanner = new PathPlanner(nh, robot, regGridConRad, progressDisplayFrequency);
    // This causes the planner to pause for the desired amount of time and display the search tree, useful for debugging
    pathPlanner->setDebugDelay(0.1);
    ROS_INFO("planner object created");
    DistanceHeuristic distanceHeuristic(nh, false);
    distanceHeuristic.setEndPose(end.p);
    distanceHeuristic.setTolerance2Goal(distanceToGoal);
    pathPlanner->setHeuristicFucntion(&distanceHeuristic);

    // Generate Grid Samples and visualise it
    pathPlanner->generateRegularGrid(gridStartPose, gridSize, 5.0, false);
    std::vector<geometry_msgs::Point> searchSpaceNodes = pathPlanner->getSearchSpace();
    std::cout << "\n" << QString("\n---->>> Total Nodes in search Space =%1").arg(searchSpaceNodes.size()).toStdString();

    // Connect nodes and visualise it
    pathPlanner->connectNodes();
    std::cout << "\nSpace Generation took:" << timer.elapsed() / double(1000.00) << " secs";
    std::vector<geometry_msgs::Point> searchSpaceConnections = pathPlanner->getConnections();
    visualTools->publishPath(searchSpaceConnections, rviz_visual_tools::BLUE, rviz_visual_tools::LARGE,"search_space");

    ROS_INFO("\nstarting the loop");
    //Dhanhani start loop here with start as the input changing


    temp_x = 0.0f;
    temp_y = 0.0f;
    std::vector<geometry_msgs::Point> pathSegments;
    geometry_msgs::PoseArray robotPose, sensorPose;
    geometry_msgs::Point linePoint;
    double dist=0,threshold2Dist=0.5;
    double yaw;
    //TODO: Ultimately this should be a vector of paths: one per UAV
    Node * path = NULL;
    Node * wayPoint = NULL;
    ros::Rate loopRate(10);
    while (ros::ok())
    {
        // TODO: allow the handling of multiple paths
        if(path)
        {
            Node * path2Display = path;
            robotPose.poses.clear();
            pathSegments.clear();
            while(path2Display !=NULL)
            {
                tf::Quaternion qt(path2Display->pose.p.orientation.x,path2Display->pose.p.orientation.y,path2Display->pose.p.orientation.z,path2Display->pose.p.orientation.w);
                yaw = tf::getYaw(qt);
                if (path2Display->next !=NULL)
                {
                    linePoint.x = path2Display->pose.p.position.x;
                    linePoint.y = path2Display->pose.p.position.y;
                    linePoint.z = path2Display->pose.p.position.z;
                    robotPose.poses.push_back(path2Display->pose.p);
                    for(int i =0; i<path2Display->senPoses.size();i++)
                        sensorPose.poses.push_back(path2Display->senPoses[i].p);
                    pathSegments.push_back(linePoint);

                    linePoint.x = path2Display->next->pose.p.position.x;
                    linePoint.y = path2Display->next->pose.p.position.y;
                    linePoint.z = path2Display->next->pose.p.position.z;
                    robotPose.poses.push_back(path2Display->next->pose.p);
                    for(int i =0; i<path2Display->next->senPoses.size();i++)
                        sensorPose.poses.push_back(path2Display->next->senPoses[i].p);
                    pathSegments.push_back(linePoint);

                    dist=dist+ Dist(path2Display->next->pose.p,path2Display->pose.p);
                }
                path2Display = path2Display->next;
            }
            // Redraw
            visualTools->resetMarkerCounts();
            for(int i=0;i<robotPose.poses.size();i++)
            {
                visualTools->publishArrow(robotPose.poses[i],rviz_visual_tools::YELLOW, rviz_visual_tools::LARGE,0.3);
            }
            visualTools->publishSpheres(searchSpaceNodes,rviz_visual_tools::PURPLE,0.1,"search_space_nodes");
            visualTools->publishPath(searchSpaceConnections, rviz_visual_tools::BLUE, rviz_visual_tools::LARGE,"search_space");
            if(pathSegments.size()>0)
                visualTools->publishPath(pathSegments, rviz_visual_tools::RED, rviz_visual_tools::LARGE,"generated_path");
        }
        if(tasks.tasks.size()>0)
        {
            ROS_INFO("Path Planning for a new task");
            // Remove one task and serve it
            kuri_msgs::NavTask task = tasks.tasks.at(0);
            tasks.tasks.pop_back();
            kuri_msgs::Object currentObj = task.object;
            endPose = currentObj.pose.pose;
            ROS_INFO("New Destination(x,y,z): (%g,%g,%g) Current Location(x,y,z): (%g,%g,%g)",endPose.position.x,endPose.position.y,endPose.position.z,currentPose.position.x,currentPose.position.y,currentPose.position.z );
            // Find path and visualise it
            timer.restart();
            //To avoid crawling on the ground, always start at 10m altitude
            Pose start(currentPose.position.x, currentPose.position.y, 10.0, DTOR(0.0));
            //To allow visual servoying assisted landing, always hover at predefined z: 10m
            Pose end(endPose.position.x, endPose.position.y, 10, DTOR(0.0));
            distanceHeuristic.setEndPose(end.p);
            ROS_INFO("Starting search");
            Node * retval = pathPlanner->startSearch(start);
            std::cout << "\nPath Finding took:" << (timer.elapsed() / double(1000.00)) << " secs";
            //path print and visualization
            if (retval)
            {
                pathPlanner->printNodeList();
            }
            else
            {
                std::cout << "\nNo Path Found";
                continue;
            }
            path     = pathPlanner->path;
            // Assign the first waypoint to the first node on the path
            wayPoint = path;
        }
        if(wayPoint)
        {
            if(Dist(wayPoint->pose.p,currentPose) < threshold2Dist)
            {
                wayPoint = wayPoint->next;
                path     = wayPoint;
                 ROS_INFO("Sending a New WayPoint(x,y,z):(%g,%g,%g)",wayPoint->pose.p.position.x,wayPoint->pose.p.position.y,wayPoint->pose.p.position.z);
            }
            if(wayPoint)
            {
                geometry_msgs::PoseStamped uavWayPoint ;
                uavWayPoint.pose = wayPoint->pose.p;
                posePub.publish(uavWayPoint) ;
            }
        }
        ros::spinOnce();
        loopRate.sleep();
    }
    delete robot;
    delete pathPlanner;
    return 0;
}



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

#include <kuri_msgs/Tasks.h>
#include <kuri_msgs/NavTask.h>
#include <kuri_msgs/NavTasks.h>

using namespace SSPP;

geometry_msgs::Pose currentPose;
geometry_msgs::Pose endPose;
geometry_msgs::Pose target_coord;
kuri_msgs::Tasks tasks;

void startPositionCallback(const geometry_msgs::PoseStamped& msg)
{
    currentPose = msg.pose;
}

void navTasksCallback(const kuri_msgs::Tasks newtasks)
{
    std::cout<<"Received a new Task\n";
    tasks = newtasks;
}

void visualizePath(Node * path,rviz_visual_tools::RvizVisualToolsPtr visualTools, rviz_visual_tools::colors color)
{

    std::vector<geometry_msgs::Point> pathSegments;
    geometry_msgs::PoseArray robotPose;
    geometry_msgs::Point linePoint;
    double dist=0;
    if(path)
    {
        Node * path2Display = path;
        robotPose.poses.clear();
        pathSegments.clear();
        while(path2Display !=NULL)
        {
            tf::Quaternion qt(path2Display->pose.p.orientation.x,path2Display->pose.p.orientation.y,path2Display->pose.p.orientation.z,path2Display->pose.p.orientation.w);
            if (path2Display->next !=NULL)
            {
                linePoint.x = path2Display->pose.p.position.x;
                linePoint.y = path2Display->pose.p.position.y;
                linePoint.z = path2Display->pose.p.position.z;
                robotPose.poses.push_back(path2Display->pose.p);

                pathSegments.push_back(linePoint);

                linePoint.x = path2Display->next->pose.p.position.x;
                linePoint.y = path2Display->next->pose.p.position.y;
                linePoint.z = path2Display->next->pose.p.position.z;
                robotPose.poses.push_back(path2Display->next->pose.p);

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
        if(pathSegments.size()>0)
            visualTools->publishPath(pathSegments, color, rviz_visual_tools::LARGE,"generated_path");
    }
}

int main(int argc, char ** argv)
{
    ROS_INFO("started");
    ros::init(argc, argv, "muti_uav_navigation");
    ros::NodeHandle nh;
    ros::Subscriber currentPoseSub = nh.subscribe("/uav_1/mavros/local_position/pose", 1, startPositionCallback);
    ros::Subscriber taskSub        = nh.subscribe("kuri_msgs/Tasks", 1, navTasksCallback);
    ros::Publisher  posePub        = nh.advertise<geometry_msgs::PoseStamped>("/uav_1/mavros/setpoint_position/local", 10);
    rviz_visual_tools::RvizVisualToolsPtr visualTools;
    visualTools.reset(new rviz_visual_tools::RvizVisualTools("map","/sspp_visualisation"));
    visualTools->deleteAllMarkers();
    visualTools->setLifetime(0.2);

    QTime timer;
    geometry_msgs::Pose gridStartPose;
    geometry_msgs::Vector3 gridSize;
    gridStartPose.position.x = 0;
    gridStartPose.position.y = 0;

    gridStartPose.position.z = 0;
    gridSize.x = 12;
    gridSize.y = 6;
    gridSize.z = 30;
    PathPlanner * pathPlanner;

    double robotH = 0.9, robotW = 0.5, narrowestPath = 0.987;
    double distanceToGoal = 1.5, regGridConRad = 3;
    double gridRes = 2;

    QPointF robotCenter(-0.3f, 0.0f);
    Robot *robot = new Robot("Robot", robotH, robotW, narrowestPath, robotCenter);

    // Every how many iterations to display the tree
    int progressDisplayFrequency = 1;
    pathPlanner = new PathPlanner(nh, robot, regGridConRad, progressDisplayFrequency);
    // This causes the planner to pause for the desired amount of time and display the search tree, useful for debugging
    pathPlanner->setDebugDelay(0.0);
    ROS_INFO("planner object created");
    DistanceHeuristic distanceHeuristic(nh, false);
    distanceHeuristic.setTolerance2Goal(distanceToGoal);
    pathPlanner->setHeuristicFucntion(&distanceHeuristic);
    pathPlanner->setMultiAgentSupport(true);

    // Generate Grid Samples and visualise it
    pathPlanner->generateRegularGrid(gridStartPose, gridSize, gridRes, false);
    std::vector<geometry_msgs::Point> searchSpaceNodes = pathPlanner->getSearchSpace();
    std::cout << "\n" << QString("\n---->>> Total Nodes in search Space =%1").arg(searchSpaceNodes.size()).toStdString();

    // Connect nodes and visualise it
    pathPlanner->connectNodes();
    std::cout << "\nSpace Generation took:" << timer.elapsed() / double(1000.00) << " secs";
    std::vector<geometry_msgs::Point> searchSpaceConnections = pathPlanner->getConnections();
    visualTools->publishPath(searchSpaceConnections, rviz_visual_tools::BLUE, rviz_visual_tools::LARGE,"search_space");

    ROS_INFO("\nstarting the loop");

    // Find path and visualise it
    timer.restart();
    Pose start = Pose(0, 0, 0.0, DTOR(0.0));
    Pose end   = Pose(12, 6, 30, DTOR(0.0));
    distanceHeuristic.setEndPose(end.p);
    ROS_INFO("Starting search");
    Node * path1 = pathPlanner->startSearch(start);
    std::cout << "\nPath Finding took:" << (timer.elapsed() / double(1000.00)) << " secs";
    //path print and visualization
    if (path1)
    {
        pathPlanner->printLastPath();
    }
    else
    {
        std::cout << "\nNo Path Found";
    }

    ROS_INFO("Starting search for second path");
    start = Pose(12, 0, 0 , DTOR(0.0));
    end   = Pose(0,  6, 30, DTOR(0.0));
    distanceHeuristic.setEndPose(end.p);
    Node *path2 = pathPlanner->startSearch(start);
    std::cout << "\nPath Finding took:" << (timer.elapsed() / double(1000.00)) << " secs";
    //path print and visualization
    if (path2)
    {
        pathPlanner->printLastPath();
    }
    else
    {
        std::cout << "\nNo Path Found";
    }


    ROS_INFO("Starting search for third path");
    start = Pose(12, 3, 15, DTOR(0.0));
    end   = Pose(0,  3, 15, DTOR(0.0));
    distanceHeuristic.setEndPose(end.p);
    Node *path3 = pathPlanner->startSearch(start);
    std::cout << "\nPath Finding took:" << (timer.elapsed() / double(1000.00)) << " secs";
    //path print and visualization
    if (path3)
    {
        pathPlanner->printLastPath();
    }
    else
    {
        std::cout << "\nNo Path Found";
    }

    ros::Rate loopRate(10);
    while (ros::ok())
    {
        visualizePath(path1,visualTools,rviz_visual_tools::RED);
        visualizePath(path2,visualTools,rviz_visual_tools::MAGENTA);
        visualizePath(path3,visualTools,rviz_visual_tools::ORANGE);
        //visualTools->publishPath(searchSpaceConnections, rviz_visual_tools::BLUE, rviz_visual_tools::LARGE,"search_space");
        //visualTools->publishSpheres(searchSpaceNodes,rviz_visual_tools::PURPLE,0.1,"search_space_nodes");
        ros::spinOnce();
        loopRate.sleep();
    }

    delete robot;
    delete pathPlanner;
    return 0;
}

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

#include "Navigator.h"

using namespace SSPP;

Navigator::Navigator(void)
{
	
}

void Navigator::startPositionCallback(const geometry_msgs::PoseStamped& msg) {
    currentPose = msg.pose;
}

void Navigator::navTasksCallback(const kuri_msgs::Tasks newtasks) {
    std::cout << "Received a new Task\n";
    tasks = newtasks;
}

nav_msgs::Path Navigator::navigate(const kuri_msgs::Tasks newtasks)
{
	tasks = newtasks;
    ros::Subscriber currentPoseSub = nh.subscribe("/uav_1/mavros/local_position/pose", 1, &Navigator::startPositionCallback,this);
    ros::Subscriber taskSub = nh.subscribe("kuri_msgs/Tasks", 1, &Navigator::navTasksCallback,this);
    ros::Publisher posePub = nh.advertise<geometry_msgs::PoseStamped>("/uav_1/mavros/setpoint_position/local", 10);
    rviz_visual_tools::RvizVisualToolsPtr visualTools;
    visualTools.reset(new rviz_visual_tools::RvizVisualTools("map", "/sspp_visualisation"));
    visualTools->deleteAllMarkers();
    visualTools->setLifetime(0.2);

    QTime timer;
    geometry_msgs::Pose gridStartPose;
    geometry_msgs::Vector3 gridSize;
//    gridStartPose.position.x = -50;
//    gridStartPose.position.y = -30;
    gridStartPose.position.x = 0;
    gridStartPose.position.y = 0;

    gridStartPose.position.z = 0;
    // Dhanhani: including start to drop zone the arena is 140 x 60
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

    // Generate Grid Samples and visualise it
    pathPlanner->generateRegularGrid(gridStartPose, gridSize, gridRes, false);
    std::vector<geometry_msgs::Point> searchSpaceNodes = pathPlanner->getSearchSpace();
    std::cout << "\n" << QString("\n---->>> Total Nodes in search Space =%1").arg(searchSpaceNodes.size()).toStdString();

    // Connect nodes and visualise it
    pathPlanner->connectNodes();
    std::cout << "\nSpace Generation took:" << timer.elapsed() / double(1000.00) << " secs";
    std::vector<geometry_msgs::Point> searchSpaceConnections = pathPlanner->getConnections();
    visualTools->publishPath(searchSpaceConnections, rviz_visual_tools::BLUE, rviz_visual_tools::LARGE, "search_space");

    pathPlanner->disconnectNodes();
    std::cout<<"\n Nodes are now disconnected";

    ROS_INFO("\nstarting the loop");

    std::vector<geometry_msgs::Point> pathSegments;
    geometry_msgs::PoseArray robotPose, sensorPose;
    geometry_msgs::Point linePoint;
    double dist = 0, threshold2Dist = 0.5;
    double yaw;
    //TODO: Ultimately this should be a vector of paths: one per UAV
    Node * path = NULL;
    Node * wayPoint = NULL;
    ros::Rate loopRate(10);
    while (ros::ok()) {
        // TODO: allow the handling of multiple paths
        if (path) {
			Node * path2Send = path;
			nav_msgs::Path result;
			result.header.frame_id="/map";
			while (path2Send != NULL) {
				geometry_msgs::PoseStamped uavWayPoint;
                uavWayPoint.pose = path2Send->pose.p;
				result.poses.push_back(uavWayPoint);
				path2Send = path2Send->next;
			}
			geometry_msgs::PoseStamped uavWayPoint;
			uavWayPoint.pose = endPose;
			result.poses.push_back(uavWayPoint);
			return result;
        }
        if (tasks.tasks.size() > 0) {
            ROS_INFO("Path Planning for a new task");
            // Remove one task and serve it
            kuri_msgs::Task task = tasks.tasks.at(0);
            tasks.tasks.pop_back();
            kuri_msgs::Object currentObj = task.object;
            endPose = currentObj.pose.pose;
            ROS_INFO("New Destination(x,y,z): (%g,%g,%g) Current Location(x,y,z): (%g,%g,%g)", endPose.position.x, endPose.position.y, endPose.position.z, currentPose.position.x, currentPose.position.y, currentPose.position.z);
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
            if (retval) {
                pathPlanner->printNodeList();
            } else {
                std::cout << "\nNo Path Found";
                continue;
            }
            path = pathPlanner->path;
        }

        ros::spinOnce();
        loopRate.sleep();
    }
    delete robot;
    delete pathPlanner;
}

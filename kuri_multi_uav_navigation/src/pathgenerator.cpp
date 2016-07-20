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

/*
 * ToFix: starting point can overlap causing the path to not include the starting path, is that a problem?
*/


#include "pathgenerator.h"

using namespace SSPP;

PathGenerator::PathGenerator(void)
{

}

void PathGenerator::uav1_startPositionCallback(const geometry_msgs::PoseStamped& msg) {
    uav1_currentPose = msg.pose;
}

void PathGenerator::uav2_startPositionCallback(const geometry_msgs::PoseStamped& msg) {
    uav2_currentPose = msg.pose;
}

void PathGenerator::uav3_startPositionCallback(const geometry_msgs::PoseStamped& msg) {
    uav3_currentPose = msg.pose;
}

void PathGenerator::navTasksCallback(const kuri_msgs::Tasks newtasks) {
    std::cout << "Received a new Task\n";
    tasks = newtasks;
}

kuri_msgs::NavTasks PathGenerator::generatePaths(const kuri_msgs::Tasks newtasks)
{
    tasks = newtasks;
    ros::Subscriber uav1_currentPoseSub = nh.subscribe("/uav_1/mavros/local_position/pose", 1, &PathGenerator::uav1_startPositionCallback,this);
    ros::Subscriber uav2_currentPoseSub = nh.subscribe("/uav_2/mavros/local_position/pose", 1, &PathGenerator::uav2_startPositionCallback,this);
    ros::Subscriber uav3_currentPoseSub = nh.subscribe("/uav_3/mavros/local_position/pose", 1, &PathGenerator::uav3_startPositionCallback,this);
    //ros::Subscriber taskSub = nh.subscribe("kuri_msgs/Tasks", 1, &PathGenerator::navTasksCallback,this);
    //ros::Publisher posePub = nh.advertise<geometry_msgs::PoseStamped>("/uav_1/mavros/setpoint_position/local", 10);
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
    pathPlanner->setMultiAgentSupport(true);
    pathPlanner->connectNodes();
    std::cout << "\nSpace Generation took:" << timer.elapsed() / double(1000.00) << " secs";
    std::vector<geometry_msgs::Point> searchSpaceConnections = pathPlanner->getConnections();
    visualTools->publishPath(searchSpaceConnections, rviz_visual_tools::BLUE, rviz_visual_tools::LARGE, "search_space");

    //pathPlanner->disconnectNodes();
    std::cout<<"\n Nodes are now disconnected";

    ROS_INFO("\nstarting the loop");

    std::vector<geometry_msgs::Point> pathSegments;
    geometry_msgs::PoseArray robotPose, sensorPose;
    geometry_msgs::Point linePoint;
    double dist = 0, threshold2Dist = 0.5;
    double yaw;
    //TODO: Ultimately this should be a vector of paths: one per UAV
    //Node * path = NULL;
    //std::vector<Node*> paths;
    Node * wayPoint = NULL;
    ros::Rate loopRate(10);
    while (ros::ok()) {
        // TODO: allow the handling of multiple paths
        if (tasks.tasks.size()<1) {
            std::vector<Node*> paths = pathPlanner->paths;
            kuri_msgs::NavTasks nav_tasks;
            int i = 0;
            for(std::vector<Node*>::iterator it = paths.begin(); it!=paths.end();it++)
            {
                Node * path2Add = *it;
                    nav_msgs::Path result;
                    result.header.frame_id="/map";
                    while (path2Add != NULL) {
                        geometry_msgs::PoseStamped uavWayPoint;
                        uavWayPoint.pose = path2Add->pose.p;
                        result.poses.push_back(uavWayPoint);
                        path2Add = path2Add->next;
                    }
                    geometry_msgs::PoseStamped uavWayPoint;
                    uavWayPoint.pose = endPoses.at(i);
                    i++;
                    //endPoses.pop_back();
                    result.poses.push_back(uavWayPoint);
                kuri_msgs::NavTask nav_task;
                nav_task.path = result;
                nav_tasks.nav_tasks.push_back(nav_task);
            }


//            Node * path2Send = path;
//            nav_msgs::Path result;
//            result.header.frame_id="/map";
//            while (path2Send != NULL) {
//                geometry_msgs::PoseStamped uavWayPoint;
//                uavWayPoint.pose = path2Send->pose.p;
//                result.poses.push_back(uavWayPoint);
//                path2Send = path2Send->next;
//            }
//            geometry_msgs::PoseStamped uavWayPoint;
//            uavWayPoint.pose = endPose;
//            result.poses.push_back(uavWayPoint);
            return nav_tasks;
        }else{
            ROS_INFO("Path Planning for a new task");
            // Remove one task and serve it
            kuri_msgs::Task task = tasks.tasks.back();
            tasks.tasks.pop_back();
            kuri_msgs::Object currentObj = task.object;
            endPoses.push_back(currentObj.pose.pose);
            // Find path and visualise it
            timer.restart();
            //To avoid crawling on the ground, always start at 10m altitude
             geometry_msgs::Pose currentPose;
            if(task.uav_id==1){
                currentPose = uav1_currentPose;
            }else if(task.uav_id==2){
                currentPose = uav2_currentPose;
            }else if(task.uav_id==3){
                currentPose = uav3_currentPose;
            }
            ROS_INFO("New Destination(x,y,z): (%g,%g,%g) Current Location(x,y,z): (%g,%g,%g)", currentObj.pose.pose.position.x, currentObj.pose.pose.position.y, currentObj.pose.pose.position.z, currentPose.position.x, currentPose.position.y, currentPose.position.z);
            Pose start(currentPose.position.x, currentPose.position.y, 10.0, DTOR(0.0));
            //To allow visual servoying assisted landing, always hover at predefined z: 10m
            Pose end(currentObj.pose.pose.position.x, currentObj.pose.pose.position.y, 10, DTOR(0.0));

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
           // path = pathPlanner->paths[0];
        }

        ros::spinOnce();
        loopRate.sleep();
    }
    delete robot;
    delete pathPlanner;
}


//int main(int argc, char** argv)
//{
//    ros::init(argc, argv, "path_generator");
//    PathGenerator PathGenerator();
//    ros::spin();
//    return 0;
//}

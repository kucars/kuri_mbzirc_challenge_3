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

/*
 * ToFix: starting point can overlap causing the path to not include the starting path, is that a problem?
*/


#include "exploration_waypoints.h"

using namespace SSPP;

ExpWaypointsGenerator::ExpWaypointsGenerator(void)
{
    robotPoseSSPub    = nh.advertise<geometry_msgs::PoseArray>("SS_robot_pose", 10);
    sensorPoseSSPub   = nh.advertise<geometry_msgs::PoseArray>("SS_sensor_pose", 10);
    connectionsPub    = nh.advertise<visualization_msgs::Marker>("connections", 10);

}

void ExpWaypointsGenerator::uav1_startPositionCallback(const geometry_msgs::PoseStamped& msg)
{
    uav1_currentPose = msg.pose;
}

void ExpWaypointsGenerator::uav2_startPositionCallback(const geometry_msgs::PoseStamped& msg)
{
    uav2_currentPose = msg.pose;
}

void ExpWaypointsGenerator::uav3_startPositionCallback(const geometry_msgs::PoseStamped& msg)
{
    uav3_currentPose = msg.pose;
}


kuri_msgs::NavTask ExpWaypointsGenerator::generateExpWaypoints(const int uav_id)
{
    //TODO: later decide if you want to use them as the starting point for the exploration or not
    ros::Subscriber uav1_currentPoseSub = nh.subscribe("/uav_1/mavros/local_position/pose", 1, &ExpWaypointsGenerator::uav1_startPositionCallback,this);
    ros::Subscriber uav2_currentPoseSub = nh.subscribe("/uav_2/mavros/local_position/pose", 1, &ExpWaypointsGenerator::uav2_startPositionCallback,this);
    ros::Subscriber uav3_currentPoseSub = nh.subscribe("/uav_3/mavros/local_position/pose", 1, &ExpWaypointsGenerator::uav3_startPositionCallback,this);

    rviz_visual_tools::RvizVisualToolsPtr visualTools;
    visualTools.reset(new rviz_visual_tools::RvizVisualTools("map", "/sspp_visualisation"));
    visualTools->deleteAllMarkers();
    visualTools->setLifetime(0.2);

    ros::Time timer_start = ros::Time::now();
    geometry_msgs::Pose gridStartPose;
    geometry_msgs::Vector3 gridSize;

    gridStartPose.position.x = -80;//-18
    gridStartPose.position.y = -40 ;//-25
    gridStartPose.position.z = 20 ;//1
    gridSize.x = 160;//36
    gridSize.y = 80;//50
    gridSize.z = 0;//15

    PathPlanner * pathPlanner;

    double robotH = 0.9, robotW = 0.5, narrowestPath = 0.987;
    double regGridConRad = 2;
    double gridRes = 2;

    geometry_msgs::Point robotCenter;
    robotCenter.x = -0.3f;
    robotCenter.y = 0.0f;
    Robot *robot = new Robot("Robot", robotH, robotW, narrowestPath, robotCenter);
    Sensors sensor1(100,89.81,0.255,0.01,100.0,640,480,Vec3f(0,0.0,-0.046), Vec3f(0,1.57,0));
    std::vector<Sensors> sensors;
    sensors.push_back(sensor1);

    // Every how many iterations to display the tree
    int progressDisplayFrequency = 1;
    pathPlanner = new PathPlanner(nh, robot, regGridConRad, progressDisplayFrequency,sensors);
    // This causes the planner to pause for the desired amount of time and display the search tree, useful for debugging
    pathPlanner->setDebugDelay(0.0);
    ROS_INFO("planner object created");

    double coverageTolerance=0.15, targetCov=20; // using 20 as a testing example, it should be 100 to cover arena
    std::string collisionCheckModelPath = ros::package::getPath("component_test") + "/src/mesh/arena_modified_3.obj";
    std::string occlusionCullingModelName = "arena_modified_3.pcd";
    CoveragePathPlanningHeuristic coveragePathPlanningHeuristic(nh,collisionCheckModelPath,occlusionCullingModelName,false, true, SurfaceCoverageH);
    coveragePathPlanningHeuristic.setCoverageTarget(targetCov);
    coveragePathPlanningHeuristic.setCoverageTolerance(coverageTolerance);
    pathPlanner->setHeuristicFucntion(&coveragePathPlanningHeuristic);

    // Generate Grid Samples and visualise it
    pathPlanner->generateRegularGrid(gridStartPose, gridSize,gridRes,false,360,false);

    // Connect nodes and visualise it
    pathPlanner->connectNodes();
    std::vector<geometry_msgs::Point> searchSpaceConnections = pathPlanner->getConnections();
    visualization_msgs::Marker connectionsMarker = drawLines(searchSpaceConnections,10000,3,100000000,0.03);
    std::vector<geometry_msgs::Point> searchSpaceNodes = pathPlanner->getSearchSpace();
    std::cout<<"\n\n---->>> Total Nodes in search Space ="<<searchSpaceNodes.size();
    geometry_msgs::PoseArray robotPoseSS,sensorPoseSS;
    pathPlanner->getRobotSensorPoses(robotPoseSS,sensorPoseSS);

    robotPoseSS.header.frame_id= "map";
    robotPoseSS.header.stamp = ros::Time::now();
    robotPoseSSPub.publish(robotPoseSS);

    sensorPoseSS.header.frame_id= "map";
    sensorPoseSS.header.stamp = ros::Time::now();
    sensorPoseSSPub.publish(sensorPoseSS);
    connectionsPub.publish(connectionsMarker);

    ROS_INFO("Path Planning for arena exploration task");

    geometry_msgs::Pose currentPose;
    if(uav_id==1){
        currentPose = uav1_currentPose;
    }else if(uav_id==2){
        currentPose = uav2_currentPose;
    }else if(uav_id==3){
        currentPose = uav3_currentPose;
    }

    // TODO: later we should decide if we are going to take it from current UAV position or define it previously
    // Pose start(currentPose.position.x, currentPose.position.y, 20.0, DTOR(0.0));
    Pose start(-63.0,17,20,DTOR(0.0));//-60.1 18 20 //-50.1,-15.1,20


    ROS_INFO("Starting search");
    Node * path = pathPlanner->startSearch(start);
    std::cout<<"\nPath Finding took:"<<double(ros::Time::now().toSec() - timer_start.toSec())<<" secs";
    //path print and visualization
    if (path) {
        // pathPlanner->printNodeList();
    } else {
        std::cout << "\nNo Path Found";
    }


    Node * path2Add = path;
    nav_msgs::Path result;
    result.header.frame_id="/map";
    while (path2Add != NULL) {
        geometry_msgs::PoseStamped uavWayPoint;
        uavWayPoint.pose = path2Add->pose.p;
        result.poses.push_back(uavWayPoint);
        path2Add = path2Add->next;
    }

    kuri_msgs::NavTask nav_task;
    kuri_msgs::Task task;
    task.header.frame_id ="/map";
    task.uav_id = uav_id;
    nav_task.path = result;
    nav_task.task=task;


    delete robot;
    delete pathPlanner;

    return nav_task;


}



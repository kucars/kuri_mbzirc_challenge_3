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

//#include <component_test/occlusion_culling_gpu.h>
//#include <component_test/occlusion_culling.h>
//#include "coverage_path_planning_heuristic.h"
#include "distance_heuristic.h"
#include "rviz_drawing_tools.h"
#include "rviz_visual_tools/rviz_visual_tools.h"
//#include <mavros/mavros.h>
#include "geometry_msgs/PoseStamped.h"

#include <kuri_msgs/NavTask.h>
#include <kuri_msgs/NavTasks.h>

using namespace SSPP;

geometry_msgs::Pose start_coord;
geometry_msgs::Pose end_coord;
geometry_msgs::Pose target_coord;
float temp_x;
float temp_y;
kuri_msgs::NavTasks tasks;

void startPositionCallback(const geometry_msgs::PoseStamped& msg) {
    start_coord = msg.pose;
    //ROS_INFO("Current start position0: (%g, %g, %g)", start_coord.position.x, start_coord.position.y, start_coord.position.z);
}

void endPositionCallback(const geometry_msgs::PoseStamped& msg) {
    end_coord = msg.pose;
    //ROS_INFO("Current end position: (%g, %g, %g)", end_coord.position.x, end_coord.position.y, end_coord.position.z);
}

void navTasksCallback(const kuri_msgs::NavTasks newtasks){
    tasks = newtasks;
}

int main(int argc, char ** argv) {
    ROS_INFO("started");
    ros::init(argc, argv, "path_planning");
    ros::NodeHandle nh;
    
    QTime timer;
    geometry_msgs::Pose gridStartPose;
    geometry_msgs::Vector3 gridSize;
    gridStartPose.position.x = 0;
    gridStartPose.position.y = 0;
    gridStartPose.position.z = 0;
    // Dhanhani: including start to drop zone the arena is 140 x 60
    gridSize.x = 50;
    gridSize.y = 70;
    gridSize.z = 20;

    PathPlanner * pathPlanner;
    //    Pose start(0.0, 0.0, 0, DTOR(0.0));
    Pose end(19.0, 7.0, 2, DTOR(0.0));

    double robotH = 0.9, robotW = 0.5, narrowestPath = 0.987; //is not changed
    //Dhanhani: multiplied both of the next variables by 10
    double distanceToGoal = 10, regGridConRad = 15;

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
    pathPlanner->generateRegularGrid(gridStartPose, gridSize, 10.0, false);
    std::vector<geometry_msgs::Point> searchSpaceNodes = pathPlanner->getSearchSpace();
    std::cout << "\n" << QString("\n---->>> Total Nodes in search Space =%1").arg(searchSpaceNodes.size()).toStdString();

    // Connect nodes and visualise it
    pathPlanner->connectNodes();
    std::cout << "\nSpace Generation took:" << timer.elapsed() / double(1000.00) << " secs";

    ROS_INFO("starting the loop");
    //Dhanhani start loop here with start as the input changing
    ros::Subscriber start_sub = nh.subscribe("/uav_1/mavros/local_position/pose", 1, startPositionCallback);
    ros::Subscriber end_sub = nh.subscribe("/uav1_target_location", 1, endPositionCallback);
    ros::Subscriber tasks_sub = nh.subscribe("kuri_msgs/NavTasks", 1, navTasksCallback);

    temp_x = 0.0f;
    temp_y = 0.0f;
    while (ros::ok()) {
        ros::Rate loopRate(10);
        if (((target_coord.position.x != end_coord.position.x && target_coord.position.y != end_coord.position.y))) {
            ROS_INFO("target not equal to end");
        }
        if(tasks.tasks.size()>0)
        {
            ROS_INFO("assigning first task");
            kuri_msgs::NavTask task1 = tasks.tasks[0];
            ROS_INFO("assigning first object");
            kuri_msgs::Object currentObj = task1.object;
            ROS_INFO("assigning pose");
            geometry_msgs::PoseWithCovariance pose = currentObj.pose;
            end_coord = pose.pose;
            ROS_INFO("end vs target: (%g/%g/%g, %g/%g/%g, %g/%g/%g)", start_coord.position.x, target_coord.position.x, end_coord.position.x, start_coord.position.y, target_coord.position.y, end_coord.position.y, start_coord.position.z, target_coord.position.z, end_coord.position.z);
        }
        if (((target_coord.position.x != end_coord.position.x && target_coord.position.y != end_coord.position.y))&&(start_coord.position.x != end_coord.position.x || start_coord.position.y != end_coord.position.y)) {
            target_coord = end_coord;
            ROS_INFO("updated target");

            //Dhanhani: update the starting point from the uav location
            //         
            // Find path and visualise it
            timer.restart();
            Pose end(end_coord.position.x, end_coord.position.y, end_coord.position.z, DTOR(0.0));
            distanceHeuristic.setEndPose(end.p);
            Pose start(start_coord.position.x, start_coord.position.y, start_coord.position.z, DTOR(0.0));
            ROS_INFO("starting search");
            Node * retval = pathPlanner->startSearch(start);
            ROS_INFO("ended search");
            std::cout << "\nPath Finding took:" << (timer.elapsed() / double(1000.00)) << " secs";

            //path print and visualization
            if (retval) {
                pathPlanner->printNodeList();
            } else {
                std::cout << "\nNo Path Found";
            }

            Node * path = pathPlanner->path;
            geometry_msgs::Point linePoint;
            std::vector<geometry_msgs::Point> pathSegments;
            //geometry_msgs::PoseArray robotPose, sensorPose;
            double dist = 0;
            double yaw;

            //Dhanhani: Publisher code
            ros::Publisher map_pub = nh.advertise<geometry_msgs::Pose>("map", 1);
            //end
            ROS_INFO("while path is not null:");
            bool final_target = false;
            while (path != NULL) {
                tf::Quaternion qt(path->pose.p.orientation.x, path->pose.p.orientation.y, path->pose.p.orientation.z, path->pose.p.orientation.w);
                yaw = tf::getYaw(qt);
                // if (path->next != NULL) {
                //                        linePoint.x = path->pose.p.position.x;
                //                        linePoint.y = path->pose.p.position.y;
                //                        linePoint.z = path->pose.p.position.z;
                //   ROS_INFO("publishing");
                ros::spinOnce();
                //robotPose.poses.push_back(path->pose.p);
                //sensorPose.poses.push_back(path->senPose.p);
                //pathSegments.push_back(linePoint);

                //linePoint.x = path->next->pose.p.position.x;
                //linePoint.y = path->next->pose.p.position.y;
                //linePoint.z = path->next->pose.p.position.z;
                //robotPose.poses.push_back(path->next->pose.p);
                //sensorPose.poses.push_back(path->next->senPose.p);
                //pathSegments.push_back(linePoint);

                //dist = dist + Dist(path->next->pose.p, path->pose.p);
                //}
                if (!final_target&&(abs(path->pose.p.position.x - start_coord.position.x) > 1.0f || abs(path->pose.p.position.y - start_coord.position.y) > 1.0f)) {
                    map_pub.publish(path->pose.p);
                    if (temp_x != start_coord.position.x || start_coord.position.y != temp_y) {
                        temp_x = start_coord.position.x;
                        temp_y = start_coord.position.y;
                        double secs = ros::Time::now().toSec();
                        //ROS_INFO("Current start position: (%g, %g, %g)", start_coord.position.x, start_coord.position.y, start_coord.position.z);
                        ROS_INFO("Current start/target position(%f): (%g/%g, %g/%g, %g/%g)", secs, start_coord.position.x, path->pose.p.position.x, start_coord.position.y, path->pose.p.position.y, start_coord.position.z, path->pose.p.position.z);
                    }
                    // ROS_INFO("Current target position2: (%g, %g, %g)", path->pose.p.position.x, path->pose.p.position.y, path->pose.p.position.z);
                    // ROS_INFO("Current end position2: (%g, %g, %g)", end_coord.position.x, end_coord.position.y, end_coord.position.z);
                } else if (path->next != NULL) {
                    ROS_INFO("Current start position2: (%g, %g, %g)", start_coord.position.x, start_coord.position.y, start_coord.position.z);
                    ROS_INFO("Current target position2: (%g, %g, %g)", path->pose.p.position.x, path->pose.p.position.y, path->pose.p.position.z);
                    ROS_INFO("Current end position2: (%g, %g, %g)", end_coord.position.x, end_coord.position.y, end_coord.position.z);
                    ROS_INFO("iterated to next path\niterated to next path\niterated to next path\niterated to next path\niterated to next path\n");
                    path = path->next;
                    map_pub.publish(path->pose.p);
                    ROS_INFO("Next target position2: (%g, %g, %g)", path->pose.p.position.x, path->pose.p.position.y, path->pose.p.position.z);
                } else {
                    final_target = true;
                    double secs = ros::Time::now().toSec();
                    ROS_INFO("No next path, last iteration(%f): (%g/%g/%g, %g/%g/%g, %g/%g/%g)", secs, start_coord.position.x, path->pose.p.position.x, end_coord.position.x, start_coord.position.y, path->pose.p.position.y, end_coord.position.y, start_coord.position.z, path->pose.p.position.z, end_coord.position.z);
                    map_pub.publish(end_coord);
                }
                if((abs(end_coord.position.x - start_coord.position.x) < 1.0f && abs(end_coord.position.y - start_coord.position.y) < 1.0f)){
                    path = NULL;
                }
            }
            ROS_INFO("path is null;");
            std::cout << "\nDistance calculated from the path: " << dist << "m\n";

        }
        ros::spinOnce();
        loopRate.sleep();
    }            //while ros is ok end here
    delete robot;
    delete pathPlanner;
    return 0;
}



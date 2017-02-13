#ifndef occupancy_mapping_H
#define occupancy_mapping_H

#include <ros/ros.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#include <iostream>
#include <queue>
#include <sstream>
#include <string>
#include <time.h>
#include <algorithm> 
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/NavSatFix.h>
#include <actionlib/server/simple_action_server.h>
#include "kuri_msgs/MappingAction.h"
#include "kuri_msgs/Object.h"
#include "kuri_msgs/Objects.h"
#include "kuri_msgs/ObjectsMap.h"
#include "std_msgs/Bool.h"
#include <thread>
#include <mutex>



class Object_mapping
{

public:

    Object_mapping();
    void mapcallback(const  kuri_msgs::Objects objects);
    void ObjectsRemovalcallback(const kuri_msgs::Objects objectsR);
    void UpdateMap(const kuri_msgs::Objects objects,int objectsNum , int Add_Remove);
    void StoreMap(actionlib::SimpleActionServer<kuri_msgs::MappingAction> *actionServer,kuri_msgs::MappingResult result);

private:
    ros::NodeHandle ph;
    ros::Publisher map_pub;
    ros::Publisher map_pub1;
    ros::Subscriber map_sub;
    ros::Subscriber map_sub1;
    nav_msgs::OccupancyGrid map;
    int objectsNum;
    std::string actionName;
    std_msgs::Bool flag;
    int vertex;
    std::mutex m;
    bool flag_success;

};


#endif // occupancy_mapping_H

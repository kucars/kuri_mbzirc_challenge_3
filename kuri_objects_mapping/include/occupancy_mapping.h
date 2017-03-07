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
#include <sensor_msgs/NavSatFix.h>
#include <actionlib/server/simple_action_server.h>
#include "kuri_msgs/MappingAction.h"
#include "kuri_msgs/Object.h"
#include "kuri_msgs/Objects.h"
#include "kuri_msgs/ObjectsMap.h"
#include "std_msgs/Bool.h"
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <thread>
#include <mutex>



class Object_mapping
{

public:
    Object_mapping();
    void mapcallback(const kuri_msgs::Object object);
    void ObjectsRemovalcallback(const kuri_msgs::Objects objectsR);
    void UpdateMap(const kuri_msgs::Objects objects,int objectsNum , int Add_Remove);
    void StoreMap(actionlib::SimpleActionServer<kuri_msgs::MappingAction> *actionServer,kuri_msgs::MappingResult result);
    void publishMap (void );

private:
    ros::NodeHandle ph;
    ros::Publisher map_pub;
    ros::Publisher map_pub1;
    ros::Subscriber map_sub;
    ros::Subscriber map_sub1;
    grid_map::GridMap map;
    int objectsNum;
    std::string actionName;
    std_msgs::Bool flag;
    int vertex;
    std::mutex m;
    bool flag_success;
    kuri_msgs::Objects newObjects;

};


#endif // occupancy_mapping_H

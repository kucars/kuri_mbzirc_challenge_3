/***************************************************************************
 *   Copyright (C) 2017 by                                                 *
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

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <geometry_msgs/PoseArray.h>

#include <signal.h>
#include <termios.h>
#include <iostream>
#include <queue>
#include <sstream>
#include <string>
#include <time.h>
#include <algorithm>
#include <unistd.h>
#include <pthread.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>
#include <float.h>
#include <std_msgs/Float64.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/NavSatFix.h>
#include <stdio.h>
#include <stdlib.h>
#include <boost/filesystem.hpp>
#include <ros/package.h>
#include <geo.h>

ros::ServiceClient takeoff;

geometry_msgs::Point        real;
geometry_msgs::PoseArray    waypoints;
geometry_msgs::PoseArray    globalWaypoints;
geometry_msgs::PoseStamped  goalPose;
mavros_msgs::State          current_state;

int count        = 0 ;
double tolerance = 0.4;
double errorX    = 0;
double errorY    = 0;
double errorZ    = 0;
double ref_la;
double ref_lo;
double lat_ref;
double lon_ref;


void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}


void globalPoseCallback(const sensor_msgs::NavSatFix :: ConstPtr& msg)
{


    //check if it reached to the position (converting the current global position to local based on the map reference that the points were generated accroding to)
    map_projection_global_init(lat_ref, lon_ref,1);

    float p_x,p_y,p_z;
    globallocalconverter_tolocal(msg ->latitude,msg ->longitude,-1*msg ->altitude,&p_y,&p_x,&p_z);
    real.x = p_x;
    real.y = p_y;
    real.z = p_z;

    errorX =  waypoints.poses[count].position.x - real.x;
    errorY =  waypoints.poses[count].position.y - real.y;
    errorZ =  waypoints.poses[count].position.z - real.z;

    //std::cout<<"LOCAL : "<<real.x<<" "<<real.y<<" "<<real.z<<std::endl;
    //std::cout<<"error : "<<errorX<<" "<<errorY<<" "<<errorZ<<std::endl;
    if ((fabs(errorX) < tolerance) && (fabs(errorY) < tolerance) && (fabs(errorZ) < tolerance))
    {
        count++;
        if(count<waypoints.poses.size())
        {
            mavros_msgs::CommandTOL point;

            point.request.latitude = globalWaypoints.poses[count].position.x;
            point.request.longitude = globalWaypoints.poses[count].position.y;
            point.request.altitude = globalWaypoints.poses[count].position.z;
            bool result = takeoff.call(point);
            std::cout<<"done ...> next"<<std::endl;

        }else  std::cout<<"Finished exploration"<<std::endl;

    }

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "takeoff_cmd");
    ros::NodeHandle nh;


    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("/uav_1/mavros/state", 10, state_cb);
    ros::Publisher pos_pub = nh.advertise<mavros_msgs::GlobalPositionTarget>
            ("/uav_1/mavros/setpoint_raw/global", 10);
    ros::Publisher local_pos_pub = nh.advertise<mavros_msgs::PositionTarget>
            ("/uav_1/mavros/setpoint_raw/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/uav_1/mavros/cmd/arming");
    takeoff = nh.serviceClient<mavros_msgs::CommandTOL>
            ("/uav_1/mavros/cmd/takeoff");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/uav_1/mavros/set_mode");
    ros::Subscriber globalPoseSub = nh.subscribe
            ("/uav_1/mavros/global_position/global", 1000, globalPoseCallback);


    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }


    double locationx,locationy,locationz,qy;
    geometry_msgs::Pose pose;
    geometry_msgs::Pose globalPose;

    //read setpoints from file
    std::string str1 = ros::package::getPath("kuri_mbzirc_sim")+"/config/con_2_100_GPU_arena_ch3.txt";
    const char * filename1 = str1.c_str();

    assert(filename1 != NULL);
    filename1 = strdup(filename1);
    FILE *file1 = fopen(filename1, "r");
    if (!file1)
    {
        std::cout<<"\nCan not open File";
        fclose(file1);
    }

    //transfer exploration generated waypoints in terms of the reference that was chosen in creating the search space ( in simulation it is the world 0,0,0 which is represented as zurich 47.3977419 , 8.5455938
    // but in the real experiments it should be based on one of the corners of the arena of the challenge
    ros::param::param("~ref_lat", lat_ref, 1.0);
    ros::param::param("~ref_lon", lon_ref, 1.0);

    std::cout<<(double)lat_ref<<" "<<(double)lon_ref<<std::endl;
    map_projection_global_init(lat_ref, lon_ref,1);

    while (!feof(file1))
    {
        fscanf(file1,"%lf %lf %lf %lf\n",&locationx,&locationy,&locationz,&qy);
        pose.position.x = locationx;
        pose.position.y = locationy;
        pose.position.z = locationz;
        pose.orientation.x = qy;
        waypoints.poses.push_back(pose);

        double lat;
        double lon;
        float alt;

        globallocalconverter_toglobal(locationy,locationx,locationz,&lat,&lon,&alt);
        globalPose.position.x = lat;
        globalPose.position.y = lon;
        globalPose.position.z = alt*-1;
        std::cout<<"global : lat "<<lat<<" lon "<<lon<<" alt "<<alt<<std::endl;
        std::cout<<"local : x "<<locationx<<" y "<<locationy<<" z "<<locationz<<std::endl;

        globalWaypoints.poses.push_back(globalPose);

    }


    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "AUTO.TAKEOFF";

    mavros_msgs::CommandBool arm_cmd;


    ros::Time last_request = ros::Time::now();
    bool armed=false;
    bool flag=false;
    while(ros::ok()){
            arm_cmd.request.value = true;
        if( current_state.mode != "AUTO.TAKEOFF" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.success){
                ROS_INFO("AUTO.TAKEOFF enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0)) && !(armed)){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                    armed=true;
                }
                last_request = ros::Time::now();
            }
        }

        //just to takeoff to the first position (it is done once)
        if(!flag )
        {
            mavros_msgs::CommandTOL point;

            point.request.latitude = globalWaypoints.poses[count].position.x;
            point.request.longitude = globalWaypoints.poses[count].position.y;
            point.request.altitude = globalWaypoints.poses[count].position.z;
            bool result = takeoff.call(point);
            flag=true;
        }


        ros::spinOnce();
        rate.sleep();
    }



    return 0;
}

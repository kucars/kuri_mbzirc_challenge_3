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
#include <stdio.h>
#include <stdlib.h>
#include <boost/filesystem.hpp>
#include <ros/package.h>

#define CONSTANTS_ONE_G					9.80665f		/* m/s^2		*/
#define CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C		1.225f			/* kg/m^3		*/
#define CONSTANTS_AIR_GAS_CONST				287.1f 			/* J/(kg * K)		*/
#define CONSTANTS_ABSOLUTE_NULL_CELSIUS			-273.15f		/* °C			*/
#define CONSTANTS_RADIUS_OF_EARTH	6371000	/* meters (m)		*/

/* lat/lon are in radians */
struct map_projection_reference_s {
    long double lat_rad;
    long double lon_rad;
    long double sin_lat;
    long double cos_lat;
    bool init_done;
    uint64_t timestamp;
};

struct globallocal_converter_reference_s {
    double alt;
    bool init_done;
};

static struct map_projection_reference_s mp_ref = {0.0, 0.0, 0.0, 0.0, false, 0};
static struct globallocal_converter_reference_s gl_ref = {0.0f, false};

ros::ServiceClient takeoff;

geometry_msgs::Point        real;
geometry_msgs::PoseArray    waypoints;
geometry_msgs::PoseArray    globalWaypoints;
geometry_msgs::PoseStamped  goalPose;
mavros_msgs::State          current_state;

int count       = 0 ;
double tolerance = 0.3;
double errorX    = 0;
double errorY    = 0;
double errorZ    = 0;


void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}



bool map_projection_initialized(const struct map_projection_reference_s *ref)
{
    return ref->init_done;
}

bool map_projection_global_initialized()
{
    return map_projection_initialized(&mp_ref);
}


int map_projection_reproject(const struct map_projection_reference_s *ref, double x, double y, long double *lat,
                      long double *lon)
{
    if (!map_projection_initialized(ref)) {
        return -1;
    }

    long double x_rad = x / CONSTANTS_RADIUS_OF_EARTH;
    long double y_rad = y / CONSTANTS_RADIUS_OF_EARTH;
    long double c = sqrtf(x_rad * x_rad + y_rad * y_rad);
    long double sin_c = sin(c);
    long double cos_c = cos(c);

    long double lat_rad;
    long double lon_rad;

    if (fabs(c) > DBL_EPSILON) {
        lat_rad = asin(cos_c * ref->sin_lat + (x_rad * sin_c * ref->cos_lat) / c);
        lon_rad = (ref->lon_rad + atan2(y_rad * sin_c, c * ref->cos_lat * cos_c - x_rad * ref->sin_lat * sin_c));

    } else {
        lat_rad = ref->lat_rad;
        lon_rad = ref->lon_rad;
    }

    *lat = lat_rad * 180.0 / M_PI;
    *lon = lon_rad * 180.0 / M_PI;

    return 0;
}

int map_projection_global_reproject(double x, double y, long double *lat, long double *lon)
{
    return map_projection_reproject(&mp_ref, x, y, lat, lon);
}



int globallocalconverter_toglobal(double x, double y, double z,  long double *lat,long double *lon, double *alt)
{
    if (!map_projection_global_initialized()) {
        //return -1;
    }

    map_projection_global_reproject(x, y, lat, lon);
    *alt = gl_ref.alt - z;

    return 0;
}

void globalPoseCallback(const geometry_msgs :: PoseStamped :: ConstPtr& msg)
{
    real.x=msg ->pose.position.x;
    real.y=msg ->pose.position.y;
    real.z=msg ->pose.position.z;

    errorX =  waypoints.poses[count].position.x - real.x;
    errorY =  waypoints.poses[count].position.y - real.y;
    errorZ =  waypoints.poses[count].position.z - real.z;

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

        }

    }

}

int map_projection_init_timestamped(struct map_projection_reference_s *ref, double lat_0, double lon_0,
        uint64_t timestamp) //lat_0, lon_0 are expected to be in correct format: -> 47.1234567 and not 471234567
{

    ref->lat_rad = lat_0 * M_PI / 180.0;
    ref->lon_rad = lon_0 * M_PI / 180.0;
    ref->sin_lat = sin(ref->lat_rad);
    ref->cos_lat = cos(ref->lat_rad);

    ref->timestamp = timestamp;
    ref->init_done = true;

    return 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;


//    double lat_ref = 47.397742;
//    double lon_ref =8.5455938;
//    std_msgs::Float64 lat_ref1 = (std_msgs::Float64)47.397742;
//    std_msgs::Float64 lon_ref1 =(std_msgs::Float64)8.5455938;



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
            ("/uav_1/mavros/local_position/pose", 1000, globalPoseCallback);

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

    double lat_ref = ref_la;
    double lon_ref = ref_lo;
    std::cout<<(double)lat_ref<<" "<<(double)lon_ref<<std::endl;
    map_projection_init_timestamped(&mp_ref,lat_ref,lon_ref,1);

    while (!feof(file1))
    {
        fscanf(file1,"%lf %lf %lf %lf\n",&locationx,&locationy,&locationz,&qy);
        pose.position.x = locationx;
        pose.position.y = locationy;
        pose.position.z = locationz;
        pose.orientation.x = qy;
        waypoints.poses.push_back(pose);

        long double lat;
        long double lon;
        double alt;

        globallocalconverter_toglobal(locationy,locationx,locationz,&lat,&lon,&alt);
        globalPose.position.x = lat;
        globalPose.position.y = lon;
        globalPose.position.z = alt*-1;
        std::cout<<"global : lat "<<lat<<" lon "<<lon<<" alt "<<alt<<std::endl;
        std::cout<<"local : x "<<locationx<<" y "<<locationy<<" z "<<locationz<<std::endl;

        globalWaypoints.poses.push_back(globalPose);

    }


    mavros_msgs::GlobalPositionTarget pp;
//    pp.header.frame_id = "fcu";
    pp.latitude = 95;
    pp.longitude= 707406378;
    pp.altitude = -10;
    pp.coordinate_frame = mavros_msgs::GlobalPositionTarget::FRAME_GLOBAL_INT;
    pp.type_mask = (1 << 11) | (7 << 6) | (7 << 3);

    mavros_msgs::PositionTarget p;
    p.position.x= 5;
    p.position.y = -5;
    p.position.z = 10;
    p.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    p.type_mask = (1 << 11) | (7 << 6) | (7 << 3);


    //send a few setpoints before starting
//     for(int i = 100; ros::ok() && i > 0; --i){
//         pos_pub.publish(pp);
//         local_pos_pub.publish(p);
//         ros::spinOnce();
//         rate.sleep();
//     }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "AUTO.TAKEOFF";

    mavros_msgs::CommandBool arm_cmd;
    mavros_msgs::CommandTOL point;
    point.request.altitude = 10;
    point.request.latitude = 47.3977003;//47.3977419
    point.request.longitude = 8.5455939;//8.5455939

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

//        local_pos_pub.publish(p);
//        pos_pub.publish(pp);

        if(!flag)
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

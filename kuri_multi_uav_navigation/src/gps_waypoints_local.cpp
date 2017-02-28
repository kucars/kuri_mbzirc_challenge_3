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
#include <sensor_msgs/NavSatFix.h>

mavros_msgs::PositionTarget p;
geometry_msgs::Pose         globalPose;
std::vector<mavros_msgs::PositionTarget> localPoses;
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

double lat_ref;
double lon_ref;

bool flagRef = false;
bool finished = false;


// taken from the geo.c in PX4/Firmware
#define CONSTANTS_ONE_G					9.80665f		/* m/s^2		*/
#define CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C		1.225f			/* kg/m^3		*/
#define CONSTANTS_AIR_GAS_CONST				287.1f 			/* J/(kg * K)		*/
#define CONSTANTS_ABSOLUTE_NULL_CELSIUS			-273.15f		/* °C			*/
#define CONSTANTS_RADIUS_OF_EARTH	6371000	/* meters (m)		*/


// taken from the geo.c in PX4/Firmware
/* lat/lon are in radians */
struct map_projection_reference_s {
    long double lat_rad;
    long double lon_rad;
    long double sin_lat;
    long double cos_lat;
    bool init_done;
    uint64_t timestamp;
};

// taken from the geo.c in PX4/Firmware
struct globallocal_converter_reference_s {
    double alt;
    bool init_done;
};

// taken from the geo.c in PX4/Firmware
static struct map_projection_reference_s mp_ref = {0.0, 0.0, 0.0, 0.0, false, 0};
static struct globallocal_converter_reference_s gl_ref = {0.0f, false};


void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}


// taken from the geo.c in PX4/Firmware
bool map_projection_initialized(const struct map_projection_reference_s *ref)
{
    return ref->init_done;
}

// taken from the geo.c in PX4/Firmware
bool map_projection_global_initialized()
{
    return map_projection_initialized(&mp_ref);
}

// taken from the geo.c in PX4/Firmware
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

// taken from the geo.c in PX4/Firmware
int map_projection_global_reproject(double x, double y, long double *lat, long double *lon)
{
    return map_projection_reproject(&mp_ref, x, y, lat, lon);
}

// taken from the geo.c in PX4/Firmware
int globallocalconverter_toglobal(double x, double y, double z,  long double *lat,long double *lon, double *alt)
{
    if (!map_projection_global_initialized()) {
        //return -1;
    }

    map_projection_global_reproject(x, y, lat, lon);
    *alt = gl_ref.alt - z;

    return 0;
}

// taken from the geo.c in PX4/Firmware
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

// taken from the geo.c in PX4/Firmware
int map_projection_project(const struct map_projection_reference_s *ref, double lat, double lon, double *x,
                           double *y)
{
    if (!map_projection_initialized(ref)) {
        return -1;
    }
    std::setprecision(10);
    double lat_rad = lat * M_PI / 180.0  ;
    double lon_rad = lon * M_PI / 180.0;

    double sin_lat = sin(lat_rad);
    double cos_lat = cos(lat_rad);
    double cos_d_lon = cos(lon_rad - ref->lon_rad);

    double arg = ref->sin_lat * sin_lat + ref->cos_lat * cos_lat * cos_d_lon;

    if (arg > 1.0) {
        arg = 1.0;

    } else if (arg < -1.0) {
        arg = -1.0;
    }

    double c = acos(arg);
    double k = (fabs(c) < DBL_EPSILON) ? 1.0 : (c / sin(c));

    *x = k * (ref->cos_lat * sin_lat - ref->sin_lat * cos_lat * cos_d_lon) * CONSTANTS_RADIUS_OF_EARTH;
    *y = k * cos_lat * sin(lon_rad - ref->lon_rad) * CONSTANTS_RADIUS_OF_EARTH;

    return 0;
}

// taken from the geo.c in PX4/Firmware
int map_projection_global_project(double lat, double lon, double *x, double *y)
{
    return map_projection_project(&mp_ref, lat, lon, x, y);

}

// taken from the geo.c in PX4/Firmware
int globallocalconverter_tolocal(double lat, double lon, double alt, double *x, double *y, double *z)
{
    if (!map_projection_global_initialized()) {
        return -1;
    }

    map_projection_global_project(lat, lon, x, y);
    *z = gl_ref.alt - alt;

    return 0;
}


void reached(geometry_msgs::Point msg)
{

    double x,y,z;
    //transfered to local based on the uav home position as a map reference
    globallocalconverter_tolocal(msg.x,msg.y,msg.z,&y,&x,&z);
    std::cout<<"current global position: x"<<msg.x<<" y "<<msg.y<<" z "<<msg.z<<std::endl;

    //check reached accroding to the home position of the uav
    errorX =  p.position.x - x;
    errorY =  p.position.y - y;
    errorZ =  p.position.z - z;
    std::cout<<"p: x"<<p.position.x<<" y "<<p.position.y<<" z "<<p.position.z<<std::endl;
    std::cout<<"current local position ref to the uav home position: x"<<x<<" y "<<y<<" z "<<z<<std::endl;

    std::cout<<"error: x"<<fabs(errorX)<<" y "<<fabs(errorY)<<" z "<<fabs(errorZ)<<std::endl;
    if ((fabs(errorX) < tolerance) && (fabs(errorY) < tolerance))
    {
        count++;
        if(count<waypoints.poses.size())
        {

            localPoses.push_back(p);
            std::cout<<"**************REACHED***************"<<std::endl;

        }else finished=true;


    }
}
void globalPoseCallback(const sensor_msgs::NavSatFix:: ConstPtr& msg)
{
    real.x=msg->latitude;
    real.y=msg->longitude;
    real.z=msg->altitude;

    if(!flagRef)
    {
        if(real.x != 0 && real.y != 0)
        {
            lat_ref = real.x;
            lon_ref = real.y;
            flagRef=true;
        }

    }else if(!finished){
        reached(real);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "local_control");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("/uav_2/mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<mavros_msgs::PositionTarget>
            ("/uav_2/mavros/setpoint_raw/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/uav_2/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/uav_2/mavros/set_mode");
    ros::Subscriber globalPoseSub = nh.subscribe
            ("/uav_2/mavros/global_position/global", 1, globalPoseCallback);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }


    //read setpoints from file
    std::string str1 = ros::package::getPath("kuri_system_coordinator")+"/config/exploration_waypoints_50.txt";
    const char * filename1 = str1.c_str();

    assert(filename1 != NULL);
    filename1 = strdup(filename1);
    FILE *file1 = fopen(filename1, "r");
    if (!file1)
    {
        std::cout<<"\nCan not open File";
        fclose(file1);
    }



    double locationx,locationy,locationz,qy;
    geometry_msgs::Pose pose;

    //transfer exploration generated waypoints in terms of the reference that was chosen in creating the search space ( in simulation it is the world 0,0,0 which is represented as zurich 47.3977419 , 8.5455938
    // but in the real experiments it should be based on one of the corners of the arena of the challenge
    double wpt_lat_ref = 47.3977419;
    double wpt_lon_ref = 8.5455938;
    std::cout<<" The local waypoints map reference: "<<(double)wpt_lat_ref<<" "<<(double)wpt_lon_ref<<std::endl;
    map_projection_init_timestamped(&mp_ref,wpt_lat_ref,wpt_lon_ref,1);
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
        //std::cout<<"global : lat "<<lat<<" lon "<<lon<<" alt "<<alt<<std::endl;
        //std::cout<<"local : x "<<locationx<<" y "<<locationy<<" z "<<locationz<<std::endl;

        globalWaypoints.poses.push_back(globalPose);

    }


    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;

    ros::Time last_request = ros::Time::now();
    bool flag=false;
    while(ros::ok()){
        arm_cmd.request.value = true;
        if( current_state.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.success){
                ROS_INFO("OFFBOARD enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                    (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                        arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        if(flagRef && !finished)
        {
            std::cout<<(double)lat_ref<<" "<<(double)lon_ref<<std::endl;
            map_projection_init_timestamped(&mp_ref,lat_ref,lon_ref,1);


            //transfer exploration global waypoints in terms of the uav gps home position
            globallocalconverter_tolocal(globalWaypoints.poses[count].position.x,globalWaypoints.poses[count].position.y,-1*globalWaypoints.poses[count].position.z,&p.position.y,&p.position.x,&p.position.z);

            std::cout<<"local waypoint based on the uav home position : x "<<p.position.x<<" y "<<p.position.y<<" z "<<p.position.z<<std::endl;

            p.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
            p.type_mask = (1 << 11) | (7 << 6) | (7 << 3);

            local_pos_pub.publish(p);
        }
        else
        {
            local_pos_pub.publish(p);
        }
        ros::spinOnce();
        rate.sleep();
    }



    return 0;
}

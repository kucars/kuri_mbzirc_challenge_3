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
#include <stdio.h>
#include <stdlib.h>
#include <boost/filesystem.hpp>
#include <ros/package.h>
#include <sensor_msgs/NavSatFix.h>
#include <geo.h>

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

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}


void reached(geometry_msgs::Point msg)
{

    float x,y,z;
    //transfered to local based on the uav home position as a map reference
    globallocalconverter_tolocal(msg.x,msg.y,msg.z,&y,&x,&z);
    //std::cout<<"current global position: x"<<msg.x<<" y "<<msg.y<<" z "<<msg.z<<std::endl;

    //check reached accroding to the home position of the uav
    errorX =  p.position.x - x;
    errorY =  p.position.y - y;
    errorZ =  p.position.z - z;
    //std::cout<<"p: x"<<p.position.x<<" y "<<p.position.y<<" z "<<p.position.z<<std::endl;
    //std::cout<<"current local position ref to the uav home position: x"<<x<<" y "<<y<<" z "<<z<<std::endl;

    //std::cout<<"error: x"<<fabs(errorX)<<" y "<<fabs(errorY)<<" z "<<fabs(errorZ)<<std::endl;
    if ((fabs(errorX) < tolerance) && (fabs(errorY) < tolerance))
    {
        count++;
        if(count<waypoints.poses.size())
        {

            localPoses.push_back(p);
            std::cout<<"**************REACHED --> next ***************"<<std::endl;

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
    double wpt_lat_ref,wpt_lon_ref;
    ros::param::param("~ref_lat", wpt_lat_ref, 1.0);
    ros::param::param("~ref_lon", wpt_lon_ref, 1.0);

    std::cout<<" The local waypoints map reference: "<<(double)wpt_lat_ref<<" "<<(double)wpt_lon_ref<<std::endl;
    map_projection_global_init(wpt_lat_ref, wpt_lon_ref,1);

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
            map_projection_global_init(lat_ref, lon_ref,1);


            //transfer exploration global waypoints in terms of the uav gps home position
            float p_x,p_y,p_z;
            globallocalconverter_tolocal(globalWaypoints.poses[count].position.x,globalWaypoints.poses[count].position.y, -1*globalWaypoints.poses[count].position.z,&p_y,&p_x,&p_z);
            p.position.x=p_x;
            p.position.y=p_y;
            p.position.z=p_z;

            //std::cout<<"local waypoint based on the uav home position : x "<<p.position.x<<" y "<<p.position.y<<" z "<<p.position.z<<std::endl;

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

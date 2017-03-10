/***************************************************************************
 *   Copyright (C) 2017 by                                                 *
 *      Tarek Taha, KURI  <tataha@tarektaha.com>                           *
 *      Randa Almadhoun   <randa.almadhoun@gmail.com>                      *
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
#include <mavros_msgs/State.h>
#include <sensor_msgs/NavSatFix.h>
#include <geo.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int64.h>
#include <nav_msgs/Path.h>

enum{idle,followPath,pick,drop};

mavros_msgs::State current_state;
geometry_msgs::PoseStamped localPoseH; //with respect to home position
geometry_msgs::PoseStamped localPoseR; //with respect to a defined reference (ex: zurich, or one of the arena corners)
sensor_msgs::NavSatFix globalPose;

ros::Publisher homeGlobalPub;
ros::Publisher finished_pub;

int uav_id;
int uavState     = idle;
int count        = 0 ;
double tolerance = 0.3;
double errorX    = 0;
double errorY    = 0;
double errorZ    = 0;


bool homePoseFlag       = false;
bool transformDoneFlag  = false;
std_msgs::Bool finishedFlag;

int stateNum;

double home_lat;
double home_lon;

std::vector<sensor_msgs::NavSatFix>     globalWaypoints;
geometry_msgs::PoseArray                newLocalWaypoints;
geometry_msgs::PoseStamped              goalPose;
geometry_msgs::PoseStamped              idlePose;
nav_msgs::Path                          path;

void stateCb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

// idle, followpath, pick, drop
void stateNumCb(const std_msgs::Int64::ConstPtr& msg){
    stateNum = msg->data;
    std::cout<<"**************** stateNum ***************** "<<stateNum<<std::endl;
    if(stateNum == idle)
    {
        uavState = idle;
        std::cout<<"uav_state: IDLE "<<uavState<<std::endl;
    }
    else if(stateNum == followPath)
    {
        uavState = followPath;
        std::cout<<"uav_state: FOLLOW PATH "<<uavState<<std::endl;
    }

    else if(stateNum == pick)
    {
        uavState = pick;
        std::cout<<"uav_state: PICK "<<uavState<<std::endl;
    }
    else if(stateNum == drop)
    {
        uavState = drop;
        std::cout<<"uav_state: DROP "<<uavState<<std::endl;
    }
}

void navPathCb(const nav_msgs::Path::ConstPtr& msg){
    if(homePoseFlag)
    {
        finishedFlag.data=false;
        double wpt_lat_ref,wpt_lon_ref;
        ros::param::param("~ref_lat", wpt_lat_ref, 47.3977419);
        ros::param::param("~ref_lon", wpt_lon_ref, 8.5455938);

        std::cout<<" The local waypoints map reference: "<<(double)wpt_lat_ref<<" "<<(double)wpt_lon_ref<<std::endl;
        for(int i=0; i<msg->poses.size(); i++)
        {
            map_projection_global_init(wpt_lat_ref, wpt_lon_ref,1);
            geometry_msgs::PoseStamped p;
            p.pose.position = msg->poses[i].pose.position;
            path.poses.push_back(p);

            //convert to global
            double lat,lon;
            float alt;
            globallocalconverter_toglobal(path.poses[i].pose.position.y,path.poses[i].pose.position.x,path.poses[i].pose.position.z,&lat,&lon,&alt);
            globalPose.latitude = lat;
            globalPose.longitude = lon;
            globalPose.altitude = alt*-1;
            globalWaypoints.push_back(globalPose);

            //convert to local with repsect to the uav home position
            map_projection_global_init(home_lat, home_lon,1);
            printf(" uav home position: x %f y %f \n",home_lat,home_lon);
            float p_x,p_y,p_z;
            globallocalconverter_tolocal(globalPose.latitude,globalPose.longitude, -1*globalPose.altitude,&p_y,&p_x,&p_z);
            geometry_msgs::Pose pt;
            pt.position.x = p_x;
            pt.position.y = p_y;
            pt.position.z = p_z;
            printf(" new local pose: x %f y %f z %f \n",p_x,p_y,p_z);
            newLocalWaypoints.poses.push_back(pt);
        }
        goalPose.pose.position.x = newLocalWaypoints.poses[count].position.x;
        goalPose.pose.position.y = newLocalWaypoints.poses[count].position.y;
        goalPose.pose.position.z = newLocalWaypoints.poses[count].position.z;
        std::cout<<"**************** transformation Done ***************** "<<std::endl;
        transformDoneFlag = true;
    }
}


void localPoseCb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    localPoseH.pose.position.x= msg->pose.position.x;
    localPoseH.pose.position.y= msg->pose.position.y;
    localPoseH.pose.position.z= msg->pose.position.z;

    //std::cout<<"uav local pose x: "<<localPoseH.pose.position.x<<" y: "<< localPoseH.pose.position.y<<" z: "<< localPoseH.pose.position.z<<std::endl;

    if(uavState == followPath)
    {
        errorX =  goalPose.pose.position.x - localPoseH.pose.position.x;
        errorY =  goalPose.pose.position.y - localPoseH.pose.position.y;
        errorZ =  goalPose.pose.position.z - localPoseH.pose.position.z;
        //std::cout<<"error x: "<<errorX<<" y: "<< errorY<<" z: "<< errorZ<<std::endl;

        if ((fabs(errorX) < tolerance) && (fabs(errorY) < tolerance) && (fabs(errorZ) < tolerance))
        {
            count++;
            if(count<newLocalWaypoints.poses.size())
            {
                std::cout<<"new waypoints x: "<<newLocalWaypoints.poses[count].position.x<<" y: "<< newLocalWaypoints.poses[count].position.y<<" z: "<< newLocalWaypoints.poses[count].position.z<<std::endl;

                // ROS_INFO("UAV %i : Sending a New uav local WayPoint(x,y,z):(%g,%g,%g)",uav_id,newLocalWaypoints.poses[count].position.x,newLocalWaypoints.poses[count].position.y,newLocalWaypoints.poses[count].position.z);
                // ROS_INFO("UAV %i : Sending a New uav global WayPoint(x,y,z):(%g,%g,%g)",uav_id,globalWaypoints[count].latitude,globalWaypoints[count].longitude,globalWaypoints[count].altitude);

                goalPose.pose.position.x = newLocalWaypoints.poses[count].position.x;
                goalPose.pose.position.y = newLocalWaypoints.poses[count].position.y;
                goalPose.pose.position.z = newLocalWaypoints.poses[count].position.z;

            }

            if(count>=newLocalWaypoints.poses.size())
            {
                transformDoneFlag=false;
                idlePose.pose.position.x = newLocalWaypoints.poses[count-1].position.x;
                idlePose.pose.position.y = newLocalWaypoints.poses[count-1].position.y;
                idlePose.pose.position.z = newLocalWaypoints.poses[count-1].position.z;
                uavState=idle;
                std::cout<<"uav_state: IDLE "<<uavState<<std::endl;

                count=0;
                newLocalWaypoints.poses.erase(newLocalWaypoints.poses.begin(),newLocalWaypoints.poses.end());
                globalWaypoints.erase(globalWaypoints.begin(), globalWaypoints.end());
                finishedFlag.data=true;
                finished_pub.publish(finishedFlag);
                finishedFlag.data=false;
            }

        }
    }

}

void globalPoseCb(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    double lat,lon,alt;
    lat=msg->latitude;
    lon=msg->longitude;
    alt=msg->altitude;

    if(!homePoseFlag)
    {
        if(lat != 0 && lon != 0)
        {
            home_lat = lat;
            home_lon = lon;
            homePoseFlag=true;
        }

    }

    if(homePoseFlag)
    {
        geometry_msgs::Point pt;
        pt.x = home_lat;
        pt.y = home_lon;
        homeGlobalPub.publish(pt);
    }

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "navigator_node");
    ros::NodeHandle nh;
    std::cout<<"starting"<<std::endl;
    uav_id = atoi( argv[1] );
    std::cout<<"uav_id: "<<uav_id<<std::endl;

    //publishers and subscribers
    ros::Subscriber state_num_sub = nh.subscribe<std_msgs::Int64>
            ("/uav_"+boost::lexical_cast<std::string>(uav_id)+"/state_num", 10, stateNumCb);
    ros::Subscriber path_sub = nh.subscribe<nav_msgs::Path>
            ("/uav_"+boost::lexical_cast<std::string>(uav_id)+"/path", 10, navPathCb);
    homeGlobalPub = nh.advertise<geometry_msgs::Point>
            ("/uav_"+boost::lexical_cast<std::string>(uav_id)+"/global_home_position", 10);
    finished_pub = nh.advertise<std_msgs::Bool>
            ("/uav_"+boost::lexical_cast<std::string>(uav_id)+"/finished_flag", 10);


    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("/uav_"+boost::lexical_cast<std::string>(uav_id)+"/mavros/state", 10, stateCb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/uav_"+boost::lexical_cast<std::string>(uav_id)+"/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/uav_"+boost::lexical_cast<std::string>(uav_id)+"/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/uav_"+boost::lexical_cast<std::string>(uav_id)+"/mavros/set_mode");
    ros::Subscriber current_pose = nh.subscribe
            ("/uav_"+boost::lexical_cast<std::string>(uav_id)+"/mavros/local_position/pose", 1000, localPoseCb);
    ros::Subscriber global_pose = nh.subscribe
            ("/uav_"+boost::lexical_cast<std::string>(uav_id)+"/mavros/global_position/global", 1000, globalPoseCb);


    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    //initial takeoff and flag initiation (TODO: put initiation checks before doing the takeoff)
    idlePose.pose.position.x = 0;
    idlePose.pose.position.y = 0;
    idlePose.pose.position.z = 10;
    finishedFlag.data = false;
    std::cout<<"uav_state: IDLE "<<uavState<<std::endl;

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if(  (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))  )
        {
            if( (set_mode_client.call(offb_set_mode) && offb_set_mode.response.success) )
            {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        else
        {
            if( (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) )
            {
                if( (arming_client.call(arm_cmd) && arm_cmd.response.success) )
                {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }


        switch(uavState)
        {
        case idle:
            local_pos_pub.publish(idlePose);break;
        case followPath:
            //once the transform to local in terms of the home position is done
            if(transformDoneFlag)
            {

                local_pos_pub.publish(goalPose);
                finished_pub.publish(finishedFlag);
            }
            else
            {
                local_pos_pub.publish(idlePose);
                finished_pub.publish(finishedFlag);
            }
            break;
        case pick:
            std::cout<<"uav_state: PICK "<<uavState<<std::endl;
            break; //TODO: add the pick part
        case drop:
            std::cout<<"uav_state: DROP "<<uavState<<std::endl;
            break; //TODO: add the drop part
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

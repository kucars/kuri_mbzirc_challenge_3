/***************************************************************************
 *   Copyright (C) 2006 - 2016 by                                          *
 *      Tarek Taha, KURI  <tataha@tarektaha.com>                           *
 *      Randa Almadhoun   <randa.almadhoun@kustar.ac.ae>                   *
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

#include "sspp/pathplanner.h"
#include "ros/ros.h"
#include <ros/package.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <geometry_msgs/Pose.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include "geometry_msgs/PoseStamped.h"

#include <kuri_msgs/Tasks.h>
#include <kuri_msgs/NavTask.h>
#include <kuri_msgs/NavTasks.h>
#include <kuri_msgs/FollowPathAction.h>
#include <kuri_msgs/ConvertPoseAction.h>
#include <std_msgs/Float32.h>
#include <actionlib/server/simple_action_server.h>
#include <nav_msgs/Path.h>
#include <navigator.h>
#include <geo.h>

class Converter_action_server
{
public:

    Converter_action_server(std::string name) :
        actionServer(nh, name, boost::bind(&Converter_action_server::goalCB, this, _1), false),
        actionName(name)
    {
        //register the goal and feeback callbacks
        // actionServer.registerGoalCallback(boost::bind(&Converter_action_server::goalCB, this));
        actionServer.registerPreemptCallback(boost::bind(&Converter_action_server::preemptCB, this));
        uavHome1Sub = nh.subscribe("/uav_1/global_home_position", 1000, &Converter_action_server::globalUAV1Home, this);
        uavHome2Sub = nh.subscribe("/uav_2/global_home_position", 1000, &Converter_action_server::globalUAV2Home, this);
        uavHome3Sub = nh.subscribe("/uav_3/global_home_position", 1000, &Converter_action_server::globalUAV3Home, this);

        actionServer.start();
        ROS_INFO("Action Server Ready for Orders: %s", actionName.c_str());

    }

    ~Converter_action_server(void)
    {
    }

    void globalUAV1Home(const geometry_msgs::Point& msg)
    {
        uav1Home.x = msg.x;
        uav1Home.y = msg.y;
    }
    void globalUAV2Home(const geometry_msgs::Point& msg)
    {
        uav2Home.x = msg.x;
        uav2Home.y = msg.y;
    }
    void globalUAV3Home(const geometry_msgs::Point& msg)
    {
        uav3Home.x = msg.x;
        uav3Home.y = msg.y;
    }
    geometry_msgs::Pose convert2LocalRef(geometry_msgs::Pose uavLocalPose,int uav_id)
    {
        if(uav_id == 1)
            map_projection_global_init(uav1Home.x, uav1Home.y,1);
        else if(uav_id == 2)
            map_projection_global_init(uav2Home.x, uav2Home.y,1);
        else if(uav_id == 3)
            map_projection_global_init(uav3Home.x, uav3Home.y,1);

        double lat,lon;
        float alt;
        globallocalconverter_toglobal(uavLocalPose.position.y,uavLocalPose.position.x,uavLocalPose.position.z,&lat,&lon,&alt);

        double wpt_lat_ref,wpt_lon_ref;
        ros::param::param("~ref_lat", wpt_lat_ref, 47.3977419);
        ros::param::param("~ref_lon", wpt_lon_ref, 8.5455938);
        map_projection_global_init(wpt_lat_ref, wpt_lon_ref,1);

        float p_x,p_y,p_z;
        globallocalconverter_tolocal(lat,lon,alt,&p_y,&p_x,&p_z);
        geometry_msgs::Pose refUavPose;
        refUavPose.position.x = p_x;
        refUavPose.position.y = p_y;
        refUavPose.position.z = p_z;

        return refUavPose;
    }

    void goalCB(const kuri_msgs::ConvertPoseGoalConstPtr &goal)
    {
        // accept the new goal
        ROS_INFO("Accepting Goal for action %s", actionName.c_str());


        ROS_INFO("started converting");
        geometry_msgs::Pose ref1LocalPose = convert2LocalRef(goal->uav1LocalPose,1);
        geometry_msgs::Pose ref2LocalPose = convert2LocalRef(goal->uav2LocalPose,2);
        geometry_msgs::Pose ref3LocalPose = convert2LocalRef(goal->uav3LocalPose,3);

        printf(" uav 1: x %f, y %f, z %f \n\n",ref1LocalPose.position.x,ref1LocalPose.position.y,ref1LocalPose.position.z);
        printf(" uav 2: x %f, y %f, z %f \n\n",ref2LocalPose.position.x,ref2LocalPose.position.y,ref2LocalPose.position.z);
        printf(" uav 3: x %f, y %f, z %f \n\n",ref3LocalPose.position.x,ref3LocalPose.position.y,ref3LocalPose.position.z);

        result.ref1LocalPose.position = ref1LocalPose.position;
        result.ref2LocalPose.position = ref2LocalPose.position;
        result.ref3LocalPose.position = ref3LocalPose.position;


        ROS_INFO("%s: Succeeded", actionName.c_str());
        actionServer.setSucceeded(result);

    }

    void preemptCB()
    {
        ROS_INFO("%s: Preempted", actionName.c_str());
        // set the action state to preempted
        actionServer.setPreempted();
    }



protected:

    ros::NodeHandle nh;
    ros::Subscriber uavHome1Sub,uavHome2Sub,uavHome3Sub;
    geometry_msgs::Point uav1Home,uav2Home,uav3Home;
    actionlib::SimpleActionServer<kuri_msgs::ConvertPoseAction> actionServer;
    std::string actionName;
    kuri_msgs::ConvertPoseGoal goal;
    kuri_msgs::ConvertPoseResult result;
    int uav_i;

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Converter_action_server");
    Converter_action_server Converter_action_server1("Converter_action_server");


    ros::spin();
    return 0;
}

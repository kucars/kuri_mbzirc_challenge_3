#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/NavSatFix.h>
#include <actionlib/server/simple_action_server.h>
#include <occupancy_mapping.h>

class Mapping_action_server
{
public:

    Mapping_action_server(std::string name) :
        actionServer(nh, name, false),
        actionName(name)
    {
        //register the goal and feeback callbacks
        ROS_INFO("Registering callbacks for action %s", actionName.c_str());
        actionServer.registerGoalCallback(boost::bind(&Mapping_action_server::goalCB, this));
        actionServer.registerPreemptCallback(boost::bind(&Mapping_action_server::preemptCB, this));
        MapObject = new Object_mapping();
        ROS_INFO("Starting server for action %s", actionName.c_str());
        actionServer.start();
    }

    ~Mapping_action_server(void){}


    void goalCB()
    {
        progressCount = 0;
        // accept the new goal
        ROS_INFO("Accepting Goal for action %s", actionName.c_str());
        goal = actionServer.acceptNewGoal()->uav_id;
        ROS_INFO("started Mapping");
        MapObject->StoreMap(&actionServer,result);
    }

    void preemptCB()
    {
        ROS_INFO("%s: Preempted", actionName.c_str());
        // set the action state to preempted
        actionServer.setPreempted();
    }


protected:

    ros::NodeHandle nh;
    kuri_msgs::MappingFeedback feedback;
    kuri_msgs::MappingResult   result;
    kuri_msgs::MappingGoalConstPtr GoalConstPtr;
    actionlib::SimpleActionServer<kuri_msgs::MappingAction> actionServer;
    std::string actionName;
    float progressCount;
    int goal;
    Object_mapping *MapObject;

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Object_Mapping");
    Mapping_action_server Mapping_action_server("create_map");  // This added for testing
    ros::spin();
    return 0;
}


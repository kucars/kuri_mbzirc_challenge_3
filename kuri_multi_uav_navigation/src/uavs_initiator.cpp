/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state1;
mavros_msgs::State current_state2;
mavros_msgs::State current_state3;
geometry_msgs::PoseStamped pose1;
geometry_msgs::PoseStamped pose2;
geometry_msgs::PoseStamped pose3;

void state_cb1(const mavros_msgs::State::ConstPtr& msg){
    current_state1 = *msg;
}
void state_cb2(const mavros_msgs::State::ConstPtr& msg){
    current_state2 = *msg;
}
void state_cb3(const mavros_msgs::State::ConstPtr& msg){
    current_state3 = *msg;
}
void localPoseCb1(const geometry_msgs :: PoseStamped :: ConstPtr& msg){
    pose1.pose.position.x=msg ->pose.position.x;
    pose1.pose.position.y=msg ->pose.position.y;
    pose1.pose.position.z=msg ->pose.position.z;
}
void localPoseCb2(const geometry_msgs :: PoseStamped :: ConstPtr& msg){
    pose2.pose.position.x=msg ->pose.position.x;
    pose2.pose.position.y=msg ->pose.position.y;
    pose2.pose.position.z=msg ->pose.position.z;
}
void localPoseCb3(const geometry_msgs :: PoseStamped :: ConstPtr& msg){
    pose3.pose.position.x=msg ->pose.position.x;
    pose3.pose.position.y=msg ->pose.position.y;
    pose3.pose.position.z=msg ->pose.position.z;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node_ch3");
    ros::NodeHandle nh;
    
    //uav1
    ros::Subscriber state_sub1 = nh.subscribe<mavros_msgs::State>
            ("/uav_1/mavros/state", 10, state_cb1);
    ros::Publisher local_pos_pub1 = nh.advertise<geometry_msgs::PoseStamped>
            ("/uav_1/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client1 = nh.serviceClient<mavros_msgs::CommandBool>
            ("/uav_1/mavros/cmd/arming");
    ros::ServiceClient set_mode_client1 = nh.serviceClient<mavros_msgs::SetMode>
            ("/uav_1/mavros/set_mode");
    ros::Subscriber current_pose1 = nh.subscribe("/uav_1/mavros/local_position/pose", 1000, localPoseCb1);
	    
	   
    //uav2	    
    ros::Subscriber state_sub2 = nh.subscribe<mavros_msgs::State>
            ("/uav_2/mavros/state", 10, state_cb2);
    ros::Publisher local_pos_pub2 = nh.advertise<geometry_msgs::PoseStamped>
            ("/uav_2/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client2 = nh.serviceClient<mavros_msgs::CommandBool>
            ("/uav_2/mavros/cmd/arming");
    ros::ServiceClient set_mode_client2 = nh.serviceClient<mavros_msgs::SetMode>
            ("/uav_2/mavros/set_mode");
    ros::Subscriber current_pose2 = nh.subscribe("/uav_2/mavros/local_position/pose", 1000, localPoseCb2);	    
    //uav3	    
	        ros::Subscriber state_sub3 = nh.subscribe<mavros_msgs::State>
            ("/uav_3/mavros/state", 10, state_cb3);
    ros::Publisher local_pos_pub3 = nh.advertise<geometry_msgs::PoseStamped>
            ("/uav_3/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client3 = nh.serviceClient<mavros_msgs::CommandBool>
            ("/uav_3/mavros/cmd/arming");
    ros::ServiceClient set_mode_client3 = nh.serviceClient<mavros_msgs::SetMode>
            ("/uav_3/mavros/set_mode");
    ros::Subscriber current_pose3 = nh.subscribe("/uav_3/mavros/local_position/pose", 1000, localPoseCb3);	    
	    
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state1.connected && current_state2.connected && current_state3.connected){
        ros::spinOnce();
        rate.sleep();
    }


    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    int r = 0;
    while(ros::ok()){
        if(  (current_state1.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))  &&
	  (current_state2.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))) &&
	  (current_state3.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
	){
            if( (set_mode_client1.call(offb_set_mode) && offb_set_mode.response.success) && 
	      (set_mode_client2.call(offb_set_mode) && offb_set_mode.response.success)   &&
	      (set_mode_client3.call(offb_set_mode) && offb_set_mode.response.success)
	    ){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( (!current_state1.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) && 
	      (!current_state2.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) && 
	      (!current_state3.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) ){
                if( (arming_client1.call(arm_cmd) && arm_cmd.response.success) &&
		  (arming_client2.call(arm_cmd) && arm_cmd.response.success) &&
		  (arming_client3.call(arm_cmd) && arm_cmd.response.success)
		){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
//         std::cout<<"Current Mode uav1 is: "<<current_state1.mode<<"\n"; fflush(stdout);
//         std::cout<<"Current Mode uav2 is: "<<current_state2.mode<<"\n"; fflush(stdout);
//         std::cout<<"Current Mode uav3 is: "<<current_state3.mode<<"\n"; fflush(stdout);

//         local_pos_pub1.publish(pose1);
// 	   local_pos_pub2.publish(pose2);
//         local_pos_pub3.publish(pose3);


	
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

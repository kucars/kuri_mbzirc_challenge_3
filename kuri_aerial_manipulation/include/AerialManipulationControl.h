#ifndef AERIALMANIPULATIONCONTROL_H
#define AERIALMANIPULATIONCONTROL_H

#include "ros/ros.h"
#include <ros/package.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include <eigen_conversions/eigen_msg.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

#include <actionlib/server/simple_action_server.h>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
//msg headers.
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <sensor_msgs/NavSatFix.h>

#include <nav_msgs/Odometry.h>
#include <iostream>

//pcl
#include <pcl_ros/point_cloud.h>
#include <pcl/common/common_headers.h>
#include <ctime>

#include "kuri_msgs/Object.h"
#include "kuri_msgs/Objects.h"

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

class ObjectPicker
{
public:
  ObjectPicker(const ros::NodeHandle &_nh, const ros::NodeHandle &_nhPrivate);
  ~ObjectPicker(){}
  void mavrosStateCallback(const mavros_msgs::State::ConstPtr& msg);
  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void headingCallback(const std_msgs::Float64::ConstPtr& msg);

private:
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate;
  ros::Time goalLastReceived;
  ros::Publisher velPub;
  ros::Publisher pidPub;
  ros::Subscriber goalSub;
  ros::Subscriber compassSub;
  ros::ServiceClient armingClient;
  geometry_msgs ::Point real;
  geometry_msgs ::TwistStamped twist;
  geometry_msgs ::TwistStamped stopTwist;
  geometry_msgs::PoseStamped goalPose;
  mavros_msgs::State currentState;
  mavros_msgs::CommandBool armCommand;

  bool firstDataFlag;
  double roll, pitch, yaw;
  int counter;
  float w;

  double tolerance_2_goal;
  double kpx;
  double kix;
  double kdx;
  double kpy;
  double kiy;
  double kdy;
  double kpz;
  double kiz;
  double kdz;
  double kpw;
  double kiw;
  double kdw;

  float errorX;
  float errorY;
  float errorZ;
  float errorW;
  float prevErrorX;
  float prevErrorY;
  float prevErrorZ;
  float prevErrorW;
  float rise;
  float nonstop;
  float pX;
  float pY;
  float pZ;
  float pW;
  float iX;
  float iY;
  float iZ;
  float iW;
  float dX;
  float dY;
  float dZ;
  float dW;
  float aX;
  float aY;
  float aZ;
  float aW;
  bool mustExit;
  int waypointNum ;
};

class AerialManipulationControl
{
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, nav_msgs::Odometry> MySyncPolicy;

public:
    bool waitforResults (kuri_msgs::Object goal_R ); 
    AerialManipulationControl() ;
    ~AerialManipulationControl();

protected:
    ros::NodeHandle nh_;
    kuri_msgs::Object goal;
    bool flag_2 ; 
    ros::Publisher velocity_pub_;
    message_filters::Subscriber<sensor_msgs::Image> *img_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> *camera_info_sub_;
    message_filters::Subscriber<nav_msgs::Odometry> *uav_odom_sub_;
    message_filters::Synchronizer<MySyncPolicy> *sync;
    tf::Transform BaseToCamera;
};
#endif // AERIALMANIPULATIONCONTROL_H

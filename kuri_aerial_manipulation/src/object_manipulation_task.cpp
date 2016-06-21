/***************************************************************************
 *   Copyright (C) 2006 - 2016 by                                          *
 *      Tarek Taha, KURI  <tataha@tarektaha.com>                           *
 *      Randa Almadhoun   <randa.almadhoun@kustar.ac.ae>                   *
 *      Ahmed AlDhanhani  <ahmed.aldhanhani@kustar.ac.ae>                                                                   *
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

#include <nav_msgs/Odometry.h>
#include <iostream>

//pcl
#include <pcl_ros/point_cloud.h>
#include <pcl/common/common_headers.h>
#include <ctime>

#include "kuri_msgs/Object.h"
#include "kuri_msgs/Objects.h"

#include "kuri_msgs/PickObjectAction.h"

bool firstFlag = true ;
bool landing = false  ;  // true just for testing the landing step

kuri_msgs::Object CurrentObj ;

double u , v , X, Y ;
namespace enc = sensor_msgs::image_encodings;
double roll, pitch, yaw;
// cuda
float  process_in_cuda(double *_a, double *_b, double *_c, cv::Mat *dev_img, pcl::PointCloud<pcl::PointXYZRGB> *PC);

int LowerH = 110;
int LowerS = 150;
int LowerV = 150;
int UpperH = 130;
int UpperS = 255;
int UpperV = 255;

int LowerHB = 110;
int LowerSB = 150;
int LowerVB = 150;
int UpperHB = 130;
int UpperSB = 255;
int UpperVB = 255;

int LowerHR = 0;
int LowerSR = 100;
int LowerVR = 100;
int UpperHR = 20;
int UpperSR = 255;
int UpperVR = 255;

int LowerHG = 60;
int LowerSG = 100;
int LowerVG = 100;
int UpperHG = 60;
int UpperSG = 255;
int UpperVG = 255;

class AerialManipulationAction
{
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, nav_msgs::Odometry> MySyncPolicy;
public:

    AerialManipulationAction(std::string name) :
        actionServer(nh_, name, false),
        actionName(name)
    {
        //register the goal and feeback callbacks
        actionServer.registerGoalCallback(boost::bind(&AerialManipulationAction::goalCB, this));
        actionServer.registerPreemptCallback(boost::bind(&AerialManipulationAction::preemptCB, this));

	//publish pointcloud msgs:
	std::string topic1 = nh_.resolveName("imagetoground");
	pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(topic1, 1);
	std::string topic2 = nh_.resolveName("/uav_1/mavros/setpoint_velocity/cmd_vel");
	velocity_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(topic2, 1);
	std::string topic3 = nh_.resolveName("/uav_1/mavros/setpoint_position/local");
	orien_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(topic3, 1);

	img_sub_  = new message_filters::Subscriber<sensor_msgs::Image>(nh_, "/uav_1/downward_cam/camera/image", 10);
	camera_info_sub_ = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh_, "/uav_1/downward_cam/camera/camera_info", 10);
	uav_odom_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(nh_, "/uav_1/mavros/local_position/odom", 10);

	sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(3), *img_sub_, *camera_info_sub_, *uav_odom_sub_);
	sync->registerCallback(boost::bind(&AerialManipulationAction::imageCallback, this, _1, _2, _3));

	//initialize base_link to camera optical link
	BaseToCamera.setOrigin(tf::Vector3(0.0, 0.0, -0.2));
	BaseToCamera.setRotation(tf::Quaternion(0.707, -0.707, 0.000, -0.000));
	
	//subscribe to the data topic of interest
	//sub = nh_.subscribe("/random_number", 1, &AerialManipulationAction::analysisCB, this);
	std::cout << " Start the action server " << std::endl << std::flush ; 
	actionServer.start();
	ROS_INFO("Action name %s" , actionName.c_str()) ; 
    }

    //process to pointcloud
  void p2p(const sensor_msgs::ImageConstPtr& img,
           const sensor_msgs::CameraInfoConstPtr& cam_info,
           const nav_msgs::OdometryConstPtr& odom);
  
    ~AerialManipulationAction(void)
    {
    }
   /*  void paramsCallback(kuri_aerial_manipulation::ColorsParamConfig &config, uint32_t level)
	{
     	 LowerHB << config.LowerHB ; 
         LowerSB << config.LowerSB ; 
	 LowerVB << config.LowerVB ; 
	 UpperHB << config.UpperHB ;
	 UpperSB << config.UpperSB ; 
	 UpperVB << config.UpperVB ;

     	 LowerHR << config.LowerHR ; 
         LowerSR << config.LowerSR ; 
	 LowerVR << config.LowerVR ; 
	 UpperHR << config.UpperHR ;
	 UpperSR << config.UpperSR ; 
	 UpperVR << config.UpperVR ;

     	 LowerHG << config.LowerHG ; 
         LowerSG << config.LowerSG ; 
	 LowerVG << config.LowerVG ; 
	 UpperHG << config.UpperHG ;
	 UpperSG << config.UpperSG ; 
	 UpperVG << config.UpperVG ;
	}*/
    //call back, for processing
    void imageCallback(const sensor_msgs::ImageConstPtr& img,
                     const sensor_msgs::CameraInfoConstPtr& cam_info,
                     const nav_msgs::OdometryConstPtr& odom
                    )
    {
       // make sure that the action hasn't been canceled
        if (!actionServer.isActive())
            return;

        std::cout << "Server Active" << std::endl << std::flush ; 

	  cv_bridge::CvImagePtr cv_ptr;
	try
	{
	  cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
	  ROS_ERROR("cv_bridge exception: %s", e.what());
	  return;
	}
	//cv::imshow("view", cv_ptr->image);

	cv::Mat img_mask, img_hsv, combined_Image;
	cv::cvtColor(cv_ptr->image, img_hsv, CV_BGR2HSV);

	if (goal.color  == "blue")
	  cv::inRange(img_hsv, cv::Scalar(LowerHB, LowerSB, LowerVB), cv::Scalar(UpperHB, UpperSB, UpperVB), img_mask);
	else if (goal.color == "red")
	  cv::inRange(img_hsv, cv::Scalar(LowerHR, LowerSR, LowerVR), cv::Scalar(UpperHR, UpperSR, UpperVR), img_mask);
	else if (goal.color == "green")
	  cv::inRange(img_hsv, cv::Scalar(LowerHG, LowerSG, LowerVG), cv::Scalar(UpperHG, UpperSG, UpperVG), img_mask);
	else
	  return;

	ROS_INFO("Now here");
	// cv::imshow("mask", img_hsv);
	// cv::imshow("view", cv_ptr->image);
	cv::Mat locations;   // output, locations of non-zero pixels
	cv::findNonZero(img_mask, locations);

	// access pixel coordinates
	cv::Point pnt;
	int sumx = 0.0 ;
	int sumy = 0.0 ;
	int i ;
	for (i = 0 ; i < locations.total() ; i++)
	{
	  pnt = locations.at<cv::Point>(i);
	  sumx = sumx + pnt.x ;
	  sumy = sumy + pnt.y ;
	}
	if (locations.total() > 0)
	{
	  ROS_INFO("Found Object ");
	  u = sumx / locations.total();
	  v = sumy / locations.total() ;

	}
	else
	{
	  ROS_INFO("No Object ");

	}

	  p2p(img, cam_info, odom
);

            
       

    }



    void goalCB()
    {
        progressCount = 0;
        // accept the new goal
        goal = actionServer.acceptNewGoal()->object_2_pick;
    }

    void preemptCB()
    {
        ROS_INFO("%s: Preempted", actionName.c_str());
        // set the action state to preempted
        actionServer.setPreempted();
    }

   
protected:

    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<kuri_msgs::PickObjectAction> actionServer;
    std::string actionName;
    float progressCount;
    kuri_msgs::Object goal;
    kuri_msgs::PickObjectFeedback feedback;
    kuri_msgs::PickObjectResult   result;

    ros::Subscriber sub;
    ros::Publisher pointcloud_pub_;
    ros::Publisher velocity_pub_;
    ros::Publisher orien_pub_;
    ros::Subscriber object_to_pick ;
    
    geometry_msgs::Pose currentPose;

    message_filters::Subscriber<sensor_msgs::Image> *img_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> *camera_info_sub_;
    message_filters::Subscriber<nav_msgs::Odometry> *uav_odom_sub_;
    message_filters::Synchronizer<MySyncPolicy> *sync;
    tf::Transform BaseToCamera;
    //int LowerH , LowerS , LowerV, UpperH, UpperS , UpperV ; 
    //int LowerHB, LowerSB,LowerVB, UpperHB, UpperSB , UpperVB ;
    //int LowerHR ,LowerSR , LowerVR , UpperHR , UpperSR , UpperVR ;
    //int LowerHG ,LowerSG , LowerVG , UpperHG , UpperSG , UpperVG ;

#define Ground_Z 0.0
  //test
  tf::TransformBroadcaster br;
};

void AerialManipulationAction::p2p(const sensor_msgs::ImageConstPtr& img,
                             const sensor_msgs::CameraInfoConstPtr& cam_info,
                             const nav_msgs::OdometryConstPtr& odom)
{


  tf::Transform extrisic;
  cv::Mat P(3, 4, CV_64FC1);
  cv::Mat P_Mat_G(3, 4, CV_64FC1);
  tf::Pose tfpose;
  tfScalar extrisic_data[4 * 4];
  pcl::PointCloud<pcl::PointXYZRGB> Pointcloud;
  Pointcloud.header.frame_id = "/world";
  Pointcloud.height = img->height;
  Pointcloud.width = img->width;
  Pointcloud.resize(img->height * img->width);
  Pointcloud.is_dense = true;
  cv::Mat cvimg = cv_bridge::toCvShare(img, "bgr8")->image.clone();
  tf::poseMsgToTF(odom->pose.pose, tfpose);
  extrisic = BaseToCamera * tfpose.inverse();
  //to test if the tf is correct, create testframe_to_camera
  //br.sendTransform(tf::StampedTransform(extrisic, ros::Time::now(), "/testframe_to_camera", "/world"));
  //pinv of projection matrix...
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 4; j++)
    {
      P.at<double>(i, j) = cam_info->P.at(i * 4 + j);
      //  std::cout << "PP" << P  << std::endl ;
    }
  //however, this P is in camera coordinate..
  extrisic.getOpenGLMatrix(extrisic_data);
  cv::Mat E_MAT(4, 4, CV_64FC1, extrisic_data);
  P_Mat_G = P * (E_MAT.t());
  // now is the ground, namely, world coordinate
  double a[4], b[4], c[4];
  a[0] = P_Mat_G.at<double>(0, 0);
  a[1] = P_Mat_G.at<double>(0, 1);
  a[2] = P_Mat_G.at<double>(0, 2);
  a[3] = P_Mat_G.at<double>(0, 3);
  b[0] = P_Mat_G.at<double>(1, 0);
  b[1] = P_Mat_G.at<double>(1, 1);
  b[2] = P_Mat_G.at<double>(1, 2);
  b[3] = P_Mat_G.at<double>(1, 3);
  c[0] = P_Mat_G.at<double>(2, 0);
  c[1] = P_Mat_G.at<double>(2, 1);
  c[2] = P_Mat_G.at<double>(2, 2);
  c[3] = P_Mat_G.at<double>(2, 3);
  std::clock_t start;
  double duration;
  start = std::clock();

  //gpu
#if defined(GPU_EN)
  process_in_cuda(a, b, c, &cvimg, &Pointcloud);
#else

  // ************************** find 3D point ******************** //

  // just for the detected obstacle
  float B[2][2], bvu[2];
  B[0][0] = u * c[0] - a[0];
  B[0][1] = u * c[1] - a[1];
  B[1][0] = v * c[0] - b[0];
  B[1][1] = v * c[1] - b[1];
  bvu[0] = a[2] * Ground_Z + a[3] - u * c[2] * Ground_Z - u * c[3];
  bvu[1] = b[2] * Ground_Z + b[3] - v * c[2] * Ground_Z - v * c[3];
  float DomB = B[1][1] * B[0][0] - B[0][1] * B[1][0];
  float objectPoseX = (B[1][1] * bvu[0] - B[0][1] * bvu[1]) / DomB ;
  float objectPoseY = (B[0][0] * bvu[1] - B[1][0] * bvu[0]) / DomB ;
  //      float temp = objectPoseX ;
  //    objectPoseX = -1 * objectPoseY ;
  //     objectPoseY = temp ;
  std::cout << "****new msg **** " << std::endl << std::flush ;
  std::cout << "u      " << u  << "   v:  " << v  << std::endl << std::flush ;

  std::cout << "Object "  << objectPoseX  << "\t" <<   objectPoseY << std::endl  << std::flush ;
  std::cout << "uav_x  " << odom->pose.pose.position.x  << "  uavy: " << odom->pose.pose.position.y  << std::endl << std::flush ;
  std::cout << "errorx " << odom->pose.pose.position.x - objectPoseX  << " errory " << odom->pose.pose.position.y - objectPoseY  << std::endl << std::flush ;

  tf::Quaternion q(odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z, odom->pose.pose.orientation.w);
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

  // *************************** Controller ********************* //
  float distance = sqrt(((odom->pose.pose.position.x - objectPoseX) * (odom->pose.pose.position.x - objectPoseX)) + ((odom->pose.pose.position.y - objectPoseY) * (odom->pose.pose.position.y - objectPoseY)));
  float vector3x = objectPoseX - odom->pose.pose.position.x ;
  float vector3y = objectPoseY - odom->pose.pose.position.y ;
  float vector3z = odom->pose.pose.position.z ;
  
  // Test 1 
  float vector3Mag1 = sqrt ((vector3x*vector3x) + (vector3y*vector3y) ) ;
  float unitVector3x = vector3x/vector3Mag1 ; 
  float unitVector3y = vector3y/vector3Mag1 ; 
  
  // Test 2
  float vector3Mag2 = sqrt ((vector3x*vector3x) + (vector3y*vector3y) + (vector3z*vector3z)) ;
  float T_unitVector3x = vector3x/vector3Mag2 ;
  float T_unitVector3y = vector3y/vector3Mag2 ; 
  float T_unitVector3z = vector3z/vector3Mag2 ; 
  // Test 2
  geometry_msgs::TwistStamped new_cmd_vel ;
  std::cout << "Pose in Z" << odom->pose.pose.position.z  << std::endl << std::flush ;
  if ( distance < 0.1 && odom->pose.pose.position.z  < 0.4 )
     {
      std::cout << "Landed" << std::endl << std::flush ;
      // landing = false ;
      new_cmd_vel.twist.linear.x = 0 ;
      new_cmd_vel.twist.linear.y = 0 ;
      new_cmd_vel.twist.linear.z = 0.0 ;
      velocity_pub_.publish(new_cmd_vel) ;
      result.success = true ; 
      actionServer.setSucceeded(result);


     }
     else 
     {
      std::cout << "It shoud move" << std::endl << std::flush ;
      new_cmd_vel.twist.linear.x = T_unitVector3x * 0.5 ;
      new_cmd_vel.twist.linear.y = T_unitVector3y * 0.5;
      new_cmd_vel.twist.linear.z = T_unitVector3z * -0.2 ;
      velocity_pub_.publish(new_cmd_vel) ;
       
     }



#endif

  //get the duration....
  duration = (std::clock() - start) / (double) CLOCKS_PER_SEC;
  std::cout << "process_time is " << duration << " second" << '\n';
  //publish pointcloud

}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "aerialAction");
    AerialManipulationAction planGeneration(ros::this_node::getName());
    ros::spin();
    return 0;
}


  /*
  jsk_mbzirc_task
  */

  // Author: Chen

  #include <ros/ros.h>
  #include <image_transport/image_transport.h>
  #include <opencv2/highgui/highgui.hpp>
  #include <cv_bridge/cv_bridge.h>
  #include <message_filters/subscriber.h>
  #include <message_filters/synchronizer.h>
  #include <message_filters/time_synchronizer.h>
  #include <message_filters/sync_policies/approximate_time.h>
  #include <tf/tf.h>
  //msg headers.
  #include <sensor_msgs/Image.h>
  #include <sensor_msgs/CameraInfo.h>
  #include <sensor_msgs/PointCloud2.h>
  #include <jsk_recognition_msgs/VectorArray.h>
  #include <nav_msgs/Odometry.h>
  #include <tf/transform_listener.h>
  #include <geometry_msgs/Pose.h>
  #include <geometry_msgs/Twist.h>
  #include <std_msgs/Float64.h>
  #include <iostream>
  //for test
  #include <tf/transform_broadcaster.h>
  //pcl
  #include <pcl_ros/point_cloud.h>
  #include <pcl/common/common_headers.h>
  #include <ctime>
  #include <tf/transform_datatypes.h>
  #include "geometry_msgs/TwistStamped.h"
  #include "geometry_msgs/PoseStamped.h"
  #include "kuri_msgs/Object.h"
  #include "kuri_msgs/Objects.h"


    int LowerH = 110;
    int LowerS = 150;
    int LowerV = 150;
    int UpperH = 130;
    int UpperS = 255;
    int UpperV = 255;
    double u ,v ,X,Y ; 
    namespace enc = sensor_msgs::image_encodings;
    double roll, pitch, yaw;
    // cuda
    float  process_in_cuda(double *_a, double *_b, double *_c, cv::Mat *dev_img, pcl::PointCloud<pcl::PointXYZRGB> *PC);

  class uav_img2pointcloud
  {
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo,nav_msgs::Odometry,kuri_msgs::Object> MySyncPolicy;

  private:
      ros::Publisher pointcloud_pub_;
      ros::Publisher velocity_pub_;
      ros::Publisher orien_pub_;
     // ros::Subscriber object_to_pick ; 

      ros::Publisher param_matrix_pub_;
      sensor_msgs::PointCloud2 cloud_msg;
      message_filters::Subscriber<sensor_msgs::Image> *img_sub_;
      message_filters::Subscriber<sensor_msgs::CameraInfo> *camera_info_sub_;
      message_filters::Subscriber<nav_msgs::Odometry> *uav_odom_sub_;
      message_filters::Subscriber<kuri_msgs::Object> *object_to_pick;

      ros::NodeHandle nh_;
      message_filters::Synchronizer<MySyncPolicy> *sync;
      tf::Transform BaseToCamera;
      kuri_msgs::Object CurrentObj ; 
      
  #define Ground_Z 0.0
    //test
      tf::TransformBroadcaster br;
  public:
      void init()
      {
	  //publish pointcloud msgs:
	  std::string topic = nh_.resolveName("imagetoground");
	  pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(topic, 1);
	  std::string topic3 = nh_.resolveName("/uav_1/mavros/setpoint_velocity/cmd_vel");
	  velocity_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(topic3, 1);	  
	  std::string topic2 = nh_.resolveName("paramatrix");
	  param_matrix_pub_ = nh_.advertise<jsk_recognition_msgs::VectorArray>(topic2,1);
	  std::string topic4 = nh_.resolveName("/uav_1/mavros/setpoint_position/local");
	  orien_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(topic4, 1);
	  //for test the image
	  cv::namedWindow("view");
	  cv::startWindowThread();
	  img_sub_  = new message_filters::Subscriber<sensor_msgs::Image>(nh_,"/uav_1/downward_cam/camera/image",10);
	  camera_info_sub_ = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh_,"/uav_1/downward_cam/camera/camera_info", 10);
	  uav_odom_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(nh_,"/uav_1/mavros/local_position/odom",10);
	  object_to_pick = new message_filters::Subscriber<kuri_msgs::Object>(nh_,"/ObjectToPick",10);

	 // object_to_pick = nh_.Subscriber("ObjectToPick", 1000, &uav_img2pointcloud::objectToPickCallback,this );

	  sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(4), *img_sub_, *camera_info_sub_, *uav_odom_sub_,*object_to_pick);
	  sync->registerCallback(boost::bind(&uav_img2pointcloud::imageCallback,this,_1,_2,_3,_4));
	
	  /*
	  if(!nh_.getParam("enableGPU",GPUFLAG))
	      std::cout<<"fail to load the param enableGPU, Using CPU instead"<<std::endl;
	  else
	      std::cout<<"With GPU support flag = " << GPUFLAG<<std::endl;
	      */
	  //initialize base_link to camera optical link
	  BaseToCamera.setOrigin(tf::Vector3(0.0,0.0,-0.2));
	  BaseToCamera.setRotation(tf::Quaternion(0.707, -0.707, 0.000, -0.000));

      }
//      void objectToPickCallback(const kuri_msgs::ObjectConstPtr& obj) ;

      //call back, for processing
      void imageCallback(const sensor_msgs::ImageConstPtr& img,
			const sensor_msgs::CameraInfoConstPtr& cam_info,
			const nav_msgs::OdometryConstPtr& odom,
			 const kuri_msgs::ObjectConstPtr& obj);
      //process to pointcloud
      void p2p(const sensor_msgs::ImageConstPtr& img,
	      const sensor_msgs::CameraInfoConstPtr& cam_info,
	      const nav_msgs::OdometryConstPtr& odom,
	      const kuri_msgs::ObjectConstPtr& obj);
      // class's family...
      ~uav_img2pointcloud()
      {    }
  };

  
  

  void uav_img2pointcloud::p2p(const sensor_msgs::ImageConstPtr& img,
			      const sensor_msgs::CameraInfoConstPtr& cam_info,
			      const nav_msgs::OdometryConstPtr& odom,
			     const kuri_msgs::ObjectConstPtr& obj)
  {

      tf::Transform extrisic;
      cv::Mat P(3,4,CV_64FC1);
      cv::Mat P_Mat_G(3,4,CV_64FC1);
      tf::Pose tfpose;
      tfScalar extrisic_data[4*4];
      pcl::PointCloud<pcl::PointXYZRGB> Pointcloud;
      jsk_recognition_msgs::VectorArray param_vector;
      Pointcloud.header.frame_id = "/world";
      Pointcloud.height = img->height; Pointcloud.width = img->width;
      Pointcloud.resize(img->height*img->width);
      Pointcloud.is_dense = true;
      cv::Mat cvimg = cv_bridge::toCvShare(img,"bgr8")->image.clone();
      tf::poseMsgToTF(odom->pose.pose,tfpose);
      extrisic = BaseToCamera*tfpose.inverse();
      //to test if the tf is correct, create testframe_to_camera
      //br.sendTransform(tf::StampedTransform(extrisic, ros::Time::now(), "/testframe_to_camera", "/world"));
      //pinv of projection matrix...
      for(int i = 0; i < 3; i++)
	  for(int j = 0; j < 4; j++)
	  {
	      P.at<double>(i,j) = cam_info->P.at(i*4+j);
	    //  std::cout << "PP" << P  << std::endl ;
	  }
      //however, this P is in camera coordinate..
      extrisic.getOpenGLMatrix(extrisic_data);
      cv::Mat E_MAT(4,4,CV_64FC1,extrisic_data);
      P_Mat_G = P*(E_MAT.t());
      // now is the ground, namely, world coordinate
      double a[4],b[4],c[4];
      a[0] = P_Mat_G.at<double>(0,0); a[1] = P_Mat_G.at<double>(0,1); a[2] = P_Mat_G.at<double>(0,2); a[3] = P_Mat_G.at<double>(0,3);
      b[0] = P_Mat_G.at<double>(1,0); b[1] = P_Mat_G.at<double>(1,1); b[2] = P_Mat_G.at<double>(1,2); b[3] = P_Mat_G.at<double>(1,3);
      c[0] = P_Mat_G.at<double>(2,0); c[1] = P_Mat_G.at<double>(2,1); c[2] = P_Mat_G.at<double>(2,2); c[3] = P_Mat_G.at<double>(2,3);
      std::clock_t start;
      double duration;
      start = std::clock();

      //gpu
  #if defined(GPU_EN)
      process_in_cuda(a, b, c, &cvimg, &Pointcloud);
  #else
  
	   
	      // just for the detected obstacle  
	      float B[2][2],bvu[2];
	      B[0][0] = v*c[0] - a[0]; B[0][1] = v*c[1] - a[1];
	      B[1][0] = u*c[0] - b[0]; B[1][1] = u*c[1] - b[1];
	      bvu[0]= a[2]*Ground_Z + a[3] - v*c[2]*Ground_Z - v*c[3];
	      bvu[1] = b[2]*Ground_Z + b[3] - u*c[2]*Ground_Z - u*c[3];
	      float DomB = B[1][1]*B[0][0]-B[0][1]*B[1][0];
	      
	    //  Pointcloud.points[u*Pointcloud.width+v].x = (B[1][1]*bvu[0]-B[0][1]*bvu[1])/DomB;
	     // Pointcloud.points[u*Pointcloud.width+v].y = (B[0][0]*bvu[1]-B[1][0]*bvu[0])/DomB;
	    //  Pointcloud.points[u*Pointcloud.width+v].z = (float)Ground_Z;

	      
	   
	     // std::cout << "U: " << u << std::endl ; 
	    //  std::cout << "V: " << v << std::endl ; 
	   //   std::cout << "A[0][0]: " << B[0][0] << std::endl ; 
	   //   std::cout << "A[0][1]: " << B[0][1] << std::endl ; 
	   //   std::cout << "A[1][0]: " << B[1][0] << std::endl ; 
	   //   std::cout << "A[1][1]: " << B[1][1] << std::endl ; 

	      std::cout << "Object"  <<  (B[1][1]*bvu[0]-B[0][1]*bvu[1])/DomB << "\t" <<   (B[0][0]*bvu[1]-B[1][0]*bvu[0])/DomB << std::endl ; 

	      double xold = (B[1][1]*bvu[0]-B[0][1]*bvu[1])/DomB ; 
	      double yold =  (B[0][0]*bvu[1]-B[1][0]*bvu[0])/DomB ; 
     	      double xnew =  -1 * yold ; 
	      double ynew = xold ; 
	      
	            
	      tf::Quaternion q( odom->pose.pose.orientation.x,odom->pose.pose.orientation.y,odom->pose.pose.orientation.z,odom->pose.pose.orientation.w);
              tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
	      
     	     /* geometry_msgs::PoseStamped orin ;
	      orin.pose.position.x = odom->pose.pose.position.x; 
	      orin.pose.position.y = odom->pose.pose.position.y ; 
	      orin.pose.posros::NodeHandleition.z = odom->pose.pose.position.z ; 
	      orin.pose.orientation.x = 1.40 ; 
	      orin.pose.orientation.y = 1.40 ; 
	      orin.pose.orientation.z = 0.0  ; 
	      orin.pose.orientation.w = 1.0  ;
	      orien_pub_.publish(orin); 
	      ros::Duration(5.0).sleep();*/
	     
	     //bool xu=false , xd=false ,yr=false,yl=false ; 
	     geometry_msgs::TwistStamped vel ;
	     float theta3 = atan(-yold/xold) ; 
	     float theta1 = yaw ; 

	    /*  if(xold >   odom->pose.pose.position.x ) 
		xu = true ; 
	        
	      switch(xu):
	      {
		case 0:
		  //down
		  if(yold >  odom->pose.pose.position.y)
		    vel.twist.linear.x = 0.5 * cos(theta3)  ; 
		  else
		    vel.twist.linear.x = 0.5 * cos(theta3)  ;    
		  break ; 
		case 1: 
		    if(yold >  odom->pose.pose.position.y)
		    vel.twist.linear.x = 0.5 * cos(theta3)  ; 
		  else
		    vel.twist.linear.x = 0.5 * cos(theta3)  ;    
		  break ; 
		default: 
	      }
*/
             float xobject =  odom->pose.pose.position.x - xold; float yobject =  odom->pose.pose.position.y -yold  ; 
	    float normv= sqrt(xobject*xobject + yobject*yobject) ; 
	    float normx = xobject / normv ; 
	    float normy = yobject /normv ; 
	      
	   
	    
	    if( normv > 1.2 )
	    {
	      	      // just for the detected obstacle  
	      float B[2][2],bvu[2];
	      B[0][0] = v*c[0] - a[0]; B[0][1] = v*c[1] - a[1];
	      B[1][0] = u*c[0] - b[0]; B[1][1] = u*c[1] - b[1];
	      bvu[0]= a[2]*Ground_Z + a[3] - v*c[2]*Ground_Z - v*c[3];
	      bvu[1] = b[2]*Ground_Z + b[3] - u*c[2]*Ground_Z - u*c[3];
	      float DomB = B[1][1]*B[0][0]-B[0][1]*B[1][0];
	      
	    //  Pointcloud.points[u*Pointcloud.width+v].x = (B[1][1]*bvu[0]-B[0][1]*bvu[1])/DomB;
	     // Pointcloud.points[u*Pointcloud.width+v].y = (B[0][0]*bvu[1]-B[1][0]*bvu[0])/DomB;
	    //  Pointcloud.points[u*Pointcloud.width+v].z = (float)Ground_Z;

	      
	   
	     // std::cout << "U: " << u << std::endl ; 
	    //  std::cout << "V: " << v << std::endl ; 
	   //   std::cout << "A[0][0]: " << B[0][0] << std::endl ; 
	   //   std::cout << "A[0][1]: " << B[0][1] << std::endl ; 
	   //   std::cout << "A[1][0]: " << B[1][0] << std::endl ; 
	   //   std::cout << "A[1][1]: " << B[1][1] << std::endl ; 

	//************************ Object from msgs ********************************* // 
    CurrentObj.pose.pose.position.x	= obj->pose.pose.position.x ; 
    CurrentObj.pose.pose.position.y 	= obj->pose.pose.position.y ; 
    CurrentObj.pose.pose.position.z	= obj->pose.pose.position.z ; 
    CurrentObj.pose.pose.orientation.x	= obj->pose.pose.orientation.x ; 
    CurrentObj.pose.pose.orientation.y	= obj->pose.pose.orientation.y ; 
    CurrentObj.pose.pose.orientation.z	= obj->pose.pose.orientation.z ; 
    CurrentObj.pose.pose.orientation.w	= obj->pose.pose.orientation.w ; 
    CurrentObj.velocity.linear.x	= obj->velocity.linear.x;
    CurrentObj.velocity.linear.y 	= obj->velocity.linear.y;
    CurrentObj.velocity.linear.z 	= obj->velocity.linear.z;
    CurrentObj.width 			= obj->width ; 
    CurrentObj.height 			= obj->height ; 
    CurrentObj.color 			= obj->color ;    
	// ************************************************************************** // 
	      
	   
    // From Image (transform from 2D to 3D assuming Z = 0 ) 
    std::cout << "Object"  <<  (B[1][1]*bvu[0]-B[0][1]*bvu[1])/DomB << "\t" <<   (B[0][0]*bvu[1]-B[1][0]*bvu[0])/DomB << std::endl ; 
    double xold = (B[1][1]*bvu[0]-B[0][1]*bvu[1])/DomB ; 
    double yold =  (B[0][0]*bvu[1]-B[1][0]*bvu[0])/DomB ; 
    //  double xnew =  -1 * yold ; 
    //	double ynew = xold ; 
	      
	    float xobject =  odom->pose.pose.position.x - xold; float yobject =  odom->pose.pose.position.y -yold  ; 
	    float normv= sqrt(xobject*xobject + yobject*yobject) ; 
	    float normx = xobject / normv ; 
	    float normy = yobject /normv ; 
	    std::cout << "NORMv " << normv << std::endl ; 

      	    geometry_msgs::PoseStamped pose2 ;

	    pose2.pose.position.x = yold;
	    pose2.pose.position.y = xold;
	    pose2.pose.position.z = odom->pose.pose.position.z;

	    //normx = xobject / normv ; 
	    //normy = yobject /normv ; 
	    orien_pub_.publish(pose2) ;
	    //vel.twist.linear.x = 0.5 * normx ; // - 0.5 * sin(theta3); 
	    //vel.twist.linear.y = 0.5 * normy ; // +  0.5*cos(theta3) ; 
            //velocity_pub_.publish(vel) ; 
	    //xobject =  odom->pose.pose.position.x - xold; float yobject =  odom->pose.pose.position.y -yold  ; 
	    //normv= sqrt(xobject*xobject + yobject*yobject) ;
	      
	    }
	    else if (normv < 1.2) 
	    {
	    std::cout << "normv " << normv << std::endl ; 

	    geometry_msgs::TwistStamped vel2 ;
	    vel2.twist.linear.x = 0    ; // - 0.5 * sin(theta3); 
	    vel2.twist.linear.y = 0    ; // +  0.5*cos(theta3) ; 
	    vel2.twist.linear.y = -0.2 ; // +  0.5*cos(theta3) ; 
  
	    velocity_pub_.publish(vel2) ; 
	    }
	      //geometry_msgs::TwistStamped new_cmd_vel ; 
	      //new_cmd_vel.twist.linear.x = xnew * cos(yaw) - ynew *sin(yaw) ; 
	      //new_cmd_vel.twist.linear.y = xnew * sin(yaw) + ynew *cos(yaw) ; 
	      //new_cmd_vel.twist.linear.x = xold * cos(yaw) - yold *sin(yaw) ; 
	      //new_cmd_vel.twist.linear.y = xold * sin(yaw) + yold *cos(yaw) ; 
	      
     	      
	
	      
	      
	      
	/*      
	      std::cout << "uav_odom: "  << odom->pose.pose.position.x  << "\t" << odom->pose.pose.position.y << std::endl  ; 

	      std::cout << "cmd_vel new: " <<  xnew * cos(yaw) - ynew *sin(yaw) <<  "\t" << xnew * sin(yaw) + ynew *cos(yaw) << std::endl ;
	      std::cout << "cmd_vel old: " << xold * cos(yaw) - yold *sin(yaw) <<  "\t" << xnew * sin(yaw) + ynew *cos(yaw) << std::endl ; 
	      std::cout << "old=  " << xold  <<  "\t" << yold << std::endl ; 
	      std::cout << "new=  " << xnew  <<  "\t" << ynew << std::endl ; 

	      std::cout << "YAW : " << yaw  << "Rad : " << yaw * 180.0 / 3.14 <<  std::endl ; 
	      std::cout << "YAW : " << yaw  << "deg : " << yaw * 3.14 /  180.0 <<  std::endl ; 
	      double theta = atan((ynew - odom->pose.pose.position.y) / (xnew - odom->pose.pose.position.x)) ;
	      double theta1 = atan(odom->pose.pose.position.y / odom->pose.pose.position.x) ;

	      std::cout << "Theta: " << theta << "\t" << theta * 180.0 / 3.14 << std::endl ; 
	      std::cout << "Theta1: " << theta1 << "\t" << theta1 * 180.0 / 3.14 << std::endl ; 
	      
	      std::cout << "cmd_vel new with theta: " <<  xnew * cos(theta1 + theta ) - ynew *sin(theta1 + theta) <<  "\t" << xnew * sin(theta1 + theta) + ynew *cos(theta1+theta) << std::endl ;
	     // std::cout << "cmd_vel old with theta: " << xold * cos(yaw) - yold *sin(yaw) <<  "\t" << xnew * sin(yaw) + ynew *cos(yaw) << std::endl ; 

	      std::cout << "vx " <<  cos(theta + theta1) <<  "\t" << "vy " << sin(theta + theta1)  << std::endl ;
*/




	   
	   
  #endif
      //get the duration....
      duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
      std::cout<<"process_time is "<< duration << " second" <<'\n';
      //publish pointcloud
    
  }

  
  
  
  
  


  void uav_img2pointcloud::imageCallback(const sensor_msgs::ImageConstPtr& img,
					const sensor_msgs::CameraInfoConstPtr& cam_info,
					const nav_msgs::OdometryConstPtr& odom,
					const kuri_msgs::ObjectConstPtr& obj)
  {
    
    
    
    
    
      std::cout<< "imageCallback" << std::endl ; 
      try
      {
	  cv::imshow("view", cv_bridge::toCvShare(img,"bgr8")->image);
	  cv_bridge::CvImagePtr cv_ptr;
	try
	{
	    //Always copy, returning a mutable CvImage
	    //OpenCV expects color images to use BGR channel order.
	    cv_ptr = cv_bridge::toCvCopy(img, enc::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
	    //if there is an error during conversion, display it
	    ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
	    return;
	}
    

	cv::Mat img_mask_blue,img_hsv_blue,combined_Image; 
	cv::cvtColor(cv_ptr->image,img_hsv_blue,CV_BGR2HSV);
	cv::inRange(img_hsv_blue,cv::Scalar(LowerH,LowerS,LowerV),cv::Scalar(UpperH,UpperS,UpperV),img_mask_blue); 
    
	cv::Mat locations;   // output, locations of non-zero pixels
	cv::findNonZero(img_mask_blue, locations);
	
    // access pixel coordinates
	cv::Point pnt; 
	double sumx = 0.0 ;  
	double sumy= 0.0 ; 
	for (int i = 0 ; i < locations.total() ; i++) 
	      {
	    pnt = locations.at<cv::Point>(i); 
	    sumx = sumx + pnt.x ; 
	    sumy= sumy + pnt.y ; 
	      }
  //#########################################
	  u = sumx /locations.total(); 
	  v = sumy / locations.total() ; 
	  
	  //process to pointcloud
	  p2p(img,cam_info,odom,obj);
	  cv::waitKey(10);
      }

      catch (cv_bridge::Exception& e)
      {
	  ROS_ERROR("Could not convert from '%s' to 'bgr8'.", img->encoding.c_str());
      }
  }

  int main(int argc, char **argv)
  {
      ros::init(argc, argv, "image_to_pointcloud");
      uav_img2pointcloud u_i2p;
      u_i2p.init();

      ros::spin();
      cv::destroyWindow("view");
  }
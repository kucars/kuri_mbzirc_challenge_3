#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

// openCV libraries 
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>



//Store all constants for image encodings in the enc namespace to be used later.
namespace enc = sensor_msgs::image_encodings;
 
//Declare a string with the name of the window that we will create using OpenCV where processed images will be displayed.
static const char WINDOW[] = "Image Processed";
 
//Use method of ImageTransport to create image publisher
image_transport::Publisher pub;
 


int LowerH = 110;
int LowerS = 150;
int LowerV = 150;
int UpperH = 130;
int UpperS = 255;
int UpperV = 255;
//This function is called everytime a new image is published
void imageCallback(const sensor_msgs::ImageConstPtr& original_image)
{
    //Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        //Always copy, returning a mutable CvImage
        //OpenCV expects color images to use BGR channel order.
        cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        //if there is an error during conversion, display it
        ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
        return;
    }
 
 /*   //Invert Image
    //Go through all the rows
    for(int i=0; i<cv_ptr->image.rows; i++)
    {
        //Go through all the columns
        for(int j=0; j<cv_ptr->image.cols; j++)
        {
            //Go through all the channels (b, g, r)
            for(int k=0; k<cv_ptr->image.channels(); k++)
            {
                //Invert the image by subtracting image data from 255               
                cv_ptr->image.data[i*cv_ptr->image.rows*4+j*3 + k] = 255-cv_ptr->image.data[i*cv_ptr->image.rows*4+j*3 + k];
            }
        }
    }
   */ 
     cv::Mat img_mask_blue,img_hsv_blue,combined_Image; 
     cv::cvtColor(cv_ptr->image,img_hsv_blue,CV_BGR2HSV);
    cv::inRange(img_hsv_blue,cv::Scalar(LowerH,LowerS,LowerV),cv::Scalar(UpperH,UpperS,UpperV),img_mask_blue); 
   
     cv::Mat img_mask_red,img_hsv_red; 
     cv::cvtColor(cv_ptr->image,img_hsv_red,CV_BGR2HSV);
     cv::inRange(img_hsv_red,cv::Scalar(17, 15, 100), cv::Scalar(10, 255, 255),img_mask_red); 
	
    cv::addWeighted ( img_mask_red, 1, img_mask_blue, 1, 0.0, combined_Image);
     
    // cv::Mat img_mask_green,img_hsv_green; 
    /// cv::cvtColor(cv_ptr->image,img_hsv_green,CV_BGR2HSV);
    // cv::inRange(img_hsv_green,cv::Scalar(LowerH,LowerS,LowerV),cv::Scalar(UpperH,UpperS,UpperV),img_mask_green); 
   
     
     
     //Display the image using OpenCV
     cv::imshow(WINDOW, combined_Image);
     //cv::imshow(WINDOW, img_mask_red);
    // cv::imshow(WINDOW, img_mask_green);

     //Add some delay in miliseconds. The function only works if there is at least one HighGUI window created and the window is active. If there are several HighGUI windows, any of them can be active.
     cv::waitKey(3);
     //Calculate the moments of the thresholded image
   

 
         //Convert the CvImage to a ROS image message and publish it on the "camera/image_processed" topic.
    pub.publish(cv_ptr->toImageMsg());
 
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "image_processor");
  ros::NodeHandle n;
  //Create an ImageTransport instance, initializing it with our NodeHandle.
   image_transport::ImageTransport it(n);
   
   
	cv::namedWindow("Objects");
	cv::createTrackbar("LowerH","Ball",&LowerH,180,NULL);
	cv::createTrackbar("UpperH","Ball",&UpperH,180,NULL);
	cv::createTrackbar("LowerS","Ball",&LowerS,256,NULL);
	cv::createTrackbar("UpperS","Ball",&UpperS,256,NULL);
	cv::createTrackbar("LowerV","Ball",&LowerV,256,NULL);
	cv::createTrackbar("UpperV","Ball",&UpperV,256,NULL);

   
   
   //OpenCV HighGUI call to create a display window on start-up.
   cv::namedWindow(WINDOW, CV_WINDOW_AUTOSIZE);
    /**
    * Subscribe to the "camera/image_raw" base topic. The actual ROS topic subscribed to depends on which transport is used. 
    * In the default case, "raw" transport, the topic is in fact "camera/image_raw" with type sensor_msgs/Image. ROS will call 
    * the "imageCallback" function whenever a new image arrives. The 2nd argument is the queue size.
    * subscribe() returns an image_transport::Subscriber object, that you must hold on to until you want to unsubscribe. 
    * When the Subscriber object is destructed, it will automatically unsubscribe from the "camera/image_raw" base topic.
    */
    image_transport::Subscriber sub = it.subscribe("/uav_1/downward_cam/camera/image", 1, imageCallback);
    //OpenCV HighGUI call to destroy a display window on shut-down.
    cv::destroyWindow(WINDOW);
    /**
    * The advertise() function is how you tell ROS that you want to
    * publish on a given topic name. This invokes a call to the ROS
    * master node, which keeps a registry of who is publishing and who
    * is subscribing. After this advertise() call is made, the master
    * node will notify anyone who is trying to subscribe to this topic name,
    * and they will in turn negotiate a peer-to-peer connection with this
    * node.  advertise() returns a Publisher object which allows you to
    * publish messages on that topic through a call to publish().  Once
    * all copies of the returned Publisher object are destroyed, the topic
    * will be automatically unadvertised.
    *
    * The second parameter to advertise() is the size of the message queue
    * used for publishing messages.  If messages are published more quickly
    * than we can send them, the number here specifies how many messages to
    * buffer up before throwing some away.
    */
    pub = it.advertise("camera/image_processed", 1);
    /**
    * In this application all user callbacks will be called from within the ros::spin() call. 
    * ros::spin() will not return until the node has been shutdown, either through a call 
    * to ros::shutdown() or a Ctrl-C.
    */
    ros::spin();
    //ROS_INFO is the replacement for printf/cout.
    ROS_INFO("tutorialROSOpenCV::main.cpp::No error.");
 
}
  

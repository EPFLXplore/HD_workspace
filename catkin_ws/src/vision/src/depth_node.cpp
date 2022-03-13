#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include<sensor_msgs/image_encodings.h>
//#include<sensor_msgs/ImageMessage.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
using namespace std;
using namespace cv;

cv:: Mat depthFeed;
void depthImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        depthFeed = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO16)->;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'mono16'.", msg->encoding.c_str());
        return;
    }

    // If the frame is empty, break immediately
    if (depthFeed.empty())
      cout << "frame is empty " << endl;

    // Display the resulting frame
    else {
      //
        //if(cameraFeed.rows > 60 && cameraFeed.cols > 60){
          //circle(cameraFeed, cv::Point(50, 50), 10, CV_RGB(255,0,0));
       //}  check how to draw on this, matybe crete a copy...
      imshow( "depth frame", depthFeed );
    }

    // Press  ESC on keyboard to exit
    char c=(char)waitKey(25);

    //cv::waitKey(1);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "depth_image_listiner");
  ros::NodeHandle nh_D;
  //cv::namedWindow("view");

  image_transport::ImageTransport it(nh_D);
  image_transport::Subscriber sub = it.subscribe("/camera/depth/image_rect_raw", 10, depthImageCallback);
  ros::spin();
  // When everything done, release the video capture object
    destroyAllWindows();
  
  //cv::destroyWindow("view");
}

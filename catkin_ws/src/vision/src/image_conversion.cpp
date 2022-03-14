/*

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include<iostream>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()    
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("camera/color/image_raw", 1,&ImageConverter::imageCb, this);  ///camera/image_raw
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    cv::Mat frame =cv_ptr->image;
    std::cout<<"publisher works"<<std::endl;
    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60){
      //cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
      std::cout<<cv_ptr->image.rows <<std::endl;
      std::cout<<int(cv_ptr->image.at<uchar>(5,5))<<std::endl;
      
      
    
    }
    else std::cout<<"publisher not working"<<std::endl;
    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, frame);
    std::cout<<int(frame.at<uchar>(5,5))<<std::endl; //
    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}

*/



#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include<sensor_msgs/image_encodings.h>
//#include<sensor_msgs/ImageMessage.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <cstdlib>
# include <opencv2/aruco.hpp>
# include <opencv2/highgui.hpp>
# include <opencv2/imgproc.hpp>
# include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/opencv.hpp>
# include <vector>
# include <iostream>
#include <unistd.h>
#include <string>
using namespace std;
using namespace cv;
using namespace cv::aruco;

Ptr<Dictionary> dictionary = getPredefinedDictionary(DICT_5X5_250);
static cv:: Mat cameraFeed, output_display;  // make it static
void imageCallback(const sensor_msgs::ImageConstPtr& msg){  //`imageCallback(boost::shared_ptr<sensor_msgs::Image_<std::allocator<void> > const> const&)'
    
        cv_bridge::CvImagePtr cv_ptr;
    try{
         cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);  
        //cameraFeed = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
        //cameraFeed = cv_bridge::toCvCopy(msg)->image;
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        return;
    }

    // If the frame is empty, will get seg fault ...
    if (cv_ptr->image.empty())
      cout << "frame is empty " << endl;
//
   // // Display the resulting frame
    else {
      //cameraFeed.copyTo(output_display);
      Point center (40,40);
      cameraFeed = cv_ptr->image;
      //cv::drawMarker(cv_ptr->image, cv::Point(cv_ptr->image.cols/2, cv_ptr->image.rows/2),  cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 10, 1);
      //if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      //cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
      //cv::circle(cv_ptr->image,center,1,Scalar(0, 0, 255));
      vector<int> ids;
      vector<vector<Point2f> > corners;  //each tag has 4 courners each corner is a 2d points so we get a vector vector two ints matrix to store all of that// the small red square is the top left corner of the marker
      //cv::aruco::detectMarkers(cameraFeed,dictionary,corners,ids);

      //drawDetectedMarkers(cameraFeed, corners, ids);
      cv::drawMarker(cameraFeed, cv::Point(cameraFeed.cols/2, cameraFeed.rows/2),  cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 10, 1);
      imshow( "Frame",cameraFeed);
      cv::waitKey(3);
      
    
      
      if (ids.size() > 0){
      output_display = cameraFeed.clone();
      drawDetectedMarkers(output_display, corners, ids);
      imshow( "new_Frame",output_display);
      
      cv::waitKey(3);

      }
    }

    // Press  ESC on keyboard to exit
    //char c=(char)waitKey(25);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  //cv::namedWindow("view");

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/camera/color/image_raw", 1, imageCallback);
  ros::spin();
  // When everything done, release the video capture object
    destroyAllWindows();
  
  //cv::destroyWindow("view");
}



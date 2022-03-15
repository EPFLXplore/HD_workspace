//
//#include "ros/ros.h"
//#include "std_msgs/String.h"
//
//#include <sstream>
//#include "sensor_msgs/CameraInfo.h"
//
///**
// * This tutorial demonstrates simple sending of messages over the ROS system.
// */
//#include <stdint.h>
//#include <std_msgs/Float64.h> //to use a float64 type
//
//using namespace std ;
//static uint32_t height; //image dimension
//static uint32_t width;  //image dimension
//static string distortion_model; //type of distortion model used
//static std_msgs::Float64 D [16];  // need to change the size of the array ... distortion params, size depends on distortion model
//static std_msgs::Float64 K [9]; // intrinsic camera matrix for the raw (so distorted image)
//static std_msgs::Float64 R [9]; //rectification matrix, for stereo cameras
//static std_msgs::Float64 projection [12] ;//projection matrix of the rectified image tha's the one i'l probably be using
//static uint32_t binning_x ; //not importnant i guess related to the resolution
//static uint32_t binning_y ;
//
//
//
//void callbackCameraInfo(const sensor_msgs::CameraInfo::ConstPtr& msg){
//    height = msg->height;
//    cout <<"image height is:"<< height<< endl;
//    //int i(0);
//    //cout<<"camera matrix is : ["<<P[i]<<" "<< P[i+1]<<" "<<P[i+2]<<" "<< P[i+3]<<" " << endl;
//    //cout<<"camera matrix is : ["<<P[i+4]<<" "<< P[i+5]<<" "<<P[i+6]<<" "<< P[i+7]<<" " << endl;
//    //cout<<"camera matrix is : ["<<P[i+8]<<" "<< P[i+9]<<" "<<P[i+10]<<" "<< P[i+11]<<" " << endl;
//    array<double, 12UL> projection; //array of 12 bit wide doubles U is units cf tp robotique
//    for (int i(0);i<12;++i){
//      projection[i]= msg->P[i];
//      cout << "first :" << projection[i] <<endl;
//    }
//
//    distortion_model=msg->distortion_model;
//    cout << "distortion model is :" << distortion_model <<endl;
//}
//
//
//int main(int argc, char **argv)
//{
//
//  ros::init(argc, argv, "camera");
//
//  ros::NodeHandle n;
//  ros::Subscriber sub = n.subscribe<sensor_msgs::CameraInfo>("/camera/color/camera_info",10, callbackCameraInfo);
//  ros::spin();
//
//
//  return 0;
//}
//
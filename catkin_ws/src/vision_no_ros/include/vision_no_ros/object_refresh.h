#ifndef OBJECT_REFRESH_H
#define OBJECT_REFRESH_H
#include <opencv2/opencv.hpp> 
#include <vector>

#include <vision_no_ros/cntrl_pnl.h>
#include <ros/ros.h>
#include <vision_no_ros/panel_object.h> 

using namespace std;
using namespace cv;

void refresh_object(vision_no_ros::panel_object& object,const int& id,const cv::Vec3d& rvecs,const cv::Vec3d& tvecs,
                       const cntrl_pnl::ArTag& ar_1,const cntrl_pnl::Object& obj);// should maybe add another artgag argument for the objects needing 2 artags at least to localize them...






void refresh_object(vision_no_ros::panel_object& object,const int& id,const cv::Vec3d& rvecs,const cv::Vec3d& tvecs,const cntrl_pnl::ArTag& ar_1,const cntrl_pnl::Object& obj){ 

  
  object.id = id; //this should be the object id no the ar tag... each objects position will be deduced from a specific ar tag
  object.reliability=100;  //what  exactly should i do here 
  cntrl_pnl::Position offset = cntrl_pnl::distance_from_ARtag(ar_1,obj);
  object.x_pos =offset.x_coor+tvecs[0]*1000; //casting and representing the foats with ints cf bens idea...
  object.y_pos =offset.y_coor-tvecs[1]*1000;
  object.z_pos =tvecs[2]*1000;//need to add condition on the depth source!(my algo or the intel's depth frame)
  object.x_rot =rvecs[0]*180/M_PI; //will give the ar tags rotations then the gripper can stay at that angle
  object.y_rot =rvecs[1]*180/M_PI; //add rotation relative to gripper
  object.z_rot =rvecs[2]*180/M_PI; //add rotation relative to gripper

 
}

#endif
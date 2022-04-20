#ifndef OBJECT_REFRESH_H
#define OBJECT_REFRESH_H
#include <opencv2/opencv.hpp> 
#include <vector>

#include <vision_no_ros/cntrl_pnl.h>
#include <ros/ros.h>
#include <vision_no_ros/panel_object.h> 

using namespace std;
using namespace cv;

void refresh_object(vision_no_ros::panel_object& object,const vector<int>& ids,const vector<cv::Vec3d>& rvecs,const vector<cv::Vec3d>& tvecs,
                       const cntrl_pnl::ArTag& ar_1,const cntrl_pnl::Object& obj);// should maybe add another artgag argument for the objects needing 2 artags at least to localize them...





/*
Function refreshes the desired object's params
arguments are:
-object: custom ros message type for a control panel object
-id: ID of the object to be refereshed
-rvecs: rotation vector of the AR tag
-tvecs: translation vector of the AR tag
-ar_1:  reference AR tag struct
-obj:   contol panel object struct of the required object

*/
void refresh_object(vision_no_ros::panel_object& object,const vector<int>& ids,const vector<cv::Vec3d>& rvecs,const vector<cv::Vec3d>& tvecs,const cntrl_pnl::ArTag& ar_1,const cntrl_pnl::Object& obj){ 


  for (int i(0);i < ids.size();++i){
    if (ids[i]==ar_1.id){
      object.id = obj.id; //this should be the object id no the ar tag... each objects position will be deduced from a specific ar tag
      object.reliability=100;  //what  exactly should i do here 
      cntrl_pnl::Position offset = cntrl_pnl::distance_from_ARtag(ar_1,obj);
      object.x_pos =offset.x_coor+tvecs[i][0]*1000; //casting and representing the foats with ints cf bens idea...
      object.y_pos =offset.y_coor-tvecs[i][1]*1000;
      object.z_pos =tvecs[i][2]*1000;//need to add condition on the depth source!(my algo or the intel's depth frame)
      object.x_rot =rvecs[i][0]*180/M_PI; //will give the ar tags rotations then the gripper can stay at that angle
      object.y_rot =rvecs[i][1]*180/M_PI; //add rotation relative to gripper
      object.z_rot =rvecs[i][2]*180/M_PI; //add rotation relative to gripper
      break;
    }
    else {
      object.id = obj.id; //this should be the object id no the ar tag... each objects position will be deduced from a specific ar tag
      object.reliability=0; 
    }
  }
}

#endif
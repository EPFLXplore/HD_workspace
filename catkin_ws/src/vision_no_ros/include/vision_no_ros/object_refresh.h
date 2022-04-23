#ifndef OBJECT_REFRESH_H
#define OBJECT_REFRESH_H
#include <opencv2/opencv.hpp> 
#include <vector>
#include <librealsense2/rs.hpp>

#include <vision_no_ros/cntrl_pnl.h>
#include <ros/ros.h>
#include <vision_no_ros/panel_object.h> 

using namespace std;
using namespace cv;

void refresh_object(vision_no_ros::panel_object& object,const vector<int>& ids,const vector<cv::Vec3d>& rvecs,const vector<cv::Vec3d>& tvecs,
                    const cntrl_pnl::ArTag& ar_1,const cntrl_pnl::Object& obj,const rs2::depth_frame& depth,const vector<vector<Point2f>>& corners);// should maybe add another artgag argument for the objects needing 2 artags at least to localize them...





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
void refresh_object(vision_no_ros::panel_object& object,const vector<int>& ids,const vector<cv::Vec3d>& rvecs,const vector<cv::Vec3d>& tvecs,const cntrl_pnl::ArTag& ar_1,const cntrl_pnl::Object& obj,const rs2::depth_frame& depth,const vector<vector<Point2f>>& corners){ 
// need to create a similar functio that refreshes the active target without seeing any ar tag , first use this one to home onto the active targert then y=use the other function

  for (int i(0);i < ids.size();++i){
    if (ids[i]==ar_1.id){
      object.id = obj.id; //this should be the object id no the ar tag... each objects position will be deduced from a specific ar tag
      object.reliability=100;  //what  exactly should i do here 
      cntrl_pnl::Position offset = cntrl_pnl::distance_from_ARtag(ar_1,obj);
      object.x_pos =offset.x_coor+tvecs[i][0]*1000; //casting and representing the foats with ints cf bens idea...
      object.y_pos =offset.y_coor-tvecs[i][1]*1000;
      object.x_rot =rvecs[i][0]*180/M_PI; //will give the ar tags rotations then the gripper can stay at that angle
      object.y_rot =rvecs[i][1]*180/M_PI; //add rotation relative to gripper
      object.z_rot =rvecs[i][2]*180/M_PI; //add rotation relative to gripper
      float tag_center_x = (corners[i][0].x+corners[i][2].x)/2;// what if you cant see the ar tag anymore which depth to take ??
      float tag_center_y = (corners[i][0].y+corners[i][2].y)/2;
      float dist=depth.get_distance(int(tag_center_x),int(tag_center_y));//work on getting depth from intel
      //cout<<"dist= "<< dist <<endl;//use this it works
      //object.z_pos =tvecs[i][2]*1000;//need to add condition on the depth source!(my algo or the intel's depth frame)
      object.z_pos=dist*1000;
      break;
    }
    else { //send an error message or a reset arm position command
      object.id = obj.id; //this should be the object id no the ar tag... each objects position will be deduced from a specific ar tag
      object.reliability=0;
      object.z_pos=depth.get_distance(depth.get_width()/2,depth.get_height()/2); //give the depth of the center pixel
    }
  }
}

#endif
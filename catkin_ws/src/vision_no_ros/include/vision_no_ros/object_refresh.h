#ifndef OBJECT_REFRESH_H
#define OBJECT_REFRESH_H
#include <opencv2/opencv.hpp> 
#include <vector>
#include <librealsense2/rs.hpp>
#include <cmath>

#include <vision_no_ros/cntrl_pnl.h>
#include <ros/ros.h>
#include <vision_no_ros/panel_object.h> 

using namespace std;
using namespace cv;

void refresh_object(vision_no_ros::panel_object& object,const vector<int>& ids,const vector<cv::Vec3d>& rvecs,const vector<cv::Vec3d>& tvecs,
                    const cntrl_pnl::ArTag& ar_1,const cntrl_pnl::Object& obj,const rs2::depth_frame& depth,const vector<vector<Point2f>>& corners,
                    const rs2_intrinsics& intrinsics);// should maybe add another artgag argument for the objects needing 2 artags at least to localize them...

void get_euler_angle(const rs2_intrinsics& intrinsics,const float& tag_center_pxl_x,const float& tag_center_pxl_y,const float& distance_to_center);

#define USE_RS2_PROJECTION

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
void refresh_object(vision_no_ros::panel_object& object,const vector<int>& ids,const vector<cv::Vec3d>& rvecs,const vector<cv::Vec3d>& tvecs,
                    const cntrl_pnl::ArTag& ar_1,const cntrl_pnl::Object& obj,const rs2::depth_frame& depth,const vector<vector<Point2f>>& corners,
                    const rs2_intrinsics& intrinsics){ 
// need to create a similar functio that refreshes the active target without seeing any ar tag , first use this one to home onto the active targert then y=use the other function

  for (int i(0);i < ids.size();++i){
    if (ids[i]==ar_1.id){
      object.id = obj.id; //this should be the object id no the ar tag... each objects position will be deduced from a specific ar tag
      object.reliability=100;  //what  exactly should i do here 
      cntrl_pnl::Position offset = cntrl_pnl::distance_from_ARtag(ar_1,obj);
      float tag_center_x = (corners[i][0].x+corners[i][2].x)/2;// what if you cant see the ar tag anymore which depth to take ??
      float tag_center_y = (corners[i][0].y+corners[i][2].y)/2;
      float dist=depth.get_distance(int(tag_center_x),int(tag_center_y));
      #ifdef USE_RS2_PROJECTION
        float pixel[2]= {tag_center_x,tag_center_y};
        float point[3];
        //canter of AR Tag
        rs2_deproject_pixel_to_point(point,&intrinsics,pixel,dist);
        object.x_pos =offset.x_coor+point[0]*1000; //.
        object.y_pos =offset.y_coor-point[1]*1000;//minus because the camera yaxis points down
        object.z_pos=point[2]*1000; 
        //right corner of AR tag
        pixel[0]=corners[i][1].x;
        pixel[1]=corners[i][1].y;
        dist=depth.get_distance(corners[i][1].x,corners[i][1].y);
        rs2_deproject_pixel_to_point(point,&intrinsics,pixel,dist);
        float angle_right=asin(point[2]/sqrt(point[0]*point[0]+point[2]*point[2]))*180/M_PI;
        //left corner of AR tag
        pixel[0]=corners[i][0].x;
        pixel[1]=corners[i][0].y;
        dist=depth.get_distance(corners[i][0].x,corners[i][0].y);
        rs2_deproject_pixel_to_point(point,&intrinsics,pixel,dist);
        float angle_left=asin(point[2]/sqrt(point[0]*point[0]+point[2]*point[2]))*180/M_PI;
        //yaw is right angle - left angle to be positive in anticlockwise rotation
        float yaw = angle_right-angle_left;
      #else  //test which method is more accurate
        object.x_pos =offset.x_coor+tvecs[i][0]*1000; //casting and representing the foats with ints cf bens idea...
        object.y_pos =offset.y_coor-tvecs[i][1]*1000;
        object.z_pos=dist*1000;
         //object.z_pos =tvecs[i][2]*1000;//need to add condition on the depth source!(my algo or the intel's depth frame)
     
      #endif
     
      //get_euler_angle(intrinsics,tag_center_x,tag_center_y,dist);
      //rvecs is a rodrigues angle not a classic euler angle so that sucx do simple geometry to estimate euler angles
      object.x_rot =yaw; //will give the ar tags rotations then the gripper can stay at that angle
      object.y_rot =rvecs[i][1]*180/M_PI; //add rotation relative to gripper
      object.z_rot =rvecs[i][2]*180/M_PI; //add rotation relative to gripper
      
      break;
    }
    else { //send an error message or a reset arm position command
      object.id = obj.id; //this should be the object id no the ar tag... each objects position will be deduced from a specific ar tag
      object.reliability=0;
      object.z_pos=depth.get_distance(depth.get_width()/2,depth.get_height()/2); //give the depth of the center pixel
    }
  }
}


void get_euler_angle(const rs2_intrinsics& intrinsics,const float& tag_center_pxl_x,const float& tag_center_pxl_y,const float& distance_to_center){
   
  //can do it with the rotation vector then rotation matrix to find coordinates in the camera system
  // cv::Mat rotation_matrix;
  // rotation_matrix = Rodrigues(rvecs,rotation_matrix);
  // cv::Mat transformation_matrix= ;
  // trying the same thing with the intel deprojection functions




}




#endif
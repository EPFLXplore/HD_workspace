#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <opencv2/aruco.hpp>
#include <vector>

#include <vision_no_ros/cntrl_pnl.h>
#include <vision_no_ros/cv-helpers.hpp>
#include <ros/ros.h>
//#include <geometry_msgs/Point.h>
#include <vision_no_ros/vector_msg.h> //even though this file doesnt exist, the .msg one does

using namespace std;
 using namespace cv;

cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_250);
void publisher_setup();//add argc and argv to it
void publisher_refresh(ros::Publisher& pub,const int& id,const cv::Vec3d& rvecs,const cv::Vec3d& tvecs,const vector<Point2f>& corner,const cntrl_pnl::ControlPanel& my_panel);// this publisher will be only for one specific topic

int main(int argc, char **argv) try {   
    //publisher_setup();
    ros::init(argc, argv, "detected_elements_publisher");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<vision_no_ros::vector_msg>("detected_elements", 1);
    cntrl_pnl::ControlPanel my_panel;
    cntrl_pnl::setup_control_panel(my_panel);
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Start streaming with default recommended configuration
    pipe.start(); //maybe should try to optimze the start parameters for bandwidth gains and other stuff when integratingg

   
    const auto window_name = "Display Image";
    namedWindow(window_name, WINDOW_AUTOSIZE);

    while (waitKey(1) < 0 && getWindowProperty(window_name, WND_PROP_AUTOSIZE) >= 0)
    {
        rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
        rs2::frame color = data.get_color_frame();
        rs2::depth_frame depth =data.get_depth_frame();
        //float width = depth.get_width();
        //float height = depth.get_height();
        //float dist_to_center = depth.get_distance(width / 2, height / 2);
        //std::cout << "The camera is facing an object " << dist_to_center << " meters away \r"<< std::endl;

        //// Query frame size (width and height)
        //const int w = color.as<rs2::video_frame>().get_width();
        //const int h = color.as<rs2::video_frame>().get_height();

        //// Create OpenCV matrix of size (w,h) from the colorized depth data
        //Mat image(Size(w, h), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);

        // Update the window with new data
        //cv::Mat cameraMatrix = Mat(3,3, CV_64F, float(0));
        //cv::Mat distCoeffs = Mat(1,4, CV_64F, float(0));
        cv::Mat cameraMatrix ;
        cv::Mat distCoeffs ;
        get_field_of_view(pipe,cameraMatrix,distCoeffs);
        
       // std::cout << "Principal Point" << cameraMatrix.at<float>(0,0) << std::endl; this float cast messes thing up
       
        static vector<int> ids;
        static vector<vector<Point2f> > corners;
        std::vector<cv::Vec3d> rvecs, tvecs;

        cv::Mat image = frame_to_mat(color);
        cv::aruco::detectMarkers(image,dictionary,corners,ids);
        if (ids.size()>0){
            cv::Mat output_image=image.clone();
            cv::aruco::drawDetectedMarkers(output_image,corners,ids);
            //cv::circle(output_image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
            cv::aruco::estimatePoseSingleMarkers(corners, 0.05, cameraMatrix, distCoeffs, rvecs, tvecs);
            for(int i=0; i<ids.size(); i++){
                cv::aruco::drawAxis(output_image, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1); //tvecs is in meters
                //std::cout <<"ar tag depth is: " << tvecs[i][2]<< std::endl;
                // calcul with depth
                //float dist_to_center=depth.get_distance(int(corners[i][0].x),int(corners[i][0].y));
                //std::cout << "The camera is facing an object " << dist_to_center << " meters away \r"<< std::endl;
                //std::cout <<"difference is : "<<tvecs[i][2]-dist_to_center <<std::endl;
                //cntrl_pnl::Position offset = cntrl_pnl::distance_from_ARtag(my_panel.panelA.artg1,my_panel.panelA.switch1);
                //std::cout << "distance to the on the x axis is : " <<tvecs[i][0]<< std::endl;// /////////////////////////////////////the axis axis is actually drawn in the wrong direction so switch the signs of the expression using tvecs in the following two lines
                //std::cout << "distance to the first switch on the x axis is : " <<offset.x_coor+tvecs[i][0]*1000<<" mm "<< std::endl; //no x is ni iverted, it's just that im getting clodet to the object when I goto the negative x so tvecs and offset should have opposite sighns so when 
                //std::cout << "distance to the first switch on the y axis is : " <<offset.y_coor-tvecs[i][1]*1000<<" mm "<< std::endl;
                publisher_refresh(pub,ids[i],rvecs[i],tvecs[i],corners[i],my_panel);
            }
            imshow(window_name, output_image);
        }
        imshow("input feed",image);
    }

   
    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}


// stop piepline streming

//rs2_pipeline_stop(pipeline, &e);
//    check_error(e);
//
//    // Release resources
//    free(buffer);
//    rs2_delete_pipeline_profile(pipeline_profile);
//    rs2_delete_stream_profiles_list(stream_profile_list);
//    rs2_delete_stream_profile(stream_profile);
//    rs2_delete_config(config);
//    rs2_delete_pipeline(pipeline);
//    rs2_delete_device(dev);
//    rs2_delete_device_list(device_list);
//    rs2_delete_context(ctx);
//
//    return EXIT_SUCCESS;
//}

/*
void publisher_setup() {
 
  ros::init(argc, argv, "detected_elements_publisher");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<vision_no_ros::vector_msg>("detected_elements", 1);
 // ros::Rate loop_rate(0.5);
}
*/
void publisher_refresh(ros::Publisher& pub,const int& id,const cv::Vec3d& rvecs,const cv::Vec3d& tvecs,const vector<Point2f>& corners,const cntrl_pnl::ControlPanel& my_panel){
//add a for loop for ids .size then modify the msg so it can be a vector of size detected ids of the ints we specified

 // the message to be published
  vision_no_ros::vector_msg msg;
  msg.id = id; //this should be the object id no the ar tag... each objects position will be deduced from a specific ar tag
  msg.reliability=100;
  cntrl_pnl::Position offset = cntrl_pnl::distance_from_ARtag(my_panel.panelA.artg1,my_panel.panelA.switch1);
  msg.x_pos =offset.x_coor+tvecs[0]*1000; //casting and representing the foats with ints cf bens idea...
  msg.y_pos =offset.y_coor-tvecs[1]*1000;
  msg.z_pos =tvecs[2]*1000;//need to add condition on the depth source!(my algo or the intel's depth frame)
  msg.x_rot =500; //will give the ar tags rotations then the gripper can stay at that angle need to test which units though
  msg.y_rot =600; //need dims
  msg.z_rot =700; //need dims


  pub.publish(msg);
  ros::spinOnce();


}
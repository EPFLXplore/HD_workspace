#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <opencv2/aruco.hpp>
#include <vector>
#include <ros/ros.h>

#include <iostream>
#include "std_msgs/Int16.h"

//includes for my headers
#include <vision_no_ros/cntrl_pnl.h> //included in  object_refresh
#include <vision_no_ros/cv-helpers.hpp>
#include <vision_no_ros/object_refresh.h>
//custom messages includes
#include <vision_no_ros/panel_object.h> //even though this file doesnt exist, the .msg one does
#include <vision_no_ros/object_list.h>

using namespace std;
using namespace cv;

static bool show_input_image(0);
static bool show_output_image(1);
#define SAMPLES 30


////////////////////// vectors required for AR tag detection ///////////////////////////////////
cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_250);
static cv::Mat cameraMatrix ;
static cv::Mat distCoeffs ;
static vector<int> ids;
static vector<vector<Point2f> > corners;
static std::vector<cv::Vec3d> rvecs, tvecs;

void fsm_callback(const std_msgs::Int16& msg){
  // this should comtain the ros ok whuile loop but with another condition

        cout << msg.data << endl;
}
 

int main(int argc, char **argv) try {   
    
    //////////// ROS node initialisation ////////////////
    ros::init(argc, argv, "detected_elements_publisher");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<vision_no_ros::object_list>("detected_elements", 1);
    ros::Subscriber sub = n.subscribe("vision_FSM", 1000, fsm_callback);
    //ros::spin();//this is needed for the calbacks to be actually called its an infinite loop...

    //////////// control panel initialisation ////////////
    cntrl_pnl::ControlPanel my_panel;
    cntrl_pnl::setup_control_panel(my_panel);
    
    ////////// RealSense pipeline initialisation /////////
    rs2::pipeline pipe;
    //setup custom streaming configuration 
    /*
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH,1280, 720, RS2_FORMAT_Z16, 30); //this is the best resolutio for the depth stream to be accurate
    cfg.enable_stream(RS2_STREAM_COLOR,1280, 720, RS2_FORMAT_BGR8, 30);
    pipe.start(cfg);
    */
    pipe.start(); // Start streaming with default recommended configuration//maybe should try to optimze the start parameters for bandwidth gains and other stuff when integratingg
    rs2::align align_to_color(RS2_STREAM_COLOR);//expensive keep it outside the loop


    while (ros::ok()){ // need the loop to keep getting new frames replace the condition with something from the commands i get from the fsm maybe I should add an idle state to this{
        ///////////////get new depth and color frames and align them///////////////////////////
        rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
        data = align_to_color.process(data); //for aligning the depth and color frames
        rs2::frame color = data.get_color_frame();
        rs2::depth_frame depth =data.get_depth_frame();
       

        ///////////////// AR tag detection and camera calibration /////////////////////////
        rs2_intrinsics intrinsics = get_field_of_view(pipe,cameraMatrix,distCoeffs); //function to get the camera intrinsics and copy them into the right matrices
        cv::Mat image = frame_to_mat(color);  //using the cv helpers to convert an rs2 frame to a cv mat
        cv::aruco::detectMarkers(image,dictionary,corners,ids);
        
        
        
        if (ids.size()>0){
            
            cv::aruco::estimatePoseSingleMarkers(corners, 0.044, cameraMatrix, distCoeffs, rvecs, tvecs);// dont forget to modify the ar tag size!! //this function might become obsolete this will need to be added inside the ifs because the ar tag size changes
            
            uint command=0;// test variable , replaces the topic I should be subscribed to to know which on=bject to manipulate
            static int active_sample=0;
            ///////////////////////////////////////////////// start refreshing the objects ///////////////////////////////////          
            vision_no_ros::object_list objects;//decalre objects list
            
            //// panel A
            
            if (command==0 or command==1){
                //declare object and refresh it
                vision_no_ros::panel_object main_switch;
                refresh_object(main_switch,ids,rvecs,tvecs,my_panel.panelA.artg1,my_panel.panelA.switchMain,depth,corners,intrinsics,SAMPLES);//need to make a function that gets the rvecs and tvecs for the ar tag with id hard coded
                //push back the object to the list to be published
                if (active_sample==SAMPLES) objects.detected_objects.push_back(main_switch);
            }
            if (command==0 or command==2){
                vision_no_ros::panel_object switch_1;
                refresh_object(switch_1,ids,rvecs,tvecs,my_panel.panelA.artg1,my_panel.panelA.switch1,depth,corners,intrinsics,SAMPLES);
                if (active_sample==SAMPLES) objects.detected_objects.push_back(switch_1);
            }

            if (command==0 or command==3){
                vision_no_ros::panel_object switch_2;
                refresh_object(switch_2,ids,rvecs,tvecs,my_panel.panelA.artg1,my_panel.panelA.switch2,depth,corners,intrinsics,SAMPLES);
                if (active_sample==SAMPLES) objects.detected_objects.push_back(switch_2);
            }

            if (command==0 or command==4){
                vision_no_ros::panel_object switch_3;
                refresh_object(switch_3,ids,rvecs,tvecs,my_panel.panelA.artg1,my_panel.panelA.switch3,depth,corners,intrinsics,SAMPLES);
                if (active_sample==SAMPLES) objects.detected_objects.push_back(switch_3);
            }

            //// panel B1

            if (command==0 or command==5){
                vision_no_ros::panel_object button;
                refresh_object(button,ids,rvecs,tvecs,my_panel.panelB1.artg2,my_panel.panelB1.button,depth,corners,intrinsics,SAMPLES);
                if (active_sample==SAMPLES) objects.detected_objects.push_back(button);
            }

            if (command==0 or command==6){
                vision_no_ros::panel_object outlet;
                refresh_object(outlet,ids,rvecs,tvecs,my_panel.panelB1.artg3,my_panel.panelB1.outlet,depth,corners,intrinsics,SAMPLES);
                if (active_sample==SAMPLES) objects.detected_objects.push_back(outlet);
            }

            if (command==0 or command==7){
                vision_no_ros::panel_object emagLock;
                refresh_object(emagLock,ids,rvecs,tvecs,my_panel.panelB1.artg3,my_panel.panelB1.emagLock,depth,corners,intrinsics,SAMPLES);
                if (active_sample==SAMPLES) objects.detected_objects.push_back(emagLock);
            }

            //// panel B2

            if (command==0 or command==8){
                vision_no_ros::panel_object ethernet;
                refresh_object(ethernet,ids,rvecs,tvecs,my_panel.panelB2.artg4,my_panel.panelB2.ethernet,depth,corners,intrinsics,SAMPLES);
                if (active_sample==SAMPLES) objects.detected_objects.push_back(ethernet);
            }

            /////////////////////////////////////////////// end object referesh /////////////////////////////////////////////////



            if (active_sample < SAMPLES ){
                ++active_sample;
            }else {
                active_sample=0;
                //publish the list               
                pub.publish(objects);
                ros::spinOnce();
            }
                
        
            if (show_output_image){
                cv::Mat output_image=image.clone();
                cv::aruco::drawDetectedMarkers(output_image,corners,ids);
                //for(int i=0; i<ids.size(); i++){ 
                //  cv::aruco::drawAxis(output_image, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1); removed his for the jetson (uses a version thats not compatible with this)
                //} 
                imshow("output image", output_image);
                waitKey(1);
            }
        }else {
            // no ar tags are visible call the blind functions
        }
        if(show_input_image){
            imshow("input feed",image);
            waitKey(1);
        }
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

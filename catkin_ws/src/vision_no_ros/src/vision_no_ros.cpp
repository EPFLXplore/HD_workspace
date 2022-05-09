#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <opencv2/aruco.hpp>
#include <vector>
#include <ros/ros.h>
//#define ROS_IMAGE_PUB
//#ifdef  ROS_IMAGE_PUB
//#include<sensor_msgs/image_encodings.h>
//#include <image_transport/image_transport.h>
//#include <cv_bridge/cv_bridge.h>
//#endif
//includes for my headers
#include <vision_no_ros/cntrl_pnl.h> //included in  object_refresh
#include <vision_no_ros/cv-helpers.hpp>
#include <vision_no_ros/object_refresh.h>
//custom messages includes
#include <vision_no_ros/panel_object.h> //even though this file doesnt exist, the .msg one does
#include <vision_no_ros/object_list.h>

using namespace std;
using namespace cv;

cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_250);

int main(int argc, char **argv) try {   
    ros::init(argc, argv, "detected_elements_publisher");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<vision_no_ros::object_list>("detected_elements", 1);
    
    cntrl_pnl::ControlPanel my_panel;
    cntrl_pnl::setup_control_panel(my_panel);
    // Declare RealSense pipeline, encapsulating the actual device and sensors
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
   
    const auto window_name = "Display Image";
    namedWindow(window_name, WINDOW_AUTOSIZE);

    while (waitKey(0) > -1 && getWindowProperty(window_name, WND_PROP_AUTOSIZE) >= 0) //this was waitkey(1) <0
    {
        rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
        
        
        data = align_to_color.process(data);


        rs2::frame color = data.get_color_frame();
        rs2::depth_frame depth =data.get_depth_frame();
       

        static cv::Mat cameraMatrix ;
        static cv::Mat distCoeffs ;
        rs2_intrinsics intrinsics = get_field_of_view(pipe,cameraMatrix,distCoeffs); //function to get the camera intrinsics and copy them into the right matrices
        
   
        // vectors required for AR tag detection
        static vector<int> ids;
        static vector<vector<Point2f> > corners;
        static std::vector<cv::Vec3d> rvecs, tvecs;

        cv::Mat image = frame_to_mat(color);  //using the cv helpers to convert an rs2 frame to a cv mat

        cv::aruco::detectMarkers(image,dictionary,corners,ids);
        if (ids.size()>0){
            cv::Mat output_image=image.clone();
            cv::aruco::drawDetectedMarkers(output_image,corners,ids);
            cv::aruco::estimatePoseSingleMarkers(corners, 0.044, cameraMatrix, distCoeffs, rvecs, tvecs);// dont forget to modify the ar tag size!! //this function might become obsolete
            
            uint command=1;// test variable , replaces the topic I should be subscribed to to know which on=bject to manipulate
            for(int i=0; i<ids.size(); i++){
               // cv::aruco::drawAxis(output_image, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1); //tvecs is in meters
                                
                //start refreshing the objects
                vision_no_ros::object_list objects;//decalre objects list
                
                if (command==0 or command==1){
                    //declare object and refresh it
                    vision_no_ros::panel_object main_switch;
                    refresh_object(main_switch,ids,rvecs,tvecs,my_panel.panelA.artg1,my_panel.panelA.switchMain,depth,corners,intrinsics);//need to make a function that gets the rvecs and tvecs for the ar tag with id hard coded
                    //push back the object to the list to be published
                    objects.detected_objects.push_back(main_switch);
                }
                if (command==0 or command==2){
                    vision_no_ros::panel_object switch_1;
                    refresh_object(switch_1,ids,rvecs,tvecs,my_panel.panelA.artg1,my_panel.panelA.switch1,depth,corners,intrinsics);
                    objects.detected_objects.push_back(switch_1);
                }

                if (command==0 or command==3){
                    vision_no_ros::panel_object switch_2;
                    refresh_object(switch_2,ids,rvecs,tvecs,my_panel.panelA.artg1,my_panel.panelA.switch2,depth,corners,intrinsics);
                    objects.detected_objects.push_back(switch_2);
                }

                if (command==0 or command==4){
                    vision_no_ros::panel_object switch_3;
                    refresh_object(switch_3,ids,rvecs,tvecs,my_panel.panelA.artg1,my_panel.panelA.switch3,depth,corners,intrinsics);
                    objects.detected_objects.push_back(switch_3);
                }
                
                
                //publish the list               
                pub.publish(objects);
                ros::spinOnce();
            
            }
            imshow(window_name, output_image);
        }
        imshow("input feed",image);
    }
    // destroyAllWindows(); // doesnt do much here
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

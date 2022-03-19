#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <opencv2/aruco.hpp>
#include <vector>

#include <vision_no_ros/cv-helpers.hpp>

using namespace std;

cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250);

int main(int argc, char * argv[]) try
{   
    
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Start streaming with default recommended configuration
    pipe.start();

    using namespace cv;
    const auto window_name = "Display Image";
    namedWindow(window_name, WINDOW_AUTOSIZE);

    while (waitKey(1) < 0 && getWindowProperty(window_name, WND_PROP_AUTOSIZE) >= 0)
    {
        rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
        rs2::frame color = data.get_color_frame();

        //// Query frame size (width and height)
        //const int w = color.as<rs2::video_frame>().get_width();
        //const int h = color.as<rs2::video_frame>().get_height();

        //// Create OpenCV matrix of size (w,h) from the colorized depth data
        //Mat image(Size(w, h), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);

        // Update the window with new data
        cv::Mat cameraMatrix = Mat(3,3, CV_64F, float(0));
        cv::Mat distCoeffs = Mat(1,4, CV_64F, float(0));
        get_field_of_view(pipe,cameraMatrix,distCoeffs);
        
        std::cout << "Principal Point" << cameraMatrix.at<float>(0,0) << std::endl;
       
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
                cv::aruco::drawAxis(output_image, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
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


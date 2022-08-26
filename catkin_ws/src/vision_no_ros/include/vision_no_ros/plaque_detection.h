#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <vector>
#include <iostream>

using namespace std;
using namespace cv;
//read image
//transform to grayscale
//transform to binary (tresholding)
//use contour detection to detect the contours of the wghite parts in the image (aka the objects) black pixels are cconsidered background
void find_plaque(const Mat& image){

    Mat blurred_image;
    GaussianBlur(image,blurred_image,Size(7,7),0);
    Mat image_grey;
    cvtColor(blurred_image,image_grey,COLOR_BGR2GRAY);
    Mat thresh_image;
    threshold(image_grey,thresh_image,150,255,THRESH_BINARY);
        
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(thresh_image,contours,hierarchy,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE); //RETR EXTERNAL is used to detect only the external contour of an object (ie just the parent)using chain appprox none is slower because it finds and saves all of the points on the cntour not just the end points
    //drawContours(image,contours,-1,Scalar(0,255,0),2);
    imshow("detected contours",image);
    
    
    vector<vector<Point>> approx_contours;
    approx_contours.resize(contours.size());
    int count_rectangles=0;
    cv::Mat output_image=image.clone();

    for (size_t k=0 ; k<contours.size() ; ++k){
        approxPolyDP(Mat(contours[k]),approx_contours[k],50,true); //the bigger the number the worse the approx which is better for me   // i need a bad approximation for this to work, blurring the edgeds might work I need a low pass filtered image to have the rectangluar shape without any irregu;arities
        if (approx_contours[k].size()==4){
            //for (size_t i=0 ; i<approx_contours.size() ; ++i){

               // circle(output_image,approx_contours[k][i],50,Scalar(0,0,255),1);
            //}
            circle(output_image,approx_contours[k][0],50,Scalar(0,0,255),1);
            circle(output_image,approx_contours[k][2],50,Scalar(255,0,255),1);

            cout<< "contour number : " << k << "has four sides" <<endl;
            //Point pt1(contours[k][0].x,contours[k][0].y); //contours[k][0];
            //Point pt2(contours[k][2].x,contours[k][2].y); //contours[k][2];
            rectangle(output_image,approx_contours[k][0],approx_contours[k][2],Scalar(0,255,0),5);
            ++count_rectangles;
        }
    }
    
   
    drawContours(output_image,approx_contours,-1,Scalar(255,0,0),2);
    //cout << contours.size() << endl;
    imshow("detected contours",output_image);

    //imshow("blurred iage",blurred_image);
    
    waitKey(1);
   // destroyAllWindows();
}

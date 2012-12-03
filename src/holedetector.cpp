#include "holedetector.h"
#include "ros/message.h"
#include "sensor_msgs/Image.h"
#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

static const Scalar colours[] =
{
    Scalar(0,0,255),
    Scalar(0,128,255),
    Scalar(0,255,255),
    Scalar(0,255,0),
    Scalar(255,128,0),
    Scalar(255,255,0),
    Scalar(255,0,0),
    Scalar(255,0,255),
    Scalar(128,255,255)
};


HoleDetector::HoleDetector()
{
}

void HoleDetector::asusCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    std::cout << "ASUS-Callback!!! Image encoding: " << msg->encoding << std::endl;
    int rows = msg->height;
    int cols = msg->width;
    signed char* data = (signed char*)&(msg->data.front());

    ROS_INFO("This is HoleDetector (asusCallback). Saving Image data... rows: [%d] cols: [%d]", rows, cols);

    // value encoded in mm! 1.5m = 1500mm
    cv::Mat image = cv::Mat(rows, cols, CV_16UC1, data);
    cv::Mat temp;
    image.convertTo(temp, CV_32FC1);
    cv::threshold(temp, temp, 1500.0, 1500.0, cv::THRESH_TRUNC);
    //cv::threshold(temp, temp, 1500.0, 1500.0, cv::THRESH_TOZERO_INV);
    cv::normalize(temp, image, 0, 255, CV_MINMAX);


    GaussianBlur( image, image, Size(3,3), 0, 0, BORDER_DEFAULT );
    cv::imwrite("depth.jpg", image);
    std::cout << "image written" << std::endl;



    // GRADIENT APPROACH
//    int scale = 1;
//    int delta = 0;
//    int ddepth = CV_16S;
//    std::string grad_name = "Gradient";
//    std::string binary_name = "Binary of Gradient";

//    /// Convert it to gray
//    Mat grad;

//    /// Create window
//    namedWindow( grad_name, CV_WINDOW_AUTOSIZE );
//    namedWindow( binary_name, CV_WINDOW_AUTOSIZE );

//    /// Generate grad_x and grad_y
//    Mat grad_x, grad_y;
//    Mat abs_grad_x, abs_grad_y;

//    /// Gradient X
//    //Scharr( src_gray, grad_x, ddepth, 1, 0, scale, delta, BORDER_DEFAULT );
//    Sobel( image, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );
//    convertScaleAbs( grad_x, abs_grad_x );

//    /// Gradient Y
//    //Scharr( src_gray, grad_y, ddepth, 0, 1, scale, delta, BORDER_DEFAULT );
//    Sobel( image, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );
//    convertScaleAbs( grad_y, abs_grad_y );

//    /// Total Gradient (approximate)
//    addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad );

//    Mat binary;
//    threshold(grad, binary, 5, 255, THRESH_BINARY);
//    imshow( grad_name, grad );
//    imshow(binary_name, binary);




    // MSER APPROACH

    Mat image_8bit, contour_image, image_vis;
    string msers_name = "MSERs";
    string image_name = "Image";
    image.convertTo(image_8bit, CV_8UC1);
    std::vector< vector<Point> > msers;
    Mat mask;
    mask = image_8bit >= 0x50;


    MSER extractor;
    extractor(image_8bit, msers, mask);

    cvtColor(image_8bit, contour_image, CV_GRAY2BGR);


    std::cout << "MSERS: " << msers.size() << std::endl;
    double area;
    for (size_t index = 0; index < msers.size(); index ++)
    {
        area = contourArea(msers[index]);
        if(area > 100)
          drawContours(contour_image, msers, index, colours[index%9], 1);
    }


    Mat image_8bit_bgr;
    cvtColor(image_8bit, image_8bit_bgr, CV_GRAY2BGR);
    image_vis = image_8bit_bgr*0.5 + contour_image*0.5;


    imshow( msers_name, mask );
    imshow(image_name, image_vis);

    //cv::imwrite("labels.jpg", labels*2000);

    waitKey(1);



}

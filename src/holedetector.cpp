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
    //std::cout << "ASUS-Callback!!! Image encoding: " << msg->encoding << std::endl;
    int height = msg->height;
    int width = msg->width;
    signed char* data = (signed char*)&(msg->data.front());

    //ROS_INFO("This is HoleDetector (asusCallback). Saving Image data... rows: [%d] cols: [%d]", height, width);

    // value encoded in mm! 1.5m = 1500mm
    cv::Mat image = cv::Mat(height, width, CV_16UC1, data);
    cv::Mat temp;
    image.convertTo(temp, CV_32FC1);
    cv::threshold(temp, temp, 1500.0, 1500.0, cv::THRESH_TRUNC); // truncate depth to 1,5m
    float factor = 255.0f/1500.f;
    temp = temp*factor;
    temp.convertTo(image, CV_16UC1);

    GaussianBlur( image, image, Size(3,3), 0, 0, BORDER_DEFAULT );
    cv::imwrite("depth.jpg", image);
    //std::cout << "image written" << std::endl;



    // GRADIENT APPROACH
    int scale = 1;
    int delta = 0;
    int ddepth = CV_16S;
    std::string grad_name = "Gradient";
    std::string binary_name = "Binary of Gradient";
    std::string laplace_name = "Laplacian";
    std::string contour_name = "Contours";

    /// Convert it to gray
    Mat grad;

    /// Create window
    //namedWindow( grad_name, CV_WINDOW_AUTOSIZE );
    //namedWindow( binary_name, CV_WINDOW_AUTOSIZE );
    //namedWindow( laplace_name, CV_WINDOW_AUTOSIZE );
    namedWindow( contour_name, CV_WINDOW_AUTOSIZE );

    /// Generate grad_x and grad_y
    Mat grad_x, grad_y;
    Mat abs_grad_x, abs_grad_y;

    /// Gradient X
    Sobel( image, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );
    convertScaleAbs( grad_x, abs_grad_x );

    /// Gradient Y
    Sobel( image, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );
    convertScaleAbs( grad_y, abs_grad_y );

    /// Total Gradient (approximate)
    addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad );

    //Mat laplace;
    //Laplacian( image, laplace, image.depth(), 3, 1, 0, BORDER_DEFAULT );
    //convertScaleAbs( laplace, laplace );
    //Laplacian(image, laplace, image.depth());

    // Now operate on 8 bit mode
    grad.convertTo(grad, CV_8UC1);

    // Threshold the Gradient image
    Mat binary;
    threshold(grad, binary, 10, 255, THRESH_BINARY);

    // Find Contours
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours( binary, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

    // Filter Contours and Display the Result
    Mat contour_image;
    image.convertTo(contour_image, CV_8UC1);
    cvtColor(contour_image, contour_image, CV_GRAY2BGR);
    float cont_area;
    float bb_area;
    float bw_ratio;
    float area_ratio;
    float min_area_ratio = 0.5f;
    float max_area = (float)height*0.5*width*0.5;
    float min_area = (float)height*0.1*width*0.1;
    float max_bw_ratio = 3.0f;
    float min_bw_ratio = 0.8f;

    int colour_count = 0;

    for (size_t index = 0; index < contours.size(); index ++)
    {
        Rect rect = boundingRect(contours[index]);
        cont_area = contourArea(contours[index]);
        bb_area = rect.area();
        area_ratio = cont_area/bb_area;
        bw_ratio = (float)rect.width/rect.height;
        if(area_ratio > min_area_ratio && bw_ratio < max_bw_ratio && bw_ratio > min_bw_ratio && bb_area > min_area && bb_area < max_area)
        {
            drawContours(contour_image, contours, index, colours[colour_count], 1);

            Point center = Point(rect.x+rect.width/2, rect.y+rect.height/2);
            line(contour_image, center + Point(0, rect.height/2), center - Point(0, rect.height/2), colours[colour_count], 3);
            line(contour_image, center + Point(rect.width/2, 0), center - Point(rect.width/2, 0), colours[colour_count], 3);
            rectangle(contour_image, rect, colours[colour_count%9]);
            std::cout << "stats: [" << index << "] area: " << bb_area << ",  w/h: " << bw_ratio << std::endl;

            colour_count = (colour_count+1)%9;
        }

    }



    //imshow( grad_name, grad );
    //imshow(binary_name, binary);
    //imshow(laplace_name, laplace);
    imshow(contour_name, contour_image);



    // MSER APPROACH

//    Mat image_8bit, contour_image, image_vis;
//    string msers_name = "MSERs";
//    string image_name = "Image";
//    image.convertTo(image_8bit, CV_8UC1);
//    std::vector< vector<Point> > msers;
//    Mat mask;
//    mask = image_8bit >= 0x50;

//    //             int _delta, int _min_area,     int _max_area,   float _max_variation, float _min_diversity, int _max_evolution, double _area_threshold, double _min_margin, int _edge_blur_size
//    MSER extractor(4,          width*height/1000, width*height/10, 0.5f,                 0.5f,                 256,                0.1,                    40.0f,              3);
//    extractor(image_8bit, msers, mask);

//    cvtColor(image_8bit, contour_image, CV_GRAY2BGR);

//    double area;

//    std::cout << "MSERS: " << msers.size() << std::endl;
//    for (size_t index = 0; index < msers.size(); index ++)
//    {
//        Rect rect = boundingRect(msers[index]);
//        if((float)rect.width/rect.height < 2.5f)
//        {
//           drawContours(contour_image, msers, index, colours[index%9], 1);

//           Point center = Point(rect.x+rect.width/2, rect.y+rect.height/2);
//           line(contour_image, center + Point(0, rect.height/2), center - Point(0, rect.height/2), Scalar(0,0,0), 3);
//           line(contour_image, center + Point(rect.width/2, 0), center - Point(rect.width/2, 0), Scalar(0,0,0), 3);
//        }
//        rectangle(contour_image, rect, colours[index%9]);

//        area = contourArea(msers[index]);
//        std::cout << "stats: [" << index << "] area: " << area << ",  h/w: " << (float)rect.width/rect.height << std::endl;

//    }


//    Mat image_8bit_bgr;
//    cvtColor(image_8bit, image_8bit_bgr, CV_GRAY2BGR);
//    image_vis = image_8bit_bgr*0.5 + contour_image*0.5;


//    imshow( msers_name, image_8bit );
//    imshow(image_name, image_vis);

//    //cv::imwrite("labels.jpg", labels*2000);





    waitKey(1);



}

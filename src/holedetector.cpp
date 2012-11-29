#include "holedetector.h"
#include "ros/message.h"
#include "sensor_msgs/Image.h"
#include "opencv2/opencv.hpp"

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


    cv::imwrite("depth.jpg", image);
    std::cout << "image written" << std::endl;
}

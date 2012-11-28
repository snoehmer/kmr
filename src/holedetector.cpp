#include "holedetector.h"
#include "ros/message.h"
#include "sensor_msgs/Image.h"
#include "opencv2/opencv.hpp"

HoleDetector::HoleDetector()
{
}

void HoleDetector::asusCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    std::cout << "ASUS-Callback!!!" << std::endl;
    int rows = msg->height;
    int cols = msg->width;
    signed char* data = (signed char*)&(msg->data.front());
    ROS_INFO("This is HoleDetector (asusCallback). Saving Image data... rows: [%d] cols: [%d]", rows, cols);
    cv::Mat image = cv::Mat(rows, cols, CV_8SC1, data);
    cv::imwrite("depth.png", image);
}

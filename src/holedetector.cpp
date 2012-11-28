#include "holedetector.h"
#include "ros/message.h"
#include "sensor_msgs/Image.h"

HoleDetector::HoleDetector()
{
}

void HoleDetector::asusCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    std::cout << "ASUS-Callback!!!" << std::endl;
}

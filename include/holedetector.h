#ifndef HOLEDETECTOR_H
#define HOLEDETECTOR_H

#include "sensor_msgs/Image.h"

class HoleDetector
{
public:
    HoleDetector();

    void asusCallback(const sensor_msgs::Image::ConstPtr& msg);
};

#endif // HOLEDETECTOR_H

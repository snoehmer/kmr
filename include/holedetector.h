#ifndef HOLEDETECTOR_H
#define HOLEDETECTOR_H

#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"


class HoleDetector
{
private:
    sensor_msgs::CameraInfo camera_info_;
    bool need_cam_info_;

public:
    HoleDetector();

    void asusCallback(const sensor_msgs::Image::ConstPtr& msg);
    void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
};

#endif // HOLEDETECTOR_H

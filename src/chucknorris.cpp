#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <chucknorris.h>
#include <ros/ros.h>
#include "controller.h"
#include "holedetector.h"



int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "chucknorris");
    ros::NodeHandle n;
    //for registering the node

    Controller* controller = new Controller();
    HoleDetector* hd_ptr = controller->getHoleDetector();

    ros::Subscriber asus = n.subscribe("/depth/image_raw", 1, &HoleDetector::asusCallback, hd_ptr);


    std::cout << "I'll find you!!!" << std::endl;
    ros::spin();


    return 0;
}

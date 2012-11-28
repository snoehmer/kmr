#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <chucknorris.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "chucknorris");
    //ros::NodeHandle nh;
    //for registering the node

    std::cout << "I'll find you!!!" << std::endl;
    ros::spin();


    return 0;
}

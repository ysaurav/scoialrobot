#include <ros/ros.h>

#include "kinect_proxy.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "social_robot_main");

    init_kinect();        

    return 0;
}

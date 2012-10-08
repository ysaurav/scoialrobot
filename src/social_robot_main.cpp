#include <ros/ros.h>

#include "ImageHandler.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "social_robot_main");

    ImageHandler ih;

    ros::spin();

    return 0;
}

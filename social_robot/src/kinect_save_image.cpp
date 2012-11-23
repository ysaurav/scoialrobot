#include <ros/ros.h>
#include "kinect_proxy.h"

int main ( int argc, char **argv )
{
  ros::init ( argc, argv, "kinect_save_image" );
  init_kinect();
  return 0;
}

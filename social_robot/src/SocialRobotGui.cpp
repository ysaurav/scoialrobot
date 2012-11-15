#include "SocialRobotGui.h"

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "cv_utils.h"

using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;

SocialRobotGui::SocialRobotGui ( int argc, char** argv ) :
    init_argc ( argc ),
    init_argv ( argv )
{

}

SocialRobotGui::~SocialRobotGui()
{
  ros::shutdown(); // calling explicitly because we called ros::start()
  std::cout << "Waiting for ros thread to finish." << std::endl;
  wait();
}

void SocialRobotGui::init()
{
  ros::init ( init_argc,init_argv, "social_robot_gui" );
  ros::start(); // our node handles go out of scope, so we want to control shutdown explicitly.
  ros::NodeHandle nh;
  rgb_subscriber = nh.subscribe ( "/camera/rgb/image_color", 1, &SocialRobotGui::rgb_cb, this );
  rgb_rois_subs = nh.subscribe ( "/social_robot/rgb_rois", 1, &SocialRobotGui::rgb_rois_cb, this );
  depth_rois_subs = nh.subscribe ( "/social_robot/depth/rois", 1, &SocialRobotGui::depth_rois_cb, this );
  start();
}

void SocialRobotGui::run()
{
  ros::spin();
}

void SocialRobotGui::rgb_cb ( const sensor_msgs::ImageConstPtr &msg )
{
  try
    {
      Mat image_rgb = cv_bridge::toCvCopy ( msg, enc::BGR8 )->image;
      draw_depth_faces ( image_rgb, depth_rois );
      emit update_image ( image_rgb );
    }
  catch ( cv_bridge::Exception& e )
    {
      ROS_ERROR ( "cv_bridge exception: %s", e.what() );
      return;
    }
}

void SocialRobotGui::rgb_rois_cb ( const social_robot::RegionOfInterests &msg )
{
  vector<RegionOfInterest> rois = msg.rois;
  vector<Rect> tmprgb_rois = ros_utils.rosrois2cvrects ( rois );
  rgb_rois.swap ( tmprgb_rois );
}

void SocialRobotGui::depth_rois_cb ( const social_robot::RegionOfInterests &msg )
{
  vector<RegionOfInterest> rois = msg.rois;
  vector<Rect> tmpdepth_rois = ros_utils.rosrois2cvrects ( rois );
  depth_rois.swap ( tmpdepth_rois );
}
#include "SocialRobotGui.h"

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>

using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;

SocialRobotGui::SocialRobotGui ( int argc, char** argv ) :
    init_argc ( argc ),
    init_argv ( argv )
{
  display_rgb_faces = false;
  display_depth_faces = false;
  display_track_faces = true;
  display_rgb_image = true;  
  display_depth_image = false;
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
  depth_subscriber = nh.subscribe ( "/camera/depth_registered/image_raw", 1, &SocialRobotGui::depth_cb, this );
  rgb_rois_subs = nh.subscribe ( "/social_robot/rgb/rois", 1, &SocialRobotGui::rgb_rois_cb, this );
  depth_rois_subs = nh.subscribe ( "/social_robot/depth/rois", 1, &SocialRobotGui::depth_rois_cb, this );
  track_rois_subs = nh.subscribe ( "/social_robot/track/rois", 1, &SocialRobotGui::track_rois_cb, this );
  depth_update_client = nh.serviceClient<std_srvs::Empty> ( "/social_robot/depth/update" );
  track_update_client = nh.serviceClient<std_srvs::Empty> ( "/social_robot/track/update" );
  start();
}

void SocialRobotGui::run()
{
  ros::spin();
}

void SocialRobotGui::depth_cb ( const sensor_msgs::ImageConstPtr& msg )
{
  if ( display_depth_image )
    {
      try
        {
          cv_bridge::CvImagePtr cv_depth = cv_bridge::toCvCopy ( msg );
          Mat image_depth = cv_depth->image;
          if ( display_rgb_faces )
            {
              cv_utils.draw_rgb_faces ( image_depth, rgb_rois );
            }
          if ( display_depth_faces )
            {
              cv_utils.draw_depth_faces ( image_depth, depth_rois );
            }
          if ( display_track_faces )
            {
              cv_utils.draw_depth_faces ( image_depth, track_rois );
            }
          emit update_image ( image_depth );
        }
      catch ( cv_bridge::Exception& e )
        {
          ROS_ERROR ( "cv_bridge exception: %s", e.what() );
          return;
        }
    }
}

void SocialRobotGui::rgb_cb ( const sensor_msgs::ImageConstPtr &msg )
{
  if ( display_rgb_image )
    {
      try
        {
          Mat image_rgb = cv_bridge::toCvCopy ( msg, enc::BGR8 )->image;
          if ( display_rgb_faces )
            {
              cv_utils.draw_rgb_faces ( image_rgb, rgb_rois );
            }
          if ( display_depth_faces )
            {
              cv_utils.draw_depth_faces ( image_rgb, depth_rois );
            }
          if ( display_track_faces )
            {
              cv_utils.draw_depth_faces ( image_rgb, track_rois );
            }
          emit update_image ( image_rgb );
        }
      catch ( cv_bridge::Exception& e )
        {
          ROS_ERROR ( "cv_bridge exception: %s", e.what() );
          return;
        }
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

void SocialRobotGui::track_rois_cb ( const social_robot::RegionOfInterests &msg )
{
  vector<RegionOfInterest> rois = msg.rois;
  vector<Rect> tmptrack_rois = ros_utils.rosrois2cvrects ( rois );
  track_rois.swap ( tmptrack_rois );
}

void SocialRobotGui::threshold_template_matching_3d ( double threshold )
{
  ros::NodeHandle nh;
  nh.setParam ( "/social_robot/depth/match3D_thr", threshold );
  depth_update_client.call<std_srvs::Empty> ( empty );
}

void SocialRobotGui::threshold_scales ( int threshold )
{
  ros::NodeHandle nh;
  nh.setParam ( "/social_robot/depth/scales", threshold );
  depth_update_client.call<std_srvs::Empty> ( empty );
}

void SocialRobotGui::threshold_chamfer ( double threshold )
{
  ros::NodeHandle nh;
  nh.setParam ( "/social_robot/depth/chamfer_thr", threshold );
  depth_update_client.call<std_srvs::Empty> ( empty );
}

void SocialRobotGui::threshold_arc_low ( int threshold )
{
  ros::NodeHandle nh;
  nh.setParam ( "/social_robot/depth/arc_thr_low", threshold );
  depth_update_client.call<std_srvs::Empty> ( empty );
}

void SocialRobotGui::threshold_arc_high ( int threshold )
{
  ros::NodeHandle nh;
  nh.setParam ( "/social_robot/depth/arc_thr_high", threshold );
  depth_update_client.call<std_srvs::Empty> ( empty );
}

void SocialRobotGui::threshold_confidence ( double threshold )
{
  ros::NodeHandle nh;
  nh.setParam ( "/social_robot/track/confidence_level_thr", threshold );
  track_update_client.call<std_srvs::Empty> ( empty );
}

void SocialRobotGui::threshold_detection ( double threshold )
{
  ros::NodeHandle nh;
  nh.setParam ( "/social_robot/track/detection_confidence_thr", threshold );
  track_update_client.call<std_srvs::Empty> ( empty );
}

void SocialRobotGui::threshold_eucdis ( double threshold )
{
  ros::NodeHandle nh;
  nh.setParam ( "/social_robot/track/track_thr", threshold );
  track_update_client.call<std_srvs::Empty> ( empty );
}
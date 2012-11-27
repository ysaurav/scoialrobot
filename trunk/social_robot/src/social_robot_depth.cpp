#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <stereo_msgs/DisparityImage.h>
#include <social_robot/RegionOfInterests.h>
#include <omp.h>

#include "kinect_proxy.h"
#include "CvUtils.h"
#include "RosUtils.h"
#include "social_robot_constants.h"

using namespace std;
using namespace cv;
using namespace sensor_msgs;

namespace enc = image_encodings;

// for publications
RosUtils ros_utils;
social_robot::RegionOfInterests depth_pub_rois;
ros::Publisher depth_pub;

// values that can be chanegd during the runtime
double canny_thr1 = 5;
double canny_thr2 = 7;
double chamfer_thr = 10;
double arc_thr_low = 7;
double arc_thr_high = 20;
double approx_poly_thr = 1;
double max_suppression = 0.1;
double scale_factor = 0.75;
double match3D_thr = 0.4;
int scales = 6;
int framenum = 0;
int update_rate = 15;

Mat image_depth;
Mat image_disparity;

CvUtils cv_utils;

bool update_param_cb ( std_srvs::Empty::Request&, std_srvs::Empty::Response& )
{
  // TODO: update depth face detector from here
  ROS_INFO ( "Updating parameter of social_robot_depth" );

  double chamfer_thr_tmp = chamfer_thr;
  double arc_thr_low_tmp = arc_thr_low;
  double arc_thr_high_tmp = arc_thr_high;
  int scales_tmp = scales;
  double max_suppression_tmp = max_suppression;
  double match3D_thr_tmp = match3D_thr;
  double update_rate_tmp = update_rate;

  ros::NodeHandle nh;

  nh.param ( "/social_robot/depth/chamfer_thr", chamfer_thr_tmp, chamfer_thr_tmp );
  nh.param ( "/social_robot/depth/scales", scales_tmp, scales_tmp );
  nh.param ( "/social_robot/depth/arc_thr_low", arc_thr_low_tmp, arc_thr_low_tmp );
  nh.param ( "/social_robot/depth/arc_thr_high", arc_thr_high_tmp, arc_thr_high_tmp );
  nh.param ( "/social_robot/depth/max_suppression", max_suppression_tmp, max_suppression_tmp );
  nh.param ( "/social_robot/depth/match3D_thr", match3D_thr_tmp, match3D_thr_tmp );
  nh.param ( "/social_robot/depth/update_rate", update_rate_tmp, update_rate_tmp );

  chamfer_thr = chamfer_thr_tmp;
  scales = scales_tmp;
  arc_thr_low = arc_thr_low_tmp;
  arc_thr_high = arc_thr_high_tmp;
  max_suppression = max_suppression_tmp;
  match3D_thr = match3D_thr_tmp;
  update_rate = update_rate_tmp;

  return true;
}

void publish_data ( vector<Rect> depthfaces )
{
  vector<RegionOfInterest> rosrois = ros_utils.cvrects2rosrois ( depthfaces );
  depth_pub_rois.rois.swap ( rosrois );
  depth_pub.publish ( depth_pub_rois );;
}

void depth_cb ( const ImageConstPtr& msg )
{
  try
    {
      image_depth = cv_bridge::toCvCopy ( msg )->image;
    }
  catch ( cv_bridge::Exception& e )
    {
      ROS_ERROR ( "cv_bridge exception: %s", e.what() );
      return;
    }
}

void disparity_cb ( const stereo_msgs::DisparityImageConstPtr& msg )
{
  try
    {
      framenum++;
      image_disparity = cv_bridge::toCvCopy ( msg->image )->image;
      image_disparity.convertTo ( image_disparity, CV_8UC1 );

      if ( image_depth.empty() )
        {
          return;
        }
      if ( framenum >= update_rate )
        {
          framenum = 0;
          vector<Rect> depthfaces = cv_utils.detect_face_depth ( image_depth, image_disparity );
          publish_data ( depthfaces );
        }
    }
  catch ( cv_bridge::Exception& e )
    {
      ROS_ERROR ( "cv_bridge exception: %s", e.what() );
      return;
    }
}

int main ( int argc, char **argv )
{
  ros::init ( argc, argv, "social_robot_depth" );
  ros::NodeHandle nh;

  // to register the depth
  nh.setParam ( "/camera/driver/depth_registration", true );

  nh.setParam ( "/social_robot/depth/chamfer_thr", chamfer_thr );
  nh.setParam ( "/social_robot/depth/scales", scales );
  nh.setParam ( "/social_robot/depth/arc_thr_low", arc_thr_low );
  nh.setParam ( "/social_robot/depth/arc_thr_high", arc_thr_high );
  nh.setParam ( "/social_robot/depth/max_suppression", max_suppression );
  nh.setParam ( "/social_robot/depth/match3D_thr", match3D_thr );
  nh.setParam ( "/social_robot/depth/update_rate", update_rate );

  // subscribtions
  ros::ServiceServer update_srv = nh.advertiseService ( "/social_robot/depth/update", update_param_cb );
  ros::Subscriber disparity_sub = nh.subscribe ( "/camera/depth/disparity", 1, disparity_cb );
  ros::Subscriber depth_sub = nh.subscribe ( "/camera/depth/image_raw", 1, depth_cb );

  // publications
  depth_pub = nh.advertise<social_robot::RegionOfInterests> ( "/social_robot/depth/rois", 1 );

  ros::spin();

  return 0;
}

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <social_robot/RegionOfInterests.h>

#include "kinect_proxy.h"
#include "CvUtils.h"
#include "RosUtils.h"

using namespace std;
using namespace cv;
using namespace sensor_msgs;

namespace enc = image_encodings;

// for publications
RosUtils ros_utils;
social_robot::RegionOfInterests rgb_pub_rois;
ros::Publisher rgb_pub;

int framenum = 0;
int update_rate = 15;

CvUtils cv_utils;

bool update_param_cb ( std_srvs::Empty::Request&, std_srvs::Empty::Response& )
{
  ROS_INFO ( "Updating parameter of social_robot_rgb" );

  double update_rate_tmp = update_rate;

  ros::NodeHandle nh;

  nh.param ( "/social_robot/rgb/update_rate", update_rate_tmp, update_rate_tmp );

  update_rate = update_rate_tmp;

  return true;
}

void publish_data ( vector<Rect> rgbfaces )
{
  vector<RegionOfInterest> rosrois = ros_utils.cvrects2rosrois ( rgbfaces );
  rgb_pub_rois.rois.swap ( rosrois );
  rgb_pub.publish ( rgb_pub_rois );
}

void rgb_cb ( const ImageConstPtr& msg )
{
  try
    {
      framenum++;
      Mat image_rgb = cv_bridge::toCvCopy ( msg, enc::BGR8 )->image;

      if ( framenum == update_rate )
        {
          framenum = 0;
          vector<Rect> rgbfaces = cv_utils.detect_face_rgb ( image_rgb );
          publish_data ( rgbfaces );
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
  ros::init ( argc, argv, "social_robot_rgb" );
  ros::NodeHandle nh;
  
  nh.setParam ( "/social_robot/rgb/update_rate", update_rate );

  // subscribtions
  ros::ServiceServer update_srv = nh.advertiseService ( "/social_robot/rgb/update", update_param_cb );
  ros::Subscriber rgb_sub = nh.subscribe ( "/camera/rgb/image_color", 1, rgb_cb );

  // publications
  rgb_pub = nh.advertise<social_robot::RegionOfInterests> ( "/social_robot/rgb/rois", 1 );

  ros::spin();

  return 0;
}
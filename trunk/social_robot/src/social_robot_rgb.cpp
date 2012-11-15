#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <social_robot/RegionOfInterests.h>

#include "kinect_proxy.h"
#include "cv_utils.h"
#include "RosUtils.h"

using namespace std;
using namespace cv;
using namespace sensor_msgs;

namespace enc = image_encodings;

string cascade_name;
CascadeClassifier classifier;

// for publications
RosUtils ros_utils;
social_robot::RegionOfInterests rgb_pub_rois;
ros::Publisher rgb_pub;

vector<Rect> detect_face_rgb ( Mat img, CascadeClassifier &cascade )
{
  vector<Rect> tmpfaces;

  Mat gray;
  Mat frame ( cvRound ( img.rows ), cvRound ( img.cols ), CV_8UC1 );

  cvtColor ( img, gray, CV_BGR2GRAY );
  resize ( gray, frame, frame.size(), 0, 0, INTER_LINEAR );
  equalizeHist ( frame, frame );

  cascade.detectMultiScale ( frame, tmpfaces,
                             1.1, 2, 0
                             //|CV_HAAR_FIND_BIGGEST_OBJECT
                             //|CV_HAAR_DO_ROUGH_SEARCH
                             |CV_HAAR_SCALE_IMAGE
                             ,
                             Size ( 30, 30 ) );

  vector<RegionOfInterest> rosrois = ros_utils.cvrects2rosrois ( tmpfaces );
  rgb_pub_rois.rois.swap ( rosrois );
  rgb_pub.publish ( rgb_pub_rois );
  return tmpfaces;
}

bool update_param_cb ( std_srvs::Empty::Request&, std_srvs::Empty::Response& )
{
  ROS_INFO ( "Updating parameter of social_robot" );

  /*
  string head_template_tmp;
  double canny_thr1_tmp;
  double canny_thr2_tmp;
  double chamfer_thr_tmp;
  double arc_thr_tmp;
  int scales_tmp;
  double max_suppression_tmp;

  ros::NodeHandle nh;

  nh.param ( "/social_robot/head_template", head_template_tmp, head_template_tmp );
  nh.param ( "/social_robot/canny_thr1", canny_thr1_tmp, canny_thr1_tmp );
  nh.param ( "/social_robot/canny_thr2", canny_thr2_tmp, canny_thr2_tmp );
  nh.param ( "/social_robot/chamfer_thr", chamfer_thr_tmp, chamfer_thr_tmp );
  nh.param ( "/social_robot/scales", scales_tmp, scales_tmp );
  nh.param ( "/social_robot/arc_thr", arc_thr_tmp, arc_thr_tmp );
  nh.param ( "/social_robot/max_suppression", max_suppression_tmp, max_suppression_tmp );

  head_template1 = head_template_tmp;
  canny_thr1 = canny_thr1_tmp;
  canny_thr2 = canny_thr2_tmp;
  chamfer_thr = chamfer_thr_tmp;
  scales = scales_tmp;
  arc_thr_low = arc_thr_tmp;
  max_suppression = max_suppression_tmp;
  */

  return true;
}

void rgb_cb ( const ImageConstPtr& msg )
{
  try
    {
      Mat image_rgb = cv_bridge::toCvCopy ( msg, enc::BGR8 )->image;
      vector<Rect> rgbfacestmp = detect_face_rgb ( image_rgb, classifier );
    }
  catch ( cv_bridge::Exception& e )
    {
      ROS_ERROR ( "cv_bridge exception: %s", e.what() );
      return;
    }
}

bool load_classifiers ()
{
  string package_path = ros::package::getPath ( "social_robot" );

  cascade_name.append ( package_path );
  cascade_name.append ( "/rsrc/haarcascades/haarcascade_frontalface_alt.xml" );

  if ( !classifier.load ( cascade_name ) )
    {
      cerr << "ERROR: Could not load cascade classifier \"" << cascade_name << "\"" << endl;
      return false;
    }

  return true;
}

int main ( int argc, char **argv )
{
  ros::init ( argc, argv, "social_robot_rgb" );
  ros::NodeHandle nh;

  if ( !load_classifiers() )
    {
      return -1;
    }

  // subscribtions
  ros::ServiceServer update_srv = nh.advertiseService ( "/social_robot/update", update_param_cb );
  ros::Subscriber rgb_sub = nh.subscribe ( "/camera/rgb/image_color", 1, rgb_cb );

  // publications
  rgb_pub = nh.advertise<social_robot::RegionOfInterests> ( "/social_robot/rgb/rois", 1 );

  ros::spin();

  return 0;
}
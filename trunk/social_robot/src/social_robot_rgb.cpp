#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <social_robot/RegionOfInterests.h>

#include "kinect_proxy.h"
#include "CvUtils.h"
#include "RosUtils.h"
#include "particle_filter/StateData.h"
#include "particle_filter/hist.h"
#include "particle_filter/filter.h"

using namespace std;
using namespace cv;
using namespace sensor_msgs;

namespace enc = image_encodings;

CascadeClassifier classifier;

// for publications
RosUtils ros_utils;
social_robot::RegionOfInterests rgb_pub_rois;
ros::Publisher rgb_pub;

int framenum = 0;
int update_rate = 30;

CvUtils cv_utils;

vector<Rect> detect_face_rgb ( Mat img )
{
  vector<Rect> detected_faces;

  Mat gray;
  Mat frame ( cvRound ( img.rows ), cvRound ( img.cols ), CV_8UC1 );

  cvtColor ( img, gray, CV_BGR2GRAY );
  resize ( gray, frame, frame.size(), 0, 0, INTER_LINEAR );
  equalizeHist ( frame, frame );

  classifier.detectMultiScale ( frame, detected_faces,
                                1.1, 2, 0
                                //|CV_HAAR_FIND_BIGGEST_OBJECT
                                //|CV_HAAR_DO_ROUGH_SEARCH
                                |CV_HAAR_SCALE_IMAGE
                                ,
                                Size ( 30, 30 ) );

  return detected_faces;
}

bool update_param_cb ( std_srvs::Empty::Request&, std_srvs::Empty::Response& )
{
  ROS_INFO ( "Updating parameter of social_robot" );

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
          vector<Rect> rgbfaces = detect_face_rgb ( image_rgb );
          publish_data ( rgbfaces );
        }
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

  string cascade_name;
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
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

string cascade_name;
CascadeClassifier classifier;

// for publications
RosUtils ros_utils;
social_robot::RegionOfInterests rgb_pub_rois;
ros::Publisher rgb_pub;

vector<StateData> state_datas;
int framenum = 0;

CvUtils cv_utils;

vector<Rect> detect_face_rgb ( Mat img, CascadeClassifier &cascade )
{
  vector<Rect> detected_faces;

  Mat gray;
  Mat frame ( cvRound ( img.rows ), cvRound ( img.cols ), CV_8UC1 );

  cvtColor ( img, gray, CV_BGR2GRAY );
  resize ( gray, frame, frame.size(), 0, 0, INTER_LINEAR );
  equalizeHist ( frame, frame );

  cascade.detectMultiScale ( frame, detected_faces,
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

void publish_data ( void )
{
  vector<Rect> detected_faces ( state_datas.size() );
  for ( unsigned int i = 0; i < state_datas.size(); i++ )
    {
      detected_faces[i] = state_datas[i].get_target_position();
    }
  vector<RegionOfInterest> rosrois = ros_utils.cvrects2rosrois ( detected_faces );
  rgb_pub_rois.rois.swap ( rosrois );
  rgb_pub.publish ( rgb_pub_rois ); 
}

void rgb_cb ( const ImageConstPtr& msg )
{
  try
    {
      framenum++;
      Mat image_rgb = cv_bridge::toCvCopy ( msg, enc::BGR8 )->image;
      Mat gray;

      if ( framenum == 25 )
        {
          framenum = 0;
          vector<Rect> rgbfaces = detect_face_rgb ( image_rgb, classifier );
          for ( unsigned int i = 0; i < rgbfaces.size(); i++ )
            {              
              Point face_centre = cv_utils.get_rect_centre ( rgbfaces[i] );
              bool associated = false;
              for ( unsigned int j = 0; j < state_datas.size(); j++ )
                {
                  Point track_centre = cv_utils.get_rect_centre ( state_datas[j].get_target_position() );
                  double euc_dis = cv_utils.euclidean_distance ( face_centre, track_centre );

                  if ( euc_dis < 50 )
                    {
                      associated = true;
//                       state_datas[j].update_target_histogram ( image_rgb, rgbfaces[i] );
                    }
                }
              if ( !associated )
                {
                  StateData state_data;
                  state_data.initialise( 200, image_rgb, rgbfaces[i], image_rgb, 0 );
                  state_datas.push_back( state_data );
                }
            }
        }
      for ( vector<StateData>::iterator it = state_datas.begin(); it != state_datas.end();  )
        {
          it->image = image_rgb;
          it->tracking();
          if ( it->filter->confidence() < 0.025 )
            {
              it = state_datas.erase(it);
            }
          else
            {
              it++;
            }
        }
      publish_data();
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
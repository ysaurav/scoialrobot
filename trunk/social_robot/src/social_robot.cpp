#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <stereo_msgs/DisparityImage.h>
#include <social_robot/RegionOfInterests.h>
#include <omp.h>

#include "kinect_proxy.h"
#include "CvUtils.h"
#include "PixelSimilarity.h"
#include "RosUtils.h"
#include "Template.h"
#include "social_robot_constants.h"

#include "particle_filter/StateData.h"
#include "particle_filter/hist.h"
#include "particle_filter/filter.h"

using namespace std;
using namespace cv;
using namespace sensor_msgs;

namespace enc = image_encodings;

// for publications
RosUtils ros_utils;
social_robot::RegionOfInterests depth_pub_rois;
ros::Publisher depth_pub;

// TODO: put value on detetion to kill outliers or keep objects if they have over 90 threshold
double track_thr = 75;
double confidence_level_thr = 0.50;
double detection_confidence_thr = 0.75;
int num_particles = 300;
int rgb_framenum = 0;
int rgb_update_rate = 15;
int depth_framenum = 0;
int depth_update_rate = 15;

Mat image_disparity;
Mat image_depth;
Mat image_rgb;

vector<StateData> state_datas;

CvUtils cv_utils;

void publish_data ( void )
{
  vector<Rect> detected_faces ( state_datas.size() );
  for ( unsigned int i = 0; i < state_datas.size(); i++ )
    {
      detected_faces[i] = state_datas[i].get_target_position();
    }
  vector<RegionOfInterest> rosrois = ros_utils.cvrects2rosrois ( detected_faces );
  depth_pub_rois.rois.swap ( rosrois );
  depth_pub.publish ( depth_pub_rois );
}

void do_tracking ( void )
{
  int i = 0;
  for ( vector<StateData>::iterator it = state_datas.begin(); it != state_datas.end(); )
    {
      it->set_image_depth ( image_disparity );
      it->image = image_rgb;
      it->tracking ( 0.02 );

      if ( it->detection_confidence < ( detection_confidence_thr / 2 ) )
        {
          if ( cv_utils.is_there_face_rgb ( image_rgb, it->get_target_position() ) )
            {
              it->detection_confidence = 1.0;
              it++;
            }
          else
            {
              cout << i << " ### conf " << it->filter->confidence() << " detec: " << it->detection_confidence << endl;
              it = state_datas.erase ( it );
            }
        }
      else if ( it->filter->confidence() > confidence_level_thr )
        {
          it++;
        }
      else
        {
          cout << i << " *** conf " << it->filter->confidence() << " detec: " << it->detection_confidence << endl;
          it = state_datas.erase ( it );
        }
      i++;
    }
  publish_data();
}

void data_association ( vector<Rect> &faces )
{
  for ( unsigned int i = 0; i < faces.size(); i++ )
    {
//       Point3f face_centre = cv_utils.get_rect_centre_3d ( faces[i], image_depth );
      Point face_centre = cv_utils.get_rect_centre ( faces[i] );
      bool associated = false;
      for ( unsigned int j = 0; j < state_datas.size(); j++ )
        {
//           if ( state_datas[j].is_associated )
//             {
//               continue;
//             }
//           Point3f track_centre = cv_utils.get_rect_centre_3d ( state_datas[j].get_target_position(), image_depth );
          Point track_centre = cv_utils.get_rect_centre ( state_datas[j].get_target_position() );
          double euc_dis = cv_utils.euclidean_distance ( face_centre, track_centre );

//           cout << "Euc " << euc_dis << ", " << j << endl;
          if ( euc_dis < track_thr )
            {
              associated = true;
              state_datas[j].detection_confidence = 1.0;
              state_datas[j].is_associated = true;
//               state_datas[j].update_target_histogram ( image_rgb, image_disparity, faces[i] );
              break;
            }
        }
      if ( !associated )
        {
          StateData state_data;
          state_data.initialise ( num_particles, image_rgb, faces[i], image_disparity, HIST_HS );
          state_datas.push_back ( state_data );
        }
    }
  faces.clear();
}

bool update_param_cb ( std_srvs::Empty::Request&, std_srvs::Empty::Response& )
{
  ROS_INFO ( "Updating parameter of social_robot" );

  double confidence_level_thr_tmp = confidence_level_thr;
  double detection_confidence_thr_tmp = detection_confidence_thr;
  double track_thr_tmp = track_thr;

  ros::NodeHandle nh;

  nh.param ( "/social_robot/track/confidence_level_thr", confidence_level_thr_tmp, confidence_level_thr_tmp );
  nh.param ( "/social_robot/track/detection_confidence_thr", detection_confidence_thr_tmp, detection_confidence_thr_tmp );
  nh.param ( "/social_robot/track/track_thr", track_thr_tmp, track_thr_tmp );

  confidence_level_thr = confidence_level_thr_tmp;
  detection_confidence_thr = detection_confidence_thr_tmp;
  track_thr = track_thr_tmp;

  return true;
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

void rgb_cb ( const ImageConstPtr& msg )
{
  try
    {
      rgb_framenum++;
      image_rgb = cv_bridge::toCvCopy ( msg, enc::BGR8 )->image;
      
      if ( rgb_framenum == rgb_update_rate )
        {
          rgb_framenum = 0;
          vector<Rect> rgb_faces = cv_utils.detect_face_rgb ( image_rgb );
          data_association ( rgb_faces );
        }
      
      do_tracking();
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
      depth_framenum++;
      image_disparity = cv_bridge::toCvCopy ( msg->image )->image;
      image_disparity.convertTo ( image_disparity, CV_8UC1 );
      
      if ( image_depth.empty() )
        {
          return;
        }
      if ( depth_framenum == depth_update_rate )
        {
          depth_framenum = 0;
          vector<Rect> depth_faces = cv_utils.detect_face_depth ( image_depth, image_disparity );
          data_association ( depth_faces );
        }
    }
  catch ( cv_bridge::Exception& e )
    {
      ROS_ERROR ( "cv_bridge exception: %s", e.what() );
      return;
    }
}

void rgb_rois_cb ( const social_robot::RegionOfInterests &msg )
{
  vector<RegionOfInterest> rois = msg.rois;
  vector<Rect> rgb_faces = ros_utils.rosrois2cvrects ( rois );
  data_association ( rgb_faces );
}

void depth_rois_cb ( const social_robot::RegionOfInterests &msg )
{
  vector<RegionOfInterest> rois = msg.rois;
  vector<Rect> depth_faces = ros_utils.rosrois2cvrects ( rois );
  data_association ( depth_faces );
}

int main ( int argc, char **argv )
{
  ros::init ( argc, argv, "social_robot_track" );
  ros::NodeHandle nh;
//   ros::MultiThreadedSpinner spinner ( 0 );

  // to register the depth
  nh.setParam ( "/camera/driver/depth_registration", true );

  nh.setParam ( "/social_robot/track/confidence_level_thr", confidence_level_thr );
  nh.setParam ( "/social_robot/track/detection_confidence_thr", detection_confidence_thr );
  nh.setParam ( "/social_robot/track/track_thr", track_thr );

  // subscribtions
  ros::ServiceServer update_srv = nh.advertiseService ( "/social_robot/track/update", update_param_cb );
  ros::Subscriber disparity_sub = nh.subscribe ( "/camera/depth/disparity", 1, disparity_cb );
  ros::Subscriber depth_sub = nh.subscribe ( "/camera/depth/image_raw", 1, depth_cb );
  ros::Subscriber rgb_sub = nh.subscribe ( "/camera/rgb/image_color", 1, rgb_cb );

  ros::Subscriber rgb_rois_sub = nh.subscribe ( "/social_robot/rgb/rois", 1, rgb_rois_cb );
  ros::Subscriber depth_rois_sub = nh.subscribe ( "/social_robot/depth/rois", 1, depth_rois_cb );

  // publications
  depth_pub = nh.advertise<social_robot::RegionOfInterests> ( "/social_robot/track/rois", 1 );

//   spinner.spin();
  ros::spin();

  return 0;
}

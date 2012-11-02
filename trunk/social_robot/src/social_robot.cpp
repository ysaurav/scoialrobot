#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <stereo_msgs/DisparityImage.h>

#include "kinect_proxy.h"
#include "cv_utils.h"

using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;

string cascade_name;
CascadeClassifier classifier;

vector<Rect> rgb_faces;
vector<Rect> depth_faces;

// values that can be chanegd during the runtime
string head_template;
double canny_thr1 = 5;
double canny_thr2 = 7;
double chamfer_thr = 10;
double arc_thr_low = 10;
double arc_thr_high = 17;
double approx_poly_thr = 1;
int scales = 5;

bool is_rgb_turn = true;

vector<Point> head_matched_points;
vector<Point3f> head_features;

int roi_x_offset;
int roi_y_offset;
int roi_height;
int roi_width;

Mat *pyramid = new Mat[scales];
Mat *chamfer = new Mat[scales];
Mat *matching = new Mat[scales];

Mat canny_im;
Mat image_rgb;
Mat image_depth;
Mat image_disparity;

vector<Point> chamfer_matching ( Mat image );
vector<Point3f> compute_headparameters ( Mat image, vector<Point> chamfer );

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
  return tmpfaces;
}

vector<Rect> detect_face_depth ( Mat tmp_depth, Mat tmp_disparity )
{
  vector<Rect> roistmp;

  tmp_disparity.convertTo ( tmp_disparity, CV_8UC1 );

  // preprocessing
  tmp_disparity = preprocessing ( tmp_disparity );
  // FIXME there should be a way around all this conversions
  tmp_depth.convertTo ( tmp_depth, CV_8UC1 );
  tmp_depth = preprocessing ( tmp_depth );
  tmp_depth.convertTo ( tmp_depth, CV_16UC1 );

  head_matched_points = chamfer_matching ( tmp_disparity );
  head_features = compute_headparameters ( tmp_depth, head_matched_points );

  float tol = 12;

  Point3f tmpparams_av;
  vector<Point3f> output_v;
  vector<Point3f> queue;
  /*
    while ( head_features.size() > 0 )
      {
        tmpparams_av.x = head_features[0].x;
        tmpparams_av.y = head_features[0].y;
        tmpparams_av.z = head_features[0].z;

        for ( unsigned int i = 1; i < head_features.size(); i++ )
          {
            if ( abs ( tmpparams_av.x - head_features[i].x ) + abs ( tmpparams_av.y - head_features[i].y ) < tol )
              {
                tmpparams_av.x = ( tmpparams_av.x + head_features[i].x ) /2;
                tmpparams_av.y = ( tmpparams_av.y + head_features[i].y ) /2;
                tmpparams_av.z = ( ( tmpparams_av.z + head_features[i].z ) /2 ) * 1.2;
              }
            else
              {
                queue.push_back ( head_features[i] );
              }
          }

        output_v.push_back ( tmpparams_av );
        head_features.swap ( queue );
        queue.clear();
      }
    head_features.swap ( output_v );
  */
  // FIXME: make this code more eeficient
  for ( unsigned int i = 0; i < head_features.size(); i++ )
    {
      if ( head_features[i].z < 0 )
        {
          head_features[i].z = 0;
          continue;
        }

      int wh = head_features[i].z * 2;
      int tlx = head_features[i].x - head_features[i].z;
      int tly = head_features[i].y - head_features[i].z;

      Rect roi ( tlx, tly, wh, wh );
      if ( ! ( 0 <= roi.x && 0 <= roi.width && roi.x + roi.width <= canny_im.cols && 0 <= roi.y && 0 <= roi.height && roi.y + roi.height <= canny_im.rows ) )
        {
          continue;
        }
      Mat roi_canny ( canny_im, roi );
      if ( roi_canny.rows == 0 )
        {
          continue;
        }
      vector<vector<Point> > contours;
      findContours ( roi_canny, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE );

      for ( unsigned int j = 0; j < contours.size(); j++ )
        {
          vector<Point> approx;
          approxPolyDP ( contours[j], approx, approx_poly_thr, false );
          if ( approx.size() >= arc_thr_low && approx.size() <= arc_thr_high )
            {
              roistmp.push_back ( roi );
              break;
            }
        }
    }
  return roistmp;
}

vector<Point> chamfer_matching ( Mat image )
{
  canny_im.create ( image.rows, image.cols, image.depth() );
  Mat template_im = imread ( head_template, CV_LOAD_IMAGE_ANYDEPTH );

  // calculate edge detection
  Canny ( image, canny_im, canny_thr1, canny_thr2, 3, true );

  // calculate the Canny pyramid
  pyramid[0] = canny_im;
  for ( int i = 1; i < scales; i++ )
    {
      resize ( pyramid[i - 1], pyramid[i], Size(), 0.75, 0.75, INTER_NEAREST );
    }

  // calculate distance transform
  for ( int i = 0; i < scales; i++ )
    {
      distanceTransform ( ( 255 - pyramid[i] ), chamfer[i], CV_DIST_C, 3 );
      normalize ( chamfer[i], chamfer[i], 0.0, 1.0, NORM_MINMAX );
    }

  // matching with the template
  template_im = rgb2bw ( template_im );
  template_im.convertTo ( template_im, CV_32F );

  // find the best match:
  double minVal, maxVal;
  Point minLoc, maxLoc;
  double xdiff = template_im.cols / 2;
  double ydiff = template_im.rows / 2;
  Point pdiff = Point ( xdiff, ydiff );
  vector<Point> head_matched_points_tmp;

  for ( int i = 0; i < scales; i++ )
    {
      matchTemplate ( chamfer[i], template_im, matching[i], CV_TM_CCOEFF );
      normalize ( matching[i], matching[i], 0.0, 1.0, NORM_MINMAX );
      minMaxLoc ( matching[i], &minVal, &maxVal, &minLoc, &maxLoc );
      Mat matching_thr;
      threshold ( matching[i], matching_thr, 1.0 / chamfer_thr, 1.0, CV_THRESH_BINARY_INV );
      double scale = pow ( 1.0 / 0.75, i );
      get_non_zeros ( matching_thr, &head_matched_points_tmp, pdiff, scale );
    }

  return head_matched_points_tmp;
}

vector<Point3f> compute_headparameters ( Mat image, vector<Point> chamfer )
{
  vector<Point3f> parameters_head ( chamfer.size() );

  // parameters of cubic equation
  float p1 = -1.3835 * pow ( 10, -9 );
  float p2 =  1.8435 * pow ( 10, -5 );
  float p3 = -0.091403;
  float p4 =  189.38;

  for ( unsigned int i = 0; i < chamfer.size(); i++ )
    {
      int position_x = chamfer[i].x;
      int position_y = chamfer[i].y;

      unsigned short x = image.at<unsigned short> ( position_y, position_x );

      // compute height of head
      float h = ( p1 * pow ( x, 3 ) + p2 * pow ( x, 2 ) + p3 * x + p4 );

      // compute Radius of head in milimeters
      float R = 1.33 * h / 2;

      // convert Radius in pixels
      float Rp = round ( ( 1 / 1.3 ) * R );

      parameters_head[i].x = position_x;
      parameters_head[i].y = position_y;
      parameters_head[i].z = Rp;
    }

  return parameters_head;
}

bool update_param_cb ( std_srvs::Empty::Request&, std_srvs::Empty::Response& )
{
  ROS_INFO ( "Updating parameter of social_robot" );

  string head_template_tmp;
  double canny_thr1_tmp;
  double canny_thr2_tmp;
  double chamfer_thr_tmp;
  double arc_thr_tmp;
  int scales_tmp;

  ros::NodeHandle nh;

  nh.param ( "/social_robot/head_template", head_template_tmp, head_template_tmp );
  nh.param ( "/social_robot/canny_thr1", canny_thr1_tmp, canny_thr1_tmp );
  nh.param ( "/social_robot/canny_thr2", canny_thr2_tmp, canny_thr2_tmp );
  nh.param ( "/social_robot/chamfer_thr", chamfer_thr_tmp, chamfer_thr_tmp );
  nh.param ( "/social_robot/scales", scales_tmp, scales_tmp );
  nh.param ( "/social_robot/arc_thr", arc_thr_tmp, arc_thr_tmp );

  head_template = head_template_tmp;
  canny_thr1 = canny_thr1_tmp;
  canny_thr2 = canny_thr2_tmp;
  chamfer_thr = chamfer_thr_tmp;
  scales = scales_tmp;
  arc_thr_low = arc_thr_tmp;

  return true;
}

void rgb_cb ( const sensor_msgs::ImageConstPtr& msg )
{
  try
    {
      image_rgb = cv_bridge::toCvCopy ( msg, enc::BGR8 )->image;
      draw_rgb_faces ( image_rgb, rgb_faces );
      draw_depth_faces ( image_rgb, depth_faces );

      imshow ( "Social Robot", image_rgb );
      waitKey ( 3 );
    }
  catch ( cv_bridge::Exception& e )
    {
      ROS_ERROR ( "cv_bridge exception: %s", e.what() );
      return;
    }
}

void depth_cb ( const sensor_msgs::ImageConstPtr& msg )
{
  try
    {
      cv_bridge::CvImagePtr cv_depth = cv_bridge::toCvCopy ( msg );
      image_depth = cv_depth->image;
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
      cv_bridge::CvImagePtr cv_disparity = cv_bridge::toCvCopy ( msg->image );
      image_disparity = cv_disparity->image;
    }
  catch ( cv_bridge::Exception& e )
    {
      ROS_ERROR ( "cv_bridge exception: %s", e.what() );
      return;
    }
}

void timer_cb ( const ros::TimerEvent& event )
{
  if ( image_rgb.empty() || image_depth.empty() || image_disparity.empty() )
    {
      return;
    }

  Mat tmp_rgb = image_rgb.clone();
  Mat tmp_depth = image_depth.clone();
  Mat tmp_disparity = image_disparity.clone();

  if ( false )
    {
      vector<Rect> rgbfacestmp = detect_face_rgb ( tmp_rgb, classifier );
      rgb_faces.swap ( rgbfacestmp );
      is_rgb_turn =  false;
      return;
    }
  else
    {
      is_rgb_turn = true;
      vector<Rect> roistmp = detect_face_depth ( tmp_depth, tmp_disparity );
      depth_faces.swap ( roistmp );
    }
}

int main ( int argc, char **argv )
{
  ros::init ( argc, argv, "social_robot" );
  ros::NodeHandle nh;
  ros::MultiThreadedSpinner spinner ( 0 );

  string package_path = ros::package::getPath ( "social_robot" );
  head_template.append ( package_path );
  head_template.append ( "/pictures/template.png" );

  cascade_name.append ( package_path );
  cascade_name.append ( "/rsrc/haarcascades/haarcascade_frontalface_alt.xml" );

  if ( !classifier.load ( cascade_name ) )
    {
      cerr << "ERROR: Could not load cascade classifier \"" << cascade_name << "\"" << endl;
      return -1;
    }

  nh.setParam ( "/social_robot/head_template", head_template );
  nh.setParam ( "/social_robot/canny_thr1", canny_thr1 );
  nh.setParam ( "/social_robot/canny_thr2", canny_thr2 );
  nh.setParam ( "/social_robot/chamfer_thr", chamfer_thr );
  nh.setParam ( "/social_robot/scales", scales );
  nh.setParam ( "/social_robot/arc_thr_low", arc_thr_low );

  ros::ServiceServer clear_srv = nh.advertiseService ( "/social_robot/update", update_param_cb );
  ros::Subscriber disparity_sub = nh.subscribe ( "/camera/depth/disparity", 1, disparity_cb );
  ros::Subscriber rgb_sub = nh.subscribe ( "/camera/rgb/image_color", 1, rgb_cb );
  ros::Subscriber depth_sub = nh.subscribe ( "/camera/depth/image_raw", 1, depth_cb );

  namedWindow ( "Social Robot", CV_WINDOW_AUTOSIZE );
  ros::Timer timer = nh.createTimer ( ros::Duration ( 0.1 ), timer_cb );

  spinner.spin();

  return 0;
}

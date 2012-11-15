#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <stereo_msgs/DisparityImage.h>
#include <social_robot/RegionOfInterests.h>

#include "kinect_proxy.h"
#include "cv_utils.h"
#include "PixelSimilarity.h"
#include "RosUtils.h"
#include "Template.h"

using namespace std;
using namespace cv;
using namespace sensor_msgs;

namespace enc = image_encodings;

// for publications
RosUtils ros_utils;
social_robot::RegionOfInterests depth_pub_rois;
ros::Publisher depth_pub;
social_robot::RegionOfInterests rgb_pub_rois;
ros::Publisher rgb_pub;

string cascade_name;
CascadeClassifier classifier;

vector<Rect> rgb_faces;
vector<Rect> depth_faces;

// values that can be chanegd during the runtime
string head_template1;
string head_template2;
string head_template3D1;
string head_template3D2;
double canny_thr1 = 5;
double canny_thr2 = 7;
double chamfer_thr = 10;
double arc_thr_low = 7;
double arc_thr_high = 17;
double approx_poly_thr = 1;
double max_suppression = 0.001;
double scale_factor = 0.75;
int scales = 4;

bool is_rgb_turn = true;

vector<Point3f> head_matched_points;
vector<PixelSimilarity> head_features;

vector<Point3f> head_matched_points2;
vector<PixelSimilarity> head_features2;

vector<Template> templates;

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

vector<Point3f> chamfer_matching ( Mat image, Mat template_im );
vector<PixelSimilarity> compute_headparameters ( Mat image, vector<Point3f> chamfer );
vector<PixelSimilarity> false_positives( vector<PixelSimilarity> tmpparams, int thr, int thr2 );

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

vector<Rect> detect_face_depth ( Mat tmp_depth, Mat tmp_disparity )
{
  vector<Rect> roistmp;

  tmp_disparity.convertTo ( tmp_disparity, CV_8UC1 );

  // preprocessing
  Mat element = getStructuringElement( MORPH_RECT, Size( 2*5 + 1, 2*5 + 1 ), Point( 5, 5 ) );
  
  tmp_disparity = preprocessing(tmp_disparity);
  
  tmp_depth.setTo( 0, ( tmp_disparity == 0 ) );
  dilate( tmp_depth, tmp_depth, element );
  // FIXME there should be a way around all this conversions
  
  head_matched_points = chamfer_matching ( tmp_disparity, templates[0].template2d );
  head_features = compute_headparameters ( tmp_depth, head_matched_points );
  
  head_matched_points2 = chamfer_matching ( tmp_disparity, templates[1].template2d );
  head_features2 = compute_headparameters ( tmp_depth, head_matched_points2 );
  
  vector<PixelSimilarity> new_head_features, new_head_features2;
  
  new_head_features = false_positives( head_features, arc_thr_low, arc_thr_high );
  new_head_features2 = false_positives( head_features2, arc_thr_low, arc_thr_high );
   
  Rect rect;
  for (unsigned int i = 0; i < new_head_features.size(); i++)
   {
      int wh = new_head_features[i].radius * 2;
      rect = Rect(new_head_features[i].point.x - new_head_features[i].radius, new_head_features[i].point.y - new_head_features[i].radius, wh, wh);
      roistmp.push_back ( rect );
   }   
  cout << "Num rect aft: " << new_head_features.size() << endl;

    for (unsigned int i = 0; i < new_head_features2.size(); i++)
   {
      int wh = new_head_features2[i].radius * 2;
      rect = Rect(new_head_features2[i].point.x - new_head_features2[i].radius, new_head_features2[i].point.y - new_head_features2[i].radius, wh, wh);
      roistmp.push_back ( rect );
   } 
   
  vector<RegionOfInterest> rosrois = ros_utils.cvrects2rosrois(roistmp);
  depth_pub_rois.rois.swap(rosrois);  
  depth_pub.publish(depth_pub_rois);
  
  return roistmp;
}

vector<Point3f> chamfer_matching ( Mat image, Mat template_im )
{
  canny_im.create ( image.rows, image.cols, image.depth() );
  //Mat template_im = imread ( head_template, CV_LOAD_IMAGE_ANYDEPTH );

  // calculate edge detection
  Canny ( image, canny_im, canny_thr1, canny_thr2, 3, true );

  // calculate the Canny pyramid
  pyramid[0] = canny_im;
  for ( int i = 1; i < scales; i++ )
    {
      resize ( pyramid[i - 1], pyramid[i], Size(), scale_factor, scale_factor, INTER_NEAREST );
    }

  // calculate distance transform
  for ( int i = 0; i < scales; i++ )
    {
      distanceTransform ( ( 255 - pyramid[i] ), chamfer[i], CV_DIST_C, 3 );
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
  vector<Point3f> head_matched_points_tmp;

  for ( int i = 0; i < scales; i++ )
    {
      matchTemplate ( chamfer[i], template_im, matching[i], CV_TM_CCOEFF );
      normalize ( matching[i], matching[i], 0.0, 1.0, NORM_MINMAX );
      minMaxLoc ( matching[i], &minVal, &maxVal, &minLoc, &maxLoc );
      Mat matching_thr;
      threshold ( matching[i], matching_thr, 1.0 / chamfer_thr, 1.0, CV_THRESH_BINARY_INV );
      double scale = pow ( 1.0 / scale_factor, i );
      get_non_zeros ( matching_thr, matching[i], &head_matched_points_tmp, pdiff, scale );
    }

  return head_matched_points_tmp;
}

vector<PixelSimilarity> compute_headparameters ( Mat image, vector<Point3f> chamfer )
{
  vector<PixelSimilarity> parameters_head ( chamfer.size() );

  // parameters of cubic equation
  float p1 = -1.3835 * pow ( 10, -9 );
  float p2 =  1.8435 * pow ( 10, -5 );
  float p3 = -0.091403;
  float p4 =  189.38;

  for ( unsigned int i = 0; i < chamfer.size(); i++ )
    {
      int position_x = chamfer[i].x;
      int position_y = chamfer[i].y;

      float x = image.at<float> ( position_y, position_x ) * 1000; // unsigned short

      // compute height of head
      float h = ( p1 * pow ( x, 3 ) + p2 * pow ( x, 2 ) + p3 * x + p4 );

      // compute Radius of head in milimeters
      float R = 1.33 * h / 2;

      // convert Radius in pixels
      float Rp = round ( ( 1 / 1.3 ) * R );

      parameters_head[i].point = Point(position_x,position_y);
      parameters_head[i].radius = 1.1 * Rp;
      parameters_head[i].similarity = chamfer[i].z;
    }

  return parameters_head;
}

vector<PixelSimilarity> false_positives( vector<PixelSimilarity> tmpparams, int thr, int thr2 )
{
  vector<PixelSimilarity> tmpcont;
  
  for ( unsigned int i = 0; i < tmpparams.size(); i++ )
    {
      Rect roi ( tmpparams[i].point.x - tmpparams[i].radius, tmpparams[i].point.y - tmpparams[i].radius, tmpparams[i].radius * 2, tmpparams[i].radius * 2 );
      if ( ! ( 0 <= roi.x && 0 <= roi.width && roi.x + roi.width < canny_im.cols && 0 <= roi.y && 0 <= roi.height && roi.y + roi.height < canny_im.rows ) )
        {
          continue;
        }
        
      Mat tmp_mat = canny_im ( roi );
      vector<vector<Point> > contour;
      
      if( ! tmp_mat.empty() )
      {
        findContours ( tmp_mat, contour, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE );
	
	for ( unsigned int j = 0; j < contour.size(); j++ )
	{
	  vector<Point> approx;
	  approxPolyDP ( contour[j], approx, 5, false );
	  if ( approx.size() > thr && approx.size() < thr2 )
	    {
	      tmpcont.push_back(tmpparams[i]);
	      break;
	    }
        }
      }
      
    }
  /* Merged rectangles that are closed enough */  
  
  
  PixelSimilarity tmpcont_mean;
  vector<PixelSimilarity> output_v;
  vector<PixelSimilarity> queue;
  float tol = 30;

  cout << "Num rect bef: " << tmpcont.size() << endl;
  
  while(tmpcont.size() > 0)
   {

        tmpcont_mean = PixelSimilarity(tmpcont[0].point, tmpcont[0].radius, tmpcont[0].similarity);
        
        for(unsigned int i = 1; i < tmpcont.size(); i++){
          if(sqrt(pow(tmpcont_mean.point.x - tmpcont[i].point.x,2) + pow(tmpcont_mean.point.y - tmpcont[i].point.y,2)) < tol){
            if( tmpcont[i].similarity < tmpcont_mean.similarity  ) //  && tmpcont[i].similarity < 0.02
             {
               tmpcont[i] = PixelSimilarity(tmpcont[i].point, tmpcont[i].radius, tmpcont[i].similarity);
             }
          }
          else{
            //if(tmpcont[i].similarity < 0.02)
              queue.push_back(tmpcont[i]);
          }
         
        }
        output_v.push_back(tmpcont_mean);
        tmpcont.swap(queue);
        queue.clear();
   }
  
  cout << "Num rect aft: " << output_v.size() << endl;
  
  return output_v;
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

  return true;
}

void rgb_cb ( const ImageConstPtr& msg )
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

void depth_cb ( const ImageConstPtr& msg )
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
void load_templates( )
{
  
  Mat head_template_im1 = imread ( head_template1, CV_LOAD_IMAGE_ANYDEPTH );
  Mat head_template_im2 = imread ( head_template2, CV_LOAD_IMAGE_ANYDEPTH );
  Mat head_template3D_im1 = imread ( head_template3D1, CV_LOAD_IMAGE_ANYDEPTH );
  Mat head_template3D_im2 = imread ( head_template3D2, CV_LOAD_IMAGE_ANYDEPTH );
  
  if( head_template3D_im1.depth() == 3 )
    cvtColor( head_template3D_im1, head_template3D_im1, CV_RGB2GRAY );

  if( head_template3D_im2.depth() == 3 )
    cvtColor( head_template3D_im2, head_template3D_im2, CV_RGB2GRAY );
  
  templates.push_back( Template(head_template_im1, head_template3D_im1) );
  templates.push_back( Template(head_template_im2, head_template3D_im2) );
  
}
int main ( int argc, char **argv )
{
  ros::init ( argc, argv, "social_robot" );
  ros::NodeHandle nh;
  ros::MultiThreadedSpinner spinner ( 0 );

  string package_path = ros::package::getPath ( "social_robot" );
  head_template1.append ( package_path );
  head_template1.append ( "/pictures/template.png" );
  
  head_template2.append ( package_path );
  head_template2.append ( "/pictures/right_template.png" );
  
  head_template3D1.append ( package_path );
  head_template3D1.append ( "/pictures/template3D.png" );
  
  head_template3D2.append ( package_path );
  head_template3D2.append ( "/pictures/right_template3D.png" );
  
  load_templates( );
  
  cascade_name.append ( package_path );
  cascade_name.append ( "/rsrc/haarcascades/haarcascade_frontalface_alt.xml" );

  if ( !classifier.load ( cascade_name ) )
    {
      cerr << "ERROR: Could not load cascade classifier \"" << cascade_name << "\"" << endl;
      return -1;
    }
  
  nh.setParam( "/camera/driver/depth_registration", true);
  nh.setParam ( "/social_robot/head_template1", head_template1 );
  nh.setParam ( "/social_robot/canny_thr1", canny_thr1 );
  nh.setParam ( "/social_robot/canny_thr2", canny_thr2 );
  nh.setParam ( "/social_robot/chamfer_thr", chamfer_thr );
  nh.setParam ( "/social_robot/scales", scales );
  nh.setParam ( "/social_robot/arc_thr_low", arc_thr_low ); 
  nh.setParam ( "/social_robot/max_suppression", max_suppression );

  // subscribtions
  ros::ServiceServer update_srv = nh.advertiseService ( "/social_robot/update", update_param_cb );
  ros::Subscriber disparity_sub = nh.subscribe ( "/camera/depth_registered/disparity", 1, disparity_cb );
  ros::Subscriber rgb_sub = nh.subscribe ( "/camera/rgb/image_color", 1, rgb_cb );
  ros::Subscriber depth_sub = nh.subscribe ( "/camera/depth_registered/image_rect", 1, depth_cb );

  // publications
  depth_pub = nh.advertise<social_robot::RegionOfInterests>("/social_robot/depth_rois", 1);
  rgb_pub = nh.advertise<social_robot::RegionOfInterests>("/social_robot/rgb_rois", 1);
  
  namedWindow ( "Social Robot", CV_WINDOW_AUTOSIZE );
  ros::Timer timer = nh.createTimer ( ros::Duration ( 0.04 ), timer_cb );

  spinner.spin();

  return 0;
}

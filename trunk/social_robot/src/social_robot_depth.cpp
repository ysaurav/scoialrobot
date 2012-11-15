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
double arc_thr_high = 20;
double approx_poly_thr = 1;
double max_suppression = 0.1;
double scale_factor = 0.75;
double match3D_thr = 0.4;
int scales = 4;

vector<Point3f> head_matched_points;
vector<PixelSimilarity> head_features;

vector<Point3f> head_matched_points2;
vector<PixelSimilarity> head_features2;

vector<Template> templates;

Mat *pyramid = new Mat[scales];
Mat *chamfer = new Mat[scales];
Mat *matching = new Mat[scales];

Mat canny_im;
Mat image_depth;
Mat image_disparity;

vector<Point3f> chamfer_matching ( Mat image, Mat template_im );
vector<PixelSimilarity> compute_headparameters ( Mat image, vector<Point3f> chamfer );
vector<PixelSimilarity> false_positives ( vector<PixelSimilarity> tmpparams, int thr, int thr2 );
vector<PixelSimilarity> match_template3D( vector<PixelSimilarity> potentials, int n);
vector<PixelSimilarity> merge_rectangles( vector<PixelSimilarity> tmpcont);

vector<Rect> detect_face_depth ( Mat tmp_depth, Mat tmp_disparity )
{
  vector<Rect> roistmp;

  Mat element = getStructuringElement ( MORPH_RECT, Size ( 2*5 + 1, 2*5 + 1 ), Point ( 5, 5 ) );

  tmp_disparity = preprocessing ( tmp_disparity );

  tmp_depth.setTo ( 0, ( tmp_disparity == 0 ) );
  dilate ( tmp_depth, tmp_depth, element );
  
  vector<PixelSimilarity> new_head_features;
  vector<PixelSimilarity> final_head_features,final_head_features2;
  
  for ( int k = 0; k < templates.size() ; k++)
  {
    head_matched_points = chamfer_matching ( tmp_disparity, templates[k].template2d );
    head_features = compute_headparameters ( tmp_depth, head_matched_points );

    new_head_features = false_positives ( head_features, arc_thr_low, arc_thr_high );
    final_head_features = match_template3D( new_head_features, k );
  }

  final_head_features2 = merge_rectangles(final_head_features);
  
  Rect rect;
  for ( unsigned int i = 0; i < final_head_features2.size(); i++ )
    {
      int wh = final_head_features2[i].radius * 2;
      rect = Rect ( final_head_features2[i].point.x - final_head_features2[i].radius, final_head_features2[i].point.y - final_head_features2[i].radius, wh, wh );
      roistmp.push_back ( rect );
    }
    
  vector<RegionOfInterest> rosrois = ros_utils.cvrects2rosrois ( roistmp );
  depth_pub_rois.rois.swap ( rosrois );
  depth_pub.publish ( depth_pub_rois );

  return roistmp;
}

vector<Point3f> chamfer_matching ( Mat image, Mat template_im )
{
  double minVal, maxVal;
  Point minLoc, maxLoc;
  double xdiff = template_im.cols / 2;
  double ydiff = template_im.rows / 2;
  Point pdiff = Point ( xdiff, ydiff );
  vector<Point3f> head_matched_points_tmp;
  Mat matching_thr;
  
  canny_im.create ( image.rows, image.cols, image.depth() );

  Canny ( image, canny_im, canny_thr1, canny_thr2, 3, true );

  for ( int i = 0; i < scales; i++ )
    {
      resize ( canny_im, pyramid[i], Size(), pow( scale_factor,i), pow( scale_factor,i), INTER_NEAREST );
      distanceTransform ( ( 255 - pyramid[i] ), chamfer[i], CV_DIST_C, 3 );
    }

  template_im = rgb2bw ( template_im );
  template_im.convertTo ( template_im, CV_32F );
  
  for ( int i = 0; i < scales; i++ )
    {
      matchTemplate ( chamfer[i], template_im, matching[i], CV_TM_CCOEFF );
      normalize ( matching[i], matching[i], 0.0, 1.0, NORM_MINMAX );
      minMaxLoc ( matching[i], &minVal, &maxVal, &minLoc, &maxLoc );
      
      threshold ( matching[i], matching_thr, 1.0 / chamfer_thr, 1.0, CV_THRESH_BINARY_INV );
      double scale = pow ( 1.0 / scale_factor, i );
      get_non_zeros ( matching_thr, matching[i], &head_matched_points_tmp, pdiff, scale );
    }

  return head_matched_points_tmp;
}

vector<PixelSimilarity> compute_headparameters ( Mat image, vector<Point3f> chamfer)
{
  vector<PixelSimilarity> parameters_head ( chamfer.size() );

  float p1 = -1.3835 * pow ( 10, -9 );
  float p2 =  1.8435 * pow ( 10, -5 );
  float p3 = -0.091403;
  float p4 =  189.38;

  for ( unsigned int i = 0; i < chamfer.size(); i++ )
    {
      int position_x = chamfer[i].x;
      int position_y = chamfer[i].y;

      unsigned short x = image.at<unsigned short> ( position_y, position_x );// * 1000;

      float h = ( p1 * pow ( x, 3 ) + p2 * pow ( x, 2 ) + p3 * x + p4 );

      float R = 1.33 * h * 0.5;

      float Rp = round ( R / 1.3 );

      parameters_head[i].point = Point ( position_x,position_y );
      parameters_head[i].radius = 1.1 * Rp;
      parameters_head[i].similarity = chamfer[i].z;
     
    }

  return parameters_head;
}

vector<PixelSimilarity> false_positives ( vector<PixelSimilarity> tmpparams, int thr, int thr2 )
{
  vector<PixelSimilarity> tmpcont;
  vector<vector<Point> > contour;
  Mat tmp_mat;

  for ( unsigned int i = 0; i < tmpparams.size(); i++ )
    {
      Rect roi ( tmpparams[i].point.x - tmpparams[i].radius, tmpparams[i].point.y - tmpparams[i].radius, tmpparams[i].radius * 2, tmpparams[i].radius * 2 );
      if ( ! ( 0 <= roi.x && 0 <= roi.width && roi.x + roi.width < canny_im.cols && 0 <= roi.y && 0 <= roi.height && roi.y + roi.height < canny_im.rows ) )
        {
          continue;
        }

      tmp_mat = canny_im ( roi );

      if ( ! tmp_mat.empty() )
        {
          findContours ( tmp_mat, contour, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE );

          for ( unsigned int j = 0; j < contour.size(); j++ )
            {
              vector<Point> approx;
              approxPolyDP ( contour[j], approx, 5, false );
              if ( approx.size() > thr && approx.size() < thr2 )
                {
                  tmpcont.push_back ( tmpparams[i] );
                  break;
                }
            }
        }

    }
  
  vector<PixelSimilarity> output_v;
  output_v =  merge_rectangles( tmpcont );

  return output_v;
}

vector<PixelSimilarity> match_template3D( vector<PixelSimilarity> potentials, int n)
{
  
  vector<PixelSimilarity> output;
  Mat match; 
  double minVal, maxVal;
  Point minLoc, maxLoc;

  if ( potentials.empty() )
  {
    return output;
  }
  
  for (unsigned int i = 0; i < potentials.size(); i++)
  {
      Rect rect_roi ( potentials[i].point.x - potentials[i].radius, potentials[i].point.y - potentials[i].radius, 2 * potentials[i].radius, 2 * potentials[i].radius);
      Mat roi ( image_disparity, rect_roi );
      resize ( roi, roi, templates[n].template3d.size() );
      minMaxLoc ( roi, &minVal, &maxVal, 0, 0 );
      roi = roi - minVal;
      normalize ( roi, roi, 0.0, 255.0, NORM_MINMAX );
      
      matchTemplate ( roi, templates[n].template3d, match, CV_TM_CCOEFF_NORMED );

      minMaxLoc ( match, &minVal, &maxVal, &minLoc, &maxLoc );
      
      if( minVal >= match3D_thr )
      {
	output.push_back( PixelSimilarity( potentials[i].point, potentials[i].radius, potentials[i].similarity ) );
      }
	
  }
  
  return output;
}

vector<PixelSimilarity> merge_rectangles( vector<PixelSimilarity> tmpcont)
{
  PixelSimilarity tmpcont_mean;
  vector<PixelSimilarity> output_v;
  vector<PixelSimilarity> queue;
  float tol = 40;

  while ( tmpcont.size() > 0 )
    {

      tmpcont_mean = PixelSimilarity ( tmpcont[0].point, tmpcont[0].radius, tmpcont[0].similarity );

      for ( unsigned int i = 1; i < tmpcont.size(); i++ )
        {
          if ( sqrt ( pow ( tmpcont_mean.point.x - tmpcont[i].point.x,2 ) + pow ( tmpcont_mean.point.y - tmpcont[i].point.y,2 ) ) < tol )
            {
              if ( tmpcont[i].similarity < tmpcont_mean.similarity && tmpcont[i].similarity < max_suppression) //  
                {
                  tmpcont[i] = PixelSimilarity ( tmpcont[i].point, tmpcont[i].radius, tmpcont[i].similarity );
                }
            }
          else
            {
              if(tmpcont[i].similarity < max_suppression)
		queue.push_back ( tmpcont[i] );
            }

        }
      output_v.push_back ( tmpcont_mean );
      tmpcont.swap ( queue );
      queue.clear();
    }

  return output_v;
}

bool update_param_cb ( std_srvs::Empty::Request&, std_srvs::Empty::Response& )
{
  ROS_INFO ( "Updating parameter of social_robot" );

  double chamfer_thr_tmp = chamfer_thr;
  double arc_thr_low_tmp = arc_thr_low;
  double arc_thr_high_tmp = arc_thr_high;
  int scales_tmp = scales;
  double max_suppression_tmp = max_suppression;
  double match3D_thr_tmp = match3D_thr;

  ros::NodeHandle nh;

  nh.param ( "/social_robot/depth/chamfer_thr", chamfer_thr_tmp, chamfer_thr_tmp );
  nh.param ( "/social_robot/depth/scales", scales_tmp, scales_tmp );
  nh.param ( "/social_robot/depth/arc_thr_low", arc_thr_low_tmp, arc_thr_low_tmp );
  nh.param ( "/social_robot/depth/arc_thr_high", arc_thr_high_tmp, arc_thr_high_tmp );
  nh.param ( "/social_robot/depth/max_suppression", max_suppression_tmp, max_suppression_tmp );
  nh.param ( "/social_robot/depth/match3D_thr", match3D_thr_tmp, match3D_thr_tmp );

  chamfer_thr = chamfer_thr_tmp;
  scales = scales_tmp;
  arc_thr_low = arc_thr_low_tmp;
  arc_thr_high = arc_thr_high_tmp;
  max_suppression = max_suppression_tmp;
  match3D_thr = match3D_thr_tmp;

  return true;
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
      
      if ( image_depth.empty() )
        {
          return;
        }
      image_disparity.convertTo( image_disparity, CV_8UC1);
      vector<Rect> roistmp = detect_face_depth ( image_depth, image_disparity );
      Mat temp_disp;
      image_disparity.convertTo( temp_disp, CV_8UC3);
      draw_depth_faces ( temp_disp, roistmp );
      
      imshow( "Depth", temp_disp );
      waitKey( 3 );
    }
  catch ( cv_bridge::Exception& e )
    {
      ROS_ERROR ( "cv_bridge exception: %s", e.what() );
      return;
    }
}

void load_templates( )
{
  string package_path = ros::package::getPath ( "social_robot" );
  head_template1.append ( package_path );
  head_template1.append ( "/pictures/template.png" );

  head_template2.append ( package_path );
  head_template2.append ( "/pictures/template3.png" );

  head_template3D1.append ( package_path );
  head_template3D1.append ( "/pictures/template3D.png" );

  head_template3D2.append ( package_path );
  head_template3D2.append ( "/pictures/template3D.png" );
  
  Mat head_template_im1 = imread ( head_template1, CV_LOAD_IMAGE_ANYDEPTH );
  Mat head_template_im2 = imread ( head_template2, CV_LOAD_IMAGE_ANYDEPTH );
  Mat head_template3D_im1 = imread ( head_template3D1, CV_LOAD_IMAGE_ANYDEPTH );
  Mat head_template3D_im2 = imread ( head_template3D2, CV_LOAD_IMAGE_ANYDEPTH );

  if ( head_template3D_im1.depth() == 3 )
    {
      cvtColor ( head_template3D_im1, head_template3D_im1, CV_RGB2GRAY );
    }

  if ( head_template3D_im2.depth() == 3 )
    {
      cvtColor ( head_template3D_im2, head_template3D_im2, CV_RGB2GRAY );
    }

  templates.push_back ( Template ( head_template_im1, head_template3D_im1 ) );
  templates.push_back ( Template ( head_template_im2, head_template3D_im2 ) );

}

int main ( int argc, char **argv )
{
  ros::init ( argc, argv, "social_robot_depth" );
  ros::NodeHandle nh;

  load_templates( );

  // to register the depth
  nh.setParam ( "/camera/driver/depth_registration", true );
  
  nh.setParam ( "/social_robot/depth/chamfer_thr", chamfer_thr );
  nh.setParam ( "/social_robot/depth/scales", scales );
  nh.setParam ( "/social_robot/depth/arc_thr_low", arc_thr_low );
  nh.setParam ( "/social_robot/depth/arc_thr_high", arc_thr_high );
  nh.setParam ( "/social_robot/depth/max_suppression", max_suppression );
  nh.setParam ( "/social_robot/depth/match3D_thr", match3D_thr );

  // subscribtions
  ros::ServiceServer update_srv = nh.advertiseService ( "/social_robot/depth/update", update_param_cb );
  ros::Subscriber disparity_sub = nh.subscribe ( "/camera/depth/disparity", 1, disparity_cb ); // 
  ros::Subscriber depth_sub = nh.subscribe ( "/camera/depth/image_raw", 1, depth_cb );

  // publications
  depth_pub = nh.advertise<social_robot::RegionOfInterests> ( "/social_robot/depth/rois", 1 );

  ros::spin();

  return 0;
}

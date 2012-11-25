#include "DepthFaceDetector.h"
#include "CvUtils.h"
#include <ros/package.h>

#define MORPH_SIZE Size ( 11, 11 )

using namespace std;
using namespace cv;

vector<Point3f> head_matched_points;
vector<PixelSimilarity> head_features;

vector<Point3f> head_matched_points2;
vector<PixelSimilarity> head_features2;

vector<Template> templates;

Mat *pyramid;
Mat *chamfer;
Mat *matching;

Mat canny_im;
Mat image_depth;
Mat image_disparity;
CvUtils cv_utils;

DepthFaceDetector::DepthFaceDetector ( void )
{
  load_templates();
  canny_thr1 = 5;
  canny_thr2 = 7;
  chamfer_thr = 10;
  arc_thr_low = 7;
  arc_thr_high = 20;
  approx_poly_thr = 1;
  max_suppression = 0.1;
  scale_factor = 0.75;
  match3D_thr = 0.4;
  scales = 6;
  framenum = 0;
  update_rate = 15;
  pyramid = new Mat[scales];
  chamfer = new Mat[scales];
  matching = new Mat[scales];
}

vector<Rect> DepthFaceDetector::detect_face_depth ( Mat depth_image, Mat disparity_image )
{
  vector<Rect> roistmp;

  Mat element = getStructuringElement ( MORPH_RECT, Size ( 2*5 + 1, 2*5 + 1 ), Point ( 5, 5 ) );

  image_disparity = cv_utils.preprocessing ( image_disparity );

  image_depth.setTo ( 0, ( image_disparity == 0 ) );
  dilate ( image_depth, image_depth, element );

  vector<PixelSimilarity> new_head_features;
  vector<PixelSimilarity> final_head_features,final_head_features2;

  for ( unsigned int k = 0; k < templates.size() ; k++ )
    {
      head_matched_points = chamfer_matching ( image_disparity, templates[k].template2d );
      head_features = compute_headparameters ( image_depth, head_matched_points );
      new_head_features = false_positives ( head_features, arc_thr_low, arc_thr_high );
      final_head_features = match_template3D ( new_head_features, k );
    }

  final_head_features2 = merge_rectangles ( final_head_features );

  Rect rect;
  for ( unsigned int i = 0; i < final_head_features2.size(); i++ )
    {
      int wh = final_head_features2[i].radius * 2;
      rect = Rect ( final_head_features2[i].point.x - final_head_features2[i].radius, final_head_features2[i].point.y - final_head_features2[i].radius, wh, wh );
      roistmp.push_back ( rect );
    }

  return roistmp;
}

vector<Point3f> DepthFaceDetector::chamfer_matching ( Mat image, Mat template_im )
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
      resize ( canny_im, pyramid[i], Size(), pow ( scale_factor, i ), pow ( scale_factor, i ), INTER_NEAREST );
      distanceTransform ( ( 255 - pyramid[i] ), chamfer[i], CV_DIST_C, 3 );
    }

  for ( int j = 0; j < scales; j++ )
    {
      matchTemplate ( chamfer[j], template_im, matching[j], CV_TM_CCOEFF );
      normalize ( matching[j], matching[j], 0.0, 1.0, NORM_MINMAX );
      minMaxLoc ( matching[j], &minVal, &maxVal, &minLoc, &maxLoc );

      threshold ( matching[j], matching_thr, 1.0 / chamfer_thr, 1.0, CV_THRESH_BINARY_INV );
      double scale = pow ( 1.0 / scale_factor, j );
      cv_utils.get_non_zeros ( matching_thr, matching[j], &head_matched_points_tmp, pdiff, scale );
    }

  return head_matched_points_tmp;
}

vector<PixelSimilarity> DepthFaceDetector::compute_headparameters ( Mat image, vector<Point3f> chamfer )
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

      float x;

      if ( image.type( ) == 5 )
        {
          x = image.at<float> ( position_y, position_x ) * 1000;
        }
      else
        {
          unsigned short xshort = image.at<unsigned short> ( position_y, position_x );
          x = ( float ) xshort;
        }


      float h = ( p1 * pow ( x, 3 ) + p2 * pow ( x, 2 ) + p3 * x + p4 );

      float R = 1.33 * h * 0.5;

      float Rp = round ( R / 1.3 );

      parameters_head[i].point = Point ( position_x,position_y );
      parameters_head[i].radius = 1.3 * Rp;
      parameters_head[i].similarity = chamfer[i].z;

    }

  return parameters_head;
}

vector<PixelSimilarity> DepthFaceDetector::false_positives ( vector<PixelSimilarity> tmpparams, int thr, int thr2 )
{
  vector<PixelSimilarity> tmpcont;
  vector<vector<Point> > contour;
  Mat tmp_mat;

  //#pragma omp parallel for shared(tmpcont,tmpparams, thr, thr2) private(contour, tmp_mat)
  for ( unsigned int i = 0; i < tmpparams.size(); i++ )
    {
      //cout << omp_get_thread_num() << endl;
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
  output_v =  merge_rectangles ( tmpcont );

  return output_v;
}

vector<PixelSimilarity> DepthFaceDetector::match_template3D ( vector<PixelSimilarity> potentials, int n )
{
  vector<PixelSimilarity> output;
  Mat match;
  double minVal, maxVal;
  Point minLoc, maxLoc;

  if ( potentials.empty() )
    {
      return output;
    }

  for ( unsigned int i = 0; i < potentials.size(); i++ )
    {
      Rect rect_roi ( potentials[i].point.x - potentials[i].radius, potentials[i].point.y - potentials[i].radius, 2 * potentials[i].radius, 2 * potentials[i].radius );
      Mat roi ( image_disparity, rect_roi );
      resize ( roi, roi, templates[n].template3d.size() );
      minMaxLoc ( roi, &minVal, &maxVal, 0, 0 );
      roi = roi - minVal;
      normalize ( roi, roi, 0.0, 255.0, NORM_MINMAX );

      matchTemplate ( roi, templates[n].template3d, match, CV_TM_CCOEFF_NORMED );

      minMaxLoc ( match, &minVal, &maxVal, &minLoc, &maxLoc );

      if ( minVal >= match3D_thr )
        {
          output.push_back ( PixelSimilarity ( potentials[i].point, potentials[i].radius, potentials[i].similarity ) );
        }

    }

  return output;
}

vector<PixelSimilarity> DepthFaceDetector::merge_rectangles ( vector<PixelSimilarity> tmpcont )
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
              if ( tmpcont[i].similarity < tmpcont_mean.similarity && tmpcont[i].similarity < max_suppression ) //
                {
                  tmpcont[i] = PixelSimilarity ( tmpcont[i].point, tmpcont[i].radius, tmpcont[i].similarity );
                }
            }
          else
            {
              if ( tmpcont[i].similarity < max_suppression )
                queue.push_back ( tmpcont[i] );
            }

        }
      output_v.push_back ( tmpcont_mean );
      tmpcont.swap ( queue );
      queue.clear();
    }

  return output_v;
}

void DepthFaceDetector::load_templates ( void )
{
  string package_path = ros::package::getPath ( "social_robot" );

  string head_template1;
  head_template1.append ( package_path );
  head_template1.append ( "/pictures/template.png" );

  string head_template2;
  head_template2.append ( package_path );
  head_template2.append ( "/pictures/template3.png" );

  string head_template3D1;
  head_template3D1.append ( package_path );
  head_template3D1.append ( "/pictures/template3D.png" );

  string head_template3D2;
  head_template3D2.append ( package_path );
  head_template3D2.append ( "/pictures/template3D.png" );

  Mat head_template_im1 = imread ( head_template1, CV_LOAD_IMAGE_ANYDEPTH );
  Mat head_template_im2 = imread ( head_template2, CV_LOAD_IMAGE_ANYDEPTH );
  Mat head_template3D_im1 = imread ( head_template3D1, CV_LOAD_IMAGE_ANYDEPTH );
  Mat head_template3D_im2 = imread ( head_template3D2, CV_LOAD_IMAGE_ANYDEPTH );

  head_template_im1 = cv_utils.rgb2bw ( head_template_im1 );
  head_template_im1.convertTo ( head_template_im1, CV_32F );
  head_template_im2 = cv_utils.rgb2bw ( head_template_im2 );
  head_template_im2.convertTo ( head_template_im2, CV_32F );

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

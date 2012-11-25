#include <ros/ros.h>

#include "kinect_proxy.h"
#include "PixelSimilarity.h"
#include "Template.h"
#include "CvUtils.h"

#define HEAD_TEMPLATE1 "pictures/template3.png"
#define HEAD_TEMPLATE2 "pictures/template.png"
#define NUM_TEMPLATES 2
#define HEAD_TEMPLATE3D2 "pictures/template3D.png"
#define HEAD_TEMPLATE3D "pictures/template3D.png"
#define THR1          5
#define THR2          7
#define THR3          20
#define SCALES        5
#define SCALE_FACTOR  0.75//sqrt(2)

using namespace std;
using namespace cv;

string cascade_name = "rsrc/haarcascades/haarcascade_frontalface_alt.xml";
CvUtils cv_utils;

const static Scalar colors[] =
{
  CV_RGB ( 0, 0, 255 ),
  CV_RGB ( 0, 128, 255 ),
  CV_RGB ( 0, 255, 255 ),
  CV_RGB ( 0, 255, 0 ),
  CV_RGB ( 255, 128, 0 ),
  CV_RGB ( 255, 255, 0 ),
  CV_RGB ( 255, 0, 0 ),
  CV_RGB ( 255, 0, 255 )
};

Mat canny_im;

bool selectObject = false;
int trackObject = 0;
Point origin;
Rect selection;
Mat image_rgb;

void onMouse ( int event, int x, int y, int, void* )
{
  if ( selectObject )
    {
      selection.x = MIN ( x, origin.x );
      selection.y = MIN ( y, origin.y );
      selection.width = std::abs ( x - origin.x );
      selection.height = std::abs ( y - origin.y );

      selection &= Rect ( 0, 0, image_rgb.cols, image_rgb.rows );
    }

  switch ( event )
    {
    case CV_EVENT_LBUTTONDOWN:
      origin = Point ( x,y );
      selection = Rect ( x,y,0,0 );
      selectObject = true;
      break;
    case CV_EVENT_LBUTTONUP:
      selectObject = false;
      if ( selection.width > 0 && selection.height > 0 )
        trackObject = -1;
      break;
    }
}

vector<Point3f> chamfer_matching ( Mat image, Mat template_im )
{
  canny_im.create ( image.rows, image.cols, image.depth() );

  Mat* pyramid = new Mat[SCALES];
  Mat* chamfer = new Mat[SCALES];
  Mat* matching = new Mat[SCALES];

  // calculate edge detection
  Canny ( image, canny_im, THR1, THR2, 3, true );
  imshow ( "Canny", canny_im );
  waitKey ( 0 );

  // calculate the Canny pyramid
  pyramid[0] = canny_im;
  for ( int i = 1; i < SCALES; i++ )
    {
      resize ( pyramid[0], pyramid[i], Size(), pow ( SCALE_FACTOR,i ), pow ( SCALE_FACTOR,i ), INTER_NEAREST );
    }

  // calculate distance transform
  for ( int i = 0; i < SCALES; i++ )
    {
      distanceTransform ( ( 255 - pyramid[i] ), chamfer[i], CV_DIST_C, 3 );
      //normalize ( chamfer[i], chamfer[i], 0.0, 1.0, NORM_MINMAX );
    }

  // matching with the template
  template_im = cv_utils.rgb2bw ( template_im );
  template_im.convertTo ( template_im,CV_32F );
  imshow ( "template",template_im );
  waitKey ( 0 );
  // find the best match:
  double minVal, maxVal;
  Point minLoc, maxLoc;
  double xdiff = template_im.cols / 2;
  double ydiff = template_im.rows / 2;
  Point pdiff = Point ( xdiff, ydiff );
  vector<Point3f> head_matched_points;

  for ( int i = 0; i < SCALES; i++ )
    {
      matchTemplate ( chamfer[i], template_im, matching[i], CV_TM_CCOEFF );
      normalize ( matching[i], matching[i], 0.0, 1.0, NORM_MINMAX );
      minMaxLoc ( matching[i], &minVal, &maxVal, &minLoc, &maxLoc );
      Mat matching_thr;
      threshold ( matching[i], matching_thr, 1.0 / THR3, 1.0, CV_THRESH_BINARY_INV );
      double scale = pow ( 1.0 / SCALE_FACTOR , i ); // 1.0 / 0.75
      cv_utils.get_non_zeros ( matching_thr, matching[i], &head_matched_points, pdiff, scale );
    }

  return head_matched_points;
}

vector<Point3f> chamfer_matching_fusion ( Mat image, vector<Mat> template_im )
{
  canny_im.create ( image.rows, image.cols, image.depth() );

  Mat* pyramid = new Mat[SCALES];
  Mat* chamfer = new Mat[SCALES];
  Mat* matching = new Mat[SCALES];

  // calculate edge detection
  Canny ( image, canny_im, THR1, THR2, 3, true );
  imshow ( "Canny", canny_im );
  waitKey ( 0 );

  // calculate the Canny pyramid
  pyramid[0] = canny_im;
  for ( int i = 1; i < SCALES; i++ )
    {
      resize ( pyramid[0], pyramid[i], Size(), pow ( SCALE_FACTOR,i ), pow ( SCALE_FACTOR,i ), INTER_NEAREST );
    }

  // calculate distance transform
  for ( int i = 0; i < SCALES; i++ )
    {
      distanceTransform ( ( 255 - pyramid[i] ), chamfer[i], CV_DIST_C, 3 );
    }

  // matching with the template
  cout << "hola " << template_im.size() << endl;
  for ( int i = 0; i < template_im.size() ; i++ )
    {
      template_im[i] = cv_utils.rgb2bw ( template_im[i] );
      template_im[i].convertTo ( template_im[i], CV_32F );
    }


  imshow ( "template",template_im[0] );
  waitKey ( 0 );
  // find the best match:
  double minVal, maxVal;
  Point minLoc, maxLoc;
  double xdiff = template_im[0].cols / 2;
  double ydiff = template_im[0].rows / 2;
  Point pdiff = Point ( xdiff, ydiff );
  vector<Point3f> head_matched_points;
  Mat temp;



  for ( int i = 0; i < SCALES; i++ )
    {
      temp = 1000 * Mat::ones ( chamfer[i].rows - template_im[0].rows + 1, chamfer[i].cols - template_im[0].cols + 1, CV_32F );
      for ( int j = 0; j < NUM_TEMPLATES; j++ )
        {
          matchTemplate ( chamfer[i], template_im[j], matching[i], CV_TM_CCOEFF );
          temp = min ( matching[i],temp );
        }
      matching[i] = temp;
      normalize ( matching[i], matching[i], 0.0, 1.0, NORM_MINMAX );
      minMaxLoc ( matching[i], &minVal, &maxVal, &minLoc, &maxLoc );
      Mat matching_thr;
      threshold ( matching[i], matching_thr, 1.0 / THR3, 1.0, CV_THRESH_BINARY_INV );
      double scale = pow ( 1.0 / SCALE_FACTOR , i );
      cv_utils.get_non_zeros ( matching_thr, matching[i], &head_matched_points, pdiff, scale );
    }

  return head_matched_points;
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

      unsigned short x = image.at<unsigned short> ( position_y, position_x ); // unsigned short

      // compute height of head
      float h = ( p1 * pow ( x, 3 ) + p2 * pow ( x, 2 ) + p3 * x + p4 );

      // compute Radius of head in milimeters
      float R = 1.33 * h * 0.5;

      // convert Radius in pixels
      float Rp = round ( R / 1.3 );

      parameters_head[i].point = Point ( position_x, position_y );
      parameters_head[i].radius = 1.2 * Rp;
      parameters_head[i].similarity = chamfer[i].z;

    }

  return parameters_head;
}

void detect_face ( Mat &img, CascadeClassifier& cascade )
{
  int i = 0;
  double t = 0;
  vector<Rect> faces;

  Mat gray;
  Mat frame ( cvRound ( img.rows ), cvRound ( img.cols ), CV_8UC1 );

  cvtColor ( img, gray, CV_BGR2GRAY );
  resize ( gray, frame, frame.size(), 0, 0, INTER_LINEAR );
  equalizeHist ( frame, frame );

  t = ( double ) cvGetTickCount();
  cascade.detectMultiScale ( frame, faces,
                             1.1, 2, 0
                             //|CV_HAAR_FIND_BIGGEST_OBJECT
                             //|CV_HAAR_DO_ROUGH_SEARCH
                             |CV_HAAR_SCALE_IMAGE
                             ,
                             Size ( 30, 30 ) );
  t = ( double ) cvGetTickCount() - t;

  for ( vector<Rect>::const_iterator r = faces.begin(); r != faces.end(); r++, i++ )
    {
      Point center;
      Scalar color = colors[i%8];
      int radius;
      center.x = cvRound ( r->x + r->width*0.5 );
      center.y = cvRound ( r->y + r->height*0.5 );
      radius = ( int ) ( cvRound ( r->width + r->height ) *0.25 );
      circle ( img, center, radius, color, 3, 8, 0 );
      Rect newrect = cv_utils.enlarge_window ( faces[i], img, 1.0 );
      rectangle ( img, newrect, colors[0], 3, 8, 0 );
      cout << "is there face " << cv_utils.is_there_face_rgb ( img, faces[i] ) << endl;
    }
}

vector<PixelSimilarity> false_positives ( vector<PixelSimilarity> tmpparams, int thr, int thr2 )
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
                  //rectangle ( image_rgb, roi.tl(), roi.br(), cvScalar ( 255, 0, 255, 0 ), 2, 8, 0 );
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

  while ( tmpcont.size() > 0 )
    {

      tmpcont_mean = PixelSimilarity ( tmpcont[0].point, tmpcont[0].radius, tmpcont[0].similarity );

      //while(abs(tmpcont_mean.x - tmpparams[next].x) + abs(tmpcont_mean.y - tmpparams[next].y) < tol ){
      for ( unsigned int i = 1; i < tmpcont.size(); i++ )
        {
          if ( sqrt ( pow ( tmpcont_mean.point.x - tmpcont[i].point.x,2 ) + pow ( tmpcont_mean.point.y - tmpcont[i].point.y,2 ) ) < tol )
            {
              if ( tmpcont[i].similarity < tmpcont_mean.similarity ) //  && tmpcont[i].similarity < 0.02
                {
                  tmpcont[i] = PixelSimilarity ( tmpcont[i].point, tmpcont[i].radius, tmpcont[i].similarity );
                }
            }
          else
            {
              //if(tmpcont[i].similarity < 0.02)
              queue.push_back ( tmpcont[i] );
            }

        }
      output_v.push_back ( tmpcont_mean );
      tmpcont.swap ( queue );
      queue.clear();
    }

  cout << "Num rect aft: " << output_v.size() << endl;

  return output_v;
}

int main ( int argc, char **argv )
{
  namedWindow ( "CamShift Demo", 1 );
  setMouseCallback ( "CamShift Demo", onMouse, 0 );
  Mat image_dispa = imread ( argv[1], CV_LOAD_IMAGE_ANYDEPTH );
  Mat image_depth = imread ( argv[2], CV_LOAD_IMAGE_ANYDEPTH );
  image_rgb = imread ( argv[3], CV_LOAD_IMAGE_COLOR );

  Mat disp_image;
  image_rgb.copyTo ( disp_image );
  while ( true )
    {
      if( trackObject < 0 )
        {
          image_rgb.copyTo ( disp_image );
          rectangle(disp_image, selection, colors[0], 3, 8, 0);
          Rect newrect = cv_utils.enlarge_window(selection, image_rgb, 2.5);
          rectangle(disp_image, newrect, colors[1], 3, 8, 0);
          Rect newrectw = cv_utils.enlarge_window_width(selection, image_rgb, 2.5);
          rectangle(disp_image, newrectw, colors[2], 3, 8, 0);
          Rect newrecth = cv_utils.enlarge_window_height(selection, image_rgb, 2.5);
          rectangle(disp_image, newrecth, colors[3], 3, 8, 0);          
          trackObject = 0;
        }
      if ( selectObject && selection.width > 0 && selection.height > 0 )
        {
          Mat roi ( disp_image, selection );
          bitwise_not ( roi, roi );
        }

      imshow ( "CamShift Demo", disp_image );
      waitKey( 3 );
    }


  return 0;

  CascadeClassifier cascade;
  if ( !cascade.load ( cascade_name ) )
    {
      cerr << "ERROR: Could not load cascade classifier \"" << cascade_name << "\"" << endl;
      return -1;
    }

  cout << "Channels: " << image_rgb.channels() << " Type: " << image_rgb.type() << " Depth: " << image_rgb.depth() << endl;

  detect_face ( image_rgb, cascade );
  imshow ( "Arash", image_rgb );
  waitKey ( 0 );

  return 0;

  // PREPROCESSING //////////////////////////////////////////////////////////////////////////////////
  Mat element = getStructuringElement ( MORPH_RECT, Size ( 2*5 + 1, 2*5 + 1 ), Point ( 5, 5 ) );
  Mat element2 = getStructuringElement ( MORPH_RECT, Size ( 2*2 + 1, 2*2 + 1 ), Point ( 2, 2 ) );


  image_dispa = cv_utils.preprocessing ( image_dispa );

  image_depth.setTo ( 0, ( image_dispa == 0 ) );
  dilate ( image_depth, image_depth, element );

  // CHAMFER DISTANCE ///////////////////////////////////////////////////////////////////////////////
  Mat template_im = imread ( HEAD_TEMPLATE1, CV_LOAD_IMAGE_ANYDEPTH );
  Mat template_im2 = imread ( HEAD_TEMPLATE2, CV_LOAD_IMAGE_ANYDEPTH );
  Mat template3d_im = imread ( HEAD_TEMPLATE3D, CV_LOAD_IMAGE_ANYDEPTH );
  Mat template3d_im2 = imread ( HEAD_TEMPLATE3D2, CV_LOAD_IMAGE_ANYDEPTH );

  if ( template3d_im.depth() == 3 )
    cvtColor ( template3d_im, template3d_im, CV_RGB2GRAY );

  if ( template3d_im2.depth() == 3 )
    cvtColor ( template3d_im2, template3d_im2, CV_RGB2GRAY );

  vector<Template> templates;

  templates.push_back ( Template ( template_im, template3d_im ) );
  templates.push_back ( Template ( template_im2, template3d_im2 ) );

  cout << "Preprocessing done" << endl;
  vector<Point3f> head_matched_points = chamfer_matching ( image_dispa, templates[0].template2d );
  vector<Point3f> head_matched_points2 = chamfer_matching ( image_dispa, templates[1].template2d );

  int thr = atoi ( argv[4] ); // default must be 12
  int thr2 = atoi ( argv[5] ); // default must be 17

  // HEAD PARAMETERS ////////////////////////////////////////////////////////////////////////////////
  cout << "Chamfer done" << endl;
  vector<PixelSimilarity> tmpparams = compute_headparameters ( image_depth, head_matched_points );
  vector<PixelSimilarity> tmpparams2 = compute_headparameters ( image_depth, head_matched_points2 );

  cout << tmpparams.size() << ", " << tmpparams2.size() << endl;

  cout << "Head parameters computed" << endl;
  vector<PixelSimilarity> output_v, output_v2;

  output_v = false_positives ( tmpparams, thr, thr2 );
  output_v2 = false_positives ( tmpparams2, thr, thr2 );
  cout << "False positives" << endl;


  // find the best match for template3D:
  Mat match;
  double minVal, maxVal;
  Point minLoc, maxLoc;


  for ( unsigned int i = 0; i < output_v.size(); i++ )
    {
      cout << "Hola" << endl;
      Rect rect_roi ( output_v[i].point.x - output_v[i].radius, output_v[i].point.y - output_v[i].radius, 2 * output_v[i].radius, 2 * output_v[i].radius );
      Mat roi ( image_dispa, rect_roi );
      resize ( roi, roi, templates[0].template3d.size() );
      imshow ( "match", roi );
      waitKey ( 0 );

      matchTemplate ( roi, templates[0].template3d, match, CV_TM_CCOEFF_NORMED );
      // normalize ( match, match, 0.0, 1.0, NORM_MINMAX );
      cout << match.rows << ", " << match.cols << endl;
      minMaxLoc ( match, &minVal, &maxVal, &minLoc, &maxLoc );
      cout << minVal << ", " << maxVal << endl;

      if ( minVal >= 0.4 )
        circle ( image_rgb, output_v[i].point, output_v[i].radius, cvScalar ( 255, 0, 255, 0 ), 2, 8, 0 );
    }

  for ( unsigned int i = 0; i < output_v2.size(); i++ )
    {
      Rect rect_roi2 ( output_v2[i].point.x - output_v2[i].radius, output_v2[i].point.y - output_v2[i].radius, 2 * output_v2[i].radius, 2 * output_v2[i].radius );
      Mat roi2 ( image_dispa, rect_roi2 );
      resize ( roi2, roi2, templates[1].template3d.size() );
      imshow ( "match2", roi2 );
      waitKey ( 0 );

      matchTemplate ( roi2, templates[1].template3d, match, CV_TM_CCOEFF_NORMED );
      // normalize ( match, match, 0.0, 1.0, NORM_MINMAX );
      cout << match.rows << ", " << match.cols << endl;
      minMaxLoc ( match, &minVal, &maxVal, &minLoc, &maxLoc );
      cout << minVal << ", " << maxVal << endl;

      if ( minVal >= 0.4 )
        circle ( image_rgb, output_v2[i].point, output_v2[i].radius, cvScalar ( 255, 255, 0, 0 ), 2, 8, 0 );

    }

  imshow ( "Hola", image_rgb );
  waitKey ( 0 );

  return 0;
}

#include "cv_utils.h"

using namespace cv;
using namespace std;

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

Mat rgb2bw ( Mat im_rgb )
{
  Mat im_gray;
  if ( im_rgb.channels() == 3 )
    {
      cvtColor ( im_rgb, im_gray, CV_RGB2GRAY );
    }
  else
    {
      im_gray = im_rgb;
    }

  Mat im_bw;
  threshold ( im_gray, im_bw, 128, 255, CV_THRESH_BINARY );
  return im_bw;
}

Mat preprocessing ( Mat image )
{
  Mat dst;
  Mat temp;
  //  resize(image, temp, Size(), 2, 2, INTER_NEAREST);
  //  resize(temp, dst, Size(), 0.5, 0.5, INTER_NEAREST);

  // inpainting
  Mat mask; // = Mat::zeros(image.rows, image.cols, image.);
  threshold ( image, mask, 0, 255, CV_THRESH_BINARY_INV );
//   for ( int i = 0; i < image.rows; i++ )
//     {
//       unsigned char *rowi = image.ptr<unsigned char> ( i );
//       for ( int j = 0; j < image.cols; j++ )
//         {
//           if ( rowi[j] == 0 )
//             {
//               mask.at<unsigned char> ( i, j ) = 255;
//             }
//         }
//     }
  medianBlur ( image, temp, 5 );
  inpaint ( temp, mask, dst, 5, INPAINT_NS );
  medianBlur ( dst, dst, 5 );
  return dst;
}

void get_non_zeros ( Mat img, std::vector<cv::Point> *points, cv::Point pdiff, double scale )
{
  int k = 0;
  for ( int i = 0; i < img.rows; i++ )
    {
      float *rowi = img.ptr<float> ( i );
      for ( int j = 0; j < img.cols; j++ )
        {
          if ( rowi[j] != 0 )
            {
              cv::Point point = ( cv::Point ( j, i ) + pdiff ) * scale;
              points->push_back ( point );
              k++;
            }
        }
    }
}

void draw_rgb_faces ( Mat &img, vector<Rect> faces )
{
  int i = 0;
  for ( vector<Rect>::const_iterator r = faces.begin(); r != faces.end(); r++, i++ )
    {
      Point center;
      Scalar color = colors[i % 8];
      int radius;
      center.x = cvRound ( r->x + r->width * 0.5 );
      center.y = cvRound ( r->y + r->height * 0.5 );
      radius = ( int ) ( cvRound ( r->width + r->height ) * 0.25 );
      circle ( img, center, radius, color, 3, 8, 0 );
    }
}

void draw_depth_faces ( Mat &img, vector<Rect> faces )
{
  Rect tmpparams_av;
  vector<Rect> output_v;
  vector<Rect> queue;
  double tol = 7;

  while ( faces.size() > 0 )
    {
      tmpparams_av.x = faces[0].x;
      tmpparams_av.y = faces[0].y;
      tmpparams_av.width = faces[0].width;
      tmpparams_av.height = faces[0].height;

      for ( unsigned int i = 1; i < faces.size(); i++ )
        {
          if ( sqrt ( pow ( tmpparams_av.x - faces[i].x, 2 ) + pow ( tmpparams_av.y - faces[i].y, 2 ) ) < tol )
            {
              tmpparams_av.x = ( tmpparams_av.x + faces[i].x ) /2;
              tmpparams_av.y = ( tmpparams_av.y + faces[i].y ) /2;
              tmpparams_av.width = ( ( tmpparams_av.width + faces[i].width ) /2 );
              tmpparams_av.height = ( ( tmpparams_av.height + faces[i].height ) /2 );
            }
          else
            {
              queue.push_back ( faces[i] );
            }
        }
      output_v.push_back ( tmpparams_av );
      faces.swap ( queue );
      queue.clear();
    }
  faces.swap ( output_v );

  int i = 0;
  for ( vector<Rect>::const_iterator r = faces.begin(); r != faces.end(); r++, i++ )
    {
      Scalar color = colors[i % 8];
      rectangle ( img, *r, color, 3, 8, 0 );
    }
}

void region_growing ( Mat &src, Mat &dst, unsigned char threshold, vector<Point> seeds )
{
  vector<Point> queue;
  src.copyTo ( dst );
}

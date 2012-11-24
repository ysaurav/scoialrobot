#include "CvUtils.h"
#include <ros/package.h>

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

CvUtils::CvUtils ( void )
{
  string package_path = ros::package::getPath ( "social_robot" );

  string cascade_name;
  cascade_name.append ( package_path );
  cascade_name.append ( "/rsrc/haarcascades/haarcascade_frontalface_alt.xml" );
  if ( !classifier.load ( cascade_name ) )
    {
      cerr << "ERROR: Could not load cascade classifier \"" << cascade_name << "\"" << endl;
    }
}


Mat CvUtils::rgb2bw ( Mat im_rgb )
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
/**<
 * This function computes the binary image of a given one by setting to zero all the values smaller than 128.
 * @return The binary image in im_bw.
 * @param im_rgb A Mat containing an image.
 * @pre The input can be a colour or gray image.
 *
 * */

Mat CvUtils::preprocessing ( Mat image )
{
  Mat dst;
  Mat temp;

  //GaussianBlur( image, image, Size(2*3+1,2*3+1), 0.0, 0.0, BORDER_DEFAULT );
  medianBlur ( image, image, 5 );//3
  inpaint ( image, ( image == 0 ), temp, 5, INPAINT_NS );
  medianBlur ( temp, dst, 5 );
  GaussianBlur ( dst, dst, Size ( 2 * 2 + 1, 2 * 2 + 1 ), 0.0, 0.0, BORDER_DEFAULT );

  return dst;
}
/**<
 * This function fills the regions where the image values are equal to zero.
 * @return An image without zero values.
 * @param image A Mat containing an image.
 * */

void CvUtils::get_non_zeros ( Mat img, Mat prob, vector<Point3f> *points, Point pdiff, double scale )
{
  for ( int i = 0; i < img.rows; i++ )
    {
      float *rowi = img.ptr<float> ( i );
      for ( int j = 0; j < img.cols; j++ )
        {
          if ( rowi[j] != 0 )
            {
              Point3f point;
              point.x = ( Point ( j, i ).x  + pdiff.x ) * scale;
              point.y = ( Point ( j, i ).y  + pdiff.y ) * scale;
              point.z = prob.at<float> ( i,j );
              points->push_back ( point );
            }
        }
    }
}
/**<
 * This function returns a pointer to vector<Point3f> given a mask image, only the non zero values are taken into account,
 * and a probability image from a matching previous step.
 * @return void
 * @param img A Mat containing an image with 0's and 1's, it is the mask image, it has H x W dimensions.
 * @param prob A Mat containing probability values in each pixel, it has H' x W' dimensions.
 * @param *points A pointer to vector<Point3f>, it is the output parameter.
 * @param pdiff A point with the required shift in x and y coordinates to bring the point from prob to img coordinates.
 * @param scale A double for the current scale, it is used to bring the calculated coordinates to the original image size.
 * @pre The images img and prob do not have the same dimensions.
 * */


/** @name Drawing functions
*
*/
//@{
void CvUtils::draw_rgb_faces ( Mat &img, vector<Rect> faces )
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
/**<
 * This function draws the rectangles given in a vector in a colour image.
 * @return void
 * @param &img A Mat containing an image.
 * @param faces A vector<Rect> that contains the rectangles bounding the faces.
 * */

void CvUtils::draw_depth_faces ( Mat &img, vector<Rect> faces )
{
  Rect bounds ( 0, 0, img.cols, img.rows );

  int i = 0;
  for ( vector<Rect>::const_iterator r = faces.begin(); r != faces.end(); r++, i++ )
    {
      Scalar color = BLUE;
      Rect rect = *r & bounds;
      rectangle ( img, *r, color, 3, 8, 0 );
    }
}
/**<
 * This function draws the rectangles given in a vector in a depth image.
 * @return void
 * @param &img A Mat containing an image.
 * @param faces A vector<Rect> that contains the rectangles bounding the faces.
 * */
//@}

/** @name Get the center of a rectangle
*
*/
//@{
Point CvUtils::get_rect_centre ( Rect rect )
{
  Point centre;
  centre.x = rect.x + ( rect.width * 0.5 );
  centre.y = rect.y + ( rect.height * 0.5 );
  return centre;
}
/**<
 * This function calculates the center of a rectangle given a rectangle.
 * @return A Point with x and y of the center.
 * @param rect A Rect with x and y coordinates of the upper left corner, height and width.
 * */

Point3f CvUtils::get_rect_centre_3d ( Rect rect, Mat depth_image )
{
  Point3f centre;
  centre.x = rect.x + ( rect.width * 0.5 );
  centre.y = rect.y + ( rect.height * 0.5 );
  centre.z = depth_image.at<unsigned short> ( centre.y, centre.x );
  return centre;
}
/**<
 * This function calculates the center of a rectangle given the rectangle and depth information.
 * @return A Point3f with x, y and z coordinates of the center.
 * @param rect A Rect with x and y coordinates of the upper left corner, height and width.
 * @param depth_image A Mat where each pixel gives us depth information.
 * */
//@}

/** @name Euclidean Distance
*
*/
double CvUtils::euclidean_distance ( Point3f a, Point3f b )
{
  return sqrt ( pow ( a.x - b.x, 2 ) + pow ( a.y - b.y, 2 ) + pow ( a.z - b.z, 2 ) );
}
/**<
 * This function calculates the Euclidean Distance given two 3D points a and b.
 * @return A double value of the computed Euclidean Distance.
 * @param a A Point3f containing x, y and z coordinates.
 * @param b A Point3f containing x, y and z coordinates.
 * */

double CvUtils::euclidean_distance ( Point a, Point b )
{
  return sqrt ( pow ( a.x - b.x, 2 ) + pow ( a.y - b.y, 2 ) );
}
/**<
 * This function calculates the Euclidean Distance given two 2D points a and b.
 * @return A double value of the computed Euclidean Distance.
 * @param a A Point containing x and y coordinates.
 * @param b A Point containing x and y coordinates.
 * */

bool CvUtils::is_there_face ( Mat &image, Rect rect )
{
  vector<Rect> detected_faces;

  rect = enlarge_window ( rect, image, 1.1 );
  Mat roi ( image, rect );
  Mat gray;
  Mat frame ( cvRound ( roi.rows ), cvRound ( roi.cols ), CV_8UC1 );

  cvtColor ( roi, gray, CV_BGR2GRAY );
  resize ( gray, frame, frame.size(), 0, 0, INTER_LINEAR );
  equalizeHist ( frame, frame );

  classifier.detectMultiScale ( frame, detected_faces,
                                1.1, 2, 0
                                //|CV_HAAR_FIND_BIGGEST_OBJECT
                                //|CV_HAAR_DO_ROUGH_SEARCH
                                |CV_HAAR_SCALE_IMAGE
                                ,
                                Size ( 30, 30 ) );

  return !detected_faces.empty();
}

Rect CvUtils::enlarge_window ( Rect orgrect, Mat image, double scale )
{
  Rect window;
  Point center = get_rect_centre ( orgrect );

  window = Rect ( center.x - scale * orgrect.width / 2, center.y - scale * orgrect.height / 2, scale * orgrect.width, scale * orgrect.height );

  Rect bounds ( 0, 0, image.cols, image.rows );
  window = window & bounds;

  return window;
}

cv::Rect CvUtils::enlarge_window_width ( Rect orgrect, Mat image, double scale )
{
  Rect window;
  Point center = get_rect_centre ( orgrect );

  window = Rect ( center.x - scale * orgrect.width / 2, orgrect.y, scale * orgrect.width, orgrect.height );

  Rect bounds ( 0, 0, image.cols, image.rows );
  window = window & bounds;

  return window;
}

cv::Rect CvUtils::enlarge_window_height ( Rect orgrect, Mat image, double scale )
{
  Rect window;
  Point center = get_rect_centre ( orgrect );

  window = Rect ( orgrect.x, center.y - scale * orgrect.height / 2, orgrect.width, scale * orgrect.height );

  Rect bounds ( 0, 0, image.cols, image.rows );
  window = window & bounds;

  return window;
}

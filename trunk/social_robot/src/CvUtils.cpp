#include "CvUtils.h"
#include "string_utils.h"
#include <ros/package.h>

#define PI 3.14159265

DepthFaceDetector depth_face_detector;

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
  transformation_matrix = ( Mat_<double> ( 4,3 ) << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0 );

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

bool CvUtils::is_there_face_rgb ( Mat &image, Rect rect )
{
  rect = enlarge_window ( rect, image, 1.1 );
  Mat roi ( image, rect );
  vector<Rect> detected_faces = detect_face_rgb ( roi );

  return !detected_faces.empty();
}

bool CvUtils::is_there_face_depth ( Mat &depth_image, Mat &disparity_image, Rect rect )
{
  Mat depth_roi ( depth_image, rect );
  Mat disparity_roi ( disparity_image, rect );
  vector<Rect> detected_faces = detect_face_depth ( depth_roi, disparity_roi );

  return !detected_faces.empty();
}

vector<Rect> CvUtils::detect_face_rgb ( Mat image )
{
  vector<Rect> detected_faces;

  Mat gray;
  Mat frame ( cvRound ( image.rows ), cvRound ( image.cols ), CV_8UC1 );

  cvtColor ( image, gray, CV_BGR2GRAY );
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

vector<Rect> CvUtils::detect_face_depth ( Mat depth_image, Mat disparity_image )
{
  vector<Rect> detected_faces = depth_face_detector.detect_face_depth ( depth_image, disparity_image );

  return detected_faces;
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

Rect CvUtils::enlarge_window_width ( Rect orgrect, Mat image, double scale )
{
  Rect window;
  Point center = get_rect_centre ( orgrect );

  window = Rect ( center.x - scale * orgrect.width / 2, orgrect.y, scale * orgrect.width, orgrect.height );

  Rect bounds ( 0, 0, image.cols, image.rows );
  window = window & bounds;

  return window;
}

Rect CvUtils::enlarge_window_height ( Rect orgrect, Mat image, double scale )
{
  Rect window;
  Point center = get_rect_centre ( orgrect );

  window = Rect ( orgrect.x, center.y - scale * orgrect.height / 2, orgrect.width, scale * orgrect.height );

  Rect bounds ( 0, 0, image.cols, image.rows );
  window = window & bounds;

  return window;
}

double CvUtils::compute_torso_orientation ( Mat depth_image, Point head_position )
{
  unsigned short thresh = 100;        // Threshold to differentiate person from background
  unsigned short diff = 0;            // Difference among pixel intensities
  int margin = 15;                    // extra margin for background
  int current = head_position.y;      // current pixel location
  int next = head_position.y - 1;     // next pixel location
  int ref = depth_image.cols / 2;     // reference variable which divides image in two parts

  unsigned short cur_int = 0;
  unsigned short nex_int = 0;

  while ( diff < thresh )
    {
      cur_int = depth_image.at<unsigned short> ( current, head_position.x );
      nex_int = depth_image.at<unsigned short> ( next, head_position.x );
      diff = abs ( nex_int - cur_int );
      current = next;
      next--;

      if ( next == 0 )
        {
          break;
        }
    }

  int y_find = 1.5 * ( head_position.y - next ) + head_position.y;

  int next_left = 0;
  int next_right = 0;

  if ( head_position.x >  ref )    // center of the head at the right of the middle of the image
    {                              // move to left
      current = head_position.x;
      next_left = head_position.x - 1;
      diff = 0.0;
      while ( diff < thresh )
        {
          cur_int = depth_image.at<unsigned short> ( y_find, current );
          nex_int = depth_image.at<unsigned short> ( y_find, next_left );
          diff = abs ( nex_int - cur_int );
          current = next_left;
          next_left--;
          if ( next_left == 0 )
            {
              break;
            }
        }
    }
  else        // center of the head at the left of the middle of the image
    {         // move to right
      current = head_position.x;
      next_right = head_position.x + 1;
      diff = 0.0;

      while ( diff < thresh )
        {
          cur_int = depth_image.at<unsigned short> ( y_find, current );
          nex_int = depth_image.at<unsigned short> ( y_find, next_right );
          diff = abs ( nex_int - cur_int );
          current = next_right;
          next_right++;
          if ( next_right == depth_image.cols )
            {
              break;
            }
        }
    }

  int x_cor = 0;     // x-coordinate of the center of the image
  int box_w = 0;     // width of the bounding box

  if ( head_position.x >  ref )    // center of the head at the right of the middle of the image
    {
      x_cor = next_left - margin;
      box_w = 2 * ( head_position.x - next_left ) + 2 * margin;

      if ( ( x_cor + box_w ) > ( depth_image.cols - 1 ) ) // width overpasses the image size
        {
          box_w = depth_image.cols - x_cor - 1;
        }
    }
  else        // center of the head at the left of the middle of the image
    {
      x_cor = head_position.x - ( next_right - head_position.x ) - margin;
      box_w = 2 * ( head_position.x - x_cor ) + 2 * margin;

      if ( x_cor < 0 )  // corner results out of the image
        {
          x_cor = 0;
          box_w = next_right + 2 * margin;
        }
    }

  // These x-coordinates replace extreme points
  int x1 = x_cor + ( 0.25 * double ( box_w ) );    // left point
  int x2 = x_cor + ( 0.75 * double ( box_w ) );    // right point

  // Reading depth values from Depth image
  unsigned short d1 = depth_image.at<unsigned short> ( y_find, x1 );
  unsigned short d2 = depth_image.at<unsigned short> ( y_find, x2 );

  double z1 = double ( d1 );  // depth coordinate of point 1
  double z2 = double ( d2 );  // depth coordinate of point 2

  double delta_z = abs ( z2 - z1 ); // difference of depths

  x1 = double ( x1 );     // x- coordinate point 1
  x2 = double ( x2 );     // x- coordinate point 2
  double y = double ( y_find );
  
  // Points on camera frame
  Mat pxyz1 = transform_point ( Point ( x1, y ) );
  Mat pxyz2 = transform_point ( Point ( x2, y ) );

  double py1 = pxyz1.at<double> ( 1, 0 );
  double py2 = pxyz1.at<double> ( 1, 0 );

  // calculating the torso orientation
  double delta_y = abs ( py2 - py1 );  // person width

  double theta = 0.0;     // Angle of Orientation

  if ( z1 > z2 )
    {
      theta = atan ( delta_z / delta_y ) * 180 / PI;
    }
  else
    {
      theta = -atan ( delta_z / delta_y ) * 180 / PI;
    }

  return theta;
}

Mat CvUtils::get_transformation_matrix ( void )
{
  return transformation_matrix;
}

Mat CvUtils::transform_point ( Point point )
{
  Mat mat_point = ( Mat_<double> ( 3, 1 ) << point.x, point.y, 1 );
  Mat tranformation_matrix = get_transformation_matrix();
  Mat transformed_point = tranformation_matrix * mat_point;
  return transformed_point;
}

void CvUtils::write_results_to_file ( string file_name, vector<vector<Rect> > rois )
{
  string rect_file = file_name;
  rect_file.append ( "_rects.yaml" );
  string centre_file = file_name;
  centre_file.append ( "_centre.yaml" );
  FileStorage fsr ( rect_file, FileStorage::WRITE );
  FileStorage fsc ( centre_file, FileStorage::WRITE );
  fsr << "roi" << "[";
  fsc << "center" << "[";
  for ( unsigned int i = 0; i < rois.size(); i++ )
    {
      fsr << "{:" << "length" << ( int ) rois[i].size() << "rects" << "[:";
      fsc << "{:" << "length" << ( int ) rois[i].size() << "points" << "[:";
      for ( unsigned int j = 0; j < rois[i].size(); j++ )
        {
          fsr << "{:" << "x" << rois[i].at ( j ).x << "y" << rois[i].at ( j ).y << "w" << rois[i].at ( j ).width << "h" << rois[i].at ( j ).height << "}";
          Point center = get_rect_centre ( rois[i].at ( j ) );
          fsc << "{:" << "x" << center.x << "y" << center.y << "}";
        }
      fsr << "]" << "}";
      fsc << "]" << "}";
    }
  fsr.release();
  fsc.release();
}

void CvUtils::write_results_to_file ( string file_name, vector<Rect> rois )
{
  string rect_file = file_name;
  rect_file.append ( "_rects.yaml" );
  string centre_file = file_name;
  centre_file.append ( "_centre.yaml" );
  FileStorage fsr ( rect_file, FileStorage::WRITE );
  FileStorage fsc ( centre_file, FileStorage::WRITE );
  fsr<<"roi"<<"[";
  fsc<<"center"<<"[";
  for ( unsigned int i = 0; i < rois.size(); i++ )
    {
      fsr << "{:"<< "x" << rois[i].x << "y" << rois[i].y << "w" << rois[i].width << "h" << rois[i].height <<"}";
      Point center ( rois[i].x + ( rois[i].width / 2 ), rois[i].y + ( rois[i].height / 2 ) );
      fsc << "{:"<< "x" << center.x << "y" << center.y <<"}";
    }
  fsr.release();
  fsc.release();
}

void CvUtils::write_to_file ( string filename, std::vector< double > rois, double mse, double mean )
{
  string distance_file = filename;
  distance_file.append ( "_dist.yaml" );
  FileStorage fsr ( distance_file, FileStorage::WRITE );
  fsr << "stdv " << sqrt ( mse );
  fsr << "mean " << mean;
  fsr << "distance" << "[";
  for ( unsigned int i = 0; i < rois.size(); i++ )
    {
      fsr << "{:" << "d" << rois[i] << "}";
    }
  fsr.release();
}

void CvUtils::read_from_file ( string filename, vector<vector<Point> > *rois )
{
  FileStorage fs ( filename, FileStorage::READ );
  FileNode centers = fs["center"];
  FileNodeIterator it = centers.begin(), it_end = centers.end();

  for ( ; it != it_end; ++it )
    {
      int length = ( int ) ( *it ) ["length"];
      FileNode points_fn = ( *it ) ["points"];
      FileNodeIterator itp = points_fn.begin(), itp_end = points_fn.end();
      vector<Point> points ( length );
      for ( unsigned i = 0 ; itp != itp_end; ++itp, i++ )
        {
          int x = ( int ) ( *itp ) ["x"];
          int y = ( int ) ( *itp ) ["y"];
          points[i] = Point ( x, y );
        }
      rois->push_back ( points );
    }
  fs.release();
}

void CvUtils::read_from_file ( string filename, vector<Point> &rois )
{
  FileStorage fs ( filename, FileStorage::READ );
  FileNode features = fs["center"];
  FileNodeIterator it = features.begin(), it_end = features.end();

  for ( ; it != it_end; ++it )
    {
      int x = ( int ) ( *it ) ["x"];
      int y = ( int ) ( *it ) ["y"];
      rois.push_back ( Point ( x,y ) );
    }
  fs.release();
}

void CvUtils::compare_gt_results ( vector< vector< Point > > gt, vector< vector< Point > > results )
{
  unsigned int nframes = gt.size();
  if ( nframes == 0 )
    {
      // nothing to be compares
      return;
    }
  unsigned int npeople = gt[0].size();

  vector< vector< Point > > gt_per_person ( npeople );
  vector< vector< Point > > results_per_person ( npeople );
  vector< vector< Point > > outliers_per_frame ( nframes );

  for ( unsigned int i = 0; i < nframes; i++ )
    {
      vector<Point> outliers;
      vector<Point> matching;
      data_association ( gt[i], results[i], &matching, &outliers );
      for ( unsigned int j = 0; j < npeople; j++ )
        {
          gt_per_person[j].push_back ( gt[i].at ( j ) );
          results_per_person[j].push_back ( matching[j] );
        }
      outliers_per_frame.push_back ( outliers );
    }
  for ( unsigned int j = 0; j < npeople; j++ )
    {
      string file_name = "results";
      file_name.append ( inttostr ( j ) );
      compare_gt_results ( gt_per_person[j], results_per_person[j], file_name );
    }
}

void CvUtils::compare_gt_results ( vector<Point> gt, vector<Point>results, string filename )
{
  // Compute distance
  vector<double> distance;
  double d;
  double sum;
  for ( unsigned int i = 0; i < results.size(); i++ )
    {
      d = euclidean_distance ( gt[i], results[i] );
      distance.push_back ( d );
      sum += d;
    }

  // Compute mean of distances

  double size_d = results.size();
  double mean_d = sum / size_d;

  // Compute MSE
  double s_mse = 0;
  for ( int i = 0; i < size_d; i++ )
    {
      s_mse += pow ( distance[i], 2 );
    }

  double MSE = s_mse / size_d;

  write_to_file ( filename, distance, MSE, mean_d );
}

void CvUtils::data_association ( vector<Point> a, vector<Point> b, vector<Point> *matching, vector<Point> *outliers )
{
  Mat mat_compare ( a.size(), b.size(), CV_32F );
  float result;
  double minVal, maxVal;
  Point minLoc, maxLoc;

  if ( b.empty() )
    {
      for ( unsigned int i = 0; i < a.size(); i++ )
        {
          matching->push_back ( Point ( 0, 0 ) );
        }
      return;
    }

  for ( unsigned int i = 0; i < a.size(); i++ )
    {
      for ( unsigned int j = 0; j < b.size(); j++ )
        {
          result = euclidean_distance ( a[i], b[j] );
          mat_compare.at<float> ( i, j ) = result;
        }
    }

  vector<unsigned int> min_locations ( b.size() );
  for ( int k = 0; k < mat_compare.rows; k++ )
    {
      minMaxLoc ( mat_compare.row ( k ), &minVal, &maxVal, &minLoc, &maxLoc );
      if ( minVal < 75 )
        {
          matching->push_back ( b[minLoc.x] );
          min_locations[minLoc.x] = 1;
        }
      else
        {
          matching->push_back ( Point ( 0, 0 ) );
        }
    }

  for ( unsigned int k = 0; k < min_locations.size(); k++ )
    {
      if ( min_locations[k] == 0 )
        {
          outliers->push_back ( b[k] );
        }
    }

}

void CvUtils::create_combine_gt_vector ( vector<string> filenames, vector<vector<Point> > &total_gt )
{
  for ( unsigned int i = 0; i < filenames.size(); i++ )
    {
      vector<Point> gt;
      read_from_file ( filenames[i], gt );

      for ( unsigned int g = 0; g < gt.size(); g++ )
        {
          if ( total_gt.size() > g )
            {
              total_gt[g].push_back ( gt[g] );
            }
          else
            {
              vector<Point> points;
              points.push_back ( gt[g] );
              total_gt.push_back ( points );
            }
        }
    }
}

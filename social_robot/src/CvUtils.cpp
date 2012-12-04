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
  transformation_matrix = ( Mat_<float> ( 4,3 ) <<
                            0.00189980382723900, 0, -0.595945338814418,
                            0, 0.00189980382723900, -0.492084050717330,
                            0, 0, 1,
                            0, 0, 0
                          );

  string cascade_name;
  cascade_name.append ( package_path );
  cascade_name.append ( "/rsrc/haarcascades/haarcascade_frontalface_alt.xml" );
  if ( !classifier.load ( cascade_name ) )
    {
      cerr << "ERROR: Could not load cascade classifier \"" << cascade_name << "\"" << endl;
    }
  string face_cascade_name;
  face_cascade_name.append ( package_path );
  face_cascade_name.append ( "/rsrc/haarcascades/haarcascade_frontalface_alt.xml" ); // lbpcascade_frontalface
  if ( !face_classifier.load ( face_cascade_name ) )
    {
      cerr << "ERROR: Could not load cascade classifier \"" << face_cascade_name << "\"" << endl;
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

/** @name Detect faces
*
*/
//@{
bool CvUtils::is_there_face_rgb ( Mat &image, Rect rect )
{
  rect = enlarge_window ( rect, image, 1.1 );
  Mat roi ( image, rect );
  vector<Rect> detected_faces = detect_face_rgb ( roi, face_classifier );

  return !detected_faces.empty();
}
/**<
 * This function checks if there is a face in the RGB image.
 * @return A bool variable that says if a face exists.
 * @param image A Mat containing the RGB image.
 * @param rect A Rect containing the ROI.
 * */

bool CvUtils::is_there_face_depth ( Mat &depth_image, Mat &disparity_image, Rect rect )
{
  Mat depth_roi ( depth_image, rect );
  Mat disparity_roi ( disparity_image, rect );
  vector<Rect> detected_faces = detect_face_depth ( depth_roi, disparity_roi );

  return !detected_faces.empty();
}
/**<
 * This function checks if there is a face in the depth image.
 * @return A bool variable that says if a face exists.
 * @param depth_image A Mat containing the depth image.
 * @param disparity_image A Mat containing the disparity image.
 * @param rect A Rect containing the ROI.
 * */


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
/**<
 * This function detects faces in the RGB image.
 * @return A vector of Rect containing all the bounding-boxes of the detected faces.
 * @param image Mat containing the RGB image.
 * */

vector<Rect> CvUtils::detect_face_rgb ( Mat image, CascadeClassifier cascade_classifier )
{
  vector<Rect> detected_faces;

  Mat gray;
  Mat frame ( cvRound ( image.rows ), cvRound ( image.cols ), CV_8UC1 );

  cvtColor ( image, gray, CV_BGR2GRAY );
  resize ( gray, frame, frame.size(), 0, 0, INTER_LINEAR );
  equalizeHist ( frame, frame );

  cascade_classifier.detectMultiScale ( frame, detected_faces,
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
/**<
 * This function detects faces in the depth image.
 * @return A vector of Rect containing all the bounding-boxes of the detected faces.
 * @param image Mat containing the  image.
 * */

//@}

/** @name Enlarge window
*
*/
//@{
Rect CvUtils::enlarge_window ( Rect orgrect, Mat image, double scale )
{
  Rect window;
  Point center = get_rect_centre ( orgrect );

  window = Rect ( center.x - scale * orgrect.width / 2, center.y - scale * orgrect.height / 2, scale * orgrect.width, scale * orgrect.height );

  Rect bounds ( 0, 0, image.cols, image.rows );
  window = window & bounds;

  return window;
}
/**<
 * This function increases the size of the ROI.
 * @return A Rect containing enlarged ROI.
 * @param orgrect A Rect containing the original ROI.
 * @param image A Mat containing the original image.
 * @param scale A double  value containing the scale for increasing the size of the ROI.
 * */


Rect CvUtils::enlarge_window_width ( Rect orgrect, Mat image, double scale )
{
  Rect window;
  Point center = get_rect_centre ( orgrect );

  window = Rect ( center.x - scale * orgrect.width / 2, orgrect.y, scale * orgrect.width, orgrect.height );

  Rect bounds ( 0, 0, image.cols, image.rows );
  window = window & bounds;

  return window;
}
/**<
 * This function increases the width of a ROI.
 * @return A Rect containing enlarged ROI.
 * @param orgrect A Rect containing the original ROI.
 * @param image A Mat containing the original image.
 * @param scale A double  value containing the scale for increasing the width of the ROI.
 * */

Rect CvUtils::enlarge_window_height ( Rect orgrect, Mat image, double scale )
{
  Rect window;
  Point center = get_rect_centre ( orgrect );

  window = Rect ( orgrect.x, center.y - scale * orgrect.height / 2, orgrect.width, scale * orgrect.height );

  Rect bounds ( 0, 0, image.cols, image.rows );
  window = window & bounds;

  return window;
}
/**<
 * This function increases the height of a ROI.
 * @return A Rect containing enlarged ROI.
 * @param orgrect A Rect containing the original ROI.
 * @param image A Mat containing the original image.
 * @param scale A double  value containing the scale for increasing the h of the ROI.
 * */
//@}

/** @name Torso orientation
*
*/
//@{
float CvUtils::compute_torso_orientation ( Mat depth_image, Point head_position )
{
  unsigned short thresh = 100;        // Threshold to differentiate person from background
  unsigned short diff = 0;            // Difference among pixel intensities
  int margin = 15;                    // extra margin for background
  int current = head_position.y;      // current pixel location
  int next = head_position.y - 1;     // next pixel location

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

  int y_cor = next - margin;
  int y_find = 1.5 * ( head_position.y - next ) + head_position.y;
  if ( y_find > depth_image.rows )
    {
      y_find = y_cor + ( depth_image.rows - y_cor ) * 0.7;
    }

  int next_left = 0;
  int next_right = 0;

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

      //cout<<"Next Left: "<< next_left<<endl;

      if ( next_left == 0 )
        {
          break;
        }
    }

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
    
  int x_cor = 0;     // x-coordinate of the center of the image
  int box_w = 0;      // width of the bounding box

  box_w = next_right - next_left + ( 2*margin );
  x_cor = head_position.x - ( box_w/2 );

  if ( x_cor < 0 )
    {
      x_cor = 0;
    }

  if ( ( x_cor + box_w ) > depth_image.cols )
    {
      box_w = depth_image.cols - x_cor -1;
    }
            
  // These x-coordinates replace extreme points
  int x1 = x_cor + ( 0.25 * double ( box_w ) );    // left point
  int x2 = x_cor + ( 0.75 * double ( box_w ) );    // right point

  // Reading depth values from Depth image
  unsigned short d1 = depth_image.at<unsigned short> ( y_find, x1 );
  unsigned short d2 = depth_image.at<unsigned short> ( y_find, x2 );

  float z1 = float ( d1 );  // depth coordinate of point 1
  float z2 = float ( d2 );  // depth coordinate of point 2

  float delta_z = abs ( z2 - z1 ); // difference of depths

  int y = y_find;
  
  // Points on camera frame
  Mat pxyz1 = transform_point ( Point ( x1, y ) );
  Mat pxyz2 = transform_point ( Point ( x2, y ) );

  float px1 = pxyz1.at<float> ( 0, 0 ) * 1000.0;
  float px2 = pxyz2.at<float> ( 0, 0 ) * 1000.0;

  // calculating the torso orientation
  float delta_x = abs ( px2 - px1 );  // person width

  float theta = 0.0;     // Angle of Orientation

  if ( abs ( delta_x ) < 100 ) // side view of the person
    {
      theta = 90.0;
    }
  else if ( z1 > z2 )
    {
      theta = atan ( delta_z / delta_x ) * 180 / PI;
    }
  else
    {
      theta = -atan ( delta_z / delta_x ) * 180 / PI;
    }

  return theta;
}

/**<
 * This function computes the torso orientation.
 * @return A double value containing the torso orientation in degrees.
 * @param depth_image A Mat containing the original depth image.
 * @param head_position A Point containing the coordinates of the head.
 * */

Mat CvUtils::get_transformation_matrix ( void )
{
  return transformation_matrix;
}

/**<
 * This function computes projection matrix based on intrinsec and extrinsec parameters.
 * @return A Mat containing the projection matrix.
 * */

Mat CvUtils::transform_point ( Point point )
{
  Mat mat_point = ( Mat_<float> ( 3, 1 ) << point.x, point.y, 1 );
  Mat tranformation_matrix = get_transformation_matrix();
  Mat transformed_point = tranformation_matrix * mat_point;
  return transformed_point;
}
/**<
 * This function transforms the point with the projection matrix.
 * @return A Mat containing the transformed point.
 * @param point A Point containing the original point.
 * */
//@}


/** @name Write to file
*
*/
//@{
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

/**<
 * This function writes the results to a .yaml file.
 * @return 
 * @param file_name A String containing the name where the results will be saved.
 * @param rois A vector of vector of rectangles containing the ROI that will be saved.
 * */

void CvUtils::write_results_to_file ( string file_name, vector< vector< Point > > points, double outliers_ratio, ConfusionMatrix eval )
{
  string centre_file = file_name;
  centre_file.append ( "_centre.yaml" );  
  FileStorage fsc ( centre_file, FileStorage::WRITE );
  double accuracy, error_rate;
  accuracy = (double)  (eval.tp + eval.tn)/ (double) (eval.tp + eval.tn + eval.fp + eval.fn);
  error_rate = (double)  (eval.fp + eval.fn)/ (double) (eval.tp + eval.tn + eval.fp + eval.fn);
  cout << accuracy << endl;
  fsc << "True positives" << eval.tp << "True negatives" << eval.tn << "False positives" << eval.fp << "False negatives" << eval.fn;
  fsc << "Precision accuracy" << accuracy << "Error rate" << error_rate; 
  fsc << "Outliers" << outliers_ratio << "center" << "[";
  for ( unsigned int i = 0; i < points.size(); i++ )
    {
      fsc << "{:" << "length" << ( int ) points[i].size() << "points" << "[:";
      for ( unsigned int j = 0; j < points[i].size(); j++ )
        {
          Point center = points[i].at ( j );
          fsc << "{:" << "x" << center.x << "y" << center.y << "}";
        }
      fsc << "]" << "}";
    }
  fsc.release();
}
/**<
 * This function writes the results to a .yaml file.
 * @return 
 * @param file_name A String containing the name where the results will be saved.
 * @param points A vector of vector of Points containing the interest points that will be saved.
 * @param outliers_ratio A double variable containing the outliers ratio that will be saced.
 * */

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
/**<
 * This function writes the results to a .yaml file.
 * @return 
 * @param file_name A String containing the name where the results will be saved.
 * @param rois A vector of Rect containing the ROI that will be saved.
 * */

void CvUtils::write_results_to_file ( string file_name, vector<Point> points )
{
  string centre_file = file_name;
  centre_file.append ( "_centre.yaml" );
  FileStorage fsc ( centre_file, FileStorage::WRITE );
  fsc<<"center"<<"[";
  for ( unsigned int i = 0; i < points.size(); i++ )
    {
      Point center = points[i];
      fsc << "{:"<< "x" << center.x << "y" << center.y <<"}";
    }
  fsc.release();
}
/**<
 * This function writes the results to a .yaml file.
 * @return 
 * @param file_name A String containing the name where the results will be saved.
 * @param points A vector of Points containing the interest points that will be saved.
 * */

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
/**<
 * This function writes the results to a .yaml file.
 * @return 
 * @param file_name A String containing the name where the results will be saved.
 * @param rois A vector of doubles containing the interest points that will be saved to the file.
 * @param mse A double variable containing the mean square error that will be saved to the file.
 * @param mean A double variable containing the mean that will be saved to the file.
 * */
//@}


/** @name Read from file
*
*/
//@{
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
/**<
 * This function reads the results to from a .yaml file.
 * @return 
 * @param file_name A String containing the name where the results will be saved.
 * @param rois A vector of vector of points containing the interest points that will be read from file.
 * */

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
/**<
 * This function reads the results to from a .yaml file.
 * @return 
 * @param file_name A String containing the name where the results will be saved.
 * @param rois A vector of points containing the interest points that will be read from file.
 * */
//@}

/** @name Compare results
*
*/
//@{
void CvUtils::compare_gt_results ( vector< vector< Point > > gt, vector< vector< Point > > results )
{
  ConfusionMatrix eval;
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
  double outliers_ratio = 0;
  
  for ( unsigned int i = 0; i < nframes; i++ )
    {
      vector<Point> outliers;
      vector<Point> matching;
      ConfusionMatrix eval_temp = data_association ( gt[i], results[i], &matching, &outliers );
      eval.tp += eval_temp.tp;
      eval.tn += eval_temp.tn;
      eval.fp += eval_temp.fp;
      eval.fn += eval_temp.fn;
      for ( unsigned int j = 0; j < npeople; j++ )
        {
          gt_per_person[j].push_back ( gt[i].at ( j ) );
          results_per_person[j].push_back ( matching[j] );
        }
      outliers_per_frame[i] = outliers;
      outliers_ratio += outliers.size();
      
    }
  for ( unsigned int j = 0; j < npeople; j++ )
    {
      string file_name = "results";
      file_name.append ( inttostr ( j ) );
      compare_gt_results ( gt_per_person[j], results_per_person[j], file_name );
    }
    
  outliers_ratio /= nframes;
    
  write_results_to_file ( "evaluation", outliers_per_frame, outliers_ratio, eval );
}
/**<
 * This function compare the ground-truth results to the detection and tracking algorithm
 * @return 
 * @param gt A vector of vector of points containing the ground-truth
 * @param results A vector of vector of points containing the results of the implemented algorithm
 * */

void CvUtils::compare_gt_results ( vector<Point> gt, vector<Point>results, string filename )
{
  // Compute distance
  vector<double> distance;
  double d;
  double sum = 0;
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
/**<
 * This function compare the ground-truth results to the detection and tracking algorithm
 * @return 
 * @param gt A vector of points containing the ground-truth
 * @param results A vector of points containing the results of the implemented algorithm
 * */
//@}

ConfusionMatrix CvUtils::data_association ( vector<Point> gt, vector<Point> results, vector<Point> *matching, vector<Point> *outliers )
{
  ConfusionMatrix conf_mat;
  Mat mat_compare ( gt.size(), results.size(), CV_32F );
  float result;
  double minVal, maxVal;
  Point minLoc, maxLoc;

  if ( results.empty() )
    {
      for ( unsigned int i = 0; i < gt.size(); i++ )
        {
          matching->push_back ( Point ( 0, 0 ) );
          if ( gt[i].x == 0 && gt[i].y == 0 )
            {
              conf_mat.tn++;
            }
          else
            {
              conf_mat.fn++;
            }
        }
      return conf_mat;
    }

  for ( unsigned int i = 0; i < gt.size(); i++ )
    {
      for ( unsigned int j = 0; j < results.size(); j++ )
        {
          result = euclidean_distance ( gt[i], results[j] );
          mat_compare.at<float> ( i, j ) = result;
        }
    }

  vector<unsigned int> min_locations ( results.size() );
  for ( int k = 0; k < mat_compare.rows; k++ )
    {
      minMaxLoc ( mat_compare.row ( k ), &minVal, &maxVal, &minLoc, &maxLoc );
      if ( minVal < 100 )
        {
          matching->push_back ( results[minLoc.x] );
          min_locations[minLoc.x] = 1;
          conf_mat.tp++;
        }
      else
        {
          matching->push_back ( Point ( 0, 0 ) );
          if ( gt[k].x == 0 && gt[k].y == 0 )
            {
              conf_mat.tn++;
            }
          else
            {
              conf_mat.fn++;
            }
        }
    }

  for ( unsigned int k = 0; k < min_locations.size(); k++ )
    {
      if ( min_locations[k] == 0 )
        {
          outliers->push_back ( results[k] );
          conf_mat.fp++;
        }
    }

  return conf_mat;
}
/**<
 * This function associates the ground-truth results to the detection and tracking algorithm
 * and it stores the information in a ConfusionMatrix.
 * @return ConfusionMatrix
 * @param a A vector of points containing the ground-truth.
 * @param b A vector of points containing the results of the implemented algorithm.
 * @param matching A vector containing the mathing between the ground-truth and the results.
 * */


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
/**<
 * This function combines all together the results of the ground-truth
 * @return 
 * @param filenames A vector of string containing the names of the ground-truth files
 * @param total_gt A vector of vector of points containing the combined ground-truth.
 * */

void CvUtils::update_depth_face_detector ( DepthFaceDetector depthfacedetector )
{
  depth_face_detector = depthfacedetector;
}

DepthFaceDetector CvUtils::get_depth_face_detector ( void )
{
  return depth_face_detector;
}

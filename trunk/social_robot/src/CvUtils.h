#ifndef CVUTILS_H
#define CVUTILS_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <iostream>
#include <vector>
#include "particle_filter/StateData.h"

/** 
* @class CvUtils
*
* @brief This class contains useful functions for the image processing part.
* 
* @author Social Robot
* 
*/

using namespace std;
using namespace cv;

const Scalar RED = Scalar ( 0, 0, 255 );
const Scalar BLUE = Scalar ( 255, 0, 0 );
const Scalar GREEN = Scalar ( 0, 255, 0 );
const Scalar CYAN = Scalar ( 255, 255, 0 );
const Scalar MAGENTA = Scalar ( 255, 0, 255 );
const Scalar YELLOW = Scalar ( 0, 255, 255 );
const Scalar WHITE = Scalar ( 255, 255, 255 );
const Scalar BLACK = Scalar ( 0, 0, 0 );

class CvUtils
  {
  public:
    Mat rgb2bw ( Mat im_rgb );
    Mat preprocessing ( Mat image );
    void get_non_zeros ( Mat img, Mat prob, vector<Point3f> *points, Point pdiff = Point ( 0, 0 ), double scale = 1 );

    void draw_rgb_faces ( Mat &img, vector<Rect> faces );
    void draw_tracking_faces ( Mat &img, vector<StateData> state_datas );
    void draw_depth_faces ( Mat &img, vector<Rect> faces );

    Point get_rect_centre ( Rect rect );
    Point3f get_rect_centre_3d ( Rect rect, Mat depth_image );

    double euclidean_distance ( Point3f a, Point3f b );
    double euclidean_distance ( Point a, Point b );
    
    Rect enlarge_window ( Rect orgrect, Mat image, double scale = 2.0 );
  };

#endif // CVUTILS_H

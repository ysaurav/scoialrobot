#ifndef CVUTILS_H
#define CVUTILS_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <iostream>
#include <vector>
#include "DepthFaceDetector.h"

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

class DepthFaceDetector;

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

    CvUtils ( void );
    Mat rgb2bw ( Mat im_rgb );
    Mat preprocessing ( Mat image );
    void get_non_zeros ( Mat img, Mat prob, vector<Point3f> *points, Point pdiff = Point ( 0, 0 ), double scale = 1 );

    void draw_rgb_faces ( Mat &img, vector<Rect> faces );
    void draw_depth_faces ( Mat &img, vector<Rect> faces );

    Point get_rect_centre ( Rect rect );
    Point3f get_rect_centre_3d ( Rect rect, Mat depth_image );

    double euclidean_distance ( Point3f a, Point3f b );
    double euclidean_distance ( Point a, Point b );

    bool is_there_face_rgb ( Mat &image, Rect rect );
    bool is_there_face_depth ( Mat &depth_image, Mat &disparity_image, Rect rect );
    vector<Rect> detect_face_rgb ( Mat image );
    vector<Rect> detect_face_depth ( Mat depth_image, Mat disparity_image );

    Rect enlarge_window ( Rect orgrect, Mat image, double scale = 2.0 );
    Rect enlarge_window_width ( Rect orgrect, Mat image, double scale = 2.0 );
    Rect enlarge_window_height ( Rect orgrect, Mat image, double scale = 2.0 );

    double compute_torso_orientation ( Mat depth_image, Point head_position );

    void write_results_to_file ( string file_name, vector<vector<Rect> > rois );
    void write_results_to_file ( string file_name, vector<Rect> rois );
    void write_to_file ( string filename, vector<double> rois, double mse );
    
    void read_from_file ( string filename, vector<vector<Point> > *rois );
    void read_from_file ( string filename, vector<Point> &rois );
    
    void compare_gt_results ( vector<vector<Point> > gt, vector<vector<Point> > results );
    void compare_gt_results ( vector<Point> gt, vector<Point> results, string filename );
    void data_association ( vector<Point> a, vector<Point> b, vector<Point> *matching, vector<Point> *outliers );
    
    void create_combine_gt_vector ( vector<string> filenames, vector<vector<Point> > &total_gt );

  private:
    CascadeClassifier classifier;
  };

#endif // CVUTILS_H

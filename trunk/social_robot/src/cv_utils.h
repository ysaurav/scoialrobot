#ifndef CV_UTILS
#define CV_UTILS

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <iostream>
#include <vector>

cv::Mat rgb2bw ( cv::Mat im_rgb );
cv::Mat preprocessing ( cv::Mat image );
cv::Mat preprocessing_16U ( cv::Mat image );
void get_non_zeros ( cv::Mat img, cv::Mat prob, std::vector<cv::Point3f> *points, cv::Point pdiff = cv::Point ( 0, 0 ), double scale = 1 );

void draw_rgb_faces ( cv::Mat &img, std::vector<cv::Rect> faces );
void draw_depth_faces ( cv::Mat &img, std::vector<cv::Rect> faces );
void region_growing( cv::Mat &src, cv::Mat &dst, unsigned char threshold, std::vector<cv::Point> seeds );

#endif

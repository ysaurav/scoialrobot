#ifndef CV_UTILS
#define CV_UTILS

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <vector>

cv::Mat rgb2bw(cv::Mat im_rgb);
void get_non_zeros(cv::Mat img, std::vector<cv::Point> *points, cv::Point pdiff = cv::Point(0, 0), double scale = 1);

#endif

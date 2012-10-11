#include "cv_utils.h"

using namespace cv;

Mat rgb2bw(cv::Mat im_rgb)
{
  Mat im_gray;
  cvtColor(im_rgb, im_gray, CV_RGB2GRAY);
  Mat im_bw;
  threshold(im_gray, im_bw, 128, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
  return im_bw;
}

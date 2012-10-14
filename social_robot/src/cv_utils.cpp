#include "cv_utils.h"

using namespace cv;

Mat rgb2bw(Mat im_rgb)
{
  Mat im_gray;
  if (im_rgb.channels() == 3)
  {
    cvtColor(im_rgb, im_gray, CV_RGB2GRAY);
  }
  else
  {
    im_gray = im_rgb;
  }
  
  Mat im_bw;
  threshold(im_gray, im_bw, 128, 255, CV_THRESH_BINARY);
  return im_bw;
}

void get_non_zeros(Mat img, std::vector<cv::Point> *points, cv::Point pdiff, double scale)
{
  int k = 0;
  for (int i = 0; i < img.rows; i++)
  {
    float *rowi = img.ptr<float>(i);
    for (int j = 0; j < img.cols; j++)
    {
      if (rowi[j] != 0)
      {
        cv::Point point = (cv::Point(j, i) + pdiff) * scale;
        points->push_back(point);
        k++;
      }
    }
  }
}

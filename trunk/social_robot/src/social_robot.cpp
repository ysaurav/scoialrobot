#include <ros/ros.h>

#include "kinect_proxy.h"
#include "cv_utils.h"

#define HEAD_TEMPLATE "pictures/template.png"
#define THR1          5
#define THR2          7
#define THR3          10
#define SCALES        5

cv::Mat preprocessing(cv::Mat image)
{
  cv::Mat dst;
  cv::Mat temp;
  cv::resize(image, temp, cv::Size(), 2, 2, cv::INTER_NEAREST);
  cv::resize(temp, dst, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST);
  cv::medianBlur(dst, dst, 5);
  // cv::namedWindow("Preprocessing", CV_WINDOW_AUTOSIZE);
  // cv::imshow("Preprocessing", dst);
  // cv::waitKey(0);
  return dst;
}

std::vector<cv::Point> chamfer_matching(cv::Mat image)
{
  cv::Mat canny_im(image.rows, image.cols, CV_8U);
  cv::Mat template_im = cv::imread(HEAD_TEMPLATE, CV_LOAD_IMAGE_COLOR);

  cv::Mat* pyramid = new cv::Mat[SCALES];
  cv::Mat* chamfer = new cv::Mat[SCALES];
  cv::Mat* matching = new cv::Mat[SCALES];
    
  // calculate edge detection  
  cv::Canny(image, canny_im, THR1, THR2, 3, true);
  
  // calculate the Canny pyramid
  pyramid[0] = canny_im;
  for(int i = 1; i < SCALES; i++)
  {
    cv::resize(pyramid[i - 1], pyramid[i], cv::Size(), 0.75, 0.75, cv::INTER_NEAREST);
  }
  
  // calculate distance transform
  for(int i = 0; i < SCALES; i++)
  {
    cv::distanceTransform((255 - pyramid[i]), chamfer[i], CV_DIST_C, 3);
    cv::normalize(chamfer[i], chamfer[i], 0.0, 1.0, cv::NORM_MINMAX);
  }
  
  // matching with the template
  template_im = rgb2bw(template_im);
  template_im.convertTo(template_im,CV_32F);  

  // find the best match:
  double minVal, maxVal;
  cv::Point minLoc, maxLoc;
  double xdiff = template_im.cols / 2;
  double ydiff = template_im.rows / 2;
  cv::Point pdiff = cv::Point(xdiff, ydiff);
  std::vector<cv::Point> head_matched_points;

  for(int i = 0; i < SCALES; i++)
  {
    matchTemplate(chamfer[i], template_im, matching[i], CV_TM_CCOEFF);
    normalize(matching[i], matching[i], 0.0, 1.0, cv::NORM_MINMAX);
    cv::minMaxLoc(matching[i], &minVal, &maxVal, &minLoc, &maxLoc);
    cv::Mat matching_thr;
    threshold(matching[i], matching_thr, 1.0 / THR3, 1.0, CV_THRESH_BINARY_INV);
    double scale = pow(1.0 / 0.75, i);
    get_non_zeros(matching_thr, &head_matched_points, pdiff, scale);
  }

  return head_matched_points;
}

int main(int argc, char **argv)
{
  //  ros::init(argc, argv, "social_robot");  
  //  ros::spin();
  
  // temporarily reading the image from file later on change it to the live video.
  cv::Mat image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
  cvtColor(image, image, CV_RGB2GRAY);
  image = preprocessing(image);
  std::vector<cv::Point> head_matched_points = chamfer_matching(image);
  
  // TODO: remove me, just for testing
  for (unsigned int i = 0; i < head_matched_points.size(); i++)
  {
    circle(image, head_matched_points[i], 2, cvScalar(255, 255, 0, 0), 2, 8, 0);  
  }
  
  cv::namedWindow("Image", CV_WINDOW_AUTOSIZE);
  cv::imshow("Image", image);
  cv::waitKey(0);
  
  return 0;
}

#include <ros/ros.h>

#include "kinect_proxy.h"
#include "cv_utils.h"

#define HEAD_TEMPLATE "pictures/template.png"
#define THR1          4.5
#define THR2          6.5
#define SCALES        5

cv::Mat preprocessing(cv::Mat image)
{
  cv::Mat dst;
  cv::Mat temp;
  cv::resize(image, temp, cv::Size(), 2, 2, cv::INTER_NEAREST);
  cv::resize(temp, dst, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST);
  cv::medianBlur(dst, dst, 5);
  cv::namedWindow("Preprocessing", CV_WINDOW_AUTOSIZE);
  cv::imshow("Preprocessing", dst);
  cv::waitKey(0);
  return dst;
}

cv::Mat chamfer_matching(cv::Mat image)
{
  cv::Mat gray_im(image.rows, image.cols, CV_8U);
  cv::Mat canny_im(image.rows, image.cols, CV_8U);
  cv::Mat template_im = cv::imread(HEAD_TEMPLATE, CV_LOAD_IMAGE_COLOR);

  cv::Mat* pyramid = new cv::Mat[SCALES];
  cv::Mat* chamfer = new cv::Mat[SCALES];
  cv::Mat* matching = new cv::Mat[SCALES];
    
  // calculate edge detection
  cvtColor(image, gray_im, CV_RGB2GRAY);
  cv::Canny(gray_im, canny_im, THR1, THR2, 3, "true");
  
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
  cv::Point* screwCenter = new cv::Point[SCALES];

  for(int i = 0; i < SCALES; i++)
  {
    matchTemplate(chamfer[i], template_im, matching[i], CV_TM_CCOEFF);
    normalize(matching[i], matching[i], 0, 1, cv::NORM_MINMAX);
    cv::minMaxLoc(matching[i], &minVal, &maxVal, &minLoc, &maxLoc);
    // move center of detected screw to the correct position:
    screwCenter[i] = maxLoc + cv::Point(chamfer[i].cols / 2, chamfer[i].rows / 2);
    cv::namedWindow("Matching", CV_WINDOW_AUTOSIZE);
    cv::imshow("Matching", matching[i]);
    cv::waitKey(0);
  }

  return image;
}

int main(int argc, char **argv)
{
  //  ros::init(argc, argv, "social_robot");  
  //  ros::spin();
  
  // temporarily reading the image from file later on change it to the live video.
  cv::Mat image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
  cv::namedWindow("Original", CV_WINDOW_AUTOSIZE);
  cv::imshow("Original", image);
  image = preprocessing(image);
  image = chamfer_matching(image);
  
  
  return 0;
}

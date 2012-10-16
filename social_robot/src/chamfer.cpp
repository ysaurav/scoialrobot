#include <ros/ros.h>
#include <ros/package.h>
#include <string>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "cv_utils.h"

cv::Mat preprocessing(cv::Mat image)
{
  cv::Mat dst;
  cv::Mat temp;
  cv::resize(image, temp, cv::Size(), 2, 2, cv::INTER_NEAREST);
  cv::resize(temp, dst, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST);
  cv::medianBlur(dst, dst, 5);
  return dst;
}

int main(int argc, char **argv)
{
  if (argc != 6)
  {
    ROS_INFO("usage: chamfer RGB_IMAGE DEPTH_IMAGE THRS1 THRS2 MATCHING_THR");
    return 1;
  }

  cv::Mat rgb_im = cv::imread(argv[1], CV_LOAD_IMAGE_ANYDEPTH);
  cv::Mat depth_im = cv::imread(argv[2], CV_LOAD_IMAGE_ANYDEPTH);
  cv::Mat gray_im(depth_im.rows,depth_im.cols,CV_8U);
  cv::Mat canny_im(depth_im.rows,depth_im.cols,CV_8U);
  cv::Mat template_im = cv::imread("pictures/head_template.png", CV_LOAD_IMAGE_COLOR);
  
  std::cout << "1 depth " << rgb_im.depth() << " type " << rgb_im.type() << "\n";
  std::cout << "2 depth " << depth_im.depth() << " type " << depth_im.type() << "\n";
  
  std::cout << "2 Pixel: " << depth_im.at<unsigned short>(atoi(argv[3]), atoi(argv[4])) << "\n";
  return 0;
  
  float thr1 = atof(argv[3]);
  float thr2 = atof(argv[4]);
  
  int scales = 5;

  cv::Mat* DT = new cv::Mat[scales];
  cv::Mat* chamfer = new cv::Mat[scales];
  cv::Mat* matching = new cv::Mat[scales];
    
  // do processing
  
  
  // Calculate edge detection  
  rgb_im = preprocessing(rgb_im);
  cv::imshow("Hello", rgb_im);
  cv::waitKey(0);
  cv::Canny(rgb_im,canny_im,thr1,thr2,3,"true");
  
  // Calculate the pyramid
  DT[0] = canny_im;
  cv::imshow("Distance", DT[0]);
  cv::waitKey(0);
  for(int i=1;i<scales;i++)
  {
      cv::resize(DT[i-1],DT[i],cv::Size(),.75,.75,cv::INTER_NEAREST);
//      cv::imshow("Distance", DT[i]);
//      cv::waitKey(0);
  }
  
  // Calculate distance transform
  for(int i=0;i<scales;i++)
  {
      cv::distanceTransform((255-DT[i]),chamfer[i],CV_DIST_C,3);
      cv::normalize(chamfer[i], chamfer[i], 0.0, 1.0,cv::NORM_MINMAX);
//      cv::imshow("Distance",chamfer[i]);
//      cv::waitKey(0);
  }
  
  // Matching with the template
  //cvtColor(template_im, template_im, CV_RGB2GRAY);
  template_im = rgb2bw(template_im);
  template_im.convertTo(template_im, CV_32F);
  
  // display image
  //cv::namedWindow("Template", CV_WINDOW_AUTOSIZE);
  //cv::imshow("Template",template_im);
  //cv::waitKey(0);
  

  // Find a best match:
  double minVal, maxVal;
  cv::Point minLoc, maxLoc;
  double xdiff = template_im.cols / 2;
  double ydiff = template_im.rows / 2;
  cv::Point pdiff = cv::Point(xdiff, ydiff);
  
  for(int i=0;i<scales;i++)
  {
     matchTemplate(chamfer[i],template_im,matching[i],CV_TM_CCOEFF);
     normalize( matching[i], matching[i], 0.0, 1.0, cv::NORM_MINMAX);
     std::cout << "Size of matching image: " << matching[i].rows << " " << matching[i].cols << "\n";
     cv::minMaxLoc(matching[i], &minVal, &maxVal, &minLoc, &maxLoc);
     std::cout << "Min " << minVal << " Max " << maxVal << "\n";
     cv::Mat im_bw;
     threshold(matching[i], im_bw, maxVal / atof(argv[5]), 255, CV_THRESH_BINARY_INV);     
     std::vector<cv::Point> non_zero_points;
     double scale = pow(1.0 / 0.75, i);
     std::cout << "Scale is: " << scale << std::endl;
     get_non_zeros(im_bw, &non_zero_points, pdiff, scale);
     for (unsigned int j = 0; j < non_zero_points.size(); j++)
     {
       circle(rgb_im, non_zero_points[j], 10, cvScalar(255, 255, 0, 0), 2, 8, 0);         
     }
     cv::imshow("BW", rgb_im);
     cv::waitKey(0);
  }
  
  // write the result to the result folder
  std::string depth_im_result = ros::package::getPath("social_robot");
  depth_im_result.append("/logs/preprocessed_");
  depth_im_result.append(argv[2]);
  
  // bool imwrite_result = cv::imwrite(depth_im_result, depth_im);
  // std::cout << depth_im_result << " was " << imwrite_result << " (rows,cols) = "<< "(" << chamfer[4].rows << "," << chamfer[4].cols << ")" << std::endl;
  
  return 0;
}

#include <ros/ros.h>
#include <ros/package.h>
#include <string>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "cv_utils.h"


int main(int argc, char **argv)
{
  if (argc != 5)
  {
    ROS_INFO("usage: chamfer RGB_IMAGE DEPTH_IMAGE THRS1 THRS2");
    return 1;
  }

  cv::Mat rgb_im = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
  cv::Mat depth_im = cv::imread(argv[2], CV_LOAD_IMAGE_COLOR);
  cv::Mat gray_im(depth_im.rows,depth_im.cols,CV_8U);
  cv::Mat canny_im(depth_im.rows,depth_im.cols,CV_8U);
  cv::Mat template_im = cv::imread("pictures/head_template.png", CV_LOAD_IMAGE_COLOR);
  
  
  
  float thr1 = atof(argv[3]);
  float thr2 = atof(argv[4]);
  
  int scales = 5;

  cv::Mat* DT = new cv::Mat[scales];
  cv::Mat* chamfer = new cv::Mat[scales];
  cv::Mat* matching = new cv::Mat[scales];
    
  // do processing
  
  // Calculate edge detection
  cvtColor(rgb_im, gray_im, CV_RGB2GRAY);
  cv::Canny(gray_im,canny_im,thr1,thr2,3,"true");
  
  // Calculate the pyramid
  DT[0] = canny_im;
//  cv::imshow("Distance", DT[0]);
//  cv::waitKey(0);
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
  template_im.convertTo(template_im,CV_32F);
  cv::Mat template_im2(template_im.rows,template_im.cols,template_im.type());
  
  // display image
  //cv::namedWindow("Template", CV_WINDOW_AUTOSIZE);
  //cv::imshow("Template",template_im);
  //cv::waitKey(0);
  
  //filter2D(template_im,template_im2,-1,template_im);
  matchTemplate(template_im,template_im,template_im2,CV_TM_SQDIFF);
  // Find a best match:
  double minVal, maxVal;
  cv::Point minLoc, maxLoc;
  cv::minMaxLoc(template_im2, &minVal, &maxVal, &minLoc, &maxLoc);
  // Move center of detected screw to the correct position:  
  cv::Point screwCenter = maxLoc + cv::Point(template_im.cols/2, template_im.rows/2);
  std::cout << screwCenter.x << ", " << screwCenter.y << std::endl;

  
  for(int i=0;i<scales;i++)
  {
     //filter2D(chamfer[i],matching[i],-1,template_im);
     matchTemplate(chamfer[i],template_im,matching[i],CV_TM_CCOEFF);
     normalize( matching[i], matching[i], 0, 1, cv::NORM_MINMAX);
     std::cout << matching[i].rows << " " << matching[i].cols << "\n";
     cv::minMaxLoc(matching[i], &minVal, &maxVal, &minLoc, &maxLoc);
     // Move center of detected screw to the correct position:  
     cv::Point screwCenter =  cv::Point(chamfer[i].cols/2 + minLoc.x, chamfer[i].rows/2 + minLoc.y);
     std::cout << screwCenter.x << ", " << screwCenter.y << std::endl;
     rectangle(chamfer[i], minLoc, cv::Point(minLoc.x + template_im.cols, minLoc.y + template_im.rows ), cv::Scalar::all(0), 2, 8, 0 );
//     circle(chamfer[i], screwCenter, 10, cvScalar(255, 255, 0, 0), 1, 8, 0);
     cv::imshow("Hola",matching[i]);
     cv::waitKey(0);
  }
  
  for(int i=0;i<scales;i++)
  {
      cv::imshow("Distance",chamfer[i]);
      cv::waitKey(0);
  }
  
  // write the result to the result folder
  std::string depth_im_result = ros::package::getPath("social_robot");
  depth_im_result.append("/logs/preprocessed_");
  depth_im_result.append(argv[2]);
  
  bool imwrite_result = cv::imwrite(depth_im_result, depth_im);
  std::cout << depth_im_result << " was " << imwrite_result << " (rows,cols) = "<< "(" << chamfer[4].rows << "," << chamfer[4].cols << ")" << std::endl;
  
  return 0;
}

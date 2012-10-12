#include <ros/ros.h>
#include <ros/package.h>
#include <string>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


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
  for(int i=1;i<scales;i++)
  {
      cv::resize(DT[i-1],DT[i],cv::Size(),.75,.75,cv::INTER_NEAREST);
  }
  
  // Calculate distance transform
  for(int i=0;i<scales;i++)
  {
      cv::distanceTransform((255-DT[i]),chamfer[i],CV_DIST_C,3);
      cv::normalize(chamfer[i], chamfer[i], 0.0, 1.0,cv::NORM_MINMAX);
  }
  
  // Matching with the template
  cvtColor(template_im, template_im, CV_RGB2GRAY);
  cv::Mat template_im2(template_im.rows,template_im.cols,template_im.type());
  template_im.convertTo(template_im,CV_32F);
  
  //filter2D(template_im,template_im2,-1,template_im);
  for(int i=0;i<scales;i++)
  {
     filter2D(chamfer[i],matching[i],-1,template_im);
     cv::imshow("Hola",matching[i]);
     cv::waitKey(0);
  }
  
  // write the result to the result folder
  std::string depth_im_result = ros::package::getPath("social_robot");
  depth_im_result.append("/logs/preprocessed_");
  depth_im_result.append(argv[2]);
  
  // display image
  cv::namedWindow("Hola", CV_WINDOW_AUTOSIZE);
  cv::imshow("Hola",matching[0]);
  cv::waitKey(0);
  
  bool imwrite_result = cv::imwrite(depth_im_result, depth_im);
  std::cout << depth_im_result << " was " << imwrite_result << " (rows,cols) = "<< "(" << chamfer[4].rows << "," << chamfer[4].cols << ")" << std::endl;
  
  return 0;
}

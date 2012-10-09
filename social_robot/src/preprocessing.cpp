#include <ros/ros.h>
#include <ros/package.h>
#include <string>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char **argv)
{
  if (argc != 3)
  {
    ROS_INFO("usage: preprocessing RGB_IMAGE DEPTH_IMAGE");
    return 1;
  }

  cv::Mat rgb_im = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
  cv::Mat depth_im = cv::imread(argv[2], CV_LOAD_IMAGE_COLOR);

  // do processing

  // write the result to the result folder
  std::string depth_im_result = ros::package::getPath("social_robot");
  depth_im_result.append("/logs/preprocessed_");
  depth_im_result.append(argv[2]);
  bool imwrite_result = cv::imwrite(depth_im_result, depth_im);
  std::cout << depth_im_result << " was " << imwrite_result << std::endl;
  
  return 0;
}

#include "kinect_proxy.h"
#include <std_srvs/Empty.h>
#include <stereo_msgs/DisparityImage.h>

cv_bridge::CvImagePtr cv_rgb;
cv_bridge::CvImagePtr cv_depth;
cv_bridge::CvImagePtr cv_disparity;

bool save_depth(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  std::string depth_im_result = ros::package::getPath("social_robot");
  depth_im_result.append("/logs/kinect_depth_");
  depth_im_result.append(current_log_time());
  depth_im_result.append(".png");
  bool imwrite_result = cv::imwrite(depth_im_result, cv_depth->image);
  std::cout << depth_im_result << " was " << imwrite_result << std::endl;
  return imwrite_result;
}

bool save_rgb(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  std::string depth_im_result = ros::package::getPath("social_robot");
  depth_im_result.append("/logs/kinect_rgb_");
  depth_im_result.append(current_log_time());
  depth_im_result.append(".png");
  bool imwrite_result = cv::imwrite(depth_im_result, cv_rgb->image);
  std::cout << depth_im_result << " was " << imwrite_result << std::endl;
  return imwrite_result;
}

bool save_disparity(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  std::string depth_im_result = ros::package::getPath("social_robot");
  depth_im_result.append("/logs/kinect_disparity_");
  depth_im_result.append(current_log_time());
  depth_im_result.append(".png");
  bool imwrite_result = cv::imwrite(depth_im_result, cv_disparity->image);
  std::cout << depth_im_result << " was " << imwrite_result << std::endl;
  return imwrite_result;
}

void rgb_cb(const sensor_msgs::ImageConstPtr& msg)
{    
  try
  {
    cv_rgb = cv_bridge::toCvCopy(msg);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }      
}

void depth_cb(const sensor_msgs::ImageConstPtr& msg)
{    
  try
  {
    cv_depth = cv_bridge::toCvCopy(msg);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
}

void disparity_cb(const stereo_msgs::DisparityImageConstPtr& msg)
{    
  try
  {
    cv_disparity = cv_bridge::toCvCopy(msg->image);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }      
}

void init_kinect(void)
{
  ros::NodeHandle nh;
  ros::Subscriber rgb_sub = nh.subscribe("/camera/rgb/image_color", 1, rgb_cb);
  ros::Subscriber depth_sub = nh.subscribe("/camera/depth/image_raw", 1, depth_cb);
  ros::Subscriber disparity_sub = nh.subscribe("/camera/depth/disparity", 1, disparity_cb);
  ros::ServiceServer save_rgb_ser = nh.advertiseService("social_robot/save_rgb", save_rgb);
  ros::ServiceServer save_depth_ser = nh.advertiseService("social_robot/save_depth", save_depth);
  ros::ServiceServer save_disparity_ser = nh.advertiseService("social_robot/save_disparity", save_disparity);
  ros::spin();  
}



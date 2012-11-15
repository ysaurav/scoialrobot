#include "kinect_proxy.h"

#include <std_srvs/Empty.h>
#include <stereo_msgs/DisparityImage.h>

namespace enc = sensor_msgs::image_encodings;

cv_bridge::CvImagePtr cv_rgb;
cv_bridge::CvImagePtr cv_depth;
cv_bridge::CvImagePtr cv_disparity;
cv_bridge::CvImagePtr cv_ir;

bool save_all(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  std::string project_path = ros::package::getPath("social_robot");
  std::string rgb_path = project_path;
  std::string depth_path = project_path;
  std::string disparity_path = project_path;
  
  rgb_path.append("/logs/kinect_rgb_");
  rgb_path.append(current_log_time());
  rgb_path.append(".png");
  bool rgb_result = cv::imwrite(rgb_path, cv_rgb->image);
  std::cout << rgb_path << " was " << rgb_result << std::endl;
  
  depth_path.append("/logs/kinect_depth_");
  depth_path.append(current_log_time());
  depth_path.append(".png");
  bool depth_result = cv::imwrite(depth_path, cv_depth->image);
  std::cout << depth_path << " was " << depth_result << std::endl;
  
  disparity_path.append("/logs/kinect_disparity_");
  disparity_path.append(current_log_time());
  disparity_path.append(".png");
  bool disparity_result = cv::imwrite(disparity_path, cv_disparity->image);
  std::cout << disparity_path << " was " << disparity_result << std::endl;
  
  return rgb_result && depth_result && disparity_result;
}

bool save_depth(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  std::string im_path = ros::package::getPath("social_robot");
  im_path.append("/logs/kinect_depth_");
  im_path.append(current_log_time());
  im_path.append(".png");
  bool imwrite_result = cv::imwrite(im_path, cv_depth->image);
  std::cout << im_path << " was " << imwrite_result << std::endl;
  return imwrite_result;
}

bool save_rgb(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  std::string im_path = ros::package::getPath("social_robot");
  im_path.append("/logs/kinect_rgb_");
  im_path.append(current_log_time());
  im_path.append(".png");
  bool imwrite_result = cv::imwrite(im_path, cv_rgb->image);
  std::cout << im_path << " was " << imwrite_result << std::endl;
  return imwrite_result;
}

bool save_disparity(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  std::string im_path = ros::package::getPath("social_robot");
  im_path.append("/logs/kinect_disparity_");
  im_path.append(current_log_time());
  im_path.append(".png");
  bool imwrite_result = cv::imwrite(im_path, cv_disparity->image);
  std::cout << im_path << " was " << imwrite_result << std::endl;
  return imwrite_result;
}

bool save_ir(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  std::string im_path = ros::package::getPath("social_robot");
  im_path.append("/logs/kinect_ir_");
  im_path.append(current_log_time());
  im_path.append(".png");
  bool imwrite_result = cv::imwrite(im_path, cv_ir->image);
  std::cout << im_path << " was " << imwrite_result << std::endl;
  return imwrite_result;
}

void rgb_cb(const sensor_msgs::ImageConstPtr& msg)
{    
  try
  {
    cv_rgb = cv_bridge::toCvCopy(msg, enc::BGR8);
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
    cv_depth = cv_bridge::toCvCopy(msg); // , enc::MONO16
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

void ir_cb(const sensor_msgs::ImageConstPtr& msg)
{    
  try
  {
    cv_ir = cv_bridge::toCvCopy(msg);
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
  ros::Subscriber rgb_sub = nh.subscribe("/camera/rgb/image_rect_color", 1, rgb_cb);// /camera/rgb/image_rect_color
  ros::Subscriber depth_sub = nh.subscribe("/camera/depth_registered/image_rect", 1, depth_cb); // /camera/depth_registered/image_rect
  ros::Subscriber disparity_sub = nh.subscribe("/camera/depth_registered/disparity", 1, disparity_cb); //  /camera/depth_registered/disparity
  ros::Subscriber ir_sub = nh.subscribe("/camera/ir/image_rect", 1, ir_cb);
  
  ros::ServiceServer save_rgb_ser = nh.advertiseService("social_robot/save_rgb", save_rgb);
  ros::ServiceServer save_depth_ser = nh.advertiseService("social_robot/save_depth", save_depth);
  ros::ServiceServer save_disparity_ser = nh.advertiseService("social_robot/save_disparity", save_disparity);
  ros::ServiceServer save_ir_ser = nh.advertiseService("social_robot/save_ir", save_ir);
  ros::ServiceServer save_all_ser = nh.advertiseService("social_robot/save_all", save_all);
  ros::spin();  
}

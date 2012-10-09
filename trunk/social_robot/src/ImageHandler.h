#ifndef IMAGE_HANDLER
#define IMAGE_HANDLER

#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "string_utils.h"

class ImageHandler
{
  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  image_transport::Subscriber rgb_sub;
	image_transport::Subscriber depth_sub;
  image_transport::Subscriber disparity_sub;
  ros::ServiceServer service;

  cv_bridge::CvImagePtr cv_rgb;
  cv_bridge::CvImagePtr cv_depth;

  int i;
  
public:
  ImageHandler()
    : it(nh)
  {
    rgb_sub = it.subscribe("/camera/rgb/image_color", 1, &ImageHandler::rgb_cb, this);
		depth_sub = it.subscribe("/camera/depth/image_raw", 1, &ImageHandler::depth_cb, this);
		// disparity_sub = it.subscribe("/camera/depth/disparity", 1, &ImageHandler::disparity_cb, this);
    service = nh.advertiseService("save_depth", &ImageHandler::save_depth);
    i = 0;
  }

  ~ImageHandler()
  {

  }

  bool save_depth()
  {
    std::string depth_im_result = ros::package::getPath("social_robot");
    depth_im_result.append("/logs/kinect_image_");
     
    depth_im_result.append(".png");
    bool imwrite_result = cv::imwrite(depth_im_result, cv_depth->image);
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

    if (false)
    {
      std::string depth_im_result = ros::package::getPath("social_robot");
      depth_im_result.append("/logs/kinect_image_");
      
      depth_im_result.append(inttostr(i));
      depth_im_result.append(".png");
      bool imwrite_result = cv::imwrite(depth_im_result, cv_depth->image);
      std::cout << depth_im_result << " was " << imwrite_result << std::endl;
      i++;
    }      
  }

};

#endif

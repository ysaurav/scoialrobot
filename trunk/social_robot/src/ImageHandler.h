#ifndef IMAGE_HANDLER
#define IMAGE_HANDLER

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW1[] = "RGB window";
static const char WINDOW2[] = "Depth window";

class ImageHandler
{
  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  image_transport::Subscriber rgb_sub;
	image_transport::Subscriber depth_sub;

  cv_bridge::CvImagePtr cv_rgb;
  cv_bridge::CvImagePtr cv_depth;
  
public:
  ImageHandler()
    : it(nh)
  {
    rgb_sub = it.subscribe("/camera/rgb/image_color", 1, &ImageHandler::rgb_cb, this);
		depth_sub = it.subscribe("/camera/depth/image_raw", 1, &ImageHandler::depth_cb, this);

    // cv::namedWindow(WINDOW1);
    // cv::namedWindow(WINDOW2);
  }

  ~ImageHandler()
  {
    cv::destroyWindow(WINDOW1);
    cv::destroyWindow(WINDOW2);
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

    // cv::imshow(WINDOW1, cv_rgb->image);
    // cv::waitKey(3);       
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

    // cv::imshow(WINDOW2, cv_depth->image);
    // cv::waitKey(3);       
  }

  cv_bridge::CvImagePtr get_cv_rgb()
  {
    return cv_rgb;
  }

  cv_bridge::CvImagePtr get_cv_depth()
  {
    return cv_depth;
  }

};

#endif

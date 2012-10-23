#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <stereo_msgs/DisparityImage.h>

#include "kinect_proxy.h"
#include "cv_utils.h"

// values that can be chanegd during the runtime
std::string head_template = "pictures/template.png";
double canny_thr1 = 5;
double canny_thr2 = 7;
double chamfer_thr = 10;
double arc_thr = 12;
int scales = 5;

std::vector<cv::Point> head_matched_points;
std::vector<cv::Point3f> head_features;
int roi_x_offset;
int roi_y_offset;
int roi_height;
int roi_width;

cv::Mat *pyramid = new cv::Mat[scales];
cv::Mat *chamfer = new cv::Mat[scales];
cv::Mat *matching = new cv::Mat[scales];
cv::Mat canny_im;

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
  canny_im.create(image.rows, image.cols, image.depth());
  cv::Mat template_im = cv::imread(head_template, CV_LOAD_IMAGE_ANYDEPTH);
    
  // calculate edge detection  
  cv::Canny(image, canny_im, canny_thr1, canny_thr2, 3, true);
  
  // calculate the Canny pyramid
  pyramid[0] = canny_im;
  for(int i = 1; i < scales; i++)
  {
    cv::resize(pyramid[i - 1], pyramid[i], cv::Size(), 0.75, 0.75, cv::INTER_NEAREST);
  }
  
  // calculate distance transform
  for(int i = 0; i < scales; i++)
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
  std::vector<cv::Point> head_matched_points_tmp;

  for (int i = 0; i < scales; i++)
  {
    matchTemplate(chamfer[i], template_im, matching[i], CV_TM_CCOEFF);
    normalize(matching[i], matching[i], 0.0, 1.0, cv::NORM_MINMAX);
    cv::minMaxLoc(matching[i], &minVal, &maxVal, &minLoc, &maxLoc);
    cv::Mat matching_thr;
    threshold(matching[i], matching_thr, 1.0 / chamfer_thr, 1.0, CV_THRESH_BINARY_INV);
    double scale = pow(1.0 / 0.75, i);
    get_non_zeros(matching_thr, &head_matched_points_tmp, pdiff, scale);
  }

  return head_matched_points_tmp;
}

std::vector<cv::Point3f> compute_headparameters(cv::Mat image, std::vector<cv::Point> chamfer)
{
  std::vector<cv::Point3f> parameters_head(chamfer.size());

  // parameters of cubic equation
  float p1 = -1.3835 * pow(10, -9);
  float p2 =  1.8435 * pow(10, -5);
  float p3 = -0.091403;
  float p4 =  189.38;

  for (unsigned int i = 0; i < chamfer.size(); i++)
  {
    int position_x = chamfer[i].x;
    int position_y = chamfer[i].y;

    unsigned short x = image.at<unsigned short>(position_y, position_x);
    
    // compute height of head
    float h = (p1 * pow(x, 3) + p2 * pow(x, 2) + p3 * x + p4);

    // compute Radius of head in milimeters
    float R = 1.33 * h / 2;

    // convert Radius in pixels
    float Rp = round((1 / 1.3) * R);

    parameters_head[i].x = position_x;
    parameters_head[i].y = position_y;
    parameters_head[i].z = Rp;
  }

  return parameters_head;
}

bool update_param_cb(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  ROS_INFO("Updating parameter of social_robot");
  
  std::string head_template_tmp;
  double canny_thr1_tmp;
  double canny_thr2_tmp;
  double chamfer_thr_tmp;
  int scales_tmp;
	
	ros::NodeHandle nh;
	
  nh.param("head_template", head_template_tmp, head_template_tmp);
  nh.param("canny_thr1", canny_thr1_tmp, canny_thr1_tmp);
  nh.param("canny_thr2", canny_thr2_tmp, canny_thr2_tmp);
  nh.param("chamfer_thr", chamfer_thr_tmp, chamfer_thr_tmp);
  nh.param("scales", scales_tmp, scales_tmp);
  
  head_template = head_template_tmp;
  canny_thr1 = canny_thr1_tmp;
  canny_thr2 = canny_thr2_tmp;
  chamfer_thr = chamfer_thr_tmp;
  scales = scales_tmp;  
  
  return true;
}

void rgb_cb(const sensor_msgs::ImageConstPtr& msg)
{    
  try
  {
    cv::Mat image_rgb = cv_bridge::toCvCopy(msg)->image;
    // FIXME: too slow
    for (unsigned int i = 0; i < head_features.size(); i++)
    {
      if (head_features[i].z < 0)
      {
        head_features[i].z = 0;
        continue;
      }
      int wh = head_features[i].z * 2;
      int tlx = head_features[i].x - head_features[i].z;
      int tly = head_features[i].y - head_features[i].z;
      cv::Rect roi(tlx, tly, wh, wh);
      if (!(0 <= roi.x && 0 <= roi.width && roi.x + roi.width <= canny_im.cols && 0 <= roi.y && 0 <= roi.height && roi.y + roi.height <= canny_im.rows))
      {
        continue;
      }
      cv::Mat roi_canny = canny_im(roi);
      std::vector<std::vector<cv::Point> > contours;
      cv::findContours(roi_canny, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
      for (unsigned int j = 0; j < contours.size(); j++)
      {
        std::vector<cv::Point> approx;
        cv::approxPolyDP(contours[j], approx, 5, false);          
        if (approx.size() > arc_thr)
        {
          rectangle(image_rgb, roi.tl(), roi.br(), cvScalar(255, 0, 255, 0), 2, 8, 0);
          break;
        }
      }
      // circle(image_rgb, cv::Point(head_features[i].x, head_features[i].y), head_features[i].z, cvScalar(255, 255, 0, 0), 2, 8, 0);  
    }
    rectangle(image_rgb, cv::Point(roi_x_offset, roi_y_offset), cv::Point(roi_x_offset + roi_width, roi_y_offset + roi_height), cvScalar(255, 0, 255, 0), 2, 8, 0);
    cv::imshow("Image", image_rgb);
    cv::waitKey(3);
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
    cv_bridge::CvImagePtr cv_depth = cv_bridge::toCvCopy(msg);
    cv::Mat depth_image = cv_depth->image;
    depth_image = preprocessing(depth_image);
    head_features = compute_headparameters(depth_image, head_matched_points);
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
    cv_bridge::CvImagePtr cv_disparity = cv_bridge::toCvCopy(msg->image);
    cv::Mat disparity_image = cv_disparity->image;
    disparity_image.convertTo(disparity_image, CV_8U);
    disparity_image = preprocessing(disparity_image);
    head_matched_points = chamfer_matching(disparity_image);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }      
}

void roi_cb(const sensor_msgs::RegionOfInterest& msg)
{
  try
  {
    roi_x_offset = msg.x_offset;
    roi_y_offset = msg.y_offset;
    roi_height = msg.height;
    roi_width = msg.width;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }      
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "social_robot");
  ros::NodeHandle nh;

  nh.setParam("head_template", head_template);
  nh.setParam("canny_thr1", canny_thr1);
  nh.setParam("canny_thr2", canny_thr2);
  nh.setParam("chamfer_thr", chamfer_thr);
  nh.setParam("scales", scales);
  
  ros::Subscriber disparity_sub = nh.subscribe("/camera/depth/disparity", 1, disparity_cb);
  ros::Subscriber rgb_sub = nh.subscribe("/camera/rgb/image_color", 1, rgb_cb);
  ros::Subscriber depth_sub = nh.subscribe("/camera/depth/image_raw", 1, depth_cb);
  ros::Subscriber roi_sub = nh.subscribe("/roi", 1, roi_cb);
  cv::namedWindow("Image", CV_WINDOW_AUTOSIZE);
  
  ros::spin();
  
  return 0;
}

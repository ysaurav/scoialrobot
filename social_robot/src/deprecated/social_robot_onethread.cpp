#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <stereo_msgs/DisparityImage.h>

#include "kinect_proxy.h"
#include "cv_utils.h"

using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;

// values that can be chanegd during the runtime
string head_template = "pictures/template.png";
double canny_thr1 = 5;
double canny_thr2 = 7;
double chamfer_thr = 20;
double arc_thr = 12;
double euc_thr = 5;
int scales = 5;

vector<Point> head_matched_points;
vector<Point3f> head_features;
vector<Rect> rois;
vector<Rect> humans;

int roi_x_offset;
int roi_y_offset;
int roi_height;
int roi_width;

Mat *pyramid = new Mat[scales];
Mat *chamfer = new Mat[scales];
Mat *matching = new Mat[scales];

Mat canny_im;
Mat image_rgb;
Mat depth_image;
Mat disparity_image;

Mat preprocessing(Mat image)
{
  Mat dst;
  Mat temp;
  resize(image, temp, Size(), 2, 2, INTER_NEAREST);
  resize(temp, dst, Size(), 0.5, 0.5, INTER_NEAREST);
  // inpainting
  Mat mask = Mat::zeros(image.rows, image.cols, CV_8U);
  Mat temp2(image.rows, image.cols, CV_8U);
  Mat temp3(image.rows, image.cols, CV_8U);
  for (int i = 0; i < image.rows; i++)
  {
    unsigned char *rowi = image.ptr<unsigned char>(i);
    for (int j = 0; j < image.cols; j++)
    {
      if (rowi[j] == 0)
      {
        mask.at<unsigned char>(i, j) = 255;
      }
    }
  }
  medianBlur(image, temp3, 5);
  inpaint(temp3, mask, dst, 5, INPAINT_NS);
  medianBlur(dst, dst, 5);
  return dst;
}

vector<Point> chamfer_matching(Mat image)
{
  canny_im.create(image.rows, image.cols, image.depth());
  Mat template_im = imread(head_template, CV_LOAD_IMAGE_ANYDEPTH);

  // calculate edge detection
  Canny(image, canny_im, canny_thr1, canny_thr2, 3, true);

  // calculate the Canny pyramid
  pyramid[0] = canny_im;
  for(int i = 1; i < scales; i++)
  {
    resize(pyramid[i - 1], pyramid[i], Size(), 0.75, 0.75, INTER_NEAREST);
  }

  // calculate distance transform
  for(int i = 0; i < scales; i++)
  {
    distanceTransform((255 - pyramid[i]), chamfer[i], CV_DIST_C, 3);
    normalize(chamfer[i], chamfer[i], 0.0, 1.0, NORM_MINMAX);
  }

  // matching with the template
  template_im = rgb2bw(template_im);
  template_im.convertTo(template_im,CV_32F);

  // find the best match:
  double minVal, maxVal;
  Point minLoc, maxLoc;
  double xdiff = template_im.cols / 2;
  double ydiff = template_im.rows / 2;
  Point pdiff = Point(xdiff, ydiff);
  vector<Point> head_matched_points_tmp;

  for (int i = 0; i < scales; i++)
  {
    matchTemplate(chamfer[i], template_im, matching[i], CV_TM_CCOEFF);
    normalize(matching[i], matching[i], 0.0, 1.0, NORM_MINMAX);
    minMaxLoc(matching[i], &minVal, &maxVal, &minLoc, &maxLoc);
    Mat matching_thr;
    threshold(matching[i], matching_thr, 1.0 / chamfer_thr, 1.0, CV_THRESH_BINARY_INV);
    double scale = pow(1.0 / 0.75, i);
    get_non_zeros(matching_thr, &head_matched_points_tmp, pdiff, scale);
  }

  return head_matched_points_tmp;
}

vector<Point3f> compute_headparameters(Mat image, vector<Point> chamfer)
{
  vector<Point3f> parameters_head(chamfer.size());

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

  string head_template_tmp;
  double canny_thr1_tmp;
  double canny_thr2_tmp;
  double chamfer_thr_tmp;
  double arc_thr_tmp;
  int scales_tmp;

  ros::NodeHandle nh;

  nh.param("/social_robot/head_template", head_template_tmp, head_template_tmp);
  nh.param("/social_robot/canny_thr1", canny_thr1_tmp, canny_thr1_tmp);
  nh.param("/social_robot/canny_thr2", canny_thr2_tmp, canny_thr2_tmp);
  nh.param("/social_robot/chamfer_thr", chamfer_thr_tmp, chamfer_thr_tmp);
  nh.param("/social_robot/scales", scales_tmp, scales_tmp);
  nh.param("/social_robot/arc_thr", arc_thr_tmp, arc_thr_tmp);

  head_template = head_template_tmp;
  canny_thr1 = canny_thr1_tmp;
  canny_thr2 = canny_thr2_tmp;
  chamfer_thr = chamfer_thr_tmp;
  scales = scales_tmp;
  arc_thr = arc_thr_tmp;

  return true;
}

void rgb_cb(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    image_rgb = cv_bridge::toCvCopy(msg, enc::BGR8)->image;
    for (unsigned int i = 0; i < rois.size(); i++)
    {
      rectangle(image_rgb, rois[i], cvScalar(255, 0, 255, 0), 2, 8, 0);
    }
    imshow("Image", image_rgb);
    waitKey(3);
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
    depth_image = cv_depth->image;
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
    disparity_image = cv_disparity->image;
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

void timer_cb(const ros::TimerEvent& event)
{
  rois.clear();
  Mat tmp_rgb = image_rgb.clone();
  Mat tmp_depth = depth_image.clone();
  Mat tmp_disparity = disparity_image.clone();
  if (tmp_rgb.empty() || tmp_depth.empty() || disparity_image.empty())
  {
    return;
  }
  tmp_disparity.convertTo(tmp_disparity, CV_8U);
//  tmp_depth.convertTo(tmp_depth, CV_8U);
  // preprocessing
  tmp_disparity = preprocessing(tmp_disparity);
//  tmp_depth = preprocessing(tmp_depth);

  head_matched_points = chamfer_matching(tmp_disparity);
  head_features = compute_headparameters(tmp_depth, head_matched_points);
  for (unsigned int i = 0; i < head_features.size(); i++)
  {
    unsigned int n = i + 1;
    if (n == head_features.size())
    {
      n = 0;
    }
    double euc = sqrt(pow(head_features[i].x - head_features[n].x, 2) + pow(head_features[i].y - head_features[n].y, 2));
    if (euc <= euc_thr)
    {
      continue;
    }

    if (head_features[i].z < 0)
    {
      head_features[i].z = 0;
      continue;
    }
    int wh = head_features[i].z * 2;
    int tlx = head_features[i].x - head_features[i].z;
    int tly = head_features[i].y - head_features[i].z;
    Rect roi(tlx, tly, wh, wh);
    if (!(0 <= roi.x && 0 <= roi.width && roi.x + roi.width <= canny_im.cols && 0 <= roi.y && 0 <= roi.height && roi.y + roi.height <= canny_im.rows))
    {
      continue;
    }
    Mat roi_canny = canny_im(roi);
    vector<vector<Point> > contours;
    findContours(roi_canny, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    for (unsigned int j = 0; j < contours.size(); j++)
    {
      vector<Point> approx;
      approxPolyDP(contours[j], approx, 5, false);
      if (approx.size() > arc_thr)
      {
        rois.push_back(roi);
        break;
      }
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "social_robot");
  ros::NodeHandle nh;

  nh.setParam("/social_robot/head_template", head_template);
  nh.setParam("/social_robot/canny_thr1", canny_thr1);
  nh.setParam("/social_robot/canny_thr2", canny_thr2);
  nh.setParam("/social_robot/chamfer_thr", chamfer_thr);
  nh.setParam("/social_robot/scales", scales);
  nh.setParam("/social_robot/arc_thr", arc_thr);

  ros::ServiceServer clear_srv = nh.advertiseService("/social_robot/update", update_param_cb);
  ros::Subscriber disparity_sub = nh.subscribe("/camera/depth/disparity", 1, disparity_cb);
  ros::Subscriber rgb_sub = nh.subscribe("/camera/rgb/image_color", 1, rgb_cb);
  ros::Subscriber depth_sub = nh.subscribe("/camera/depth/image_raw", 1, depth_cb);
  ros::Subscriber roi_sub = nh.subscribe("/roi", 1, roi_cb);
  namedWindow("Image", CV_WINDOW_AUTOSIZE);
  ros::Timer timer = nh.createTimer(ros::Duration(0.3), timer_cb);

  ros::spin();

  return 0;
}

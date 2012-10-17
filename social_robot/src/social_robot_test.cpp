#include <ros/ros.h>

#include "kinect_proxy.h"
#include "cv_utils.h"

#define HEAD_TEMPLATE "pictures/template.png"
#define THR1          5
#define THR2          7
#define THR3          20
#define SCALES        5

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
  cv::Mat canny_im(image.rows, image.cols, image.depth());
  cv::Mat template_im = cv::imread(HEAD_TEMPLATE, CV_LOAD_IMAGE_ANYDEPTH);

  cv::Mat* pyramid = new cv::Mat[SCALES];
  cv::Mat* chamfer = new cv::Mat[SCALES];
  cv::Mat* matching = new cv::Mat[SCALES];

  // calculate edge detection
  cv::Canny(image, canny_im, THR1, THR2, 3, true);

  // calculate the Canny pyramid
  pyramid[0] = canny_im;
  for(int i = 1; i < SCALES; i++)
  {
    cv::resize(pyramid[i - 1], pyramid[i], cv::Size(), 0.75, 0.75, cv::INTER_NEAREST);
  }

  // calculate distance transform
  for(int i = 0; i < SCALES; i++)
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
  std::vector<cv::Point> head_matched_points;

  for (int i = 0; i < SCALES; i++)
  {
    matchTemplate(chamfer[i], template_im, matching[i], CV_TM_CCOEFF);
    normalize(matching[i], matching[i], 0.0, 1.0, cv::NORM_MINMAX);
    cv::minMaxLoc(matching[i], &minVal, &maxVal, &minLoc, &maxLoc);
    cv::Mat matching_thr;
    threshold(matching[i], matching_thr, 1.0 / THR3, 1.0, CV_THRESH_BINARY_INV);
    double scale = pow(1.0 / 0.75, i);
    get_non_zeros(matching_thr, &head_matched_points, pdiff, scale);
  }

  return head_matched_points;
}

std::vector<cv::Point3f> headparameters(cv::Mat image, std::vector<cv::Point> chamfer)
{
  int position_x;
  int position_y;
  unsigned short d;
  unsigned short x;

  std::vector<cv::Point3f> parametersHead(chamfer.size());
  float h;
  float R;
  float Rp;
  //parameters of cubic equation

  float p1 = -1.3835*pow(10,-9);
  float p2 = 1.8435*pow(10,-5);
  float p3 = -0.091403;
  float p4 = 189.38;

  // depthImage.convertTo(temp, CV_32F);
  for (unsigned int i = 0; i < chamfer.size(); i++)
  {
    // std::cout<<"Here"<<std::endl;
    position_x = chamfer[i].x;
    position_y = chamfer[i].y;
    //  std::cout<< "x "<< position_x<<" y "<<position_y<<std::endl;

    d = image.at<unsigned short>(position_x,position_y);
    // x = (1/(alpha*(d - beta)))*1000;
    // std::cout<< "Depth of pixel"<< image.at<unsigned short>(position_y,position_x)<<std::endl;
    //  std::cout<< "Depth of pixel "<< d <<std::endl;

    x = image.at<unsigned short>(position_x,position_y);
    // compute height of head
    h = (p1*pow(x,3) + p2*pow(x,2) + p3*x + p4);

    // compute Radius of head in milimeters
    R = 1.33*h/2;

    // convert Radius in pixels
    Rp = round((1/1.3)*R);

    // std::cout << "Height of head is: " << h << " Radius in milimeters is: " << R << " Radius in pixels: "<< Rp <<std::endl;

    parametersHead[i].x = h;
    parametersHead[i].y = R;
    parametersHead[i].z = Rp;
  }

  return parametersHead;
}


int main(int argc, char **argv)
{
  //  ros::init(argc, argv, "social_robot");
  //  ros::spin();

  // temporarily reading the image from file later on change it to the live video.
  cv::Mat image_dispa = cv::imread(argv[1], CV_LOAD_IMAGE_ANYDEPTH);
  cv::Mat image_depth = cv::imread(argv[2], CV_LOAD_IMAGE_ANYDEPTH);
  cv::Mat image_rgb = cv::imread(argv[3], CV_LOAD_IMAGE_ANYDEPTH);
  // cvtColor(image, image, CV_RGB2GRAY);
  image_dispa = preprocessing(image_dispa);
  image_depth = preprocessing(image_depth);
  std::vector<cv::Point> head_matched_points = chamfer_matching(image_dispa);

  std::vector<cv::Point3f> parametersHead = headparameters(image_depth,head_matched_points);
  int th = 20;

  cv::Point center;
  cv::Scalar color( 255, 255, 255 );
  float Rp;
  for (unsigned int i = 0; i < parametersHead.size(); i++)
  {
    Rp = parametersHead[i].z;
    center.x = head_matched_points[i].x;
    center.y = head_matched_points[i].y;
    circle(image_rgb, center, Rp, color, 1, 2);
  }

  cv::namedWindow("Image", CV_WINDOW_AUTOSIZE);
  cv::imshow("Image", image_rgb);
  cv::waitKey(0);

  return 0;
}

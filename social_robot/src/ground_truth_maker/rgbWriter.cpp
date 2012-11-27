#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <stereo_msgs/DisparityImage.h>

#include "../kinect_proxy.h"
#include "../CvUtils.h"
#include "../RosUtils.h"
#include "../string_utils.h"

using namespace std;
using namespace cv;
using namespace sensor_msgs;

namespace enc = image_encodings;

VideoWriter rgbwriter;

CvUtils cv_utils;
bool isfirstframe = true;
string file_name = "default.avi";

void rgb_cb ( const ImageConstPtr& msg )
{/*
  try
    {
      Mat image_rgb = cv_bridge::toCvCopy ( msg, enc::BGR8 )->image;

      if ( isfirstframe )
        {
          int width = image_rgb.cols;
          int height = image_rgb.rows;
          rgbwriter.open ( file_name, CV_FOURCC ( 'D', 'I', 'V', 'X' ), 30, Size ( width, height ) );
          if ( !rgbwriter.isOpened() )
            {
              cerr << "Could not open '" << "'" << endl;
            }
          isfirstframe = false;
        }

      if ( rgbwriter.isOpened() )
        {
          rgbwriter << image_rgb;
        }

    }
  catch ( cv_bridge::Exception& e )
    {
      ROS_ERROR ( "cv_bridge exception: %s", e.what() );
      return;
    }*/
}

int frame = 0;
void disparity_cb ( const stereo_msgs::DisparityImageConstPtr& msg )
{
  try
    {
      Mat image_disparity = cv_bridge::toCvCopy ( msg->image )->image;
      image_disparity.convertTo ( image_disparity, CV_8UC1 );
      image_disparity = cv_utils.preprocessing(image_disparity);
      
      std::string project_path = ros::package::getPath("social_robot");

      string filename = project_path;
      filename.append("/logs/frame");
      if (frame < 10)
      {
        filename.append("000");
      } else if ( frame < 100 )
      {
        filename.append("00");
      }
       else if ( frame < 1000)
      {
        filename.append("0");
      }
      filename.append(inttostr(frame));
      filename.append(".png");
      imwrite(filename, image_disparity);
      frame++;
    }
  catch ( cv_bridge::Exception& e )
    {
      ROS_ERROR ( "cv_bridge exception: %s", e.what() );
      return;
    }
}


int main ( int argc, char **argv )
{
  ros::init ( argc, argv, "rgbwriter" );
  ros::NodeHandle nh;
  if ( argc > 1 )
    {
      file_name = argv[1];
    }
  // subscribtions
  ros::Subscriber rgb_sub = nh.subscribe ( "/camera/rgb/image_color", 1, rgb_cb );
  ros::Subscriber disparity_sub = nh.subscribe ( "/camera/depth/disparity", 1000, disparity_cb );


  ros::spin();

  return 0;
}

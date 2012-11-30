#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include "../kinect_proxy.h"
#include "../CvUtils.h"
#include "../RosUtils.h"
#include "../string_utils.h"

using namespace std;
using namespace cv;

namespace enc = image_encodings;

VideoWriter rgbwriter;

CvUtils cv_utils;
bool isfirstframe = true;
string video_file_name = "default.avi";

void rgb_cb ( const ImageConstPtr& msg )
{
  try
    {
      Mat image_rgb = cv_bridge::toCvCopy ( msg, enc::BGR8 )->image;

      if ( isfirstframe )
        {
          int width = image_rgb.cols;
          int height = image_rgb.rows;
          rgbwriter.open ( video_file_name, CV_FOURCC ( 'D', 'I', 'V', 'X' ), 30, Size ( width, height ) );
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
    }
}

int main ( int argc, char **argv )
{
  ros::init ( argc, argv, "rgbwriter" );
  ros::NodeHandle nh;
  if ( argc > 1 )
    {
      video_file_name = argv[1];
    }

  // subscribtions
  ros::Subscriber rgb_sub = nh.subscribe ( "/camera/rgb/image_color", 1, rgb_cb );

  ros::spin();

  return 0;
}

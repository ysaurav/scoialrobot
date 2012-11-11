#ifndef SOCIALROBOTGUI_H
#define SOCIALROBOTGUI_H

#include <QThread>
#include <ros/ros.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include "RosUtils.h"
#include <social_robot/RegionOfInterests.h>

using namespace std;
using namespace cv;

class SocialRobotGui : public QThread
  {
    Q_OBJECT
  public:
    SocialRobotGui ( int argc, char **argv );
    ~SocialRobotGui();
    void init();
    void run();
    void rgb_cb ( const sensor_msgs::ImageConstPtr &msg );
    void rgb_rois_cb ( const social_robot::RegionOfInterests &msg );
    void depth_rois_cb ( const social_robot::RegionOfInterests &msg );

  signals:
    void update_image ( const Mat & );

  private:
    int init_argc;
    char ** init_argv;
    ros::Subscriber rgb_subscriber;
    ros::Subscriber rgb_rois_subs;
    ros::Subscriber depth_rois_subs;
    vector<Rect> rgb_rois;
    vector<Rect> depth_rois;
    
    RosUtils ros_utils;
  };

#endif // SOCIALROBOTGUI_H
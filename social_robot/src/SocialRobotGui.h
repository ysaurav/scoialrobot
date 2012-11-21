#ifndef SOCIALROBOTGUI_H
#define SOCIALROBOTGUI_H

#include <QThread>
#include <ros/ros.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include "RosUtils.h"
#include <social_robot/RegionOfInterests.h>
#include <std_srvs/Empty.h>

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
    void depth_cb ( const sensor_msgs::ImageConstPtr &msg );
    void rgb_rois_cb ( const social_robot::RegionOfInterests &msg );
    void depth_rois_cb ( const social_robot::RegionOfInterests &msg );

    // thresholds
    void threshold_template_matching_3d ( double threshold );
    void threshold_scales ( int threshold );
    void threshold_chamfer ( double threshold );
    void threshold_arc_low ( int threshold );
    void threshold_arc_high ( int threshold );
    void threshold_confidence ( double threshold );

    // settings
    bool display_rgb_faces;
    bool display_depth_faces;
    bool display_rgb_image;
    bool display_depth_image;

  signals:
    void update_image ( const Mat & );

  private:
    int init_argc;
    char ** init_argv;
    ros::Subscriber rgb_subscriber;
    ros::Subscriber depth_subscriber;
    ros::Subscriber rgb_rois_subs;
    ros::Subscriber depth_rois_subs;
    ros::ServiceClient update_client;
    vector<Rect> rgb_rois;
    vector<Rect> depth_rois;
    std_srvs::Empty empty;

    RosUtils ros_utils;
  };

#endif // SOCIALROBOTGUI_H

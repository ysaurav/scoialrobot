#ifndef SOCIALROBOTGUI_H
#define SOCIALROBOTGUI_H

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <std_msgs/String.h>
#include <stereo_msgs/DisparityImage.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

using namespace std;
using namespace cv;

class SocialRobotGui : public QThread
{
  Q_OBJECT
public:
    SocialRobotGui(int argc, char** argv );
    ~SocialRobotGui();
    void init();
    void init(const std::string &master_url, const std::string &host_url, const std::string &topic_name);
    void run();

    void rgb_cb(const sensor_msgs::ImageConstPtr& msg);
  
signals:
    void update_image( const Mat & );

private:
    int init_argc;
    char ** init_argv;
    ros::Subscriber rgb_subscriber;
};

#endif // SOCIALROBOTGUI_H

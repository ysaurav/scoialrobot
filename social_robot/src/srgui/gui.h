#ifndef GUI_H
#define GUI_H

#include <QMainWindow>
#include <opencv2/video/tracking.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "window_QT.h"
#include "ui_gui.h"
#include "../SocialRobotGui.h"

using namespace cv;
using namespace std;

class Gui : public QMainWindow
  {
    Q_OBJECT

  public:
    explicit Gui ( int argc, char **argv, QWidget *parent = 0 );
    ~Gui();

  private slots:
    void updateImage ( const Mat & );
    void updateImageSize ( const QSize &size );

    void readSettings ( void );
    void writeSettings ( void );

    // settings slots

    // display
    void rgb_check_box_handler ( void );
    void depth_check_box_handler ( void );
    void handle_display ( void );

    // thresholds
    void threshold_template_matching_3d ( void );
    void threshold_scales ( void );
    void threshold_chamfer ( void );
    void threshold_arc_low ( void );
    void threshold_arc_high ( void );
    void threshold_confidence ( void );

    void closeEvent ( QCloseEvent *event ); // Overloaded function

  private:
    Ui::Gui *ui;
    CvWindow *cvWindow;
    SocialRobotGui *social_robot_gui;

    // Setting
    QSettings settings;

  };

#endif // GUI_H

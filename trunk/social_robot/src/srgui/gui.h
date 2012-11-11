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
  explicit Gui(int argc, char** argv, QWidget *parent = 0);
  ~Gui();

  // Options
  QString Color;
  bool Normalization;
  QString Function;
  float low_ratio, high_ratio, aspect_are, Threshold;


private slots:
  void updateImage( const Mat & );
  void updateImageSize( const QSize &size);

  void readSettings( void );
  void writeSettings( void );
  
  void closeEvent(QCloseEvent *event); // Overloaded function

private:
  Ui::Gui *ui;
  CvWindow *cvWindow;
  SocialRobotGui *social_robot_gui;

  // Setting
  QSettings settings;

};

#endif // GUI_H

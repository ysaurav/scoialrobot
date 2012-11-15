#include "gui.h"
#include <QStringList>
#include <QFileDialog>
#include <QDebug>
#include <QMessageBox>
#include <string>

#include "imageframe.h"

Gui::Gui ( int argc, char** argv, QWidget *parent ) :
    QMainWindow ( parent ),
    ui ( new Ui::Gui )
{
  ui->setupUi ( this );
  cvWindow = new CvWindow ( QString ( "cvWindow" ), CV_WINDOW_AUTOSIZE );

  // set the location of 'cvWindow'
  cvWindow->setParent ( ui->frame );

  readSettings();

  social_robot_gui = new SocialRobotGui ( argc, argv );
  qRegisterMetaType< Mat > ( "Mat" );
  connect ( social_robot_gui, SIGNAL ( update_image ( Mat ) ), this, SLOT ( updateImage ( Mat ) ) );
  social_robot_gui->init();
  // If the frame resizes, update the image size
  connect ( ui->frame, SIGNAL ( resizeImage ( QSize ) ), this, SLOT ( updateImageSize ( QSize ) ) );
  updateImageSize ( ui->frame->size() );
}

Gui::~Gui()
{
  writeSettings();
  delete ui;
}

void Gui::readSettings ( void )
{

}

void Gui::writeSettings ( void )
{
}

void Gui::rgb_check_box_handler ( void )
{
  if ( ui->rgb_check_box->checkState() == Qt::Checked)
  {
    social_robot_gui->display_rgb_faces = true;
  }
  else
  {
    social_robot_gui->display_rgb_faces = false;
  }
}

void Gui::depth_check_box_handler ( void )
{
  if ( ui->depth_check_box->checkState() == Qt::Checked)
  {
    social_robot_gui->display_depth_faces = true;
  }
  else
  {
    social_robot_gui->display_depth_faces = false;
  }
}

void Gui::handle_display ( void )
{
  if (ui->rgbRadioButton->isChecked())
  {
    social_robot_gui->display_rgb_image = true;
    social_robot_gui->display_depth_image = false;
  }
  else
  {
    social_robot_gui->display_rgb_image = false;
    social_robot_gui->display_depth_image = true;
  }
}

void Gui::threshold_template_matching_3d ( void )
{
  social_robot_gui->threshold_template_matching_3d( (double) ui->templateMatchingThreshold->value() );
}

void  Gui::threshold_scales ( void )
{
  social_robot_gui->threshold_scales( (int) ui->scaleSpinBox->value() );
}

void  Gui::threshold_chamfer ( void )
{
  social_robot_gui->threshold_chamfer( (double) ui->chamferDoubleSpinBox->value() );
}

void  Gui::threshold_arc_low ( void )
{
  social_robot_gui->threshold_arc_low( (int) ui->arcLowSpinBox->value() );
}

void  Gui::threshold_arc_high ( void )
{
  social_robot_gui->threshold_arc_high( (int) ui->arcHighSpinBox->value() );
}

void Gui::updateImage ( const Mat &img )
{
  CvMat c_img = img;
  cvWindow->updateImage ( &c_img );
  updateImageSize ( ui->frame->size() );
}

void Gui::updateImageSize ( const QSize &size )
{
  //  We need a little bit space for color and coordinate
  //  information below the image
//  int width = size.width();
//  int height = size.height() - 50;
//  cvWindow->getView()->setSceneRect ( QRect ( 0, 0, width, height ) );
//  cvWindow->getView()->setFixedSize ( QSize ( width, height ) );
//  cvWindow->resize ( QSize ( width, height ) );
}

void Gui::closeEvent ( QCloseEvent *event )
{
  writeSettings();
  event->accept();
}

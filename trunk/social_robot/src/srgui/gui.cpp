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
  int width = size.width();
  int height = size.height() - 50;
  cvWindow->getView()->setSceneRect ( QRect ( 0, 0, width, height ) );
  cvWindow->getView()->setFixedSize ( QSize ( width, height ) );
  cvWindow->resize ( QSize ( width, height ) );
}

void Gui::closeEvent ( QCloseEvent *event )
{
  writeSettings();
  event->accept();
}
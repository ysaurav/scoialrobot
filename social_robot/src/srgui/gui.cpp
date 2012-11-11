#include "gui.h"
#include <QStringList>
#include <QFileDialog>
#include <QDebug>
#include <QMessageBox>
#include <string>

#include "imageframe.h"

#define SIZE 100

Gui::Gui(int argc, char** argv, QWidget *parent) :
        QMainWindow(parent),
        ui(new Ui::Gui)
{
    ui->setupUi(this);
    cvWindow = new CvWindow(QString("cvWindow"), CV_WINDOW_AUTOSIZE);    

    // Set Images (icons) size in the tableView
    QSize size(SIZE, SIZE);

    // set the location of 'cvWindow'
    cvWindow->setParent(ui->frame);

    readSettings();
    
    social_robot_gui = new SocialRobotGui(argc, argv);
    qRegisterMetaType< Mat >("Mat");
    connect(social_robot_gui, SIGNAL(update_image(Mat)), this, SLOT(updateImage(Mat)));    
    social_robot_gui->init();
    // If the frame resizes, update the image size
    connect(ui->frame, SIGNAL(resizeImage(QSize)), this, SLOT(updateImageSize(QSize)));
    updateImageSize(ui->frame->size());
    Mat im = imread("/home/safir/ros_workspace/scoialrobot/social_robot/pictures/rgb01.png");
    updateImage(im);
}

Gui::~Gui()
{
    writeSettings();
    delete ui;
}

//void Gui::default_settings( void )
//{

//}

void Gui::readSettings( void )
{
    // Main windows settings
    settings.beginGroup("MainWindow");

    resize(settings.value("size", QSize(400, 400)).toSize());
    move(settings.value("pos", QPoint(200, 200)).toPoint());
    restoreState(settings.value("state", QByteArray()).toByteArray());

    settings.endGroup();


    // Options
    settings.beginGroup("Options");
    Color = settings.value("Color").toString();
    Normalization = settings.value("Normalization").toBool();

    Function   = settings.value("Function").toString();
    low_ratio  = settings.value("low_ratio").toFloat();
    high_ratio = settings.value("high_ratio").toFloat();
    aspect_are = settings.value("aspect_are").toFloat();
    Threshold  = settings.value("Threshold").toFloat();
    settings.endGroup();
}

void Gui::writeSettings( void )
{
    // Main windows settings
    settings.beginGroup("MainWindow");

    // Save postion/size of main window
    settings.setValue("size", size());
    settings.setValue("pos", pos());
    settings.setValue("state", saveState());

    settings.endGroup();


    // Options
    settings.beginGroup("Options");
    settings.setValue("Color", Color);
    settings.setValue("Normalization", Normalization);
    settings.setValue("Function", Function);
    settings.setValue("low_ratio", low_ratio);
    settings.setValue("high_ratio", high_ratio);
    settings.setValue("aspect_are", aspect_are);
    settings.setValue("Threshold", Threshold);
    settings.endGroup();
}

void Gui::updateImage( const Mat &img )
{
    CvMat c_img = img;
    cvWindow->updateImage( &c_img );
    updateImageSize(ui->frame->size());
}

void Gui::updateImageSize(const QSize &size) {
    //  We need a little bit space for color and coordinate
    //  information below the image
    int width = size.width();
    int height = size.height()-50;
    cvWindow->getView()->setSceneRect(QRect(0,0,width,height));
    cvWindow->getView()->setFixedSize(QSize(width, height));
    cvWindow->resize(QSize(width, height));
}

void Gui::closeEvent(QCloseEvent *event)
{
    writeSettings();
    event->accept();
}
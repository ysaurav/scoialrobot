#include <QtGui/QApplication>
#include "gui.h"

int main(int argc, char *argv[])
{
  QApplication app(argc, argv);

  QCoreApplication::setOrganizationName("SocialRobot");
  QCoreApplication::setApplicationName("Social Robot");

  Gui w(argc, argv);
  w.show();
  app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));

  return app.exec();
}

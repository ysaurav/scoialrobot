#include "imageframe.h"

#include <QResizeEvent>

ImageFrame::ImageFrame(QWidget *parent) :
    QFrame(parent) {

}

void ImageFrame::resizeEvent(QResizeEvent *event) {
  QFrame::resizeEvent(event);
  //  Inform the gui window that the frame's size has changed
  emit resizeImage(event->size());
}

#ifndef IMAGEFRAME_H
#define IMAGEFRAME_H

#include <QFrame>
#include <QSize>

class ImageFrame : public QFrame
{
    Q_OBJECT
public:
    explicit ImageFrame(QWidget *parent = 0);

signals:
    void resizeImage(QSize newSize);

public slots:
    void resizeEvent(QResizeEvent *);
};

#endif // IMAGEFRAME_H

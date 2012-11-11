#-------------------------------------------------
#
# Project created by QtCreator 2012-11-10T20:28:37
#
#-------------------------------------------------

TARGET = srgui
TEMPLATE = app

unix {
    # using pkg-config
    CONFIG += link_pkgconfig
    PKGCONFIG += opencv-2.3.1
}

RESOURCES += window_QT.qrc

SOURCES += main.cpp gui.cpp imageframe.cpp window_QT.cpp

HEADERS  += gui.h imageframe.h window_QT.h

FORMS    += gui.ui

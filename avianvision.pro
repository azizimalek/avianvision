#-------------------------------------------------
#
# Project created by QtCreator 2018-03-01T15:04:18
#
#-------------------------------------------------

QT       += core gui opengl widgets
QMAKE_CXXFLAGS += -std=c++11

#greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = avianvision
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES += \
        main.cpp \
        mainwindow.cpp \
    capture_camera.cpp \
    uidisplaycontroller.cpp \
    uibuttoncontroller.cpp \
    stereoprocessing.cpp \
    opengldisplay.cpp \
    logmessagehandler.cpp \
    chessboarddetection.cpp \
    stereocalibration.cpp \
    v4l2capture.cpp \
    opticalflow.cpp \
    panaromicprocessing.cpp \
    natnethandler.cpp

HEADERS += \
        mainwindow.h \
    capture_camera.h \
    uidisplaycontroller.h \
    uibuttoncontroller.h \
    stereoprocessing.h \
    opengldisplay.h \
    logmessagehandler.h \
    chessboarddetection.h \
    masterheader.h \
    stereocalibration.h \
    v4l2capture.h \
    opticalflow.h \
    panaromicprocessing.h \
    natnethandler.h

FORMS += \
        mainwindow.ui

INCLUDEPATH += /usr/local/lib \
               /home/azinyer/Documents/NatNetLinux

LIBS +=`pkg-config opencv --cflags --libs`\
        -lboost_system\
        -lboost_thread

RESOURCES += \
    texture.qrc

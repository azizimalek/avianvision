#ifndef UIDISPLAYCONTROLLER_H
#define UIDISPLAYCONTROLLER_H

#include <QPixmap>
#include <QObject>
#include <opencv/highgui.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv/cv.h"
#include "opencv2/videoio.hpp"
#include <thread>
#include <chrono>
#include "masterheader.h"

enum DISPLAYSELECT{
    LEFTCAM = 0,
    RIGHTCAM,
    STEREOMERGE,
    STEREOSTITCH,
    STEREOBM,
    STEREOREMAP,
    STEREOBMMAP,
    CALIBRATEDSTEREOMAP
};

class uidisplaycontroller : public QObject
{
    Q_OBJECT

public:
    uidisplaycontroller();
    ~uidisplaycontroller();
    void getFrame(cv::Mat frame){ frame.copyTo(gframe);}
    void startDisplay();
    void stream();
    void setRefreshTime(int refreshtime){grefreshtime = refreshtime;}
    QPixmap mat2pix(cv::Mat);
    cv::Mat gframe;
    int grefreshtime = 10;
signals:
    void updateDisplay(QPixmap newPix);
    void updateDisplayGL();
    void updateFrame();

private:
    logmessagehandler* mlogmessage = new  logmessagehandler();


};

#endif // UIDISPLAYCONTROLLER_H

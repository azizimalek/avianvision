#ifndef CAPTURE_CAMERA_H
#define CAPTURE_CAMERA_H

#include <opencv/highgui.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv/cv.h"
#include "opencv2/videoio.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <thread>

#include <libv4l2.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <iostream>

#include "v4l2capture.h"
#include "masterheader.h"

using namespace cv;
using namespace std;

/*!
 * \brief Frame size struture for capture image Mat variable
 */
struct frameSize{
    int width;
    int height;
};

/*!
 * \brief The capture_camera class is for handler class for opencv camera capture
 */
class capture_camera
{
public:
    capture_camera();
    void init(logmessagehandler);
    void start_camera(int);
    void start_camera_v4l2(int cam);
    void stop_camera();
    void capture();
    void capturev4l2();
    bool successful();
    void update_frame(Mat);
    Mat getRGBframe();
    Mat getframe();
    void flipFrame(Mat,Mat);
    void enableFlip();
    void setSizeProperties(int, int);
    void disableSizeProperties(){enableSizing = false;}
    void setFPSProperties(int);
    void disableFPSProperties(){enableFPSControl = false;}
    void disableCameraAutoExposure();
    void storeFrameBuffer(){if(!gframe.empty())gframeBuffer.push_back(gframe);}
    std::vector<cv::Mat> getFrameBuffer(){if(!gframeBuffer.empty())return gframeBuffer;}
    void clearFrameBuffer(){gframeBuffer.clear();}
    int getFrameBufferSize(){return gframeBuffer.size();}
    void enableFPSProperties(){enableFPSControl = true;}

    Mat gframe;
    std::vector<cv::Mat> gframeBuffer;
    frameSize gframeSize;
    int gFPS;

private:
    VideoCapture cap;
    v4l2capture capv4l2;
    logmessagehandler* mlogmessage = new logmessagehandler();
    int camport = 0;
    bool enableFlipping = false;
    bool enableSizing = false;
    bool enableFPSControl = false;
};

#endif // CAPTURE_CAMERA_H

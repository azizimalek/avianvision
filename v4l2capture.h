#ifndef V4L2CAPTURE_H
#define V4L2CAPTURE_H

#include "masterheader.h"


static int isEmpty = -1;

class v4l2capture
{


public:
    v4l2capture();
    cv::Mat capture2Mat(int camport);
    void setCameraPowerLineControl(int );
    static int xioctl(int , int , void *);

    int bufferindex;
    uint8_t *bufferv4l2;

};

#endif // V4L2CAPTURE_H

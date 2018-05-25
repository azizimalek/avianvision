#include "v4l2capture.h"

v4l2capture::v4l2capture()
{

}

void v4l2capture::setCameraPowerLineControl(int camport){
    int fd;
    struct v4l2_control ctrl;

    std::string camPath =  "/dev/video" + std::to_string(camport);
    const char *ccamPath = camPath.c_str();
    fd = open(ccamPath, O_RDWR);
    if (fd == -1) return;

    ctrl.id = V4L2_CID_POWER_LINE_FREQUENCY;
    ctrl.value = V4L2_CID_POWER_LINE_FREQUENCY_50HZ;
    if (isEmpty == xioctl(fd, VIDIOC_S_CTRL, &ctrl))
    {
        perror("Setting Power line Control");
        return;
    }

    close(fd);
}

cv::Mat v4l2capture::capture2Mat(int camport){
    int fd;
    cv::Mat emptyMat;
    std::string camPath =  "/dev/video" + std::to_string(camport);
    const char *ccamPath = camPath.c_str();
    fd = open(ccamPath, O_RDWR);
    if (fd == -1)
    {
        // couldn't find capture device
        perror("Opening Video device");
        return emptyMat;
    }

    struct v4l2_capability caps = {camport};
    if (isEmpty ==  xioctl(fd, VIDIOC_QUERYCAP, &caps))
    {
        perror("Querying Capabilites");
        return emptyMat;
    }

    struct v4l2_format fmt = {camport};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    //fmt.fmt.pix.width = 320;
    //fmt.fmt.pix.height = 240;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;


    if (isEmpty == xioctl(fd, VIDIOC_S_FMT, &fmt))
    {
        perror("Setting Pixel Format");
        return emptyMat;
    }

    struct v4l2_control ctrl;
    ctrl.id = V4L2_CID_POWER_LINE_FREQUENCY;
    ctrl.value = V4L2_CID_POWER_LINE_FREQUENCY_50HZ;
    if (isEmpty == xioctl(fd, VIDIOC_S_CTRL, &ctrl))
    {
        perror("Setting Power line Control");
        return emptyMat;
    }

    struct v4l2_requestbuffers req = {camport};
    req.count = 2;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (isEmpty == xioctl(fd, VIDIOC_REQBUFS, &req))
    {
        perror("Requesting Buffer");
        return emptyMat;
    }

    struct v4l2_buffer buf = {camport};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = bufferindex;
    if(isEmpty == xioctl(fd, VIDIOC_QUERYBUF, &buf))
    {
        perror("Querying Buffer");
        return emptyMat;
    }

    bufferv4l2 = static_cast<uint8_t*>(mmap (NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset)) ;

    if(isEmpty == xioctl(fd, VIDIOC_STREAMON, &buf.type))
    {
        perror("Start Capture");
        return emptyMat;
    }

    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(fd, &fds);
    struct timeval tv = {camport};
    tv.tv_sec = 2;
    int r = select(fd+1, &fds, NULL, NULL, &tv);
    if(-1 == r)
    {
        perror("Waiting for Frame");
        return emptyMat;
    }

    if(isEmpty == xioctl(fd, VIDIOC_DQBUF, &buf))
    {
        perror("Retrieving Frame");
        return emptyMat;
    }

    close(fd);
    cv::Mat v4l2Mat = cv::Mat(720, 1280, CV_8UC4, (void*)bufferv4l2);

    if(!v4l2Mat.empty()) return v4l2Mat;

}

int v4l2capture::xioctl(int fd, int request, void *arg)
{
    int r;
        do r = ioctl (fd, request, arg);
        while (isEmpty == r && EINTR == errno);
        return r;
}

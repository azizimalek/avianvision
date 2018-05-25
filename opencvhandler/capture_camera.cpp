#include "capture_camera.h"

capture_camera::capture_camera()
{

}

void capture_camera::capture(){
    Mat frame;

    cap.open(camport);

    if(!cap.isOpened()) return;

    if(enableSizing){
        cap.set(CV_CAP_PROP_FRAME_WIDTH,gframeSize.width);
        cap.set(CV_CAP_PROP_FRAME_HEIGHT,gframeSize.height);
    }

    if(enableFPSControl){
        cap.set(CV_CAP_PROP_FPS,gFPS);
    }


    while(cap.isOpened())
    {

    //double fps = cap.get(CV_CAP_PROP_FPS);

        cap.read(frame);

        // check if we succeeded
        if (frame.empty()) {
            break;
        }

        //image flipping operation: clone image->flip image to clone image (otherwise wont work)
        Mat procFrame;
        frame.copyTo(procFrame);
        if(enableFlipping)flipFrame(frame,procFrame);

        update_frame(procFrame);

        //std::this_thread::sleep_for (std::chrono::milliseconds(10));
    }
}

void capture_camera::disableCameraAutoExposure(){
    //disable auto exposure || will be implement later
//    // open capture
//    int descriptor = v4l2_open("/dev/video0", O_RDWR);

//    // manual exposure control
//    v4l2_control c;
//    c.id = V4L2_CID_EXPOSURE_AUTO;
//    c.value = V4L2_EXPOSURE_MANUAL;
//    if(v4l2_ioctl(descriptor, VIDIOC_S_CTRL, &c) == 0)
//        cout << "success";
}
void capture_camera::setSizeProperties(int width, int height){
    gframeSize.width = width;
    gframeSize.height = height;
    enableSizing = true;
}

void capture_camera::setFPSProperties(int FPS){
    gFPS = FPS;
    enableFPSControl = true;
}


void capture_camera::start_camera(int cam){
    camport = cam;
    std::thread capture_thread (&capture_camera::capture, this);
    capture_thread.detach();

}

void capture_camera::stop_camera(){
    //terminate opencv videocapture
    cap.release();
}

bool capture_camera::successful(){
    bool success;
    if(cap.isOpened()) success = true;
    else success = false;

    return success;
}

void capture_camera::update_frame(Mat frame){
    gframe = frame;
}

Mat capture_camera::getRGBframe(){
    Mat rgbframe;
    if(!gframe.empty())
    cvtColor(gframe, rgbframe, COLOR_BGR2RGB);
    return rgbframe;
}

Mat capture_camera::getframe(){
    Mat frame;
    if(!gframe.empty())
    frame = gframe;
    return frame;
}

void capture_camera::flipFrame(Mat matIn, Mat matOut){
    //if(!matIn.empty())
    cv::flip(matIn,matOut,-1);
}

void capture_camera::enableFlip(){
    enableFlipping = true;
}


#include "uidisplaycontroller.h"

uidisplaycontroller::uidisplaycontroller()
{

}

uidisplaycontroller::~uidisplaycontroller(){
    std::terminate();
}

void uidisplaycontroller::startDisplay(){
    std::thread display_thread (&uidisplaycontroller::stream, this);
    display_thread.detach();
}

void uidisplaycontroller::stream(){

    while(1){

        if(!gframe.empty()){

            emit updateDisplayGL();
            gframe.release();
        }

        else emit updateFrame();
        std::this_thread::sleep_for (std::chrono::milliseconds(grefreshtime));
    }

}


QPixmap uidisplaycontroller::mat2pix(cv::Mat matIn){
   QPixmap pixframe = QPixmap::fromImage(QImage((unsigned char*) matIn.data, matIn.cols, matIn.rows, QImage::Format_RGB888));
   return pixframe;
}



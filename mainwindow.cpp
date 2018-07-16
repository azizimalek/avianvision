#include "mainwindow.h"
#include "ui_mainwindow.h"


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    connect(leftcambutton, SIGNAL(updateButtonText(QString)), this, SLOT(update_leftcamera_bt_text(QString)));
    connect(rightcambutton, SIGNAL(updateButtonText(QString)), this, SLOT(update_rightcamera_bt_text(QString)));
    connect(runbutton, SIGNAL(updateButtonText(QString)), this, SLOT(update_run_bt_text(QString)));
    display->startDisplay();
    logmessage->startLogging();
    //connect(display, SIGNAL(updateDisplay(QPixmap)), this, SLOT(update_display(QPixmap)));
    connect(display, SIGNAL(updateDisplayGL()), this, SLOT(update_displayGL()));
    connect(display, SIGNAL(updateFrame()), this, SLOT(update_frame()));
    connect(logmessage, SIGNAL(messageReceived(QString)),this, SLOT(logmessageReceived(QString)));
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::logmessageReceived(QString logmsg){
    ui->status_te->append(logmsg);
}

void MainWindow::on_leftcamera_bt_clicked()
{
    leftcambutton->setToggleButton("Start Camera", "Stop Camera");

    if(leftcambutton->toggleButtonState()){
        ui->status_te->append("starting left camera");
        v4l2.setCameraPowerLineControl(ui->leftcam_cb->currentIndex());
        leftcam.start_camera(ui->leftcam_cb->currentIndex());
        //leftcam.start_camera_v4l2(ui->leftcam_cb->currentIndex());
        opticalproc.startOpticalFlowPyrLKPano();
    }

    else {
        ui->status_te->append("stopping left camera");
        //TEMP: quick test for optical flow
        opticalproc.stopOpticalFlowPyrLK();
        leftcam.stop_camera();
    }
    //if(leftcam.successful())

    leftcam.enableFlip();
    leftcambutton->togglebutton();



}

void MainWindow::on_rightcamera_bt_clicked()
{
    rightcambutton->setToggleButton("Start Camera", "Stop Camera");

    if(rightcambutton->toggleButtonState()){
        ui->status_te->append("starting right camera");
        v4l2.setCameraPowerLineControl(ui->rightcam_cb->currentIndex());
        rightcam.start_camera(ui->rightcam_cb->currentIndex());
        //rightcam.start_camera_v4l2(ui->rightcam_cb->currentIndex());
    }

    else {
        ui->status_te->append("stopping right camera");
        rightcam.stop_camera();
    }

    //if(rightcam.successful())
        rightcambutton->togglebutton();

}

void MainWindow::update_leftcamera_bt_text(QString newBtnText){
    ui->leftcamera_bt->setText(newBtnText);
    ui->leftcam_cb->setDisabled(!leftcambutton->buttonstate);
}

void MainWindow::update_rightcamera_bt_text(QString newBtnText){
    ui->rightcamera_bt->setText(newBtnText);
    ui->rightcam_cb->setDisabled(!rightcambutton->buttonstate);
}
void MainWindow::update_run_bt_text(QString newBtnText){
    ui->run_bt->setText(newBtnText);
}

void MainWindow::update_frame(){
    display->getFrame(leftcam.getRGBframe());
}
void MainWindow::update_display(QPixmap pixframe){
    display->getFrame(leftcam.getRGBframe());

    //ui->display_lbl->setPixmap(pixframe);
    ui->status_lbl->setText("Updating frame...");
}

void MainWindow::update_displayGL(){

    cv::Mat glframe = displayGLSelected();

    if(!glframe.empty())
    ui->openGLWidget->setDisplayTexture(glframe);
    else
        ui->status_te->append("display is not ready");

    display->gframe.release();

}

void MainWindow::on_leftcam_rb_clicked()
{
    displaySelector = 0;
}

void MainWindow::on_rightcam_rb_clicked()
{
    displaySelector = 1;
}

void MainWindow::on_stereocam_rb_clicked()
{
    displaySelector = 2;
}

void MainWindow::on_stereodepth_rb_clicked()
{
    displaySelector = 3;
}

Mat MainWindow::displayGLSelected(){
    cv::Mat glframe;
    cv:Mat panaFrame;
    chessboard[0].updateChessboardInput(leftcam.getRGBframe());
    chessboard[1].updateChessboardInput(rightcam.getRGBframe());
    switch(displaySelector){
    case DISPLAYSELECT::LEFTCAM:
        display->setRefreshTime(10);
        //glframe = leftcam.getRGBframe();
        panaFrame = panaproc.unwarpPanoramic(leftcam.getRGBframe());
        //glframe = opticalproc.drawingOpticalFlowNative(panaFrame);
        opticalproc.bindFrame(panaFrame);
        glframe = opticalproc.getDisplayFrame();
        //glframe = panaproc.drawReferenceLine(leftcam.getRGBframe());
        break;
    case DISPLAYSELECT::RIGHTCAM:
        display->setRefreshTime(10);
        glframe = rightcam.getRGBframe();
        break;
    case DISPLAYSELECT::STEREOMERGE:
        display->setRefreshTime(10);
        glframe = stereoproc.crudeMatMerge(leftcam.getRGBframe(),rightcam.getRGBframe());
        break;
    case 3:
        //TODO: will be implemented for stereo depth
        break;
    case 4:
        display->setRefreshTime(50);
        glframe = stereoproc.stitchingMatMerge(leftcam.getRGBframe(),rightcam.getRGBframe());
        break;
    case 5:
    {
        cv::Mat leftMat, rightMat;
        chessboard[0].updateChessboardFrame(leftMat);
        chessboard[1].updateChessboardFrame(rightMat);
        glframe = stereoproc.crudeMatMerge(leftMat,rightMat);
    }
        break;
    case STEREOBMMAP:
        glframe = stereoproc.stereoBM(leftcam.getframe(),rightcam.getframe());
        if(glframe.empty())ui->status_te->append("where's my frame...");
        break;
    case CALIBRATEDSTEREOMAP: //calibrated stereo map
        //TODO: remapped stereo images
        //glframe = stereoproc.stereoRemapDisplay(leftcam.getRGBframe(),rightcam.getRGBframe());
       // glframe = stereoproc.stereoBMCalibrated(leftcam.getRGBframe(),rightcam.getRGBframe());
        if(leftcam.getRGBframe().empty() || rightcam.getRGBframe().empty()) break;
        stereoproc.runStereoDisparity(leftcam.getRGBframe(),rightcam.getRGBframe());
        if(!stereoproc.noOutput()) glframe = stereoproc.getFilteredDisparityImage();
        //glframe = stereoproc.stereoDisparityFilteredCalibrated(leftcam.getRGBframe(),rightcam.getRGBframe());
            break;
    default:
        break;

    };
    if(glframe.empty()) ui->status_te->append("no frame found");

    //temperatory looper (must delete when proper has been done

    std::string recordedData = "";
    if(!opticalproc.getPixelVelocityData().empty() && writingfile && opticalproc.getPixelVelocityData().size() == 4){
        recordedData = std::to_string((std::clock() - start_time)/(double)CLOCKS_PER_SEC);
        recordedData += "\t";
        for (int i=0; i < opticalproc.getPixelVelocityData().size(); i++){
            recordedData += std::to_string(opticalproc.getPixelVelocityData()[i].x);
            recordedData += "\t";
            recordedData += std::to_string(opticalproc.getPixelVelocityData()[i].y);
            recordedData += "\t";
        }

        Point3fn rbloc;
        Quaternion4f rbquat;
        if(natnet.running()){
            logmessage->status("Recording optitrack data");
            rbloc = natnet.getRigidBodyLocation();
            rbquat = natnet.getRigidBodyOrientation();

            recordedData += std::to_string(natnet.getRigidBodyLocation().x);
            recordedData += "\t";
            recordedData += std::to_string(natnet.getRigidBodyLocation().y);
            recordedData += "\t";
            recordedData += std::to_string(natnet.getRigidBodyLocation().z);
            recordedData += "\t";

            recordedData += std::to_string(natnet.getRigidBodyOrientation().qw);
            recordedData += "\t";
            recordedData += std::to_string(natnet.getRigidBodyOrientation().qx);
            recordedData += "\t";
            recordedData += std::to_string(natnet.getRigidBodyOrientation().qy);
            recordedData += "\t";
            recordedData += std::to_string(natnet.getRigidBodyOrientation().qz);
            recordedData += "\t";
        }

        outfile << recordedData << std::endl;
    }

    return glframe;
}

void MainWindow::on_stereocalib_bt_clicked()
{
    //run stereo calibration here
    logmessage->status("running stereo calibration");
    stereocam.setBoardSize(9,6);
    stereocam.setSquareSize(22.00);
    stereocam.startStereoCalibration(leftcam.getFrameBuffer(),rightcam.getFrameBuffer());

}

void MainWindow::on_StoreCam_bt_clicked()
{
    leftcam.storeFrameBuffer();
    rightcam.storeFrameBuffer();
    std::string status = "Images Captured. Current stored images from left camera: " + std::to_string( leftcam.getFrameBufferSize());
    ui->status_te->append(QString::fromStdString(status));
    status = "Images Captured. Current stored images from right camera: " + std::to_string( rightcam.getFrameBufferSize())  + "with " + std::to_string(leftcam.getframe().cols) + "x" + std::to_string(leftcam.getframe().rows) ;
    ui->status_te->append(QString::fromStdString(status));
    bool stored = chessboardPair.verifyPairAndSaveChessBoard(leftcam.gframeBuffer,rightcam.gframeBuffer);
    if(stored) logmessage->status("Paired Image with Chessboard saved");
    else logmessage->warning("One of the Pair image does not have chessboard. Thus, deleted. Please Retry.");
}

void MainWindow::on_clearCam_bt_clicked()
{
    leftcam.clearFrameBuffer();
    rightcam.clearFrameBuffer();
}

void MainWindow::on_stereostitch_rb_clicked()
{
    displaySelector = 4;
}



void MainWindow::on_withchess_rb_toggled(bool checked)
{
    if(checked){
        ui->status_te->append("checked");
        for(int i = 0; i<2; i++){
            chessboard[i].startDrawChessCorners();
        }
        displaySelector = 5;
    }

    else{
        ui->status_te->append("unchecked");
        for(int i = 0; i<2; i++){
            chessboard[i].stopDrawChessCorners();
        }
    }
}

void MainWindow::on_stereobm_rb_toggled(bool checked)
{
    if(checked) displaySelector = STEREOBMMAP;
    else{}
}

void MainWindow::on_applycalibration_bt_clicked()
{
    //apply stereo calibration parameters
    stereoproc.setIntrinsicsParam(stereocam.readSCIntrinsicParams());
    stereoproc.setExtrinsicsParam(stereocam.readSCExtrinsicParams());
    //stereoproc.setImageSize(stereocam);
    if(stereoproc.calibrationReady()){
        stereoproc.initStereoURMfromCalibParam();
        logmessage->status("Stereo Calibration Intrinsic Parameters Applied.");
    }
    else {
        logmessage->status("Stereo Calibration Failed. Please check calibration file.");
    }

}

void MainWindow::on_stereowcalib_rb_toggled(bool checked)
{
    if(checked) displaySelector = CALIBRATEDSTEREOMAP;
    else{}
}

void MainWindow::on_run_bt_clicked()
{
    runbutton->setToggleButton("RUN", "STOP");
    runbutton->togglebutton();

    on_leftcamera_bt_clicked();
    on_rightcamera_bt_clicked();
    on_applycalibration_bt_clicked();
    if(stereoproc.calibrationReady())
    on_stereowcalib_rb_toggled(true);
}


void MainWindow::on_recorddata_pb_clicked()
{

    if(!writingfile){
        start_time = std::clock();
        std::chrono::system_clock::time_point today = std::chrono::system_clock::now();

        time_t tt;

        tt = std::chrono::system_clock::to_time_t ( today );
        String datetime = ctime(&tt);

        String filename = "data" + datetime;

        outfile.open(filename + ".txt");
        //outfile <<
        writingfile = true;

        logmessage->status("Recording data to file.");
    }

    else{

    }
}

void MainWindow::on_recorddata_pb_toggled(bool checked)
{

}

void MainWindow::on_optitrack_pb_clicked()
{
    try{
        natnet.runNatnetStream("192.168.1.104","192.168.1.100");
    }
    catch(std::exception ex){logmessage->error(ex.what());}

}

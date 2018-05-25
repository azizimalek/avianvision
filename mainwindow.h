#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QPixmap>
#include "capture_camera.h"
#include "uibuttoncontroller.h"
#include "uidisplaycontroller.h"
#include "stereoprocessing.h"
#include "chessboarddetection.h"
#include "stereocalibration.h"
#include "opticalflow.h"
#include <GL/gl.h>
#include <GL/glu.h>

using namespace cv;
using namespace std;



namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    Mat displayGLSelected();

public slots:
    void update_leftcamera_bt_text(QString);
    void update_rightcamera_bt_text(QString);
    void update_run_bt_text(QString);
    void update_display(QPixmap pixframe);
    void update_displayGL();
    void update_frame();
    void logmessageReceived(QString);

private slots:
    void on_leftcamera_bt_clicked();
    void on_rightcamera_bt_clicked();


    void on_leftcam_rb_clicked();

    void on_rightcam_rb_clicked();

    void on_stereocam_rb_clicked();

    void on_stereodepth_rb_clicked();

    void on_stereocalib_bt_clicked();

    void on_StoreCam_bt_clicked();

    void on_clearCam_bt_clicked();

    void on_stereostitch_rb_clicked();

    //void on_withchess_rb_clicked();

    void on_withchess_rb_toggled(bool checked);

    void on_stereobm_rb_toggled(bool checked);

    void on_applycalibration_bt_clicked();

    void on_stereowcalib_rb_toggled(bool checked);

    void on_run_bt_clicked();

private:
    Ui::MainWindow *ui;
    //Model section
    //camera handler
    capture_camera leftcam;
    capture_camera rightcam;
    v4l2capture v4l2;                               //mainly use for camera setting using v4l2 library (linux only)
    //chessboard detection process handler
    chessboarddetection chessboardPair;             //mainly use for stereo chessboard process
    chessboarddetection chessboard[2];              //for chessboard detection on each camera
    //stereo handler
    stereocalibration stereocam;
    stereoprocessing stereoproc;
    //optical flow handler
    opticalflow opticalproc;

    //Controller section
    logmessagehandler *logmessage = new logmessagehandler();
    uibuttonController *leftcambutton = new uibuttonController();
    uibuttonController *rightcambutton = new uibuttonController();
    uibuttonController *runbutton = new uibuttonController();
    uidisplaycontroller *display = new uidisplaycontroller();

    int displaySelector = 0;
    //QThread display;
};

#endif // MAINWINDOW_H

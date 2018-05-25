#ifndef STEREOCALIBRATION_H
#define STEREOCALIBRATION_H


#include "masterheader.h"
#include "chessboarddetection.h"

struct stereoRectifyParam{
    cv::Mat R1, R2, P1, P2, Q;
};

struct stereoCalibrateParam{
    cv::Mat R,T,E,F;
};

struct intrinsicParam{
    cv::Mat M1, M2, D1, D2;
};

struct extrinsicParam{
    cv::Mat R,T,R1,R2,P1,P2,Q;
};

class stereocalibration
{
public:
    stereocalibration();
    void startStereoCalibration(std::vector<cv::Mat> , std::vector<cv::Mat>);
    void stereoCalibrationSimple();
    double stereoCalibrate( std::vector<std::vector<std::vector<cv::Point2f>>>&, std::vector<std::vector<cv::Point3f>> , std::vector<cv::Mat>& , std::vector<cv::Mat>&,stereoCalibrateParam&, cv::Size);
    double stereoCalibrationQualityCheck(std::vector<std::vector<std::vector<cv::Point2f>>>&, std::vector<cv::Mat>& , std::vector<cv::Mat>&, stereoCalibrateParam&, int);
    std::string saveSCIntrinsicParams(std::vector<cv::Mat>, std::vector<cv::Mat>);
    void saveSCExtrinsicParams(stereoCalibrateParam,stereoRectifyParam);
    void rectify(std::vector<cv::Mat>&, std::vector<cv::Mat>& ,std::vector<cv::Rect>&,stereoCalibrateParam&, stereoRectifyParam& , cv::Size);
    void rectifyUncalibrated(std::vector<std::vector<std::vector<cv::Point2f>>>& , std::vector<cv::Mat>, stereoCalibrateParam, stereoRectifyParam, cv::Size, int);
    void stereoArrangement();
    void initStereoUndistortRectifyMap(std::vector<cv::Mat>, std::vector<cv::Mat>&,  std::vector<std::vector<cv::Mat>>&, stereoRectifyParam, cv::Size);
    cv::Mat stereoArrangement(stereoRectifyParam , cv::Size );
    void setBoardSize(int h,int w){mboardSize = cv::Size(h,w);}
    void setSquareSize(float squareSize){msquareSize = squareSize;}
    intrinsicParam readSCIntrinsicParams();
    extrinsicParam readSCExtrinsicParams();
    void setImageSize(int h,int w){mimageSize = cv::Size(h,w);}
    cv::Size getImageSize(){return mimageSize;}
    cv::Size getImageSizeAuto();
    std::vector<std::vector<cv::Mat>> initStereoURMfromCalibParam(intrinsicParam, extrinsicParam);
    void setvalidRoi(std::vector<cv::Rect> validRoi){mvalidRoi = validRoi;}
    std::vector<cv::Rect> getvalidRoi(){return mvalidRoi;}

    stereoRectifyParam gSRParam;
    stereoCalibrateParam gSCParam;

private:
    chessboarddetection chessboard;
    logmessagehandler* logmessage = new logmessagehandler();
    std::vector<cv::Mat> mleftMatBuffer;
    std::vector<cv::Mat> mrightMatBuffer;
    cv::Size mboardSize;
    float msquareSize;
    cv::Size mimageSize = cv::Size(640,480);
    std::vector<cv::Rect> mvalidRoi;

};

#endif // STEREOCALIBRATION_H

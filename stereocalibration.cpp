#include "stereocalibration.h"

stereocalibration::stereocalibration()
{

}

cv::Size stereocalibration::getImageSizeAuto(){
    //mimageSize =

}


double stereocalibration::stereoCalibrate( std::vector<std::vector<std::vector<cv::Point2f>>>& imagePoints, std::vector<std::vector<cv::Point3f> > objectPoints, std::vector<cv::Mat>& cameraMatrix,std::vector<cv::Mat>& distCoeffs, stereoCalibrateParam& SCParam, cv::Size imageSize){
    //cv::Mat cameraMatrix[2], distCoeffs[2];

    cameraMatrix[0] = cv::initCameraMatrix2D(objectPoints,imagePoints[0],imageSize,0);
    cameraMatrix[1] = cv::initCameraMatrix2D(objectPoints,imagePoints[1],imageSize,0);
    cv::Mat R, T, E, F;

    //stereo start here
    double rms = cv::stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
                    cameraMatrix[0], distCoeffs[0],
                    cameraMatrix[1], distCoeffs[1],
                    imageSize, SCParam.R, SCParam.T, SCParam.E, SCParam.F,
                    CV_CALIB_FIX_ASPECT_RATIO +
                    CV_CALIB_ZERO_TANGENT_DIST +
                    CV_CALIB_USE_INTRINSIC_GUESS +
                    CV_CALIB_SAME_FOCAL_LENGTH +
                    CV_CALIB_RATIONAL_MODEL +
                    CV_CALIB_FIX_K3 + CV_CALIB_FIX_K4 + CV_CALIB_FIX_K5,
                    cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 100, 1e-5) );
    return rms;
}

double stereocalibration::stereoCalibrationQualityCheck(std::vector<std::vector<std::vector<cv::Point2f>>>& imagePoints, std::vector<cv::Mat>& cameraMatrix, std::vector<cv::Mat>& distCoeffs,stereoCalibrateParam& SCParam,  int nimages){
    // CALIBRATION QUALITY CHECK
    // because the output fundamental matrix implicitly
    // includes all the output information,
    // we can check the quality of calibration using the
    // epipolar geometry constraint: m2^t*F*m1=0
        double err = 0;
        int npoints = 0;
        std::vector<cv::Vec3f> lines[2];

        for(int i = 0; i < nimages; i++ )
        {
            int npt = (int)imagePoints[0][i].size();
            std::vector<cv::Mat> imgpt;
            imgpt.resize(2);
            for(int k = 0; k < 2; k++ )
            {
                imgpt[k] = cv::Mat(imagePoints[k][i]);
                cv::undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], cv::Mat(), cameraMatrix[k]);
                cv::computeCorrespondEpilines(imgpt[k], k+1, SCParam.F, lines[k]);
            }
            for(int j = 0; j < npt; j++ )
            {
                double errij = fabs(imagePoints[0][i][j].x*lines[1][j][0] +
                                    imagePoints[0][i][j].y*lines[1][j][1] + lines[1][j][2]) +
                               fabs(imagePoints[1][i][j].x*lines[0][j][0] +
                                    imagePoints[1][i][j].y*lines[0][j][1] + lines[0][j][2]);
                err += errij;
            }
            npoints += npt;
        }
        double epipolarErr = err/npoints;
        logmessage->status("average epipolar err = " + std::to_string(epipolarErr));

        return err/npoints;
}

std::string stereocalibration::saveSCIntrinsicParams(std::vector<cv::Mat> cameraMatrix, std::vector<cv::Mat> distCoeffs){
    // save intrinsic parameters

    cv::FileStorage fs("intrinsics.yml", cv::FileStorage::WRITE);
    if( fs.isOpened() )
    {
        fs << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0] <<
            "M2" << cameraMatrix[1] << "D2" << distCoeffs[1];
        fs.release();
    }
    else
        logmessage->error("can not save the intrinsic parameters");

    return "Error: can not save the intrinsic parameters";
}

intrinsicParam stereocalibration::readSCIntrinsicParams(){
    intrinsicParam inParam;
    cv::FileStorage fs;
    fs.open("intrinsics.yml", cv::FileStorage::READ);
    fs["M1"]>>inParam.M1;
    fs["M2"]>>inParam.M2;
    fs["D1"]>>inParam.D1;
    fs["D2"]>>inParam.D2;

    return inParam;

}

void stereocalibration::rectify(std::vector<cv::Mat>& cameraMatrix, std::vector<cv::Mat>& distCoeffs, std::vector<cv::Rect>& validRoi, stereoCalibrateParam& SCParam ,stereoRectifyParam& SRParam , cv::Size imageSize){

    try{
        cv::stereoRectify(cameraMatrix[0], distCoeffs[0],
                      cameraMatrix[1], distCoeffs[1],
                      imageSize, SCParam.R, SCParam.T, SRParam.R1, SRParam.R2, SRParam.P1, SRParam.P2, SRParam.Q,
                      CV_CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi[0], &validRoi[1]);
    }
    catch(std::exception ex){logmessage->error("Unable to rectify stereo camera");}

}

void stereocalibration::rectifyUncalibrated( std::vector<std::vector<std::vector<cv::Point2f>>>& imagePoints, std::vector<cv::Mat> cameraMatrix,stereoCalibrateParam SCParam, stereoRectifyParam SRParam, cv::Size imageSize, int nimages){
    std::vector<cv::Point2f> allimgpt[2];
    for( int k = 0; k < 2; k++ )
    {
        for(int i = 0; i < nimages; i++ )
            std::copy(imagePoints[k][i].begin(), imagePoints[k][i].end(), std::back_inserter(allimgpt[k]));
    }
    SCParam.F = cv::findFundamentalMat(cv::Mat(allimgpt[0]), cv::Mat(allimgpt[1]), CV_FM_8POINT, 0, 0);
    cv::Mat H1, H2;
    cv::stereoRectifyUncalibrated(cv::Mat(allimgpt[0]), cv::Mat(allimgpt[1]), SCParam.F, imageSize, H1, H2, 3);

    SRParam.R1 = cameraMatrix[0].inv()*H1*cameraMatrix[0];
    SRParam.R2 = cameraMatrix[1].inv()*H2*cameraMatrix[1];
    SRParam.P1 = cameraMatrix[0];
    SRParam.P2 = cameraMatrix[1];
}
void stereocalibration::saveSCExtrinsicParams(stereoCalibrateParam SCParam ,stereoRectifyParam SRParam){

    cv::FileStorage fs("extrinsics.yml", cv::FileStorage::WRITE);
    if( fs.isOpened() )
    {
        fs << "R" << SCParam.R << "T" << SCParam.T << "R1" << SRParam.R1 << "R2" << SRParam.R2 << "P1" << SRParam.P1 << "P2" << SRParam.P2 << "Q" << SRParam.Q;
        fs.release();
    }
    else logmessage->error("Unable to save Extrinsic Parameters");
}

extrinsicParam stereocalibration::readSCExtrinsicParams(){
    extrinsicParam exParam;
    cv::FileStorage fs;
    fs.open("extrinsics.yml", cv::FileStorage::READ);
    fs["R"]>>exParam.R;
    fs["T"]>>exParam.T;
    fs["R1"]>>exParam.R1;
    fs["R2"]>>exParam.R2;
    fs["P1"]>>exParam.P1;
    fs["P2"]>>exParam.P2;
    fs["Q"]>>exParam.Q;

    return exParam;
}

void stereocalibration::initStereoUndistortRectifyMap(std::vector<cv::Mat> cameraMatrix, std::vector<cv::Mat>& distCoeffs,  std::vector<std::vector<cv::Mat>>& rmap, stereoRectifyParam SRParam, cv::Size imageSize){
    cv::initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], SRParam.R1, SRParam.P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
    cv::initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], SRParam.R2, SRParam.P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);
}

std::vector<std::vector<cv::Mat>> stereocalibration::initStereoURMfromCalibParam(intrinsicParam intrinParam, extrinsicParam extrinParam){
    std::vector<std::vector<cv::Mat>> rmap;
    cv::initUndistortRectifyMap(intrinParam.M1,intrinParam.D1, extrinParam.R1, extrinParam.P1, mimageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
    cv::initUndistortRectifyMap(intrinParam.M2,intrinParam.D2, extrinParam.R2, extrinParam.P2, mimageSize, CV_16SC2, rmap[1][0], rmap[1][1]);

    return rmap;
}

cv::Mat stereocalibration::stereoArrangement(stereoRectifyParam SRParam, cv::Size imageSize){
     bool isVerticalStereo = fabs(SRParam.P2.at<double>(1, 3)) > fabs(SRParam.P2.at<double>(0, 3));
     cv::Mat canvas;
     double sf;
     int w, h;
     if( !isVerticalStereo )
     {
         sf = 600./MAX(imageSize.width, imageSize.height);
         w = cvRound(imageSize.width*sf);
         h = cvRound(imageSize.height*sf);
         canvas.create(h, w*2, CV_8UC3);
     }
     else
     {
         sf = 300./MAX(imageSize.width, imageSize.height);
         w = cvRound(imageSize.width*sf);
         h = cvRound(imageSize.height*sf);
         canvas.create(h*2, w, CV_8UC3);
     }

     return canvas;
}

//void stereocalibration::displayStereoMat(){
//    cv::remap(img, rimg, rmap[k][0], rmap[k][1], CV_INTER_LINEAR);
//    cv::cvtColor(rimg, cimg, cv::COLOR_GRAY2BGR);
//    cv::Mat canvasPart = !isVerticalStereo ? canvas(cv::Rect(w*k, 0, w, h)) : canvas(cv::Rect(0, h*k, w, h));
//    cv::resize(cimg, canvasPart, canvasPart.size(), 0, 0, CV_INTER_AREA);
//    if( useCalibrated )
//    {
//        cv::Rect vroi(cvRound(validRoi[k].x*sf), cvRound(validRoi[k].y*sf),
//                  cvRound(validRoi[k].width*sf), cvRound(validRoi[k].height*sf));
//        cv::rectangle(canvasPart, vroi, cv::Scalar(0,0,255), 3, 8);
//    }
//}
void stereocalibration::startStereoCalibration(std::vector<cv::Mat> leftmatbuffer, std::vector<cv::Mat> rightmatbuffer){
    if(!leftmatbuffer.empty() && !rightmatbuffer.empty()){
        mleftMatBuffer = leftmatbuffer;
        mrightMatBuffer = rightmatbuffer;
    }
    else logmessage->error("No mat buffers are found");

    logmessage->status("starting stereo calibration thread");

    try{
        stereoCalibrationSimple();
        //std::thread stereocalib_thread(&stereocalibration::stereoCalibrationSimple,this);
    }
    catch(std::exception ex){logmessage->error("Problem starting stereo calibration thread");}
    //stereocalib_thread.detach();
}

void stereocalibration::stereoCalibrationSimple()
{
    logmessage->status("starting stereo calibrations");
    const int maxScale = 2;
    cv::Size boardSize = mboardSize;
    float squareSize = msquareSize;

    // ARRAY AND VECTOR STORAGE:
    std::vector<cv::Mat> leftmatbuffer = mleftMatBuffer;
    std::vector<cv::Mat> rightmatbuffer = mrightMatBuffer;
    //imagepoints array - [camera set][image buffer][chessboard points]
    std::vector<std::vector<std::vector<cv::Point2f>>> imagePoints;
    imagePoints.resize(2);
    std::vector<std::vector<cv::Point3f> > objectPoints;
    cv::Size imageSize;
    bool foundl, foundr;
    int i, j, k, nlimages = (int)leftmatbuffer.size(), nrimages = (int)rightmatbuffer.size(), nimages;
    stereoCalibrateParam sCParam;
    stereoRectifyParam sRParam;
    std::vector<cv::Mat> cameraMatrix, distCoeffs;
    cameraMatrix.resize(2);
    distCoeffs.resize(2);
    std::vector<cv::Rect> validRoi;
    validRoi.resize(2);

    //setImageSize(mleftMatBuffer[0].rows,mleftMatBuffer[0].cols);
    imageSize = getImageSize();

    logmessage->status("assign image number");
    if(nlimages == nrimages)
        nimages = (nlimages+nrimages)/2;
    else return;

    imagePoints[0].resize(nimages);
    imagePoints[1].resize(nimages);
    logmessage->status("image points buffer resized");

    for( i = j = 0; i < nimages; i++ ) //number of image
    {
        foundl = chessboard.extractChessPoints(leftmatbuffer[i],imagePoints[0][i],boardSize);
        foundr = chessboard.extractChessPoints(rightmatbuffer[i],imagePoints[1][i],boardSize);

    }
    logmessage->status("chessboard points extracted from image buffers. Total point buffer: " + std::to_string(i));

//initialize chessboard actual sizing
    try{
        chessboard.initChessboardPoints(objectPoints,nimages,boardSize,squareSize);
    }
    catch(std::exception e){logmessage->error("Problem initialize actual board size vector points");}

////Starting Stereo Calibration from here
    logmessage->status("Running stereo calibration ...");

////Calibrate stereo of both camera
    stereoCalibrate(imagePoints,objectPoints,cameraMatrix,distCoeffs,sCParam,imageSize);
    logmessage->status("Stereo calibration succesful!");

//// CALIBRATION QUALITY CHECK
    stereoCalibrationQualityCheck(imagePoints,cameraMatrix,distCoeffs,sCParam,nimages);
    //logmessage->status("Stereo calibration quality check passed.");

//// save intrinsic parameters
    saveSCIntrinsicParams(cameraMatrix,distCoeffs);
    logmessage->status("Intrinsic Parameter saved.");

////rectify stereo
    rectify(cameraMatrix,distCoeffs,validRoi,sCParam, sRParam,imageSize);

////save extrinsic parameters
    saveSCExtrinsicParams(sCParam,sRParam);

    setvalidRoi(validRoi);
}



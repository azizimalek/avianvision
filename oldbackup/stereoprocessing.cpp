#include "stereoprocessing.h"

stereoprocessing::stereoprocessing()
{

}

cv::Mat stereoprocessing::crudeMatMerge(cv::Mat matleft,cv::Mat matright){
    cv::Mat matOut;

    if(!matleft.empty() && !matright.empty()) cv::hconcat(matleft,matright, matOut);

    return matOut;
}

cv::Mat stereoprocessing::stitchingMatMerge(cv::Mat leftMat, cv::Mat rightMat){
    cv::Mat stitchedMat;
    std::vector<cv::Mat> stitchingMat;
    stitchingMat.push_back(leftMat);
    stitchingMat.push_back(rightMat);
    cv::Stitcher::Mode mode = cv::Stitcher::SCANS;
    //cv::Stitcher stitcher = cv::Stitcher::createDefault(false);
    cv::Ptr<cv::Stitcher> stitcher = cv::Stitcher::create(mode, true);
    cv::Stitcher::Status stitchstatus= stitcher->stitch(stitchingMat,stitchedMat);
    stitchingMat.clear();
    if(stitchstatus == cv::Stitcher::OK)
    return stitchedMat;

}

cv::Mat stereoprocessing::drawChessCorners(cv::Mat inoutMat, cv::Size boardSize, std::vector<cv::Point2f> corners, bool found ){
    cv::drawChessboardCorners(inoutMat, boardSize, corners, found);
    return inoutMat;
}

bool stereoprocessing::findChessCorners(cv::Mat inMat,cv::Size& boardSize, std::vector<cv::Point2f>& corners, const int maxScale = 2){

    bool found = false;
    for( int scale = 1; scale <= maxScale; scale++ )
    {
        cv::Mat tempMat;
        if( scale == 1 )
            tempMat = inMat;
        else
            cv::resize(inMat, tempMat, cv::Size(), scale, scale,cv::INTER_LINEAR_EXACT);

            found = cv::findChessboardCorners(tempMat, boardSize, corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);

        if( found )
        {
            if( scale > 1 )
            {
                cv::Mat cornersMat(corners);
                cornersMat *= 1./scale;
            }
            break;
        }
    }

    return found;
}

void stereoprocessing::extractChessPoints(cv::Mat inMat, std::vector<cv::Point2f>& corners, cv::Size boardSize, float squareSize){

    cv::Size imageSize;

    if(inMat.empty())
        return;
    if( imageSize == cv::Size() )
        imageSize = inMat.size();
    else if( inMat.size() != imageSize )
    {
       //error message std::cout << "The image " << filename << " has the size different from the first image size. Skipping the pair\n";
        return;
    }
    bool found = false;

    //find chessboard corners in image
    found = findChessCorners(inMat, boardSize, corners, 2);

    //display chessboard corners on window
    gframeWithChess = drawChessCorners(inMat, boardSize, corners, found );

    //further improve accuracy of the corner points with sub pix calculation
    if( !found )
        return;
    cv::cornerSubPix(inMat, corners, cv::Size(11,11), cv::Size(-1,-1),
                 cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS,
                              30, 0.01));
}

double stereoprocessing::stereoCalibrate( std::vector<std::vector<cv::Point2f> >& imagePoints, std::vector<std::vector<cv::Point3f> > objectPoints, std::vector<cv::Mat>& cameraMatrix,std::vector<cv::Mat>& distCoeffs, stereoCalibrateParam SCParam, cv::Size imageSize){
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

void stereoprocessing::initChessboardPoints(std::vector<std::vector<cv::Point3f>> &objectPoints, int nimages, cv::Size boardSize, float squareSize){
    objectPoints.resize(nimages);

    //get relative exact point for the chessboard that are used
    for(int i = 0; i < nimages; i++ )
    {
        for(int j = 0; j < boardSize.height; j++ )
            for(int k = 0; k < boardSize.width; k++ )
                objectPoints[i].push_back(cv::Point3f(k*squareSize, j*squareSize, 0));
    }
    //return objectPoints;
}

double stereoprocessing::stereoCalibrationQualityCheck(std::vector<std::vector<std::vector<cv::Point2f>>> imagePoints, std::vector<cv::Mat>& cameraMatrix, std::vector<cv::Mat>& distCoeffs,stereoCalibrateParam SCParam,  int nimages){
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
            cv::Mat imgpt[2];
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
       // cout << "average epipolar err = " <<  err/npoints << endl;
        return err/npoints;
}

std::string stereoprocessing::saveSCIntrinsicParams(std::vector<cv::Mat> cameraMatrix, std::vector<cv::Mat> distCoeffs){
    // save intrinsic parameters

    cv::FileStorage fs("intrinsics.yml", cv::FileStorage::WRITE);
    if( fs.isOpened() )
    {
        fs << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0] <<
            "M2" << cameraMatrix[1] << "D2" << distCoeffs[1];
        fs.release();
    }
    //else
        //cout << "Error: can not save the intrinsic parameters\n";
    return "Error: can not save the intrinsic parameters";
}

void stereoprocessing::rectify(std::vector<cv::Mat> cameraMatrix, std::vector<cv::Mat> distCoeffs, std::vector<cv::Rect>& validRoi, stereoCalibrateParam SCParam ,stereoRectifyParam& SRParam , cv::Size imageSize){
    cv::stereoRectify(cameraMatrix[0], distCoeffs[0],
                  cameraMatrix[1], distCoeffs[1],
                  imageSize, SCParam.R, SCParam.T, SRParam.R1, SRParam.R2, SRParam.P1, SRParam.P2, SRParam.Q,
                  CV_CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi[0], &validRoi[1]);
}

void stereoprocessing::rectifyUncalibrated( std::vector<std::vector<std::vector<cv::Point2f>>>& imagePoints, std::vector<cv::Mat> cameraMatrix,stereoCalibrateParam SCParam, stereoRectifyParam SRParam, cv::Size imageSize, int nimages){
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
void stereoprocessing::saveSCExtrinsicParams(stereoCalibrateParam SCParam ,stereoRectifyParam SRParam){

    cv::FileStorage fs("extrinsics.yml", cv::FileStorage::WRITE);
    if( fs.isOpened() )
    {
        fs << "R" << SCParam.R << "T" << SCParam.T << "R1" << SRParam.R1 << "R2" << SRParam.R2 << "P1" << SRParam.P1 << "P2" << SRParam.P2 << "Q" << SRParam.Q;
        fs.release();
    }
}

void stereoprocessing::initStereoUndistortRectifyMap(std::vector<cv::Mat> cameraMatrix, std::vector<cv::Mat>& distCoeffs,  std::vector<std::vector<cv::Mat>>& rmap, stereoRectifyParam SRParam, cv::Size imageSize){
    cv::initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], SRParam.R1, SRParam.P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
    cv::initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], SRParam.R2, SRParam.P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);
}

cv::Mat stereoprocessing::stereoArrangement(stereoRectifyParam SRParam, cv::Size imageSize){
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

//void stereoprocessing::displayStereoMat(){
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

void stereoprocessing::stereoCalibration(const std::vector<std::string>& imagelist, cv::Size boardSize, float squareSize, bool displayCorners = false, bool useCalibrated=true, bool showRectified=true)
{
    if( imagelist.size() % 2 != 0 )
    {
        //std::cout << "Error: the image list contains odd (non-even) number of elements\n";
        return;
    }

    const int maxScale = 2;
    // ARRAY AND VECTOR STORAGE:

    std::vector<std::vector<cv::Point2f> > imagePoints[2];
    std::vector<std::vector<cv::Point3f> > objectPoints;
    cv::Size imageSize;

    int i, j, k, nimages = (int)imagelist.size()/2;

    imagePoints[0].resize(nimages);
    imagePoints[1].resize(nimages);
    std::vector<std::string> goodImageList;

    for( i = j = 0; i < nimages; i++ ) //number of image
    {
        for( k = 0; k < 2; k++ ) //camera number
        {
            const std::string& filename = imagelist[i*2+k];
            cv::Mat img = cv::imread(filename, 0);
            if(img.empty())
                break;
            if( imageSize == cv::Size() )
                imageSize = img.size();
            else if( img.size() != imageSize )
            {
               // std::cout << "The image " << filename << " has the size different from the first image size. Skipping the pair\n";
                break;
            }
            bool found = false;
            std::vector<cv::Point2f>& corners = imagePoints[k][j];
            for( int scale = 1; scale <= maxScale; scale++ )
            {
                cv::Mat timg;
                if( scale == 1 )
                    timg = img;
                else
                    cv::resize(img, timg, cv::Size(), scale, scale,cv::INTER_LINEAR_EXACT);
                found = cv::findChessboardCorners(timg, boardSize, corners,
                   CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
                if( found )
                {
                    if( scale > 1 )
                    {
                        cv::Mat cornersMat(corners);
                        cornersMat *= 1./scale;
                    }
                    break;
                }
            }
            if( displayCorners )
            {
                //std::cout << filename << std::endl;
                cv::Mat cimg, cimg1;
                cv::cvtColor(img, cimg, cv::COLOR_GRAY2BGR);
                cv::drawChessboardCorners(cimg, boardSize, corners, found);
                double sf = 640./MAX(img.rows, img.cols);
                cv::resize(cimg, cimg1, cv::Size(), sf, sf, cv::INTER_LINEAR_EXACT);
                cv::imshow("corners", cimg1);
                //char c = (char)waitKey(500);
                //if( c == 27 || c == 'q' || c == 'Q' ) //Allow ESC to quit
                    //exit(-1);
            }
            //else
               // putchar('.');
            if( !found )
                break;
            cv::cornerSubPix(img, corners, cv::Size(11,11), cv::Size(-1,-1),
                         cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS,
                                      30, 0.01));
        }
        if( k == 2 )
        {
            goodImageList.push_back(imagelist[i*2]);
            goodImageList.push_back(imagelist[i*2+1]);
            j++;
        }
    }
    //cout << j << " pairs have been successfully detected.\n";
    nimages = j;
    if( nimages < 2 )
    {
        //cout << "Error: too little pairs to run the calibration\n";
        return;
    }

    imagePoints[0].resize(nimages);
    imagePoints[1].resize(nimages);
    objectPoints.resize(nimages);

    for( i = 0; i < nimages; i++ )
    {
        for( j = 0; j < boardSize.height; j++ )
            for( k = 0; k < boardSize.width; k++ )
                objectPoints[i].push_back(cv::Point3f(k*squareSize, j*squareSize, 0));
    }

   // cout << "Running stereo calibration ...\n";

    cv::Mat cameraMatrix[2], distCoeffs[2];
    cameraMatrix[0] = cv::initCameraMatrix2D(objectPoints,imagePoints[0],imageSize,0);
    cameraMatrix[1] = cv::initCameraMatrix2D(objectPoints,imagePoints[1],imageSize,0);
    cv::Mat R, T, E, F;

    //stereo start here
    double rms = cv::stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
                    cameraMatrix[0], distCoeffs[0],
                    cameraMatrix[1], distCoeffs[1],
                    imageSize, R, T, E, F,
                    CV_CALIB_FIX_ASPECT_RATIO +
                    CV_CALIB_ZERO_TANGENT_DIST +
                    CV_CALIB_USE_INTRINSIC_GUESS +
                    CV_CALIB_SAME_FOCAL_LENGTH +
                    CV_CALIB_RATIONAL_MODEL +
                    CV_CALIB_FIX_K3 + CV_CALIB_FIX_K4 + CV_CALIB_FIX_K5,
                    cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 100, 1e-5) );
    //cout << "done with RMS error=" << rms << endl;

// CALIBRATION QUALITY CHECK
// because the output fundamental matrix implicitly
// includes all the output information,
// we can check the quality of calibration using the
// epipolar geometry constraint: m2^t*F*m1=0
    double err = 0;
    int npoints = 0;
    std::vector<cv::Vec3f> lines[2];
    for( i = 0; i < nimages; i++ )
    {
        int npt = (int)imagePoints[0][i].size();
        cv::Mat imgpt[2];
        for( k = 0; k < 2; k++ )
        {
            imgpt[k] = cv::Mat(imagePoints[k][i]);
            cv::undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], cv::Mat(), cameraMatrix[k]);
            cv::computeCorrespondEpilines(imgpt[k], k+1, F, lines[k]);
        }
        for( j = 0; j < npt; j++ )
        {
            double errij = fabs(imagePoints[0][i][j].x*lines[1][j][0] +
                                imagePoints[0][i][j].y*lines[1][j][1] + lines[1][j][2]) +
                           fabs(imagePoints[1][i][j].x*lines[0][j][0] +
                                imagePoints[1][i][j].y*lines[0][j][1] + lines[0][j][2]);
            err += errij;
        }
        npoints += npt;
    }
   // cout << "average epipolar err = " <<  err/npoints << endl;

    // save intrinsic parameters
    cv::FileStorage fs("intrinsics.yml", cv::FileStorage::WRITE);
    if( fs.isOpened() )
    {
        fs << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0] <<
            "M2" << cameraMatrix[1] << "D2" << distCoeffs[1];
        fs.release();
    }
    //else
        //cout << "Error: can not save the intrinsic parameters\n";

    cv::Mat R1, R2, P1, P2, Q;
    cv::Rect validRoi[2];

    cv::stereoRectify(cameraMatrix[0], distCoeffs[0],
                  cameraMatrix[1], distCoeffs[1],
                  imageSize, R, T, R1, R2, P1, P2, Q,
                  CV_CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi[0], &validRoi[1]);

    fs.open("extrinsics.yml", cv::FileStorage::WRITE);
    if( fs.isOpened() )
    {
        fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
        fs.release();
    }
    //else
        //cout << "Error: can not save the extrinsic parameters\n";

    // OpenCV can handle left-right
    // or up-down camera arrangements
    bool isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));

// COMPUTE AND DISPLAY RECTIFICATION
    if( !showRectified )
        return;

    cv::Mat rmap[2][2];
// IF BY CALIBRATED (BOUGUET'S METHOD)
    if( useCalibrated )
    {
        // we already computed everything
    }
// OR ELSE HARTLEY'S METHOD
    else
 // use intrinsic parameters of each camera, but
 // compute the rectification transformation directly
 // from the fundamental matrix
    {
        std::vector<cv::Point2f> allimgpt[2];
        for( k = 0; k < 2; k++ )
        {
            for( i = 0; i < nimages; i++ )
                std::copy(imagePoints[k][i].begin(), imagePoints[k][i].end(), std::back_inserter(allimgpt[k]));
        }
        F = cv::findFundamentalMat(cv::Mat(allimgpt[0]), cv::Mat(allimgpt[1]), CV_FM_8POINT, 0, 0);
        cv::Mat H1, H2;
        cv::stereoRectifyUncalibrated(cv::Mat(allimgpt[0]), cv::Mat(allimgpt[1]), F, imageSize, H1, H2, 3);

        R1 = cameraMatrix[0].inv()*H1*cameraMatrix[0];
        R2 = cameraMatrix[1].inv()*H2*cameraMatrix[1];
        P1 = cameraMatrix[0];
        P2 = cameraMatrix[1];
    }

    //Precompute maps for cv::remap()
    cv::initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
    cv::initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);

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

    for( i = 0; i < nimages; i++ )
    {
        for( k = 0; k < 2; k++ )
        {
            cv::Mat img = cv::imread(goodImageList[i*2+k], 0), rimg, cimg;
            cv::remap(img, rimg, rmap[k][0], rmap[k][1], CV_INTER_LINEAR);
            cv::cvtColor(rimg, cimg, cv::COLOR_GRAY2BGR);
            cv::Mat canvasPart = !isVerticalStereo ? canvas(cv::Rect(w*k, 0, w, h)) : canvas(cv::Rect(0, h*k, w, h));
            cv::resize(cimg, canvasPart, canvasPart.size(), 0, 0, CV_INTER_AREA);
            if( useCalibrated )
            {
                cv::Rect vroi(cvRound(validRoi[k].x*sf), cvRound(validRoi[k].y*sf),
                          cvRound(validRoi[k].width*sf), cvRound(validRoi[k].height*sf));
                cv::rectangle(canvasPart, vroi, cv::Scalar(0,0,255), 3, 8);
            }
        }

        if( !isVerticalStereo )
            for( j = 0; j < canvas.rows; j += 16 )
                cv::line(canvas, cv::Point(0, j), cv::Point(canvas.cols, j), cv::Scalar(0, 255, 0), 1, 8);
        else
            for( j = 0; j < canvas.cols; j += 16 )
                cv::line(canvas, cv::Point(j, 0), cv::Point(j, canvas.rows), cv::Scalar(0, 255, 0), 1, 8);
        cv::imshow("rectified", canvas);
//        char c = (char)waitKey();
//        if( c == 27 || c == 'q' || c == 'Q' )
//            break;
    }
}

#include "chessboarddetection.h"

chessboarddetection::chessboarddetection()
{

}

bool chessboarddetection::verifyPairChessBoard(std::vector<cv::Mat>& leftcamBuffer, std::vector<cv::Mat>& rightcamBuffer){
    bool verified = false;
    cv::Mat leftcam = leftcamBuffer.back();
    cv::Mat rightcam = rightcamBuffer.back();
    std::vector<cv::Point2f> corners;
    cv::Size boardSize = cv::Size(9,6);
    bool foundleft = findChessCorners(leftcam,boardSize,corners, 1);
    bool foundright = findChessCorners(rightcam,boardSize, corners,1);

    if(foundleft && foundright) verified = true;
    else{
        verified = false;
    }

    return verified;
}

bool chessboarddetection::verifyPairAndSaveChessBoard(std::vector<cv::Mat>& leftcamBuffer, std::vector<cv::Mat>& rightcamBuffer){
    bool verify = verifyPairChessBoard(leftcamBuffer,rightcamBuffer);

    if(verify) return verify;
    else deleteCurrentPairMatBuffer(leftcamBuffer,rightcamBuffer);

    return verify;
}

void chessboarddetection::deleteCurrentPairMatBuffer(std::vector<cv::Mat>& leftcamBuffer, std::vector<cv::Mat>& rightcamBuffer){
    leftcamBuffer.pop_back();
    rightcamBuffer.pop_back();
}

void chessboarddetection::startDrawChessCorners(){
    std::thread drawChessCorners_thread (&chessboarddetection::drawChessCorners_independent,this);
    allowthread = true;
    drawChessCorners_thread.detach();
    //logmessage.status("thread running!");
}

void chessboarddetection::stopDrawChessCorners(){
    allowthread = false;
}

void chessboarddetection::drawChessCorners_independent(){
    while(allowthread){
        if(doneDrawChessPoint){
            if(!mliveMat.empty()) mdrawChessBoard = drawChessCorners(mliveMat, cv::Size(9,6));
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}

cv::Mat chessboarddetection::drawChessCorners(cv::Mat inoutMat, cv::Size boardSize){
    doneDrawChessPoint = false;
    std::vector<cv::Point2f> corners;
    bool found = extractChessPoints(inoutMat,corners,boardSize);
    cv::drawChessboardCorners(inoutMat, boardSize, corners, found);
    doneDrawChessPoint = true;
    return inoutMat;
}

bool chessboarddetection::findChessCorners(cv::Mat inMat,cv::Size& boardSize, std::vector<cv::Point2f>& corners, const int maxScale = 2){

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

bool chessboarddetection::extractChessPoints(cv::Mat inMat, std::vector<cv::Point2f>& corners, cv::Size boardSize){

    cv::Size imageSize;

    if(inMat.empty())
        return false;
    if( imageSize == cv::Size() )
        imageSize = inMat.size();
    else if( inMat.size() != imageSize )
    {
        logmessage.error("The Mat image has the size different from the first image size. Skipping a pair");
        return false;
    }
    bool found = false;

    //find chessboard corners in image
    found = findChessCorners(inMat, boardSize, corners, 1);

    //further improve accuracy of the corner points with sub pix calculation
    if( !found )
        return found;
    cv::TermCriteria criteria = cv::TermCriteria( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.001 );
    try{
        cv::Mat inGrayMat;
        cv::cvtColor(inMat,inGrayMat,CV_BGR2GRAY);
        cv::cornerSubPix(inGrayMat, corners, cv::Size(11,11), cv::Size(-1,-1), criteria);
    }
    catch(std::exception& e){}

    return found;
}

void chessboarddetection::initChessboardPoints(std::vector<std::vector<cv::Point3f>> &objectPoints, int nimages, cv::Size boardSize, float squareSize){
    //objectPoints.resize(nimages);

    for(int i = 0; i < nimages; i++ )
    {
        objectPoints.push_back(calculateActualChessboardSize(boardSize,squareSize));
    }
}

std::vector<cv::Point3f> chessboarddetection::calculateActualChessboardSize(cv::Size boardSize, float squareSize){
    //get relative exact point for the chessboard that are used
    std::vector<cv::Point3f> actualBoardSize;

    for(int h = 0; h < boardSize.height; h++ )
        for(int w = 0; w < boardSize.width; w++ )
            actualBoardSize.push_back(cv::Point3f(h*squareSize, w*squareSize, 0));

    return actualBoardSize;
}

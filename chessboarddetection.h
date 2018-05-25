#ifndef CHESSBOARDDETECTION_H
#define CHESSBOARDDETECTION_H

#include "masterheader.h"

class chessboarddetection
{
public:
    chessboarddetection();
    cv::Mat drawChessCorners(cv::Mat, cv::Size);
    bool findChessCorners(cv::Mat ,cv::Size& , std::vector<cv::Point2f>& , const int);
    bool extractChessPoints(cv::Mat, std::vector<cv::Point2f>& , cv::Size);
    void initChessboardPoints(std::vector<std::vector<cv::Point3f>>& , int , cv::Size, float);
    void updateChessboardInput(cv::Mat inMat){if(!inMat.empty())mliveMat = inMat;}
    void updateChessboardFrame(cv::Mat& chessMat){chessMat = mdrawChessBoard;}
    void startDrawChessCorners();
    void stopDrawChessCorners();
    void drawChessCorners_independent();
    void setboardsize(int x, int y){mboardSize.width = x; mboardSize.height = y;}
    void enableThread(){allowthread = true;}
    void disableThread(){allowthread = false;}
    bool verifyPairChessBoard(std::vector<cv::Mat>& , std::vector<cv::Mat>& );
    bool verifyPairAndSaveChessBoard(std::vector<cv::Mat>& , std::vector<cv::Mat>& );
    void deleteCurrentPairMatBuffer(std::vector<cv::Mat>& , std::vector<cv::Mat>& );
    std::vector<cv::Point3f> calculateActualChessboardSize(cv::Size , float );
    cv::Mat gframeWithChess;

private:
    bool allowthread = true;
    logmessagehandler logmessage;
    cv::Mat mliveMat;
    cv::Mat mdrawChessBoard;
    cv::Size mboardSize;
    std::thread drawChessCorners_thread;
    bool newChessBoardDraw = false;
    bool doneDrawChessPoint = true;
};

#endif // CHESSBOARDDETECTION_H

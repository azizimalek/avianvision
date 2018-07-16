#ifndef OPTICALFLOW_H
#define OPTICALFLOW_H

#include "masterheader.h"
#include <ctime>
#include <cstdio>
#include <iostream>

struct goodFeatureTrackParam{
    // maxCorners EThe maximum number of corners to return. If there are more corners
    // than that will be found, the strongest of them will be returned
    int maxCorners = 1000;

    // qualityLevel ECharacterizes the minimal accepted quality of image corners;
    // the value of the parameter is multiplied by the by the best corner quality
    // measure (which is the min eigenvalue, see cornerMinEigenVal() ,
    // or the Harris function response, see cornerHarris() ).
    // The corners, which quality measure is less than the product, will be rejected.
    // For example, if the best corner has the quality measure = 1500,
    // and the qualityLevel=0.01 , then all the corners which quality measure is
    // less than 15 will be rejected.
    double qualityLevel = 0.05;

    // minDistance EThe minimum possible Euclidean distance between the returned corners
    double minDistance = 3.;

    // mask EThe optional region of interest. If the image is not empty (then it
    // needs to have the type CV_8UC1 and the same size as image ), it will specify
    // the region in which the corners are detected
    cv::Mat mask;

    // blockSize ESize of the averaging block for computing derivative covariation
    // matrix over each pixel neighborhood, see cornerEigenValsAndVecs()
    int blockSize = 5;

    // useHarrisDetector EIndicates, whether to use operator or cornerMinEigenVal()
    bool useHarrisDetector = false;

    // k EFree parameter of Harris detector
    double k = 0.04;
};

struct opticalflowPyrLKInput{
    cv::Mat previousGray;
    cv::Mat nextGray;
    std::vector<cv::Point2f> prev_goodpoints;
    std::vector<cv::Point2f> next_goodpoints;

    //set for optical flow parameter
    void setPreviousGray(cv::Mat grayframe){if(!grayframe.empty()) previousGray = grayframe;}
    void setNextGray(cv::Mat grayframe){if(!grayframe.empty()) nextGray = grayframe;}
    void setPreviousPoints(std::vector<cv::Point2f> points){ prev_goodpoints = points;}
    void setNextPoints(std::vector<cv::Point2f> points){ next_goodpoints = points;}
};

struct opticalflowPyrLKOutput{
    std::vector<uchar> status;
    cv::Mat errors;
};

struct matchOpticalFlowPoints{
    std::vector<cv::Point2f> prev_flowPoints;
    std::vector<cv::Point2f> next_flowPoints;
};

class opticalflow
{
public:
    opticalflow();
    std::vector<cv::Point2f> computeOpticalFlowPoints(cv::Mat, goodFeatureTrackParam);
    std::vector<cv::Point2f> computeOpticalFlowPoints();
    void goodFeaturesToTrackSimplified(cv::Mat&, std::vector<cv::Point2f>&, goodFeatureTrackParam&);
    void computeOpticalFlowPyrLK(opticalflowPyrLKInput&, opticalflowPyrLKOutput&);
    void computeOpticalFlowPyrLK();
    matchOpticalFlowPoints matchingOpticalFlowPoints(std::vector<cv::Point2f>&, std::vector<cv::Point2f>&, std::vector<uchar>);
    matchOpticalFlowPoints matchingOpticalFlowPoints(opticalflowPyrLKInput&, opticalflowPyrLKOutput&);
    matchOpticalFlowPoints matchingOpticalFlowPoints();
    bool initOpticalFlowPyrLK();
    matchOpticalFlowPoints runOpticalFlowPyrLK(cv::Mat);
    void runOpticalFlowPyrLKThread(cv::Mat frame);
    void runOpticalFlowPyrLKSimple();
    void startOpticalFlowPyrLK(cv::Mat);
    void startOpticalFlowPyrLK();
    void stopOpticalFlowPyrLK(){mopLK_thread_status = false;}
    cv::Mat drawingOpticalFlowNative(cv::Mat);
    void refreshOpticalFlowForNext();
    cv::Mat drawPanoRegion(cv::Mat);
    std::vector<cv::Point2f> calculatePanoRegionVelocity(cv::Mat , matchOpticalFlowPoints);
    cv::Point2f calculateVelocityXY(matchOpticalFlowPoints);
    cv::Mat drawVelocityByPanoRegion(cv::Mat, std::vector<cv::Point2f>);
    void runOpticalFlowPyrLKPanoByRegionThread();
    void startOpticalFlowPyrLKPano();

    //--set methods
    void setGoodFeatureTrackParam(goodFeatureTrackParam gftParam){mgftParam = gftParam;}
    void setComputeFrame(cv::Mat frame){ mframe = frame;}
    //--bind methods
    void bindFrame(cv::Mat& frame){mframe = frame;}


    //get methods
    goodFeatureTrackParam getgoodFeatureTrackParam(){return mgftParam;}
    cv::Mat getFrame(){return mframe;}
    cv::Mat getDisplayFrame(){return mdispFrame;}
    std::vector<cv::Point2f> getPixelVelocityData(){return velocityPixelXYData;}

    bool initFlag = false;
    bool refreshFlag = false;

private:
    cv::Mat mframe;
    cv::Mat mdispFrame;
    goodFeatureTrackParam mgftParam;
    std::vector<cv::Point2f> prev_goodPoints;
    opticalflowPyrLKInput mofplkin;
    opticalflowPyrLKOutput mofplkout;
    bool mopLK_thread_status = false;

    std::clock_t start_time;
    std::clock_t end_time;
    double elapsed_time;
    std::vector<cv::Point2f> velocityPixelXYData;


};

#endif // OPTICALFLOW_H

#include "opticalflow.h"

opticalflow::opticalflow()
{

}

std::vector<cv::Point2f> opticalflow::computeOpticalFlowPoints(cv::Mat frame, goodFeatureTrackParam gftParam){
    std::vector<cv::Point2f> goodPoints;

    if(!frame.empty()){
        cv::cvtColor(frame,frame,CV_BGR2GRAY);
        goodFeaturesToTrackSimplified(frame,goodPoints,gftParam);
        cv::cornerSubPix(frame, goodPoints,
                                        cv::Size(10, 10), cv::Size(-1, -1),
                                        cv::TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03));
    }

    return goodPoints;

}

std::vector<cv::Point2f> opticalflow::computeOpticalFlowPoints(){
    std::vector<cv::Point2f> goodPoints = computeOpticalFlowPoints(mframe,mgftParam);
    return goodPoints;
}


void opticalflow::computeOpticalFlowPyrLK(opticalflowPyrLKInput& ofplkin, opticalflowPyrLKOutput& ofplkout){
    cv::calcOpticalFlowPyrLK(   ofplkin.previousGray,
                                ofplkin.nextGray,
                                ofplkin.prev_goodpoints,
                                ofplkin.next_goodpoints,
                                ofplkout.status,
                                ofplkout.errors
                            );
}


matchOpticalFlowPoints opticalflow::matchingOpticalFlowPoints(std::vector<cv::Point2f>& prev_goodPoints, std::vector<cv::Point2f>& next_goodPoints, std::vector<uchar> status){

    matchOpticalFlowPoints matchofPoints;

    if(!status.empty()){
        for (unsigned int i = 0; i < status.size(); ++i) {
            if (status[i] == 1) {
                matchofPoints.prev_flowPoints.push_back(prev_goodPoints[i]);
                matchofPoints.next_flowPoints.push_back(next_goodPoints[i]);
            }
        }
    }

    return matchofPoints;
}

matchOpticalFlowPoints opticalflow::matchingOpticalFlowPoints(opticalflowPyrLKInput& ofplkin, opticalflowPyrLKOutput& ofplkout){

   matchOpticalFlowPoints matchofPoints = matchingOpticalFlowPoints(ofplkin.prev_goodpoints,ofplkin.next_goodpoints,ofplkout.status);

   return matchofPoints;

}

void opticalflow::goodFeaturesToTrackSimplified(cv::Mat& frame, std::vector<cv::Point2f>& goodPoints, goodFeatureTrackParam& gftParam){
    cv::goodFeaturesToTrack(frame,goodPoints,
                            gftParam.maxCorners,gftParam.qualityLevel,
                            gftParam.minDistance,gftParam.mask,
                            gftParam.blockSize,gftParam.useHarrisDetector,gftParam.k);
}

void opticalflow::runOpticalFlowPyrLK(cv::Mat frame){
    std::vector<cv::Point2f> goodPoints;
    matchOpticalFlowPoints matchofPoints;
    setComputeFrame(frame);
    goodPoints = computeOpticalFlowPoints();
    //TODO: continuation of optical flow - still ongoing


}

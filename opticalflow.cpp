#include "opticalflow.h"

opticalflow::opticalflow()
{
    start_time = std::clock();
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

void opticalflow::computeOpticalFlowPyrLK(){
    computeOpticalFlowPyrLK(mofplkin,mofplkout);
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

matchOpticalFlowPoints opticalflow::matchingOpticalFlowPoints(){

   return matchingOpticalFlowPoints(mofplkin,mofplkout);

}

void opticalflow::goodFeaturesToTrackSimplified(cv::Mat& frame, std::vector<cv::Point2f>& goodPoints, goodFeatureTrackParam& gftParam){
    cv::goodFeaturesToTrack(frame,goodPoints,
                            gftParam.maxCorners,gftParam.qualityLevel,
                            gftParam.minDistance,gftParam.mask,
                            gftParam.blockSize,gftParam.useHarrisDetector,gftParam.k);
}

bool opticalflow::initOpticalFlowPyrLK(){
    if(!initFlag || mofplkin.previousGray.empty()){
        mofplkin.prev_goodpoints = computeOpticalFlowPoints();
        mofplkin.setPreviousGray(mframe);
        initFlag = true;
        return false;
    }

    else if(refreshFlag){
        mofplkin.prev_goodpoints = computeOpticalFlowPoints();
        mofplkin.setPreviousGray(mframe);
        return true;
    }

    else return true;
}

void opticalflow::runOpticalFlowPyrLKThread(cv::Mat frame){

    matchOpticalFlowPoints matchingPoints;

    setComputeFrame(frame);
    while(mopLK_thread_status){
        if(initOpticalFlowPyrLK()){
            mofplkin.next_goodpoints = computeOpticalFlowPoints();
            mofplkin.setNextGray(mframe);
            //compute optical flow using opencv optical flow Lucas Kanade Computation
            computeOpticalFlowPyrLK();
            //find matching points for optical flow pairing
            matchingPoints = matchingOpticalFlowPoints();
        }
    }

}

void opticalflow::runOpticalFlowPyrLKPanoByRegionThread(){

    while(mopLK_thread_status){
        if(!mframe.empty()){
            mdispFrame = drawingOpticalFlowNative(mframe);
        }
    }
}

matchOpticalFlowPoints opticalflow::runOpticalFlowPyrLK(cv::Mat frame){

    matchOpticalFlowPoints matchingPoints;

    setComputeFrame(frame);
        if(initOpticalFlowPyrLK()){
            mofplkin.next_goodpoints = computeOpticalFlowPoints();
            mofplkin.setNextGray(mframe);
            //compute optical flow using opencv optical flow Lucas Kanade Computation
            computeOpticalFlowPyrLK();
            //find matching points for optical flow pairing
            matchingPoints = matchingOpticalFlowPoints();
        }
        return matchingPoints;

}

cv::Mat opticalflow::drawingOpticalFlowNative(cv::Mat frame){
    matchOpticalFlowPoints matchingPoints;
    std::vector<cv::Point2f> velocityPixelXYByRegion;

    if(frame.empty()) return frame;
    matchingPoints = runOpticalFlowPyrLK(frame);

    if(matchingPoints.next_flowPoints.empty()) return frame;

    //cv::circle(frame,cv::Point(frame.cols/2,frame.rows/2),20,cv::Scalar(0,255,0),2);
    if(!frame.empty()){
        //matchingPoints = runOpticalFlowPyrLK(frame);
        for(int i=0; i<matchingPoints.prev_flowPoints.size();i++){
            //draw current points
            cv::circle(frame,matchingPoints.next_flowPoints[i],1,cv::Scalar(255,0,0));
            //draw previous points
            cv::circle(frame,matchingPoints.prev_flowPoints[i],1,cv::Scalar(0,0,255));
            cv::arrowedLine(frame,matchingPoints.prev_flowPoints[i],matchingPoints.next_flowPoints[i],cv::Scalar(255,255,0),2);
        }
        drawPanoRegion(frame);

        velocityPixelXYByRegion = calculatePanoRegionVelocity(frame,matchingPoints);
        velocityPixelXYData = velocityPixelXYByRegion;
        drawVelocityByPanoRegion(frame,velocityPixelXYByRegion);
        velocityPixelXYByRegion.clear();
    }
    elapsed_time = (std::clock() - start_time)/(double)CLOCKS_PER_SEC;
    if(elapsed_time > 0.1){
        start_time = std::clock();
        refreshOpticalFlowForNext();
    }
    else refreshFlag = false;

    return frame;
}

void opticalflow::refreshOpticalFlowForNext(){
    refreshFlag = true;
    //initFlag = false;
}

cv::Mat opticalflow::drawPanoRegion(cv::Mat frame){
    int part = 4;
    for(int i=1; i<part+1;i++)
    cv::line(frame,cv::Point(frame.cols/part*i,frame.rows),cv::Point(frame.cols/part*i,0),cv::Scalar(255,255,255),2);
    return frame;
}

cv::Mat opticalflow::drawVelocityByPanoRegion(cv::Mat frame, std::vector<cv::Point2f> velocityPixelXY){
    int part = 4;
    int spacer = frame.cols/part;
    if(!velocityPixelXY.empty() && velocityPixelXY.size() == part){
        for(int i=1; i<part+1;i++){
            //for x velocity
            if(velocityPixelXY[i-1].x < 100.0f && velocityPixelXY[i-1].x > -100.0f)
            cv::arrowedLine(frame,cv::Point2f(spacer*i - spacer/2,frame.rows/2),cv::Point2f(spacer*i - spacer/2 + velocityPixelXY[i-1].x*5,frame.rows/2),cv::Scalar(255,255,255),2);
            //for y velocity
            if(velocityPixelXY[i-1].y < 100.0f && velocityPixelXY[i-1].y > -100.0f)
            cv::arrowedLine(frame,cv::Point(spacer*i - spacer/2,frame.rows/2),cv::Point(spacer*i - spacer/2,frame.rows/2 - velocityPixelXY[i-1].y*2),cv::Scalar(255,255,255),2);
        }
    }

    return frame;
}

std::vector<cv::Point2f> opticalflow::calculatePanoRegionVelocity(cv::Mat frame , matchOpticalFlowPoints matchofpoints){
    int part = 4;
    std::vector<int> region = {0,0,0,0,0};
    std::vector<cv::Point2f> velocityPixelXYByRegion;
    velocityPixelXYByRegion.resize(part);

    for(int i=1; i<=part;i++)
        region[i] = frame.cols/part*i;

    std::vector<matchOpticalFlowPoints> matchofpointsByRegion;

    matchofpointsByRegion.resize(part);

    for(int j=0; j < part; j++){
        for (int i = 0; i < matchofpoints.prev_flowPoints.size(); i++) {

            if(matchofpoints.prev_flowPoints[i].x >= region[j] && matchofpoints.prev_flowPoints[i].x <= region[j+1])
            matchofpointsByRegion[j].prev_flowPoints.push_back(matchofpoints.prev_flowPoints[i]);

            if(matchofpoints.next_flowPoints[i].x >= region[j] && matchofpoints.next_flowPoints[i].x <= region[j+1])
            matchofpointsByRegion[j].next_flowPoints.push_back(matchofpoints.next_flowPoints[i]);
        }

        if(!matchofpointsByRegion[j].prev_flowPoints.empty() && !matchofpointsByRegion[j].next_flowPoints.empty()){
            //velocityPixelXYByRegion.push_back();
            velocityPixelXYByRegion[j] = calculateVelocityXY(matchofpointsByRegion[j]);
        }
    }

//    for(int j=0; j < part; j++)
//        if(!matchofpointsByRegion[j].prev_flowPoints.empty() && !matchofpointsByRegion[j].next_flowPoints.empty()){
//            //velocityPixelXYByRegion.push_back();
//            velocityPixelXYByRegion[j] = calculateVelocityXY(matchofpointsByRegion[j]);
//        }


    return velocityPixelXYByRegion;

}

cv::Point2f opticalflow::calculateVelocityXY(matchOpticalFlowPoints matchofpoints){
    cv::Point2f prevPointsSum;
    cv::Point2f nextPointsSum;

    prevPointsSum.x = 0.0f;
    prevPointsSum.y = 0.0f;
    nextPointsSum.x = 0.0f;
    nextPointsSum.y = 0.0f;

    float resultantVelocity;
    cv::Point2f velocityPixel;

    velocityPixel.x = 0.0f;
    velocityPixel.y = 0.0f;

    //calculate optical flow velocity
    for (int i = 0; i < matchofpoints.prev_flowPoints.size(); ++i) {
        prevPointsSum.x += matchofpoints.prev_flowPoints[i].x;
        prevPointsSum.y += matchofpoints.prev_flowPoints[i].y;

        nextPointsSum.x += matchofpoints.next_flowPoints[i].x;
        nextPointsSum.y += matchofpoints.next_flowPoints[i].y;
    }

    velocityPixel.x = ((prevPointsSum.x/matchofpoints.prev_flowPoints.size()) - (nextPointsSum.x/matchofpoints.next_flowPoints.size()));
    velocityPixel.y = ((prevPointsSum.y/matchofpoints.prev_flowPoints.size()) - (nextPointsSum.y/matchofpoints.next_flowPoints.size()));

    return velocityPixel;
}

//TODO: add matching points parameter as output
void opticalflow::runOpticalFlowPyrLKSimple(){
    runOpticalFlowPyrLK(mframe);
}


void opticalflow::startOpticalFlowPyrLK(cv::Mat frame){
    std::thread opLK_thread (&opticalflow::runOpticalFlowPyrLK, this, frame);
    mopLK_thread_status = true;
    opLK_thread.detach();
}

void opticalflow::startOpticalFlowPyrLK(){
    std::thread opLK_thread (&opticalflow::runOpticalFlowPyrLKSimple, this);
    mopLK_thread_status = true;
    opLK_thread.detach();
}

void opticalflow::startOpticalFlowPyrLKPano(){
    std::thread opLK_thread (&opticalflow::runOpticalFlowPyrLKPanoByRegionThread, this);
    mopLK_thread_status = true;
    opLK_thread.detach();
}

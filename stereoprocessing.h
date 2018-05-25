#ifndef STEREOPROCESSING_H
#define STEREOPROCESSING_H

#include "masterheader.h"
#include "stereocalibration.h"
#include <opencv2/ximgproc/edge_filter.hpp>
#include <opencv2/ximgproc/disparity_filter.hpp>

struct remappedStereo
{
    cv::Mat leftcam, rightcam;
};

class stereoprocessing
{
public:
    stereoprocessing();
    cv::Mat stereoBM(cv::Mat, cv::Mat);
    void startStereoBM();
    void runStereoBM();
    cv::Mat crudeMatMerge(cv::Mat,cv::Mat);
    cv::Mat stitchingMatMerge(cv::Mat,cv::Mat);
    void applyStereoCalibrationParameter(stereoCalibrateParam SCParam, stereoRectifyParam SRParam){ mSCParam = SCParam; mSRParam = SRParam;}
    logmessagehandler* logmessage = new logmessagehandler();
    void setSCParam(stereoCalibrateParam SCParam){mSCParam = SCParam;}
    stereoCalibrateParam getSCParam(){return mSCParam;}
    void setIntrinsicsParam(intrinsicParam intrinParam){mintrinParam = intrinParam;}
    intrinsicParam getIntrinsicsParam(){return mintrinParam;}
    void setExtrinsicsParam(extrinsicParam extrinParam){mextrinParam = extrinParam;}
    extrinsicParam getExtrinsicsParam(){return mextrinParam;}
    void setImageSize(int h,int w){mimageSize.height = h; mimageSize.width = w;}
    void setImageSize(cv::Size imageSize){mimageSize = imageSize;}
    cv::Size getImageSize(){return mimageSize;}
    std::vector<std::vector<cv::Mat>> initStereoURMfromCalibParam(intrinsicParam, extrinsicParam);
    void initStereoURMfromCalibParam();
    std::vector<std::vector<cv::Mat>> initStereoRMapSetup();
    remappedStereo stereoRemap(cv::Mat&,cv::Mat&);
    cv::Mat stereoRemapDisplay(cv::Mat,cv::Mat);
    cv::Mat stereoBMCalibrated(cv::Mat , cv::Mat );
    cv::Mat stereoDisparityFiltered(cv::Mat, cv::Mat);
    cv::Mat stereoDisparityFilteredCalibrated(cv::Mat, cv::Mat);
    std::vector<std::vector<cv::Mat>> initRMapVctor();
    cv::Mat cropImageByOffset(cv::Mat, int, int);
    void setFilteredDisparityImage(cv::Mat filteredDisparityImage){mfilteredDisparityImage = filteredDisparityImage;}
    cv::Mat getFilteredDisparityImage(){return mfilteredDisparityImage;}

    void startStereoDisparity(cv::Mat, cv::Mat);
    void runStereoDisparity(cv::Mat, cv::Mat);

    bool noOutput(){if(mfilteredDisparityImage.empty()) return true; else return false;}

private:
    stereoCalibrateParam mSCParam;
    stereoRectifyParam mSRParam;
    intrinsicParam mintrinParam;
    extrinsicParam mextrinParam;
    std::vector<std::vector<cv::Mat>> mrmap;
    cv::Size mimageSize = cv::Size(640,480);
    cv::Mat mfilteredDisparityImage;
};

#endif // STEREOPROCESSING_H

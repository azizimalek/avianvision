#include "stereoprocessing.h"

stereoprocessing::stereoprocessing()
{

}

void stereoprocessing::startStereoBM(){
    std::thread stereoBM_thread (&stereoprocessing::runStereoBM, this);
    stereoBM_thread.detach();
}

void stereoprocessing::runStereoBM(){
    //stereoprocessing::stereoBM();
}

void stereoprocessing::startStereoDisparity(cv::Mat leftImage, cv::Mat rightImage){
    std::thread stereoBM_thread (&stereoprocessing::runStereoDisparity,this,leftImage,rightImage);
    stereoBM_thread.detach();
}

void stereoprocessing::runStereoDisparity(cv::Mat leftImage, cv::Mat rightImage){
    mfilteredDisparityImage = stereoDisparityFilteredCalibrated(leftImage,rightImage);
}

cv::Mat stereoprocessing::stereoBM(cv::Mat matLeft, cv::Mat matRight){
    //-- And create the image in which we will save our disparities
    cv::Mat imgDisparity16S;
    cv::Mat imgDisparity8U;

    //-- 1. Convert image to gray

      if( !matLeft.empty() && !matRight.empty() ){
          cv::cvtColor(matLeft,matLeft,CV_BGR2GRAY);
          cv::cvtColor(matRight,matRight,CV_BGR2GRAY);

          //-- 2. Call the constructor for StereoBM
          int ndisparities = 16*9;   /**< Range of disparity */
          int SADWindowSize = 9; /**< Size of the block window. Must be odd */

          cv::Ptr<cv::StereoBM> sbm = cv::StereoBM::create();

//          sbm->setROI1(roi1);
//          sbm->setROI2(roi2);
          sbm->setPreFilterCap(31);
          sbm->setBlockSize(SADWindowSize);
          sbm->setMinDisparity(0);
          sbm->setNumDisparities(ndisparities);
          sbm->setTextureThreshold(10);
          sbm->setUniquenessRatio(15);
//          sbm->setSpeckleWindowSize(100);
//          sbm->setSpeckleRange(32);
          sbm->setDisp12MaxDiff(1);

//          //-- 3. Calculate the disparity image
//          sbm->setBlockSize(3);
//          sbm->setNumDisparities(16*2);
//          sbm->setPreFilterCap(4);
//          sbm->setMinDisparity( -64);
//          sbm->setUniquenessRatio(1);
//          sbm->setSpeckleWindowSize(150);
//          sbm->setSpeckleRange(2);
//          sbm->setDisp12MaxDiff(10);
//          sbm->setP1(600);
//          sbm->setP2(2400);
          
          cv::Ptr<cv::ximgproc::DisparityWLSFilter> wlsfilter = cv::ximgproc::createDisparityWLSFilter(sbm);
          cv::Ptr<cv::StereoMatcher> rightmatcher = cv::ximgproc::createRightMatcher(sbm);

          try{
              sbm->compute( matLeft, matRight, imgDisparity16S );
          }
          catch (std::exception e){}


          cv::normalize(imgDisparity16S, imgDisparity8U, 0, 255, CV_MINMAX, CV_8U);

      }
      else logmessage->warning("Missing a frame for pairing");

      //cv::ximgproc::FastGlobalSmootherFilter fgsf;
      //cv::Mat imgDisparity8Ufiltered;
      //fgsf.filter(imgDisparity8U,imgDisparity8Ufiltered);
      //cv::ximgproc::FastGlobalSmootherFilter::filter(imgDisparity8U,imgDisparity8Ufiltered);
      cv::Mat imgDisparity;
      if(!imgDisparity8U.empty()) cv::cvtColor(imgDisparity8U,imgDisparity, CV_GRAY2RGB);
      return imgDisparity;

}

cv::Mat stereoprocessing::stereoDisparityFiltered(cv::Mat matLeft, cv::Mat matRight){
    int offset = 50;
    matLeft = cropImageByOffset(matLeft,offset,offset);
    matRight = cropImageByOffset(matRight,offset,offset);

    //Following algorithm fetched from disparity filtering example code
    double matching_time, filtering_time;
    double lambda = 8000;
    double sigma = 1.5;
    double vis_mult = 1.0;
    cv::Mat conf_map = cv::Mat(matLeft.rows,matLeft.cols,CV_8U);
    conf_map = cv::Scalar(255);
    cv::Rect ROI;
    cv::Mat left_for_matcher;
    cv::Mat right_for_matcher;
    cv::Mat left_disp,right_disp, filtered_disp;

    matLeft.copyTo(left_for_matcher);
    matRight.copyTo(right_for_matcher);


    int ndisparities = 16*9;   /**< Range of disparity */
    int SADWindowSize = 9;      /**< Size of the block window. Must be odd */
    cv::Ptr<cv::StereoBM> left_matcher = cv::StereoBM::create(ndisparities,SADWindowSize);
//    left_matcher->setPreFilterCap(31);
//    left_matcher->setBlockSize(21);
//    left_matcher->setMinDisparity(4);
//    left_matcher->setNumDisparities(128);
//    left_matcher->setTextureThreshold(10);
//    left_matcher->setUniquenessRatio(15);
//    left_matcher->setSpeckleWindowSize(45);
//    left_matcher->setSpeckleRange(16);
//    left_matcher->setDisp12MaxDiff(1);

    cv::Ptr<cv::ximgproc::DisparityWLSFilter>  wls_filter = cv::ximgproc::createDisparityWLSFilter(left_matcher);
    cv::Ptr<cv::StereoMatcher> right_matcher = cv::ximgproc::createRightMatcher(left_matcher);

    cv::cvtColor(left_for_matcher,  left_for_matcher,  CV_BGR2GRAY);
    cv::cvtColor(right_for_matcher, right_for_matcher, CV_BGR2GRAY);

    //matching_time = (double)getTickCount();
    left_matcher-> compute(left_for_matcher, right_for_matcher,left_disp);
    right_matcher->compute(right_for_matcher,left_for_matcher, right_disp);
    //matching_time = ((double)getTickCount() - matching_time)/getTickFrequency();

    wls_filter->setLambda(lambda);
    wls_filter->setSigmaColor(sigma);
    //filtering_time = (double)getTickCount();
    wls_filter->filter(left_disp,matLeft,filtered_disp,right_disp);
    //filtering_time = ((double)getTickCount() - filtering_time)/getTickFrequency();
    conf_map = wls_filter->getConfidenceMap();

    // Get the ROI that was used in the last filter call:
    ROI = wls_filter->getROI();

    cv::Mat filtered_disp_vis;
    cv::ximgproc::getDisparityVis(filtered_disp,filtered_disp_vis,vis_mult);

    if(!filtered_disp_vis.empty()) cv::cvtColor(filtered_disp_vis,filtered_disp_vis, CV_GRAY2RGB);
    filtered_disp_vis = filtered_disp_vis(ROI);
    return filtered_disp_vis;


}

cv::Mat stereoprocessing::cropImageByOffset(cv::Mat imageToCrop, int offset_x, int offset_y){
    cv::Rect roi;
    roi.x = offset_x;
    roi.y = offset_y;
    roi.width = imageToCrop.size().width - (offset_x*2);
    roi.height = imageToCrop.size().height - (offset_y*2);

    cv::Mat croppedImage = imageToCrop(roi);

    return croppedImage;
}

std::vector<std::vector<cv::Mat>> stereoprocessing::initStereoURMfromCalibParam(intrinsicParam intrinParam, extrinsicParam extrinParam){
    std::vector<std::vector<cv::Mat>> rmap = initRMapVctor();
    cv::initUndistortRectifyMap(intrinParam.M1,intrinParam.D1, extrinParam.R1, extrinParam.P1, mimageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
    cv::initUndistortRectifyMap(intrinParam.M2,intrinParam.D2, extrinParam.R2, extrinParam.P2, mimageSize, CV_16SC2, rmap[1][0], rmap[1][1]);

    return rmap;
}

void stereoprocessing::initStereoURMfromCalibParam(){
    std::vector<std::vector<cv::Mat>> rmap = initRMapVctor();
    cv::initUndistortRectifyMap(mintrinParam.M1,mintrinParam.D1, mextrinParam.R1, mextrinParam.P1, mimageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
    cv::initUndistortRectifyMap(mintrinParam.M2,mintrinParam.D2, mextrinParam.R2, mextrinParam.P2, mimageSize, CV_16SC2, rmap[1][0], rmap[1][1]);

    mrmap = rmap;
}

std::vector<std::vector<cv::Mat>> stereoprocessing::initRMapVctor(){
   std::vector<std::vector<cv::Mat>> rmap;
   rmap.resize(2);
   rmap[0].resize(2);
   rmap[1].resize(2);
   return rmap;
}

std::vector<std::vector<cv::Mat>> stereoprocessing::initStereoRMapSetup(){

    return initStereoURMfromCalibParam(mintrinParam,mextrinParam);
}

remappedStereo stereoprocessing::stereoRemap(cv::Mat& leftcam, cv::Mat& rightcam){
    remappedStereo reStereo;
    cv::remap(leftcam, reStereo.leftcam, mrmap[0][0], mrmap[0][1], CV_INTER_LINEAR);
    cv::remap(rightcam, reStereo.rightcam, mrmap[1][0], mrmap[1][1], CV_INTER_LINEAR);
    return reStereo;
}

cv::Mat stereoprocessing::stereoRemapDisplay(cv::Mat leftcam, cv::Mat rightcam){
    remappedStereo reStereo = stereoRemap(leftcam, rightcam);
    return crudeMatMerge(reStereo.leftcam,reStereo.rightcam);
}

cv::Mat stereoprocessing::stereoBMCalibrated(cv::Mat leftcam, cv::Mat rightcam){
    remappedStereo reStereo = stereoRemap(leftcam, rightcam);
    return stereoBM(reStereo.leftcam,reStereo.rightcam);
}

cv::Mat stereoprocessing::stereoDisparityFilteredCalibrated(cv::Mat leftcam, cv::Mat rightcam){
    remappedStereo reStereo = stereoRemap(leftcam, rightcam);
    return stereoDisparityFiltered(reStereo.leftcam,reStereo.rightcam);
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

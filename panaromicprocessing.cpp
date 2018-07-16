#include "panaromicprocessing.h"

panaromicprocessing::panaromicprocessing()
{

}

remapMatrix panaromicprocessing::circlePanoramicMapping(int ws, int hs, float wd, float hd, float r1, float r2, float cx, float cy ){
 cv::Mat xmap = cv::Mat::zeros(cv::Size(hd,wd), CV_32FC1);
 cv::Mat ymap = cv::Mat::zeros(cv::Size(hd,wd), CV_32FC1);
 float r;
 float theta;
 float xs, ys;
 remapMatrix remapMat;

for (int y=0; y<hd-1; y++){
     for(int x=0; x<wd-1; x++){
         r = (y/hd)*(r2-r1)+r1;
         theta = (x/wd)*2.0*M_PI-M_PI/2;

         xs = cx + r*sin(theta);
         ys = cy + r*cos(theta);

         xmap.at<float>(y,x) = xs;
         ymap.at<float>(y,x) = ys;
     }
 }

 xmap.copyTo(remapMat.xmap);
 ymap.copyTo(remapMat.ymap);

 initPanoramicMappingFlag = true;
 return remapMat;

}



cv::Mat panaromicprocessing::unwarp(cv::Mat matImage, cv::Mat xmap, cv::Mat ymap){
    cv::Mat remappedImage;
    cv::remap(matImage,remappedImage,xmap,ymap,cv::INTER_LINEAR);
    return remappedImage;
}

cv::Mat panaromicprocessing::unwarpPanoramic(cv::Mat matImage){
    if(!matImage.empty()){
        initCirclePanoramicMapping(matImage);
        if(!mremapMat.xmap.empty() && !mremapMat.ymap.empty()){
            return unwarp(matImage,mremapMat.xmap,mremapMat.ymap);
        }
        else return matImage;
    }
    return matImage;
}

remapMatrix panaromicprocessing::initCirclePanoramicMapping(int ws,int hs){
    float r1 = ws*14.9/100;
    float r2 = ws*28.8/100;
    float cx = ws/2.0+ws/2.0*9.0/100;
    float cy = hs/2.0+ws/2.0*4.8/100;
//    float wd = 2.0*((r2+r1)/2)*M_PI;
//    float hd = (r2-r1);
    float wd = 680;
    float hd = 680;
    return circlePanoramicMapping(ws, hs, wd, hd, r1, r2, cx, cy);
}

void panaromicprocessing::initCirclePanoramicMapping(cv::Mat matImage){
    if(!initPanoramicMappingFlag)
    mremapMat = initCirclePanoramicMapping(matImage.cols,matImage.rows);
}

cv::Mat panaromicprocessing::drawReferenceLine(cv::Mat matImage){
    if(!matImage.empty()){
        float r1 = matImage.cols*14.9/100;
        float r2 = matImage.cols*28.8/100;
        float cx = matImage.cols/2.0+matImage.cols/2.0*9.0/100;
        float cy = matImage.rows/2.0+matImage.cols/2.0*4.8/100;

        cv::circle(matImage,cv::Point2d(cx,cy),1,cv::Scalar(0, 0, 255),5);
        //at 0 degree
        cv::circle(matImage,cv::Point2d(cx+r1,cy),1,cv::Scalar(255, 0, 0),5);
        cv::circle(matImage,cv::Point2d(cx+r2,cy),1,cv::Scalar(0, 255, 0),5);
        //at 180 degree
        cv::circle(matImage,cv::Point2d(cx-r1,cy),1,cv::Scalar(255, 0, 0),5);
        cv::circle(matImage,cv::Point2d(cx-r2,cy),1,cv::Scalar(0, 255, 0),5);
        //at 90 degree
        cv::circle(matImage,cv::Point2d(cx,cy+r1),1,cv::Scalar(255, 0, 0),5);
        cv::circle(matImage,cv::Point2d(cx,cy+r2),1,cv::Scalar(0, 255, 0),5);
        //at 270 degree
        cv::circle(matImage,cv::Point2d(cx,cy-r1),1,cv::Scalar(255, 0, 0),5);
        cv::circle(matImage,cv::Point2d(cx,cy-r2),1,cv::Scalar(0, 255, 0),5);

        cv::circle(matImage,cv::Point2d(cx,cy),r1,cv::Scalar(0, 255, 255));
        cv::circle(matImage,cv::Point2d(cx,cy),r2,cv::Scalar(0, 255, 255));

    }

    return matImage;

}

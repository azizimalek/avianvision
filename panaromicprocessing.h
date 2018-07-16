#ifndef PANAROMICPROCESSING_H
#define PANAROMICPROCESSING_H

#include "masterheader.h"
#include "math.h"

struct remapMatrix{
    cv::Mat xmap;
    cv::Mat ymap;
};

class panaromicprocessing
{
public:
    panaromicprocessing();

    bool initPanoramicMappingFlag = false;

    remapMatrix circlePanoramicMapping(int ws, int hs, float wd, float hd, float r1, float r2, float cx, float cy);
    cv::Mat unwarp(cv::Mat matImage, cv::Mat xmap, cv::Mat ymap);
    cv::Mat unwarpPanoramic(cv::Mat matImage);
    remapMatrix initCirclePanoramicMapping(int ws,int hs);
    void initCirclePanoramicMapping(cv::Mat matImage);
    cv::Mat drawReferenceLine(cv::Mat matImage);

    //set function
    void setPanoramaInnerRadius(float r1){mr1 = r1;}
    void setPanoramaOuterRadius(float r2){mr2 = r2;}
    void setPanoramaCenterX(float cx){mcx = cx;}
    void setPanoramaCenterY(float cy){mcy = cy;}

private:
    //panoramic dewarp mapping parameters (with its default value)
    int mws = 1280;
    int mhs = 720;
    float mwd;
    float mhd;
    float mr1 = 5;
    float mr2 = 50;
    float mcx = mws/2;
    float mcy = mhs/2;

    remapMatrix mremapMat;

};

#endif // PANAROMICPROCESSING_H

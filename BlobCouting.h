#pragma once

#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>

#include "cvBlob/cvblob.h"

enum LaneOrientation
{
    LO_NONE       = 0,
    LO_HORIZONTAL = 1,
    LO_VERTICAL   = 2
};

enum BlobPosition
{
    VP_NONE = 0,
    VP_A  = 1,
    VP_B  = 2
};

class BlobCouting
{
private:
    bool firstTime;
    bool showOutput;
    int key;
    cv::Mat img_input;
    cvb::CvTracks tracks;
    std::map<cvb::CvID, std::vector<CvPoint2D64f> > points;
    LaneOrientation laneOrientation;
    std::map<cvb::CvID, BlobPosition> positions;
    long countAB;
    long countBA;
    int img_w;
    int img_h;
    int showAB;

public:
    BlobCouting();
    ~BlobCouting();

    void setInput(const cv::Mat &i);
    void setTracks(const cvb::CvTracks &t);
    void process();

private:
    BlobPosition getBlobPosition(const CvPoint2D64f centroid);

    void saveConfig();
    void loadConfig();
};

#pragma once

#include "opencv2/core.hpp"
#include "cvBlob/cvblob.h"
#include "BlobCounter.h"

class BlobTracker {
public:
    BlobTracker(ICounter* counter);
    ~BlobTracker();

    void init(float processScale = 1.0f,
              float maxMatchDistance = 30.0f,
              uint inactiveFrame = 10,
              uint activeFrame = 0,
              float minScale = 0.005,
              float maxScale = 0.5);

    void reset();

    void process(const cv::Mat& frame);

protected:
    void blobAppear(cvb::CvTrack* blob);

    void blobTraced(cvb::CvTrack* blob);

    void blobDisappear(cvb::CvTrack* blob);

    void updateTrackers(const cvb::CvBlobs& blobs);

private:
    float _processScale;

    IplImage* _label;
    cvb::CvTracks _trackers;

    //Max distance to determine when a track and a blob match.
    double _distance;

    //Max number of frames a track can be inactive.
    uint _inactive;

    //If a track becomes inactive but it has been active less than thActive frames, the track will be deleted.
    uint _active;

    float _blobMinScale;
    float _blobMaxScale;

    unsigned int _blobNo;

    bool _isInited;
    ICounter* _counter;

};


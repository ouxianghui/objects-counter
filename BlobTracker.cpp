//#include "stdafx.h"
#include "BlobTracker.h"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

BlobTracker::BlobTracker(ICounter* counter) : _counter(counter) {
    _processScale = 1.0f;
    _distance = 30.0f;
    _inactive = 10;
    _active = 0;
    _blobMinScale = 0.005f;
    _blobMaxScale = 0.5f;
    _blobNo = 0;
    _isInited = false;
    cvReleaseTracks(_trackers);

    _label = NULL;
}


BlobTracker::~BlobTracker() {
    if (_label) {
        cvReleaseImage(&_label);
        _label = NULL;
    }
}

void BlobTracker::init(float processScale,
                        float maxMatchDistance,
                        uint inactiveFrame,
                        uint activeFrame,
                        float minScale,
                        float maxScale) {
    _processScale = processScale;
    _distance = maxMatchDistance;
    _inactive = inactiveFrame;
    _active = activeFrame;
    _blobMinScale = minScale;
    _blobMaxScale = maxScale;
    _blobNo = 0;
    _isInited = false;
    cvReleaseTracks(_trackers);
}

void BlobTracker::reset() {
    _blobNo = 0;
    _isInited = false;

    if (_label) {
        cvReleaseImage(&_label);
        _label = NULL;
    }

    cvReleaseTracks(_trackers);
}

void BlobTracker::process(const cv::Mat& frame) {
    if (!_isInited) {
        if (_label) {
            cvReleaseImage(&_label);
            _label = NULL;
        }

        _label = cvCreateImage(cvSize(frame.cols, frame.rows), IPL_DEPTH_LABEL, 1);

        _isInited = true;
    }

    /*std::vector<cv::Mat> vecSrc;
    cv::split(frame, vecSrc);
    std::vector<cv::Mat> vecTmp;
    vecTmp.push_back(vecSrc.at(0));
    vecTmp.push_back(vecSrc.at(1));
    vecTmp.push_back(vecSrc.at(2));
    cv::Mat src;
    cv::merge(vecTmp, src);

    cv::Mat fg;
    cv::cvtColor(src, fg, cv::COLOR_BGR2GRAY);
    */

    //    cv::Mat tmpFrame = frame.clone();
    //    cv::Mat headFrame;
    //    m_headExtraction.ExtractHead(tmpFrame, headFrame);
    //    //return;
    cv::Mat src;
    src = frame;
    IplImage foreground = frame;
    cv::cvtColor(frame, src, CV_GRAY2BGR);
    IplImage drawer = src;

    cvZero(_label);

    cvb::CvBlobs blobs;
    unsigned int result = cvLabel(&foreground, _label, blobs);

    //qDebug("result = %d", result);
    //    uint64 nArea = headFrame.rows * headFrame.cols;

    //	uint64 minArea = nArea*_blobMinScale;
    //    minArea = minArea >= 25 ? minArea : 25;

    //	uint64 maxArea = nArea*_blobMaxScale;

    //	cvb::cvFilterByArea(blobs, minArea, maxArea);
    cvb::cvRenderBlobs(_label, blobs, &drawer, &drawer, CV_BLOB_RENDER_BOUNDING_BOX);

    //cvShowImage("label", _label);
    //cvUpdateTracks(blobs, _trackers, 20., 5);
    updateTrackers(blobs);
    cvb::cvRenderTracks(_trackers, &drawer, &drawer, CV_TRACK_RENDER_ID | CV_TRACK_RENDER_BOUNDING_BOX);
    //printf("drawer.w = %d, drawer.h = %d\n", drawer.width, drawer.height);
    //_blobCounter.showDetectArea(src);
    //_blobCounter.showDetectLine(src);
    //_blobCounter.showDetectResult(src);
    cvShowImage("drawer", &drawer);
    cv::waitKey(50);
    cvb::cvReleaseBlobs(blobs);
}

void BlobTracker::blobAppear(cvb::CvTrack* blob) {
    _counter->blobAppear(blob);
}

void BlobTracker::blobTraced(cvb::CvTrack* blob) {
    _counter->blobTraced(blob);
}

void BlobTracker::blobDisappear(cvb::CvTrack* blob) {
    _counter->blobDisappear(blob);
}

void BlobTracker::updateTrackers(const cvb::CvBlobs& blobs) {
    CV_FUNCNAME("cvUpdateTrackers");
    __CV_BEGIN__;

    unsigned int nBlobs = blobs.size();
    unsigned int nTracks = _trackers.size();

    // Proximity matrix:
    // Last row/column is for ID/label.
    // Last-1 "/" is for accumulation.
    cvb::CvID *close = new unsigned int[(nBlobs + 2)*(nTracks + 2)]; // XXX Must be same type than CvLabel.

    try {
        // Inicialization:
        unsigned int i = 0;
        for (cvb::CvBlobs::const_iterator it = blobs.begin(); it != blobs.end(); ++it, i++) {
            //AB(i) = 0;
            close[((i)+(nTracks)*(nBlobs + 2))] = 0;
            //IB(i) = it->second->label;
            close[((i)+(nTracks + 1)*(nBlobs + 2))] = it->second->label;
        }

        //static CvID maxTrackID = 0;

        unsigned int j = 0;
        for (cvb::CvTracks::const_iterator jt = _trackers.begin(); jt != _trackers.end(); ++jt, j++) {
            //AT(j) = 0;
            close[((nBlobs)+(j)*(nBlobs + 2))] = 0;
            //IT(j) = jt->second->id;
            close[(((nBlobs)+1) + (j)*(nBlobs + 2))] = jt->second->id;
            if (jt->second->id > _blobNo)
                _blobNo = jt->second->id;
        }

        // Proximity matrix calculation and "used blob" list inicialization:
        for (i = 0; i<nBlobs; i++) {
            for (j = 0; j<nTracks; j++) {
                if (close[((i)+(j)*(nBlobs + 2))] = (cvb::distantBlobTrack(blobs.find(close[((i)+(nTracks + 1)*(nBlobs + 2))])->second, _trackers.find(close[(((nBlobs)+1) + (j)*(nBlobs + 2))])->second) < _distance)) {
                    //AB(i)++;
                    close[((i)+(nTracks)*(nBlobs + 2))]++;
                    //AT(j)++;
                    close[((nBlobs)+(j)*(nBlobs + 2))]++;
                }
            }
        }

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Detect inactive tracks
        for (j = 0; j<nTracks; j++) {
            //unsigned int c = AT(j);
            unsigned int c = close[((nBlobs)+(j)*(nBlobs + 2))];

            if (c == 0) {
                //cout << "Inactive track: " << j << endl;

                // Inactive track.
                //CvTrack *track = T(j);
                cvb::CvTrack* track = _trackers.find(close[(((nBlobs)+1) + (j)*(nBlobs + 2))])->second;

                track->inactive++;
                track->label = 0;
            }
        }

        // Detect new tracks
        for (i = 0; i<nBlobs; i++) {
            //unsigned int c = AB(i);
            unsigned int c = close[((i)+(nTracks)*(nBlobs + 2))];
            if (c == 0) {
                //cout << "Blob (new track): " << maxTrackID+1 << endl;
                //cout << *B(i) << endl;

                // New track.
                _blobNo++;
                //CvBlob *blob = B(i);
                cvb::CvBlob* blob = blobs.find(close[((i)+(nTracks + 1)*(nBlobs + 2))])->second;

                cvb::CvTrack *track = new cvb::CvTrack;
                track->id = _blobNo;
                track->label = blob->label;
                track->minx = blob->minx;
                track->miny = blob->miny;
                track->maxx = blob->maxx;
                track->maxy = blob->maxy;
                track->centroid = blob->centroid;
                track->lifetime = 0;
                track->active = 0;
                track->inactive = 0;
                _trackers.insert(cvb::CvIDTrack(_blobNo, track));

                blobAppear(track);
            }
        }

        // Clustering
        for (j = 0; j<nTracks; j++) {
            //unsigned int c = AT(j);
            unsigned int c = close[((nBlobs)+(j)*(nBlobs + 2))];

            if (c) {
                std::list<cvb::CvTrack*> tt;
                //tt.push_back(T(j));
                tt.push_back(_trackers.find(close[(((nBlobs)+1) + (j)*(nBlobs + 2))])->second);
                std::list<cvb::CvBlob*> bb;

                getClusterForTrack(j, close, nBlobs, nTracks, blobs, _trackers, bb, tt);

                // Select track
                cvb::CvTrack *track = NULL;
                unsigned int area = 0;
                for (std::list<cvb::CvTrack*>::const_iterator it = tt.begin(); it != tt.end(); ++it) {
                    cvb::CvTrack *t = *it;

                    unsigned int a = (t->maxx - t->minx)*(t->maxy - t->miny);
                    if (a > area) {
                        area = a;
                        track = t;
                    }
                }

                // Select blob
                cvb::CvBlob *blob = NULL;
                area = 0;
                //cout << "Matching blobs: ";
                for (std::list<cvb::CvBlob*>::const_iterator it = bb.begin(); it != bb.end(); ++it) {
                    cvb::CvBlob *b = *it;

                    //cout << b->label << " ";

                    if (b->area>area) {
                        area = b->area;
                        blob = b;
                    }
                }
                //cout << endl;

                if (track && blob) {
                    // Update track
                    //cout << "Matching: track=" << track->id << ", blob=" << blob->label << endl;
                    track->label = blob->label;
                    track->centroid = blob->centroid;
                    track->minx = blob->minx;
                    track->miny = blob->miny;
                    track->maxx = blob->maxx;
                    track->maxy = blob->maxy;
                    if (track->inactive) {
                        track->active = 0;
                    }
                    track->inactive = 0;

                    blobTraced(track);

                    // Others to inactive
                    for (std::list<cvb::CvTrack*>::const_iterator it = tt.begin(); it != tt.end(); ++it) {
                        cvb::CvTrack *t = *it;

                        if (t != track) {
                            //cout << "Inactive: track=" << t->id << endl;
                            t->inactive++;
                            t->label = 0;
                        }
                    }
                }
            }
        }
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        for (cvb::CvTracks::iterator jt = _trackers.begin(); jt != _trackers.end();) {
            if ((jt->second->inactive >= _inactive) || ((jt->second->inactive) && (_active) && (jt->second->active < _active))) {
                blobDisappear((cvb::CvTrack*)(jt->second));
                delete jt->second;
                _trackers.erase(jt++);
            } else {
                jt->second->lifetime++;
                if (!jt->second->inactive)
                    jt->second->active++;
                ++jt;
            }
        }
    } catch (...) {
        delete[] close;
        throw; // TODO: OpenCV style.
    }

    delete[] close;

    __CV_END__;
}

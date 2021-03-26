#ifndef DEPTHBLOBSEXTRACTER_H
#define DEPTHBLOBSEXTRACTER_H

#include "opencv2/core/core.hpp"
#include "BlobResult.h"
#include "IExtracter.h"

class DepthBlobsExtracter : public IExtracter
{
public:
    DepthBlobsExtracter(int steps = 50,
                        int minDepth = 400,
                        int maxDepth = 2000,
                        int minArea = 500,
                        int maxArea = 1500,
                        float minDensity = 0.45,
                        float maxDensity = 0.90,
                        int margin = 5);

    void extracts(const cv::Mat& src, cv::Mat& dst);

    void threshold(const cv::Mat& src, cv::Mat& dst, short min, short max);

private:
    int _step;
    int _minDepth;
    int _maxDepth;
    int _minArea;
    int _maxArea;
    float _minDensity;
    float _maxDensity;
    int _margin;
};

#endif // DEPTHBLOBSEXTRACTER_H

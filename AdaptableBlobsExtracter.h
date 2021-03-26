#ifndef AdaptableBlobsExtracter_H
#define AdaptableBlobsExtracter_H

#include "IExtracter.h"
#include "opencv2/core/core.hpp"
#include "mser3.hpp"

class AdaptableBlobsExtracter : public IExtracter
{
public:
    AdaptableBlobsExtracter(int minArea = 500, int maxArea = 1500, int margin = 5);

    void extracts(const cv::Mat& src, cv::Mat& dst);

private:
    cv::Ptr<cv::MSER3> _mser3;

    int _margin;
    int _minArea;
    int _maxArea;
};

#endif // AdaptableBlobsExtracter_H

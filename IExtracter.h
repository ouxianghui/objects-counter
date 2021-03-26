#ifndef IEXTRACTER_H
#define IEXTRACTER_H

#include "opencv2/core/core.hpp"

class IExtracter {
public:
    virtual void extracts(const cv::Mat& src, cv::Mat& dst) {}
};

#endif // IEXTRACTER_H

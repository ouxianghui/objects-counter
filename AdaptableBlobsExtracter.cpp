#include "AdaptableBlobsExtracter.h"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/photo/photo.hpp"
#include "mser2.hpp"
#include "mser3.hpp"
#include "BlobResult.h"

AdaptableBlobsExtracter::AdaptableBlobsExtracter(int minArea, int maxArea, int margin)
{
    _minArea = minArea;
    _maxArea = maxArea;
    _margin = margin;

    _mser3 = cv::MSER3::create(2, minArea, maxArea);
}

void AdaptableBlobsExtracter::extracts(const cv::Mat& src, cv::Mat& dst) {

    cv::normalize(src, src, 0, 255, cv::NORM_MINMAX);
    cv::Mat normalized = cv::Mat::zeros(src.rows, src.cols, CV_8UC1);
    cv::convertScaleAbs(src, normalized);

    cv::Mat mask = cv::Mat::zeros(src.rows, src.cols, CV_8UC1);
    cv::threshold(normalized, mask, 0, 255, cv::THRESH_BINARY_INV);

    cv::Mat paint = cv::Mat::zeros(src.rows, src.cols, CV_8UC1);
    cv::inpaint(normalized, mask, paint, 3, cv::INPAINT_TELEA);

    std::vector<std::vector<cv::Point>> regions;
    std::vector<cv::Rect> bboxes;

    //mser2->detectRegions(paint, regions, bboxes, 1);
    _mser3->detectRegions(paint, regions, bboxes);
    //dst = cv::Mat::zeros(src.rows, src.cols, CV_8UC1);
    dst.setTo(0);
//    cv::imshow("normalized", normalized);
//    cv::waitKey(10);

    //for (int i = 0; i < regions.size(); ++i) {
    //    std::vector<cv::Point>& contour = regions[i];
    //    cv::RotatedRect rrc = cv::minAreaRect(contour);
    //
    //    if (rrc.size.area() > 500 && rrc.size.area() < 2000) {
    //        cv::polylines(dst, contour, true, cv::Scalar(255), 1);
    //    }
    //}

    for (size_t i = 0; i < bboxes.size(); ++i) {
        double a = std::max(bboxes[i].width, bboxes[i].height);
        double b = std::min(bboxes[i].width, bboxes[i].height);
        double alpha = (a - b) / a;
        cv::Point pt;
        pt.x = bboxes[i].x + bboxes[i].width / 2.0;
        pt.y = bboxes[i].y + bboxes[i].height / 2.0;
        cv::Rect rc;
        rc.x = pt.x - 10;
        rc.y = pt.y - 10;
        rc.width = 20;
        rc.height = 20;
        if (bboxes[i].area() > _minArea*1.261829 &&
                bboxes[i].area() < _maxArea*1.261829 &&
                alpha < 0.3) {
            cv::rectangle(dst, rc, cv::Scalar(255), -1);
        }
    }

    CBlobResult blobResult = CBlobResult(dst, cv::Mat(), 2);

    // 过滤掉与图像上、下、左、右四个边缘相交的团块
    blobResult.Filter(blobResult,
                      FLT_EXCLUDE,
                      CBlobGetMaxY(),
                      FLT_GREATEROREQUAL,
                      mask.rows-_margin);

    blobResult.Filter(blobResult,
                      FLT_EXCLUDE,
                      CBlobGetMinY(),
                      FLT_LESSOREQUAL,
                      _margin);

    blobResult.Filter(blobResult,
                      FLT_EXCLUDE,
                      CBlobGetMinX(),
                      FLT_LESSOREQUAL,
                      _margin);

    blobResult.Filter(blobResult,
                      FLT_EXCLUDE,
                      CBlobGetMaxX(),
                      FLT_GREATEROREQUAL,
                      mask.cols-_margin);

    int histNum = blobResult.GetNumBlobs();

    //dst = cv::Mat::zeros(src.rows, src.cols, CV_8UC1);
    dst.setTo(0);
    for (int i = 0; i < histNum; ++i) {
        CBlob* histBlob = blobResult.GetBlob(i);
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Point> points = histBlob->GetExternalContour()->GetContourPoints();
        contours.push_back(points);
        cv::drawContours(dst, contours, -1, cv::Scalar(255), -1);
    }
}


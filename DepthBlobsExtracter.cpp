#include "DepthBlobsExtracter.h"
#include "opencv2/imgproc/imgproc.hpp"

DepthBlobsExtracter::DepthBlobsExtracter(int step,
                                         int minDepth,
                                         int maxDepth,
                                         int minArea,
                                         int maxArea,
                                         float minDensity,
                                         float maxDensity,
                                         int margin)
{
     _step = step;
     _minDepth = minDepth;
     _maxDepth = maxDepth;
     _minArea = minArea;
     _maxArea = maxArea;
     _minDensity = minDensity;
     _maxDensity = maxDensity;
     _margin = margin;
}

void DepthBlobsExtracter::extracts(const cv::Mat& src, cv::Mat& dst) {

    int currentDepthLower = _minDepth;
    int currentDepthUpper = currentDepthLower + _step;

    cv::Mat mask = cv::Mat::zeros(src.size(), CV_8UC1);

    CBlobResult historyLayerBlobs;
    CBlobResult currentLayerBlobs;
    //cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));
    while (currentDepthUpper < _maxDepth) {
        threshold(src, mask, _minDepth, currentDepthUpper);
        //cv::erode(mask, mask, element, cv::Point(-1, -1), 2);
        //cv::threshold(frame, mask, 0, 255, CV_THRESH_BINARY_INV);
        currentLayerBlobs = CBlobResult(mask, cv::Mat(), 2);
        cv::imshow("mask", mask);
        //cv::waitKey(0);
        //std::cout << "blobs num before filter = " << currentLayerBlobs.GetNumBlobs() << std::endl;

        // 根据面积过滤掉当前层中不符合条件的团块
        currentLayerBlobs.Filter(currentLayerBlobs,
                                 FLT_INCLUDE,
                                 CBlobGetRectArea(),
                                 FLT_INSIDE,
                                 _minArea,
                                 _maxArea);

        // 根据团块在最小圆中的像素密度过滤掉不符合条件的团块
        currentLayerBlobs.Filter(currentLayerBlobs,
                                 FLT_INCLUDE,
                                 CBlobGetMinEnclosingCircleAreaRatio(),
                                 FLT_INSIDE,
                                 _minDensity,
                                 _maxDensity);

//        int currNum = currentLayerBlobs.GetNumBlobs();
//        for (int i = 0; i < currNum; ++i) {
//            CBlob* currBlob = currentLayerBlobs.GetBlob(i);
//            cv::Rect currRect = currBlob->GetBoundingBox();
//            double a = std::max(currRect.width, currRect.height);
//            double b = std::min(currRect.width, currRect.height);
//            double alpha = (a - b) / a;
//            if (alpha > 0.3) {
//                currBlob->to_be_deleted = true;
//            }
//        }

//        // 过滤掉当前层中要被删除的团块
//        currentLayerBlobs.Filter(currentLayerBlobs,
//                                 FLT_EXCLUDE,
//                                 CBlobGetTBDeleted(),
//                                 FLT_EQUAL,
//                                 1);

        //std::cout << "blobs num after filter = " << currentLayerBlobs.GetNumBlobs() << std::endl;
        //cv::imshow("mask", mask);
        //cv::waitKey(10);

        // 当前层团块与历史层团块的匹配与筛选
        if (historyLayerBlobs.GetNumBlobs() == 0) {
            // 历史层还没有团块，就把当前层的所有团块作为历史层团块
            historyLayerBlobs = currentLayerBlobs;
        } else {
            // 历史层中有团块，匹配、筛选
            int histNum = historyLayerBlobs.GetNumBlobs();
            for (int i = 0; i < histNum; ++i) {
                CBlob* histBlob = historyLayerBlobs.GetBlob(i);
                // 历史团块还没有完成认证，则历史团块可能会被修改
                if (!histBlob->completed()) {
                    // 在当前层团块中查找与历史团块距离最近的团块
                    CBlob* currBlob = currentLayerBlobs.getBlobNearestTo(histBlob->getCenter());
                    // 如果距离最近的团块与历史团块有重叠的像素，且像素面积刚好与历史团块像素面交相等（历史团块被当前团块包含），
                    // 就用当前团块替换历史团块，替换后的历史团块完成认证
                    if (currBlob) {
                        //if (currBlob->overlappingPixels(histBlob) == histBlob->Area()) {
                        //        currBlob->setCompleted(true);
                        //        historyLayerBlobs.AddBlob(currBlob);
                        //        histBlob->to_be_deleted = true;
                        //        currBlob->to_be_deleted = true;
                        //}
                        cv::Rect currRect = currBlob->GetBoundingBox();
                        cv::Rect histRect = histBlob->GetBoundingBox();
                        cv::Rect intersect = histRect & currRect;
                        //double a = std::max(currRect.width, currRect.height);
                        //double b = std::min(currRect.width, currRect.height);
                        //double alpha = (a - b) / a;
                        if (intersect == histRect /*&& alpha <= 0.3*/) {
                            currBlob->setCompleted(true);
                            historyLayerBlobs.AddBlob(currBlob);
                            histBlob->to_be_deleted = true;
                            currBlob->to_be_deleted = true;
                        }
                    }
                }
            }

            // 过滤掉当前层中要被删除的团块
            currentLayerBlobs.Filter(currentLayerBlobs,
                                     FLT_EXCLUDE,
                                     CBlobGetTBDeleted(),
                                     FLT_EQUAL,
                                     1);

            // 过滤掉历史层中要被删除的团块
            historyLayerBlobs.Filter(historyLayerBlobs,
                                     FLT_EXCLUDE,
                                     CBlobGetTBDeleted(),
                                     FLT_EQUAL,
                                     1);
            //std::cout << "blobs num after filter = " << historyLayerBlobs.GetNumBlobs() << std::endl;

            // 当前层团块与历史层中的团块不相交，则是符合条件的团块，加入历史层团块列表
            int num = historyLayerBlobs.GetNumBlobs();
            int currNum = currentLayerBlobs.GetNumBlobs();
            for (int i = 0; i < currNum; ++i) {
                CBlob* currBlob = currentLayerBlobs.GetBlob(i);
                float currRadius;
                cv::Point2f currCenter;
                cv::minEnclosingCircle(currBlob->GetExternalContour()->GetContourPoints(),
                                       currCenter,
                                       currRadius);
                bool intersect = false;
                for (int j = 0; j < num; ++j) {
                    CBlob* histBlob = historyLayerBlobs.GetBlob(j);
                    float histRadius;
                    cv::Point2f histCenter;
                    cv::minEnclosingCircle(histBlob->GetExternalContour()->GetContourPoints(),
                                           histCenter,
                                           histRadius);

                    float deltaX = currCenter.x-histCenter.x;
                    float deltaY = currCenter.y-histCenter.y;
                    float dist = std::sqrt(deltaX*deltaX + deltaY*deltaY);
                    if (dist <= (currRadius + histRadius)) {
                        intersect = true;
                        break;
                    }
                }

                if (!intersect) {
                    historyLayerBlobs.AddBlob(currBlob);
                }
            }
        }

        currentDepthLower += _step;
        currentDepthUpper = currentDepthLower + _step;
    }

    // 过滤掉历史层中与图像上、下、左、右四个边缘相交的团块
    historyLayerBlobs.Filter(historyLayerBlobs,
                             FLT_EXCLUDE,
                             CBlobGetMaxY(),
                             FLT_GREATEROREQUAL,
                             mask.rows-_margin);

    historyLayerBlobs.Filter(historyLayerBlobs,
                             FLT_EXCLUDE,
                             CBlobGetMinY(),
                             FLT_LESSOREQUAL,
                             _margin);

    historyLayerBlobs.Filter(historyLayerBlobs,
                             FLT_EXCLUDE,
                             CBlobGetMinX(),
                             FLT_LESSOREQUAL,
                             _margin);

    historyLayerBlobs.Filter(historyLayerBlobs,
                             FLT_EXCLUDE,
                             CBlobGetMaxX(),
                             FLT_GREATEROREQUAL,
                             mask.cols-_margin);

    int histNum = historyLayerBlobs.GetNumBlobs();

    dst.setTo(0);
    for (int i = 0; i < histNum; ++i) {
        CBlob* histBlob = historyLayerBlobs.GetBlob(i);
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Point> points = histBlob->GetExternalContour()->GetContourPoints();
        contours.push_back(points);
        cv::drawContours(dst, contours, -1, cv::Scalar(255), -1);
    }

    //for (size_t i = 0; i < histNum; ++i) {
    //    cv::Point pt = historyLayerBlobs.GetBlob(i)->getCenter();
    //    //double a = std::max(bboxes[i].width, bboxes[i].height);
    //    //double b = std::min(bboxes[i].width, bboxes[i].height);
    //    //double alpha = (a - b) / a;
    //    //cv::Point pt;
    //    //pt.x = bboxes[i].x + bboxes[i].width / 2.0;
    //    //pt.y = bboxes[i].y + bboxes[i].height / 2.0;
    //    cv::Rect rc;
    //    rc.x = pt.x - 10;
    //    rc.y = pt.y - 10;
    //    rc.width = 20;
    //    rc.height = 20;
    //    //if (bboxes[i].area() > _minArea*1.261829 &&
    //    //        bboxes[i].area() < _maxArea*1.261829 &&
    //    //        alpha < 0.3) {
    //        cv::rectangle(dst, rc, cv::Scalar(255), -1);
    //    //}
    //}
}

//void DepthBlobsExtracter::extracts(const cv::Mat& src, cv::Mat& dst) {

//    int currentDepthLower = _minDepth;
//    int currentDepthUpper = currentDepthLower + _step;

//    cv::Mat mask = cv::Mat::zeros(src.size(), CV_8UC1);

//    CBlobResult historyLayerBlobs;
//    CBlobResult currentLayerBlobs;
//    //cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(11, 11));
//    while (currentDepthUpper < _maxDepth) {
//        threshold(src, mask, _minDepth, _maxDepth);
//        //cv::dilate(mask, mask, element);
//        //cv::threshold(frame, mask, 0, 255, CV_THRESH_BINARY_INV);
//        currentLayerBlobs = CBlobResult(mask, cv::Mat(), 2);
//        cv::imshow("mask", mask);
//        //std::cout << "blobs num before filter = " << currentLayerBlobs.GetNumBlobs() << std::endl;

//        // 根据面积过滤掉当前层中不符合条件的团块
//        currentLayerBlobs.Filter(currentLayerBlobs,
//                                 FLT_INCLUDE,
//                                 CBlobGetRectArea(),
//                                 FLT_INSIDE,
//                                 _minArea,
//                                 _maxArea);

////        // 根据团块在最小圆中的像素密度过滤掉不符合条件的团块
////        currentLayerBlobs.Filter(currentLayerBlobs,
////                                 FLT_INCLUDE,
////                                 CBlobGetMinEnclosingCircleAreaRatio(),
////                                 FLT_INSIDE,
////                                 _minDensity,
////                                 _maxDensity);

//        //std::cout << "blobs num after filter = " << currentLayerBlobs.GetNumBlobs() << std::endl;
//        //cv::imshow("mask", mask);
//        //cv::waitKey(10);

//        // 当前层团块与历史层团块的匹配与筛选
//        if (historyLayerBlobs.GetNumBlobs() == 0) {
//            // 历史层还没有团块，就把当前层的所有团块作为历史层团块
//            historyLayerBlobs = currentLayerBlobs;
//        } else {
//            // 历史层中有团块，匹配、筛选
//            int histNum = historyLayerBlobs.GetNumBlobs();
//            for (int i = 0; i < histNum; ++i) {
//                CBlob* histBlob = historyLayerBlobs.GetBlob(i);
//                // 历史团块还没有完成认证，则历史团块可能会被修改
//                if (!histBlob->completed()) {
//                    // 在当前层团块中查找与历史团块距离最近的团块
//                    CBlob* currBlob = currentLayerBlobs.getBlobNearestTo(histBlob->getCenter());
//                    // 如果距离最近的团块与历史团块有重叠的像素，且像素面积刚好与历史团块像素面交相等（历史团块被当前团块包含），
//                    // 就用当前团块替换历史团块，替换后的历史团块完成认证
//                    if (currBlob) {
//                        if (currBlob->overlappingPixels(histBlob) == histBlob->Area()) {
//                            currBlob->setCompleted(true);
//                            historyLayerBlobs.AddBlob(currBlob);
//                            histBlob->to_be_deleted = true;
//                            currBlob->to_be_deleted = true;
//                        }
//                        //cv::Rect currRect = currBlob->GetBoundingBox();
//                        //cv::Rect histRect = histBlob->GetBoundingBox();
//                        //cv::Rect intersect = histRect & currRect;
//                        //
//                        //if (intersect == histRect) {
//                        //    currBlob->setCompleted(true);
//                        //    historyLayerBlobs.AddBlob(currBlob);
//                        //    histBlob->to_be_deleted = true;
//                        //    currBlob->to_be_deleted = true;
//                        //}
//                    }
//                }
//            }

//            // 过滤掉当前层中要被删除的团块
//            currentLayerBlobs.Filter(currentLayerBlobs,
//                                     FLT_EXCLUDE,
//                                     CBlobGetTBDeleted(),
//                                     FLT_EQUAL,
//                                     1);

//            // 过滤掉历史层中要被删除的团块
//            historyLayerBlobs.Filter(historyLayerBlobs,
//                                     FLT_EXCLUDE,
//                                     CBlobGetTBDeleted(),
//                                     FLT_EQUAL,
//                                     1);
//            //std::cout << "blobs num after filter = " << historyLayerBlobs.GetNumBlobs() << std::endl;

//            // 当前层团块与历史层中的团块不相交，则是符合条件的团块，加入历史层团块列表
//            int num = historyLayerBlobs.GetNumBlobs();
//            int currNum = currentLayerBlobs.GetNumBlobs();
//            for (int i = 0; i < currNum; ++i) {
//                CBlob* currBlob = currentLayerBlobs.GetBlob(i);
//                float currRadius;
//                cv::Point2f currCenter;
//                cv::minEnclosingCircle(currBlob->GetExternalContour()->GetContourPoints(),
//                                       currCenter,
//                                       currRadius);
//                bool intersect = false;
//                for (int j = 0; j < num; ++j) {
//                    CBlob* histBlob = historyLayerBlobs.GetBlob(j);
//                    float histRadius;
//                    cv::Point2f histCenter;
//                    cv::minEnclosingCircle(histBlob->GetExternalContour()->GetContourPoints(),
//                                           histCenter,
//                                           histRadius);

//                    float deltaX = currCenter.x-histCenter.x;
//                    float deltaY = currCenter.y-histCenter.y;
//                    float dist = std::sqrt(deltaX*deltaX + deltaY*deltaY);
//                    if (dist <= (currRadius+histRadius)) {
//                        intersect = true;
//                        break;
//                    }
//                }

//                if (!intersect) {
//                    historyLayerBlobs.AddBlob(currBlob);
//                }
//            }
//        }

//        currentDepthLower += _step;
//        currentDepthUpper = currentDepthLower + _step;
//    }

//    // 过滤掉历史层中与图像上、下、左、右四个边缘相交的团块
//    historyLayerBlobs.Filter(historyLayerBlobs,
//                             FLT_EXCLUDE,
//                             CBlobGetMaxY(),
//                             FLT_GREATEROREQUAL,
//                             mask.rows-_margin);

//    historyLayerBlobs.Filter(historyLayerBlobs,
//                             FLT_EXCLUDE,
//                             CBlobGetMinY(),
//                             FLT_LESSOREQUAL,
//                             _margin);

//    historyLayerBlobs.Filter(historyLayerBlobs,
//                             FLT_EXCLUDE,
//                             CBlobGetMinX(),
//                             FLT_LESSOREQUAL,
//                             _margin);

//    historyLayerBlobs.Filter(historyLayerBlobs,
//                             FLT_EXCLUDE,
//                             CBlobGetMaxX(),
//                             FLT_GREATEROREQUAL,
//                             mask.cols-_margin);

//    int histNum = historyLayerBlobs.GetNumBlobs();

//    dst.setTo(0);
//    for (int i = 0; i < histNum; ++i) {
//        CBlob* histBlob = historyLayerBlobs.GetBlob(i);
//        std::vector<std::vector<cv::Point>> contours;
//        std::vector<cv::Point> points = histBlob->GetExternalContour()->GetContourPoints();
//        contours.push_back(points);
//        cv::drawContours(dst, contours, -1, cv::Scalar(255), -1);
//    }
//}

void DepthBlobsExtracter::threshold(const cv::Mat& src, cv::Mat& dst, short min, short max) {

    assert(src.type() == CV_16UC1);
    assert(dst.type() == CV_8UC1);
    assert(src.size == dst.size);

    int area = src.size().area();

    const short* sptr = src.ptr<short>();
    uchar* dptr = dst.ptr<uchar>();

    for (int i = 0; i < area; ++i) {
        if (*sptr < min || *sptr > max) {
            *dptr = 0;
        } else {
            *dptr = 255;
        }
        ++sptr;
        ++dptr;
    }
}


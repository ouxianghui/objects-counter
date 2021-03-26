/******************************************************************
** 文件名: BlobCounter.cpp
** Copyright (c) 2013-2014 上海犀视信息科技有限公司公司技术开发部
** 创建人: 成浩、王宗跃
** 日 期:2013-07-06
** 修改人: 欧湘辉
** 日 期: 2013-10-06
** 描 述: 运动物体计数类
**
** 版 本: v1.0
**-----------------------------------------------------------------------------

******************************************************************/

//#include "stdafx.h"
#include "BlobCounter.h"

template<typename T> bool isPointInRect(T x, T y, T rect_x1, T rect_y1, T rect_x2, T rect_y2) {
    if(((x < rect_x1 && x > rect_x2) || (x >rect_x1 && x < rect_x2)) &&
       ((y < rect_y1 && y > rect_y2) || (y >rect_y1 && y < rect_y2)) )
       return true;
    return false;
}

BlobCounter::BlobCounter() {
    _blobMap.clear();
    _rectInfoVec.clear();
    _iPeople = 0;
    _oPeople = 0;
}

BlobCounter::~BlobCounter() {
}

void BlobCounter::init(const cv::Size& sceneSize,
                       const RectScale& rectScale,
                       int lineStatus,
                       int ioDirection,
                       double distanceThreshold) {
    _sceneSize = sceneSize;
    _countingRectScale = rectScale;

    _lineStatus = lineStatus;

    if (_lineStatus == LINE_VERTICAL) {
        _countingLineScale.pt1Scale.y = _countingRectScale.ltScale.y;
        _countingLineScale.pt1Scale.x = (_countingRectScale.ltScale.x + _countingRectScale.rbScale.x)/2;

        _countingLineScale.pt2Scale.y = _countingRectScale.rbScale.y;
        _countingLineScale.pt2Scale.x = _countingLineScale.pt1Scale.x;
    } else {
        _countingLineScale.pt1Scale.x = _countingRectScale.ltScale.x;
        _countingLineScale.pt1Scale.y = (_countingRectScale.ltScale.y + _countingRectScale.rbScale.y)/2;

        _countingLineScale.pt2Scale.x = _countingRectScale.rbScale.x;
        _countingLineScale.pt2Scale.y = _countingLineScale.pt1Scale.y;
	}

    _distanceThreshold = distanceThreshold;

    _ioDirection = ioDirection;
}

void BlobCounter::blobAppear(cvb::CvTrack* blob) {
    if (!blob) {
        return;
    }

    BlobInfo obj;

    obj.minx = blob->minx;
    obj.maxx = blob->maxx;
    obj.miny = blob->miny;
    obj.maxy = blob->maxy;

    obj.centroid = blob->centroid;

    obj.sideOfLine = checkPointSide(obj.centroid);

    _blobMap[blob->id] = obj;
}

void BlobCounter::blobTraced(cvb::CvTrack* blob) {
    if (!blob) {
        return;
    }

    //统计跟踪到的运动物件
    countTracedBlob(blob);
}

void BlobCounter::blobDisappear(cvb::CvTrack* blob) {
    if (!blob ){
        return;
    }

    _blobMap.erase(_blobMap.find(blob->id));
}

int BlobCounter::checkPointSide(CvPoint2D64f pt) {
    bool bResult = false;

    if (_lineStatus == LINE_HORIZONTAL) {
        bResult = pt.y > (_countingLineScale.pt1Scale.y*_sceneSize.height) ? true : false;
        if (bResult) {
            return SIDE_BOTTOM;
        } else {
            return SIDE_TOP;
        }
    } else if (_lineStatus == LINE_VERTICAL) {
        bResult = pt.x > (_countingLineScale.pt1Scale.x*_sceneSize.width) ? true : false;

        if (bResult) {
            return SIDE_RIGHT;
        } else {
            return SIDE_LEFT;
        }
    }

    return SIDE_UNKNOWN;
}

/*****************************************************************
** 函数名:CountTracedBlob
** 输 入: tracedBlob
** tracedBlob---输入跟踪器指针
** 输 出: NULL
** 功能描述: 对跟踪到的运动物体进行跨线判断和计数处理
** 全局变量: NULL
** 调用模块: 本类函数及成员数据
** 作 者: WZY
** 日 期: 2013-07-08
** 修 改: OXH
** 日 期: 2014-11-06
** 修改描述: 检测线修改为检测区域，解决对象在检测线附近晃动时造成重复计数的问题
** 版本 v1.1
****************************************************************/
void BlobCounter::countTracedBlob(cvb::CvTrack* tracedBlob) {
    if (!tracedBlob) {
        return;
    }

    //新跟踪到是物件位置
   int tracedBlobSide = checkPointSide(tracedBlob->centroid);

    //同一物件当前的位置
    BlobInfo& obj = _blobMap[tracedBlob->id];
    int blobSide = obj.sideOfLine;

    cv::Point2d ltPt(_sceneSize.width*_countingRectScale.ltScale.x, _sceneSize.height*_countingRectScale.ltScale.y);
    cv::Point2d rbPt(_sceneSize.width*_countingRectScale.rbScale.x, _sceneSize.height*_countingRectScale.rbScale.y);

    //对象新状态不在检测区域内
    if (!isPointInRect<double>(tracedBlob->centroid.x, tracedBlob->centroid.y, ltPt.x, ltPt.y, rbPt.x, rbPt.y)) {
        //对象旧状态在检测区域内,有对象出检测区域
        if (obj.isInRect) {
            //判断对象运动方向,更新对象位置
            if (tracedBlobSide != SIDE_UNKNOWN && blobSide != SIDE_UNKNOWN && tracedBlobSide != blobSide) {
                if (_lineStatus == LINE_HORIZONTAL) {
                    double distance = fabs(tracedBlob->centroid.y - _countingLineScale.pt1Scale.y*_sceneSize.height);

                    if (distance >= _distanceThreshold*_sceneSize.height) {
                        //当前物件在下面，重新跟踪到时是在上面，往上，否则往下
                        if (tracedBlobSide == SIDE_TOP && blobSide == SIDE_BOTTOM) {
                            //move to top
                            blobAcrossDetectLine(BOTTOM_MOVE_TO_TOP);
                        } else if (tracedBlobSide == SIDE_BOTTOM && blobSide == SIDE_TOP) {
                            //move to bottom
                            blobAcrossDetectLine(TOP_MOVE_TO_BOTTOM);
                        }
                    }
                } else {
                    double distance = fabs(tracedBlob->centroid.x - _countingLineScale.pt1Scale.x*_sceneSize.width);

                    if (distance >= _distanceThreshold*_sceneSize.width) {
                        //当前物件在左侧，重新跟踪到时是在右侧，往右，否则往左
                        if (tracedBlobSide == SIDE_RIGHT && blobSide == SIDE_LEFT) {
                            //move to left
                            blobAcrossDetectLine(LEFT_MOVE_TO_RIGHT);
                        } else if (tracedBlobSide == SIDE_LEFT && blobSide == SIDE_RIGHT) {
                            //move to right
                            blobAcrossDetectLine(RIGHT_MOVE_TO_LEFT);
                        }
                    }
                }
            }
            obj.sideOfLine = checkPointSide(obj.centroid);
        }

        obj.isInRect = 0;
    } else { //有对象进检测区域
        //更新对象位置
        if (!obj.isInRect) {
            obj.sideOfLine = checkPointSide(obj.centroid);
        }
        obj.isInRect = 1;
    }

    //更新物件信息
    obj.minx = tracedBlob->minx;
    obj.maxx = tracedBlob->maxx;
    obj.miny = tracedBlob->miny;
    obj.maxy = tracedBlob->maxy;
    obj.centroid = tracedBlob->centroid;

//    //用于裁剪正样本的代码段
//    static int nCounter = 0;
//    cv::Mat img(pDetectFrame);

//    int x = obj.minx >= 4 ? obj.minx-4 : obj.minx;
//    int y = obj.miny >= 4 ? obj.miny-4 : obj.miny;
//    int w = x + (obj.maxx-obj.minx) + 8 > 352 ? (obj.maxx-obj.minx)+4 : (obj.maxx-obj.minx) + 8;
//    int h = y + (obj.maxy-obj.miny) + 8 > 288 ? (obj.maxy-obj.miny)+4 : (obj.maxy-obj.miny) + 8;;

//    cv::Mat roi(img, cv::Rect(x, y, w, h));
//    char path[64];
//    memset(path, 0, 64);
//    sprintf(path, "C:/Users/OXH/Desktop/new_pos_oxh/jmucv_%d.bmp", nCounter++);
//    cv::imwrite(path, roi);
}

void BlobCounter::reset() {
    _iPeople = 0;
    _oPeople = 0;

    _blobMap.clear();
    _rectInfoVec.clear();

    if (_countingLineScale.pt1Scale.x == _countingLineScale.pt2Scale.x) {
        _lineStatus = LINE_VERTICAL;
    } else {
        _lineStatus = LINE_HORIZONTAL;
    }

    _distanceThreshold = 0.0;
}

void BlobCounter::showDetectArea(cv::Mat& frame) {
    cv::Point2d ltPt(_sceneSize.width*_countingRectScale.ltScale.x, _sceneSize.height*_countingRectScale.ltScale.y);
    cv::Point2d rbPt(_sceneSize.width*_countingRectScale.rbScale.x, _sceneSize.height*_countingRectScale.rbScale.y);
	cv::Rect rect(ltPt, rbPt);
	cv::rectangle(frame, rect, cv::Scalar(0.0, 0.0, 255));
}

void BlobCounter::showDetectLine(cv::Mat& frame) {
    cv::Point2d pt1(_sceneSize.width * _countingLineScale.pt1Scale.x,
        _sceneSize.height * _countingLineScale.pt1Scale.y);

    cv::Point2d pt2(_sceneSize.width * _countingLineScale.pt2Scale.x,
        _sceneSize.height * _countingLineScale.pt2Scale.y);

	cv::line(frame, pt1, pt2, cv::Scalar(0.0, 0.0, 255));
}

void BlobCounter::showDetectResult(cv::Mat& frame) {
	//std::stringstream sstr;
    //sstr << "i = " << _iPeople << ", o = " << _oPeople;
    //cv::putText(frame, sstr.str(), cv::Point(10, 25), CV_FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 0, 0));

    //static int in = -1, out = -1;

    //if (in != _iPeople || out != _oPeople)
    {
        std::cout << "进：" << _iPeople << "  出："  << _oPeople << std::endl;
        //in = _iPeople;
        //out = _oPeople;
    }
}

void BlobCounter::blobAcrossDetectLine(int direction) {
    switch (_ioDirection) {
    case IO_DIRECTION_TOP_TO_BOTTOM: {
        if (direction == BOTTOM_MOVE_TO_TOP) {
			//onPeopleOutDetected(0);
            ++_oPeople;
        } else if (direction == TOP_MOVE_TO_BOTTOM) {
			//onPeopleInDetected(0);
            ++_iPeople;
		}
	}
	break;
    case IO_DIRECTION_BOTTOM_TO_TOP: {
        if (direction == BOTTOM_MOVE_TO_TOP) {
			//onPeopleInDetected(0);
            ++_iPeople;
        } else if (direction == TOP_MOVE_TO_BOTTOM) {
			//onPeopleOutDetected(0);
            ++_oPeople;
		}
	}
	break;
    case IO_DIRECTION_LEFT_TO_RIGHT: {
        if (direction == RIGHT_MOVE_TO_LEFT) {
			//onPeopleOutDetected(0);
            ++_oPeople;
        } else if (direction == LEFT_MOVE_TO_RIGHT) {
			//onPeopleInDetected(0);
            ++_iPeople;
		}
	}
	break;
    case IO_DIRECTION_RIGHT_TO_LEFT: {
        if (direction == RIGHT_MOVE_TO_LEFT) {
			//onPeopleInDetected(0);
            ++_iPeople;
        } else if (direction == LEFT_MOVE_TO_RIGHT) {
			//onPeopleOutDetected(0);
            ++_oPeople;
		}
	}
	break;
	default:
		break;
	}
}

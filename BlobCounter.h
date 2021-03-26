/******************************************************************
** 文件名: BlobCounter.h
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

#ifndef BLOBCOUNTER_H
#define BLOBCOUNTER_H

#include <vector>
#include <string>
#include "opencv2/core.hpp"
#include "cvBlob/cvblob.h"


//统计进出门数量定义
#define SIDE_UNKNOWN                   0	//无效位置信息
#define SIDE_TOP                       1	//上面
#define SIDE_BOTTOM                    2	//下面
#define SIDE_LEFT                      3	//左侧
#define SIDE_RIGHT                     4	//右侧

#define LINE_HORIZONTAL                1	//水平
#define LINE_VERTICAL                  2	//垂直

#define BOTTOM_MOVE_TO_TOP             1	//上面
#define TOP_MOVE_TO_BOTTOM             2	//下面
#define RIGHT_MOVE_TO_LEFT             3	//左侧
#define LEFT_MOVE_TO_RIGHT             4	//右侧

#define IO_DIRECTION_TOP_TO_BOTTOM     1	//进门方向：从上往下（相对屏幕坐标系）
#define IO_DIRECTION_BOTTOM_TO_TOP     2	//进门方向：从下往上
#define IO_DIRECTION_LEFT_TO_RIGHT     3	//进门方向：从左往右
#define IO_DIRECTION_RIGHT_TO_LEFT     4	//进门方向：从右往左

#ifndef _BLOB_INFO_
#define _BLOB_INFO_
typedef struct BlobInfo
{
    unsigned int minx;               //< X min.
    unsigned int maxx;               //< X max.
    unsigned int miny;               //< Y min.
    unsigned int maxy;               //< y max.

    CvPoint2D64f centroid;           //质心

    unsigned int sideOfLine;         //物件在检测线的哪一侧:Left/Right/Top/Bottom
    unsigned int isInRect;           //1 : in, 0 : out
} BlobInfo;
#endif

#ifndef _LINE_SCALE_
#define _LINE_SCALE_
typedef struct LineScale
{
    CvPoint2D64f pt1Scale;
    CvPoint2D64f pt2Scale;
} LineScale;
#endif

#ifndef _RECT_SCALE_
#define _RECT_SCALE_
typedef struct RectScale
{
    CvPoint2D64f ltScale;
    CvPoint2D64f rbScale;
} RectScale;
#endif

#ifndef _RECT_INFO_
#define _RECT_INFO_
typedef struct RectInfo
{
    int nID;
    std::string label;
    CvPoint2D64f ltPtScale;
    CvPoint2D64f rbPtScale;
} RectInfo;
#endif

class ICounter {
public:
    //新出现物件信号
    virtual void blobAppear(cvb::CvTrack* blob) = 0;

    //跟踪到物件信号
    virtual void blobTraced(cvb::CvTrack* blob) = 0;

    //物件消失信号
    virtual void blobDisappear(cvb::CvTrack* blob) = 0;
};

class BlobCounter : public ICounter{
public:
    explicit BlobCounter();

    ~BlobCounter();

    void init(const cv::Size& sceneSize,
              const RectScale& rectScale,
              int lineStatus,
              int ioDirection,
              double distanceThreshold = 0.0f);

    //重新初始化计数器
    void reset();

    void showDetectArea(cv::Mat& frame);
    void showDetectLine(cv::Mat& frame);
    void showDetectResult(cv::Mat& frame);

protected:
    //初始化
    //void Initialize();

    //判断点在线的哪一侧
    int checkPointSide(CvPoint2D64f pt);

    //对跟踪到的物体进行计数
    void countTracedBlob(cvb::CvTrack* tracedBlob);

    //运动物件跨线信号
    void blobAcrossDetectLine(int direction);

public:
    //新出现物件信号
    void blobAppear(cvb::CvTrack* blob);

    //跟踪到物件信号
    void blobTraced(cvb::CvTrack* blob);

    //物件消失信号
    void blobDisappear(cvb::CvTrack* blob);

private:
    std::map<int, BlobInfo> _blobMap;       //跟踪到的物件信息
    LineScale _countingLineScale;           //检测线
    RectScale _countingRectScale;           //检测区
    int _lineStatus;                        //检测线方向：1水平/2垂直
    std::vector<RectInfo> _rectInfoVec;		//检测区域信息表
    double _distanceThreshold;              //过线距离阈值
    cv::Size _sceneSize;
    int _ioDirection;

public:
    int _iPeople;
    int _oPeople;
};

#endif // BLOBCOUNTER_H

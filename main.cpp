#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/photo/photo.hpp"
//#include "opencv2/optflow
#include <iostream>
#include <sstream>
#include <unistd.h>
#include <vector>
#include "blob.h"
#include "BlobResult.h"
#include "BlobOperators.h"
#include <sys/time.h>
#include "cvBlob/cvblob.h"
#include "IExtracter.h"
#include "AdaptableBlobsExtracter.h"
#include "DepthBlobsExtracter.h"
#include "BlobTracker.h"
#include "BlobContour.h"
#include "Fitting.h"
#include "readerwriterqueue.h"
#include "spline.h"
#include "mser2.hpp"
#include "mser3.hpp"
#include "BlobCouting.h"
#include "BlobTracking.h"

#include "persistence1d.hpp"

static int fps_counter = 0;
static clock_t fps_tm = 0;

typedef struct IOArgSt {
    cv::Mat iImage;
    cv::Mat oImage;
    IExtracter* extracter;
} IOArgSt;

void TruncValue(cv::Mat &img, short min_val, short max_val) {
    assert(max_val >= min_val);
    unsigned short* ptr = img.ptr<unsigned short>();
    for (int i = img.size().area(); i != 0; i--)
    {
        if (*ptr > max_val)
        {
            *ptr = 0;
        }
        else if (*ptr < min_val)
        {
            *ptr = 0;
        }
        else
        {
            *ptr = 255;
        }
        ptr++;
    }
}

int get_fps() {
    const int kMaxCounter = 50;
    struct timeval start;
    fps_counter += 2;
    if (fps_counter < kMaxCounter) {
        return -1;
    }

    gettimeofday(&start, NULL);
    int elapse = start.tv_sec * 1000 + start.tv_usec / 1000 - fps_tm;
    int v = (int)(((float)fps_counter) / elapse * 1000);
    gettimeofday(&start, NULL);
    fps_tm = start.tv_sec * 1000 + start.tv_usec / 1000;

    fps_counter = 0;
    return v;
}

void threshold(const cv::Mat& src, cv::Mat& dst, const short min, const short max) {

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

void* threadFunc(void* arg) {
    IOArgSt* st = static_cast<IOArgSt*>(arg);

    IExtracter* extracter = static_cast<IExtracter*>(st->extracter);
    if (st && extracter) {
        extracter->extracts(st->iImage, st->oImage);
    }

    return NULL;
}

int main(int argc, char** argv) {
    if(argc != 2)
    {
        std::cout << "./pelpleCounting filename ";
        return -1;
    }
    char* filePath = argv[1];

    RectScale rectScale;
    rectScale.ltScale.x = 0.01;
    rectScale.ltScale.y = 0.3;
    rectScale.rbScale.x = 0.99;
    rectScale.rbScale.y = 0.7;

    BlobCounter counter;
    counter.init(cv::Size(320, 240), rectScale, LINE_HORIZONTAL, IO_DIRECTION_BOTTOM_TO_TOP);

    BlobTracker tracker(&counter);
    tracker.init();

    int cnt = 0;
    int times = 0;
    int total = 0;

    FILE *fp;
    fp = fopen(filePath, "r");
    if (!fp) {
        printf("file open failed\n");
        return -1;
    }

    char depth_buf[640*480*2 + 1];

    cv::Rect roi(70, 10, 560, 460);
    //cv::Rect roi(0, 0, 320, 240);

    cv::Mat frame;

    frame.create(480, 640, CV_16UC1);

    cv::VideoWriter vw;
    vw.open("/home/android/CrossCompile/DepthCounter/debug/test.avi", CV_FOURCC('M', 'J', 'P', 'G'), 25.0, cv::Size(320, 240), false);

    pthread_t tIds[2];
    IOArgSt argSt[2];

    moodycamel::BlockingReaderWriterQueue<cv::Mat> queue;

    //while (!feof(fp)/*counter < 10000*/) {
    while (cnt < 2700) {

        fread(depth_buf, 640 * 480 * 2, 1, fp);

        memcpy(frame.data, depth_buf, 640 * 480 * 2);

        cv::Mat src = cv::Mat::zeros(240, 320, CV_16UC1);

        cv::resize(frame(roi), src, src.size(), 0, 0, CV_INTER_NN);

        //vw << objs;

        queue.enqueue(src);

        ++cnt;
        std::cout << "counter = " << cnt << std::endl;
    }

    vw.release();
    fclose(fp);

    fps_tm = clock();
    fps_counter = 0;
//    AdaptableBlobsExtracter extracter1;
//    AdaptableBlobsExtracter extracter2;
    DepthBlobsExtracter extracter1(100, 10, 1000, 1000, 6000, 0.35, 0.9, 20);
    DepthBlobsExtracter extracter2(10, 10, 2500, 500, 6000, 0.25, 0.9, 10);
    argSt[0].extracter = &extracter1;
    argSt[1].extracter = &extracter2;
    argSt[0].oImage = cv::Mat::zeros(240, 320, CV_8UC1);
    argSt[1].oImage = cv::Mat::zeros(240, 320, CV_8UC1);
    int numThreads = 2;
    bool ready0 = false;
    bool ready1 = false;
    for (;;) {
        ready0 = false;
        ready1 = false;

        // Blocking with timeout
        if (queue.wait_dequeue_timed(argSt[0].iImage, 100000)) {
            ready0 = true;
        }

//        if (queue.wait_dequeue_timed(argSt[1].iImage, 100000)) {
//            ready1 = true;
//        }
        threadFunc(&argSt[0]);
        if (ready0 /*&& ready1*/) {
//            for(int i = 0; i < numThreads; i++) {
//                pthread_create(&tIds[i], NULL, threadFunc, &argSt[i]);
//            }

//            for(int i = 0; i < numThreads; i++) {
//                pthread_join(tIds[i], 0);
//            }

            tracker.process(argSt[0].oImage);
            //tracker.process(argSt[1].oImage);

            int fps = get_fps();
            if (fps > 0) {
                std::cout << "fps = " << fps << std::endl;
                ++times;
                total += fps;
                std::cout << "avg = " << total/times << std::endl;
            }
        }
    }

    return 0;
}


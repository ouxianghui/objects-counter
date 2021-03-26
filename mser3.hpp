#ifndef MSER_XXHH
#define MSER_XXHH

#include "opencv2/core.hpp"
#include "opencv2/flann/miniflann.hpp"

namespace cv
{

class MSER3
{
public:
    /** @brief Full consturctor for %MSER detector
    @param _delta it compares \f$(size_{i}-size_{i-delta})/size_{i-delta}\f$
    @param _min_area prune the area which smaller than minArea
    @param _max_area prune the area which bigger than maxArea
    @param _max_variation prune the area have simliar size to its children
    @param _min_diversity for color image, trace back to cut off mser with diversity less than min_diversity
    @param _max_evolution  for color image, the evolution steps
    @param _area_threshold for color image, the area threshold to cause re-initialize
    @param _min_margin for color image, ignore too small margin
    @param _edge_blur_size for color image, the aperture size for edge blur
     */
    static Ptr<MSER3> create( int _delta=5, int _min_area=60, int _max_area=14400,
          double _max_variation=0.25, double _min_diversity=.2,
          int _max_evolution=200, double _area_threshold=1.01,
          double _min_margin=0.003, int _edge_blur_size=5 );

    /** @brief Detect %MSER regions
    @param image input image (8UC1, 8UC3 or 8UC4, must be greater or equal than 3x3)
    @param msers resulting list of point sets
    @param bboxes resulting bounding boxes
    */
    virtual void detectRegions( InputArray image,
                                        CV_OUT std::vector<std::vector<Point> >& msers,
                                        CV_OUT std::vector<Rect>& bboxes ) = 0;

    virtual void setDelta(int delta) = 0;
    virtual int getDelta() const = 0;

    virtual void setMinArea(int minArea) = 0;
    virtual int getMinArea() const = 0;

    virtual void setMaxArea(int maxArea) = 0;
    virtual int getMaxArea() const = 0;

    virtual void setPass2Only(bool f) = 0;
    virtual bool getPass2Only() const = 0;
};

}

#endif

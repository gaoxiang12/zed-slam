/*
 * Copyright (c) 2016 xiang <email>
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#ifndef ZED_SLAM_FEATUREDETECTION_H_
#define ZED_SLAM_FEATUREDETECTION_H_
#include <opencv2/features2d/features2d.hpp>
#include "core/Frame.h"


namespace zed_slam
{

class Config;

/// Temporary container used for corner detection. Features are initialized from these.
struct Corner
{
    int x;        //!< x-coordinate of corner in the image.
    int y;        //!< y-coordinate of corner in the image.
    int level;    //!< pyramid level of the corner.
    float score;  //!< shi-tomasi score of the corner.
    float angle;  //!< for gradient-features: dominant gradient angle.
    Corner ( int x, int y, float score, int level, float angle ) :
        x ( x ), y ( y ), level ( level ), score ( score ), angle ( angle )
    {}
    
    // to opencv keypoint 
    inline cv::KeyPoint toCvKeyPoint() const 
    {
        return cv::KeyPoint( x, y, 0, angle, score, level, -1);
    }
};
typedef vector<Corner> Corners;

class FeatureDetection
{
public:
    FeatureDetection ( );
    ~FeatureDetection();

    // detect the FAST keypoints in the frame
    // ignore_previous: if true, delete all feature this frame already has 
    void detectFAST ( Frame::Ptr frame, bool ignore_previous = false ) ;

    // detect the ORB
    void detectORB ( Frame::Ptr frame );

    // compute the ORB descriptors in a frame
    void computeDescriptors ( Frame::Ptr frame );


protected:
    
    // paramters
    int         image_width_, image_height_;
    int         cell_size_;
    int         grid_n_rows_, grid_n_cols_;
    vector<bool>  grid_occupancy_;
    int         pyramid_levels_;
    double      detection_threshold_;
    double      detection_barrier_;
    int         max_features_;
};

} // namespace zed_slam

#endif // ZED_SLAM_FEATUREDETECTION_H_

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

#include <fast/fast.h>
#include <vikit/vision.h>
#include <opencv2/features2d/features2d.hpp>

#include "core/Featuredetection.h"
#include "core/Feature.h"
#include <core/Config.h>

namespace zed_slam
{

FeatureDetection::FeatureDetection ( )
{
    image_width_        = Config::get<int> ( "Camera_width" );
    image_height_       = Config::get<int> ( "Camera_height" );

    cell_size_          = Config::get<int> ( "cell_size" );
    pyramid_levels_     = Config::get<int> ( "scaleLevels" );

    grid_n_cols_ = ceil ( double ( image_width_ ) /cell_size_ );
    grid_n_rows_ = ceil ( double ( image_height_ ) /cell_size_ );
    grid_occupancy_ = vector<bool> ( grid_n_cols_ * grid_n_rows_, false );

    detection_threshold_ = Config::get<double> ( "detection_threshold" );
    detection_barrier_ = Config::get<double> ( "detection_barrier" );

    max_features_ = Config::get<int> ( "max_features" );

}

FeatureDetection::~FeatureDetection()
{

}


void FeatureDetection::detectFAST ( Frame::Ptr frame, bool ignore_previous )
{
    if ( ignore_previous == true )
    {
        frame->cleanFeatures();
    }
    // fill grid occupancy
    for ( auto f:frame->features_ )
    {
        const int k = static_cast<int> ( f.second->keypoint_.pt.y/cell_size_ ) *grid_n_cols_ +
                      static_cast<int> ( f.second->keypoint_.pt.x /cell_size_ );
        grid_occupancy_[k] = true;
    }

    // for fast corners
    Corners corners ( grid_n_cols_*grid_n_rows_, Corner ( 0, 0, detection_threshold_, 0, 0.f ) );
    for ( int L=0; L<pyramid_levels_; ++L )
    {
        const int scale = ( 1<<L );
        //cout<<"detecting scale "<<scale<<endl;
        // xy coordinates
        vector<fast::fast_xy> fast_corners;

        vector<cv::Mat> img_pyr = frame->image_pyramid_;
        // detect fast if we have sse
#if __SSE2__
        fast::fast_corner_detect_10_sse2 (
            ( fast::fast_byte * ) img_pyr[L].data, img_pyr[L].cols,
            img_pyr[L].rows, img_pyr[L].cols, detection_barrier_, fast_corners );
#elif HAVE_FAST_NEON
        fast::fast_corner_detect_9_neon (
            ( fast::fast_byte * ) img_pyr[L].data, img_pyr[L].cols,
            img_pyr[L].rows, img_pyr[L].cols, detection_barrier_, fast_corners );
#else
        fast::fast_corner_detect_10 (
            ( fast::fast_byte * ) img_pyr[L].data, img_pyr[L].cols,
            img_pyr[L].rows, img_pyr[L].cols, detection_barrier_, fast_corners );
#endif
        // scores and nonmax corners
        //cout<<"detect "<<fast_corners.size() <<endl;
        vector<int> scores, nm_corners;
        fast::fast_corner_score_10 ( ( fast::fast_byte * ) frame->image_pyramid_[L].data, frame->image_pyramid_[L].cols, fast_corners, detection_barrier_, scores );
        fast::fast_nonmax_3x3 ( fast_corners, scores, nm_corners );
        //cout<<"nm corners size = "<<nm_corners.size() <<endl;
        for ( auto it=nm_corners.begin(), ite=nm_corners.end(); it!=ite; it++ )
        {
            fast::fast_xy &xy = fast_corners.at ( *it );
            const int k = static_cast<int> ( ( xy.y*scale ) /cell_size_ ) *grid_n_cols_ +
                          static_cast<int> ( ( xy.x*scale ) /cell_size_ );
            //cout<<"x="<<xy.x<<","<<xy.y<<endl;
            //cout<<"k="<<k<<", and grid occupancy size="<<grid_occupancy_.size() <<endl;
            if ( grid_occupancy_[k] )
            {
                continue;    //该点已经计算过
            }

            const float score = vk::shiTomasiScore ( frame->image_pyramid_[L], xy.x, xy.y );
            // 选出该区域score最大的角点
            if ( score > corners[k].score )
            {
                // corner的x,y是真实值，乘过scale之后的。
                // TODO 但是金字塔顶层的scale比较高，精度不够, 需要refinement
                corners[k] = Corner ( xy.x*scale, xy.y*scale, score, L, 0.0f );
            }
        }
    }

    // 对于足够高分的corner，设为feature
    int number_features = frame->features_.size();
    cout<<number_features<<endl;
    int id=0;
    std::for_each ( corners.begin(), corners.end(), [&] ( Corner& c )
    {
        if ( number_features > max_features_ )
            return;
        // check the boarder 
        if ( c.x<5 || c.y<5 || c.x>frame->color_.cols-5 || c.y>frame->color_.rows-5 )
            return; 
        if ( c.score > detection_threshold_ )
        {
            // 用cv::Keypoint来表示feature,之后会方便一些
            shared_ptr<zed_slam::Feature> f ( new zed_slam::Feature() );
            f->id_ = id;
            f->frame_ = frame.get();
            f->keypoint_ = c.toCvKeyPoint();
            frame->features_.insert ( {id++,f} );
            number_features++;
        }
    } );

    frame->assignFeatureDepthAndConfidence();

    // reset the grid occupancy
    std::fill ( grid_occupancy_.begin(), grid_occupancy_.end(), false );
}

void FeatureDetection::detectORB ( Frame::Ptr frame )
{

}

void FeatureDetection::computeDescriptors ( Frame::Ptr frame )
{

}


}// namespace zed_slam

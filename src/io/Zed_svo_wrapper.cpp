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

#include <vikit/pinhole_camera.h>
#include <zed/Camera.hpp> 
#include "core/Config.h"
#include "io/Zed_svo_wrapper.h"

namespace zed_slam 
{
    
Zed_svo_wrapper::Zed_svo_wrapper( const string& filename )
: index_(0) 
{
    cout<<"reading svo file "<<filename<<endl;
    zed_ = new sl::zed::Camera( filename.c_str() );
    sl::zed::ERRCODE err = zed_->init( sl::zed::MODE::PERFORMANCE, 0, false );
    cout<<sl::zed::errcode2str(err)<<endl;
    
    zed_->setSVOPosition( 0 );
    number_of_frames_ = zed_->getSVONumberOfFrames();
    
    cout<<"this svo file contains "<<number_of_frames_<<" frames."<<endl;
    
    cout<<"image width, height = "<<zed_->getImageSize().width<<","<<zed_->getImageSize().height<<endl;
    
    // in the case of using zed wrapper, we use zed self calibration intrinsics rather than the config file!
    sl::zed::StereoParameters* parameters = zed_->getParameters();
    sl::zed::resolution resolution = zed_->getImageSize();
    camera_ = new vk::PinholeCamera(
        resolution.width, 
        resolution.height,
        parameters->LeftCam.fx, 
        parameters->LeftCam.fy, 
        parameters->LeftCam.cx, 
        parameters->LeftCam.cy, 
        0,0,0,0,0
//        parameters->LeftCam.disto[0],
//        parameters->LeftCam.disto[1],
//        parameters->LeftCam.disto[2],
//        parameters->LeftCam.disto[3],
//        parameters->LeftCam.disto[4]
    );
}

Zed_svo_wrapper::~Zed_svo_wrapper()
{
    delete zed_;
}

Frame::Ptr Zed_svo_wrapper::next()
{
    if ( index_ >= number_of_frames_ )
        return nullptr;
    zed_->grab( sl::zed::SENSING_MODE::RAW );
    
    cv::Mat color, depth, confidence;
    slMat2cvMat( zed_->retrieveImage( sl::zed::LEFT )).copyTo( color );
    
    // don't use normalizeMeasure but use retrieve measure to get the raw depth  
    // also notice ZED set depth to -999 if this depth cannot be computed
    slMat2cvMat( zed_->retrieveMeasure( sl::zed::MEASURE::DEPTH )).copyTo( depth );
    slMat2cvMat( zed_->normalizeMeasure( sl::zed::MEASURE::CONFIDENCE )).copyTo( confidence );
    // in a svo file we don't have a time stamp so replace it with index
    Frame::Ptr pFrame ( new Frame( camera_, color, depth, confidence, index_, index_ ) );
    index_++;
    
    // init this frame to create the image pyramid. 
    pFrame->init();
    return pFrame;
}

} //namespace zed_slam


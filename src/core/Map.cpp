#include <g2o/types/sba/types_six_dof_expmap.h>

#include "core/Map.h"
#include "core/Feature.h"
#include "core/Config.h"
#include "utils/SLAMTools.h"


namespace zed_slam {
    
LocalMap::LocalMap():
orb_( new cv::ORB() )
{
    distance_key_frame_ = Config::get<double>("distance_keyframe");
}

    
int LocalMap::insertKeyFrame ( Frame::Ptr frame )
{
    // we assert this frame is already checked as a key-frame 
    key_frames_.push_back( frame );
    
    // compute this frame's descriptors 
    computeKeyframeDescriptor( frame );
    
    // add this frame into key-frames and set the map points 
    key_frames_.push_back( frame );
    
    addMapPoints( frame );
    
    // set the ref frame to this frame 
    ref_key_frame_ = frame;

    return 0;
}

    
void LocalMap::computeKeyframeDescriptor( Frame::Ptr frame )
{
    vector<cv::KeyPoint> kps;
    std::for_each( frame->features_.begin(), frame->features_.end(), [&](Feature* feature) 
    {
        kps.push_back( feature->keypoint_ );
    });
    cv::Mat descriptors;
    (*orb_) ( frame->color_, cv::Mat(), kps, descriptors, true );
}

void LocalMap::addMapPoints ( Frame::Ptr frame )
{
    for ( Feature* feature: frame->features_ )
    {
        shared_ptr<MapPoint> map_point = MapPoint::createMapPoint(); 
        map_point->pos_ = feature->position_;
        map_points_.push_back( map_point );
    }
}

bool LocalMap::checkKeyFrame ( Frame::Ptr frame )
{
    Eigen::Isometry3d delta = ref_key_frame_->T_f_w_.inverse()*frame->T_f_w_;
    g2o::SE3Quat q = SLAMTools::toSE3Quat( delta ) ;
    auto d = q.log();
    
    cout<<"delta ="<<d<<endl;
    double d_norm = 0;
    for ( int i=0; i<6; i++ )
        d_norm += d(i)*d(i);
    
    cout<<"delta norm="<<d_norm<<endl;
    if ( d_norm > distance_key_frame_ )
        return true;
    return false;
}


    
    
    
}// namespace zed_slam
#ifndef ZED_SLAM_MAP_H_
#define ZED_SLAM_MAP_H_

#include <list>

#include <opencv2/features2d/features2d.hpp>

using namespace std;

#include "core/Frame.h"
#include "core/Mappoint.h"

namespace zed_slam
{

class Map
{
public:
};

class LocalMap 
{
protected:
    shared_ptr<cv::ORB>         orb_;   // for ORB descriptor
    list<shared_ptr<Frame>>     key_frames_;    // key frames in local map 
    list<shared_ptr<MapPoint>>  map_points_;    // map points in local map 
    
    Frame::Ptr  ref_key_frame_;         // the referece key-frame, used to check new key-frames
    
    double distance_key_frame_;        // distance threshold for selecting key frames
public:
    
    LocalMap(); 
    
    // check whether this frame can be a new key-frame 
    // currently use distance in estimated pose to reference key-frame 
    // TODO think better strategy of selecting key-frames 
    bool checkKeyFrame( Frame::Ptr frame );
    
    // add a keyframe, will compute the features' descriptors 
    int insertKeyFrame( Frame::Ptr frame );
    
    // optimize the local bundle adjustment
    void optimize();
    
    // compute the descriptors of features 
    void computeKeyframeDescriptor( Frame::Ptr frame );
    
    // set the map point into local map 
    void addMapPoints( Frame::Ptr frame );
    
    list<shared_ptr<MapPoint>> getMapPoints() { return map_points_; }
public:
    
};


}; // namespace zed_slam

#endif //ZED_SLAM_MAP_H_

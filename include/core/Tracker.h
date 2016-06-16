#ifndef ZED_SLAM_TRACK_H_
#define ZED_SLAM_TRACK_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/concept_check.hpp>

#include "core/Frame.h"

namespace zed_slam {
    
/****************************************
 * The tracker compute features in images and track them using optical flow to 
 * compute the pose of the camera and feature points, which is inspired by SVO.
 * But in order to relocalize and perform loop closure we need feature descriptors (ORB),
 * which will be computed only in key-frames, not in ordinary frames like ORB-SLAM does.
 * 
 */
class Tracker 
{
public:
    enum TrackState {
        NOT_READY,
        GOOD,
        LOST
    };
    
    
public:
    TrackState                          state_;         // current state of the tracker.
    Frame::Ptr                          current_frame_; // current frame
    Frame::Ptr                          last_frame_;    // last frame
    
public:
    
    Tracker();
    
    // init function 
    void init();
    
    // track a new frame and return its pose in the world 
    Eigen::Isometry3d trackImage( Frame::Ptr frame );   
    
    
public:
    // LK flow to track the FAST features in the first frame and 
    // compute their pixel coordinates in the second frame
    int trackKLT( Frame::Ptr first, Frame::Ptr second );
    
    // Pose estimation between last_frame_ and current_frame_ 
    // use sparse feature points to estimate the pose under g2o framework 
    int poseEstimationTwoFrames( Frame::Ptr first, Frame::Ptr second ); 
    
    // direct pose estimation
    int poseEstimationTwoFramesDirect( Frame::Ptr first, Frame::Ptr second );
    
    // semi-direct pose estimation
    // use image patches around the feature points 
    int poseEstimationTwoFramesSemiDirect( Frame::Ptr first, Frame::Ptr second );
    
    // use g2o to estimate the semi-direct and direct pose 
    int poseEstimationTwoFramesSemiDirectUsingG2O( Frame::Ptr first, Frame::Ptr second );
    int poseEstimationTwoFramesDirectUsingG2o ( Frame::Ptr first, Frame::Ptr second );
protected:
    // parameters 
    // KLT 
    int klt_max_iter_;
    double klt_eps_;
    int klt_window_size_;
    
    double epsilon_direct_;
    int confidence_threshold_;
};
}





#endif// ZED_SLAM_TRACK_H_
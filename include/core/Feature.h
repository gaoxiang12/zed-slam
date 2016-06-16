#ifndef ZED_SLAM_FEATURE_H_
#define ZED_SLAM_FEATURE_H_
#include <opencv2/features2d/features2d.hpp>
#include "core/Frame.h"

namespace zed_slam
{

class Feature
{
public:
    Feature();
    Feature ( Frame* frame, cv::KeyPoint& keypoint, float depth=-1, float confidence=-1 );
    Feature( const Feature& feature );
    Frame*              frame_;        // the associated frame
    cv::KeyPoint        keypoint_;     // the position in 2d image, pyramid layer and score are also saved in this kp
    float               depth_;        //
    float               confidence_;   // confidence from zed confidence map

    Eigen::Vector3d     position_;     // position in 3D world
    int                 quality_;      // quality, zero means this feature is a new one

public:
    
    // chech whether this feature is inside the observed image
    bool isInsideFrameImage( int boarder=0 );
    
    Eigen::Vector2d getPixelPosition( bool project=false );
    // get this feature's pixel value on the reference frame 
    uchar getPixelValue( bool project_to_frame = false );
    
    // compute the zero order (nearest interpolation) pixel differential 
    Eigen::Vector2d getZeroOrderJacobian( bool project_to_frame=true );
    
    // compute the pixel value and bilinear interpolation weights
    // if set project to frame = true, we use the projected position_. Otherwise use keypoint_.pt as the pixel position
    float getBilinearPixelValue ( BiLinearWeights& weights, Eigen::Matrix<double,1,2>& jacob, bool project_to_frame=false );

    // compute the jacobian from pixel to frame pose (in Lie alegbra )
    // Note: we put the intrinsics into this jacobian
    Eigen::Matrix<double,2,6> getJacobPixel2se3();
};

};


#endif // ZED_SLAM_FEATURE_H_

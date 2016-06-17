#ifndef ZED_SLAM_FRAME_H_
#define ZED_SLAM_FRAME_H_

#include <memory>
#include <string>
#include <list>
#include <unordered_map>

#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>

#include "vikit/pinhole_camera.h"

using namespace std; 
namespace zed_slam
{
    
class Feature;

struct BiLinearWeights
{
    float topLeft_ =0, topRight_ =0, bottomLeft_ =0, bottomRight_ =0;
};

/**
 * \brief This is an ordinary frame in zed slam. 
 * Keyframes are defined as Keyframe
 */
class Frame
{
public:
    typedef std::shared_ptr<Frame> Ptr;
    typedef std::unordered_map<int, shared_ptr<Feature>> Features;
    
    int                            id_;         // id of this frame
    double                         time_stamp_; // when it is recorded
    Eigen::Isometry3d              T_f_w_;      //transform from frame to world
    Features                       features_;   // features
    vk::PinholeCamera*             camera_;     // the camera model
    cv::Mat                        color_, depth_, confidence_; // three images extracted from zed 
    vector< cv::Mat >              image_pyramid_; // the pyramid
    double                         depth_scale_;// the scale of depth measurements
    
public:
    // cosntructor
    Frame( vk::PinholeCamera* camera, cv::Mat& img, cv::Mat& depth, cv::Mat& confidence, int id, double timestamp);
    
    // copy constructor
    Frame( const Frame& frame );
    ~Frame();
    
    // init the frame
    void init(); 
    
    // transform functions, note that we use three coordinates: pixel (2d), camera (3d) and the world (3d).
    // world to pixel 
    inline Eigen::Vector2d world2pixel ( const Eigen::Vector3d& point_world ) const 
    {
        return camera_->world2cam( T_f_w_ * point_world );
    }
    
    // pixel to world 
    Eigen::Vector3d pixel2world( const Eigen::Vector3d& pixel );
    
    // pixel to world, given 2d pixel coordinates
    Eigen::Vector3d pixel2world( const Eigen::Vector2d& pixel );
    
    // world to camera 
    inline Eigen::Vector3d world2camera( const Eigen::Vector3d& point_world ) const
    {
        return T_f_w_ * point_world;
    }
    
    // camera to world  
    inline Eigen::Vector3d camera2world( const Eigen::Vector3d& point_camera ) const
    {
        return T_f_w_.inverse()*point_camera; 
    }

    
    // the camera center in world 
    inline Eigen::Vector3d getCameraCenter() const 
    {
        return T_f_w_.inverse().translation();
    }
    
    // get all keypoints pixel positions into a vector 
    vector<cv::Point2f> getKeypointPixelPosition();
    
    // set the depth and confidence of the features 
    void assignFeatureDepthAndConfidence() ;
    
    // clean features 
    void cleanFeatures();
    
    // check whether a pixel is in the image 
    bool inFrame ( const Eigen::Vector2d pixel, int boarder=0 ) const; 
    
    // get the bilinear interpolated grayscale value (does not check if the pixel is inside the image so please check it first )
    float getBiLinearPixelValue( const Eigen::Vector2d& pixel, BiLinearWeights& weights );
    float getBiLinearPixelValue( const Eigen::Vector2d& pixel )
    {
        BiLinearWeights w;
        return getBiLinearPixelValue(pixel, w);
    }
    
    // compute the jacobian from (u,v) to Lie algebra 
    // input: a point in the world
    Eigen::Matrix<double, 2, 6> getJacobianUV2SE3( const Eigen::Vector3d& point ) const ;
    
    // compute the jacobian from pixel value to pixel position (u,v)
    Eigen::Matrix<double, 1, 2> getJacobianPixel2UV( const Eigen::Vector2d& pixel ) ; 
    
protected:
    // create the image pyramid 
    void createImagePyramid( int scale_levels, float scale_factor ); 
    
}; // class Frame

    

};


#endif // ZED_SLAM_FRAME_H_

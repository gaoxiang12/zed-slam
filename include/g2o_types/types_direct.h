#ifndef ZED_SLAM_TYPES_DIRECT_
#define ZED_SLAM_TYPES_DIRECT_

#include <Eigen/Geometry>
#include <g2o/core/base_unary_edge.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <opencv2/core/core.hpp>

namespace g2o
{

// project a 3d point into an image plane, the error is photometric error 
class EdgeSE3ProjectDirect: public BaseUnaryEdge< 1, double, VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    EdgeSE3ProjectDirect() {}
    
    EdgeSE3ProjectDirect( Eigen::Vector3d point, float fx, float fy, float cx, float cy, cv::Mat* image, int step=2 )
    : x_world_(point), fx_(fx), fy_(fy), cx_(cx), cy_(cy), image_(image), step_(step)
    {}
    
    virtual void computeError(); 
    
    // plus in manifold 
    virtual void linearizeOplus( ); 
    
    float getPixelValue( float x, float y ); 
    
    // dummy read and write functions because we don't care... 
    virtual bool read( std::istream& in );
    virtual bool write( std::ostream& out ) const ;
    
public:
    // 3d point in world frame 
    Eigen::Vector3d x_world_; 
    // Pinhole camera intrinsics 
    float cx_=0, cy_=0, fx_=0, fy_=0; 
    // the reference image 
    cv::Mat* image_=nullptr;
    int step_; 
    
    Eigen::Matrix<double,1,6> jacobian;

};

}




#endif // ZED_SLAM_TYPES_DIRECT_

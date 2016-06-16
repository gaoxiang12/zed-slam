#include <algorithm>
#include <vikit/vision.h>

#include "core/Frame.h"
#include "core/Config.h"
#include "core/Feature.h"

using namespace std;
namespace zed_slam
{

Frame::Frame ( vk::PinholeCamera* camera, cv::Mat& img, cv::Mat& depth, cv::Mat& confidence, int id, double timestamp ) :
    id_ ( id ), time_stamp_ ( timestamp ), camera_ ( camera )
{
    // check the type of images
    // assume the color is gray scale image
    if ( img.type() == CV_8UC4 )
    {
        //convert color image into gray image
        cv::cvtColor ( img, color_, CV_RGBA2GRAY );
    }
    else
        color_ = img;
    
    
    // color_ should be a CV_8UC1 grayscale image
    assert( color_.type() == CV_8UC1 );

    // ZED gives CV_32F for depth and 8UC4 for normalized confidence
    assert ( depth.type() == CV_16UC1 || depth.type() == CV_32F );
    assert ( confidence.type() == CV_8UC4 );

    depth_ = depth;
    confidence_ = confidence;

    depth_scale_ = Config::getInstance()->getValue<double> ( "DepthMapFactor" );
    T_f_w_ = Eigen::Isometry3d::Identity();
}

Frame::~Frame()
{
    std::for_each ( features_.begin(), features_.end(), [&] ( Feature* f )
    {
        delete f;
    } );
}

void Frame::init()
{
    createImagePyramid (
        Config::getInstance()->getValue<int> ( "scaleLevels" ),
        Config::getInstance()->getValue<float> ( "scaleFactor" )
    );
}

// although there is a scale factor but we now use half sample here!
// TODO change the half sample with the scale factor
void Frame::createImagePyramid ( int scale_levels, float scale_factor )
{
    image_pyramid_.resize ( scale_levels );
    image_pyramid_[0] = color_;
    for ( size_t i=1; i<scale_levels; ++i )
    {
        image_pyramid_[i] = cv::Mat ( image_pyramid_[i-1].rows/2,  image_pyramid_[i-1].cols/2, CV_8UC1 );
        vk::halfSample ( image_pyramid_[i-1], image_pyramid_[i] );
    }
}

vector< cv::Point2f > Frame::getKeypointPixelPosition()
{
    vector<cv::Point2f> pixels;
    std::for_each ( features_.begin(), features_.end(), [&] ( Feature* f )
    {
        pixels.push_back ( f->keypoint_.pt );
    }
                  );
    return pixels;
}

void Frame::assignFeatureDepthAndConfidence()
{
    std::for_each ( features_.begin(), features_.end(), [&] ( Feature* f )
    {
        // it is a bit nasty to get a float from uchar arrays :D
        if ( f->quality_ > 0 )
            return;
        f->depth_ = * ( float* ) & depth_.data [ int ( f->keypoint_.pt.y * depth_.step + f->keypoint_.pt.x*4 ) ];
        f->confidence_ = ( int ) confidence_.data[ int ( f->keypoint_.pt.y * confidence_.step + f->keypoint_.pt.x * 4 ) ];

        f->position_ = this->pixel2world (
                           Eigen::Vector3d (
                               f->keypoint_.pt.x, f->keypoint_.pt.y, f->depth_
                           ) );
    } );
}

Eigen::Vector3d Frame::pixel2world ( const Eigen::Vector3d& pixel )
{
    return T_f_w_.inverse() * Eigen::Vector3d (
               ( pixel[0]-camera_->cx() ) * pixel[2] / ( depth_scale_*camera_->fx() ),
               ( pixel[1]-camera_->cy() ) * pixel[2] / ( depth_scale_*camera_->fy() ),
               pixel[2]/depth_scale_
           );
}

Eigen::Vector3d Frame::pixel2world ( const Eigen::Vector2d& pixel )
{
    float depth = ( * ( float* ) & depth_.data [ int ( pixel[1] * depth_.step + pixel[0]*4 ) ] ) / depth_scale_;
    //cout<<"depth="<<depth<<endl;
    return T_f_w_.inverse() * Eigen::Vector3d (
               ( pixel[0]-camera_->cx() ) * depth / ( camera_->fx() ),
               ( pixel[1]-camera_->cy() ) * depth / ( camera_->fy() ),
               depth
           );
}

void Frame::cleanFeatures()
{

    std::for_each ( features_.begin(), features_.end(), [&] ( Feature* f )
    {
        delete f;
    });
    features_.clear();
}

bool Frame::inFrame ( const Eigen::Vector2d pixel, int boarder ) const
{
    return (pixel[0]-boarder) >= 0 && (pixel[0]+boarder)<color_.cols && (pixel[1]-boarder)>=0 && (pixel[1]+boarder)<color_.rows;
}

float Frame::getBiLinearPixelValue ( const Eigen::Vector2d& pixel, BiLinearWeights& weights )
{
    int x_small, y_small;
    float x = pixel[0], y = pixel[1];
    x_small = floor ( pixel[0] );
    y_small = floor ( pixel[1] );
    
    x = x-x_small;
    y = y-y_small;

    weights.topLeft_ = ( 1-x ) * ( 1-y );
    weights.topRight_ = x* ( 1-y );
    weights.bottomLeft_ = ( 1-x ) *y;
    weights.bottomRight_ = x*y;

    uchar* data_ptr = &color_.data[ y_small*color_.step + x_small ];
    
    int step = color_.step;
    uchar f00 = data_ptr[0];
    uchar f10 = data_ptr[1];
    uchar f01 = data_ptr[step];
    uchar f11 = data_ptr[step+1];

    return weights.topLeft_ * f00 + weights.topRight_ * f10
                    + weights.bottomLeft_ * f01
                    + weights.bottomRight_* f11;
}

Eigen::Matrix< double, 2, 6 > Frame::getJacobianUV2SE3 ( const Eigen::Vector3d& point ) const
{
    Eigen::Vector3d point_local = this->world2camera( point );
    double x = point_local[0], y=point_local[1], z=point_local[2];
    // don't ask me why z is in this form!
    Eigen::Matrix<double, 2, 6> jacob;
    float fx = camera_->fx(), fy = camera_->fy();
    
    jacob ( 0,0 ) = fx / z;
    jacob ( 0,1 ) = 0;
    jacob ( 0,2 ) = - fx * x / ( z*z );
    jacob ( 0,3 ) = - fx *x*y/ ( z*z );
    jacob ( 0,4 ) = fx+fx*x*x/ ( z*z ) ;
    jacob ( 0,5 ) = -fx*y/z;

    jacob ( 1,0 ) = 0;
    jacob ( 1,1 ) = fy/z;
    jacob ( 1,2 ) = -fy*y/ ( z*z );
    jacob ( 1,3 ) = -fy-fy*y*y/ ( z*z );
    jacob ( 1,4 ) = fy*x*y/ ( z*z );
    jacob ( 1,5 ) = fy*x/z;
    
    return jacob;
}

Eigen::Matrix< double, 1, 2 > Frame::getJacobianPixel2UV ( const Eigen::Vector2d& pixel ) 
{
    Eigen::Matrix<double, 1, 2> jacob;
    BiLinearWeights weights; 
    float p00 = getBiLinearPixelValue( pixel, weights );
    float p10 = getBiLinearPixelValue( pixel+Eigen::Vector2d(2,0), weights );
    float p01 = getBiLinearPixelValue( pixel+Eigen::Vector2d(0,2), weights );
    jacob (0,0) = 0.5*(p10-p00);
    jacob (0,1) = 0.5*(p01-p00);
    return jacob;
}




}; // namespace zed_slam

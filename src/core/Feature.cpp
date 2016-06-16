
#include "core/Feature.h"

namespace zed_slam
{

Feature::Feature()
    : depth_ ( -1.0f ), confidence_ ( -1.0f ), quality_ ( 0 )
{

}

Feature::Feature ( Frame* frame, cv::KeyPoint& keypoint, float depth, float confidence )
    : frame_ ( frame ), keypoint_ ( keypoint ), depth_ ( depth ), confidence_ ( confidence ), quality_ ( 0 )
{

}

Feature::Feature ( const Feature& feature )
    : frame_ ( feature.frame_ ), keypoint_ ( feature.keypoint_ ), depth_ ( feature.depth_ ), confidence_ ( feature.confidence_ ),
      quality_ ( feature.quality_ ), position_ ( feature.position_ )
{

}

Eigen::Vector2d Feature::getPixelPosition ( bool project )
{
    if ( project )
    {
        return frame_->world2pixel( position_ );
    }
    return Eigen::Vector2d( keypoint_.pt.x, keypoint_.pt.y );
}

bool Feature::isInsideFrameImage( int boarder )
{
    Eigen::Vector2d pixel = frame_->world2pixel ( position_ );
    return frame_->inFrame( pixel, boarder );
    //return frame_->camera_->isInFrame ( pixel.cast<int>(), true );
}

uchar Feature::getPixelValue ( bool project_to_frame )
{
    int x,y;
    if ( project_to_frame )
    {
        Eigen::Vector2d pixel = frame_->world2pixel ( position_ );
        x = round ( pixel[0] );
        y = round ( pixel[1] );
    }
    else
    {
        x = round ( keypoint_.pt.x );
        y = round ( keypoint_.pt.y );
    }
    if ( !frame_->camera_->isInFrame ( Eigen::Vector2i ( x,y ), true ) )
        return 0;
    return frame_->color_.data[ y*frame_->color_.step + x*4 ];
}

Eigen::Vector2d Feature::getZeroOrderJacobian ( bool project_to_frame )
{
    int x,y;
    if ( project_to_frame )
    {
        Eigen::Vector2d pixel = frame_->world2pixel ( position_ );
        x = round ( pixel[0] );
        y = round ( pixel[1] );
    }
    else
    {
        x = round ( keypoint_.pt.x );
        y = round ( keypoint_.pt.y );
    }
    if ( !frame_->camera_->isInFrame ( Eigen::Vector2i ( x,y ), true ) )
        return Eigen::Vector2d ( 0,0 );
    return Eigen::Vector2d (
               frame_->color_.data[ y*frame_->color_.step + ( x+1 ) *4 ] - frame_->color_.data[ y*frame_->color_.step + ( x ) *4 ],
               frame_->color_.data[ ( y+1 ) *frame_->color_.step + x*4 ] - frame_->color_.data[ y*frame_->color_.step + ( x ) *4 ]
           );
}


float Feature::getBilinearPixelValue ( BiLinearWeights& weights, Eigen::Matrix<double,1,2>& jacob, bool project_to_frame )
{
    int x_small, x_big, y_small, y_big;
    float x,y;
    if ( project_to_frame )
    {
        Eigen::Vector2d pixel = frame_->world2pixel ( position_ );
        for (int i=0; i<keypoint_.octave; i++)
        {
            pixel = pixel/2;
        }
        x_small = floor ( pixel[0] );
        x_big = x_small + 1;
        y_small = floor ( pixel[1] );
        y_big = y_small + 1;
        x = pixel[0];
        y = pixel[1];
    }
    else
    {
        x = keypoint_.pt.x;
        y = keypoint_.pt.y;
        for (int i=0; i<keypoint_.octave; i++)
        {
            x/=2;
            y/=2;
        }
        x_small = floor ( x );
        x_big = x_small + 1;
        y_small = floor ( y );
        y_big = y_small + 1;
    }
    
    cv::Mat& img_data = frame_->image_pyramid_[keypoint_.octave];
    //cout<<"x="<<x<<",y="<<y<<endl;
    /*
    cv::Mat img_show;
    cv::cvtColor ( img_data, img_show, CV_GRAY2RGB );
    cv::circle( img_show, cv::Point2f(x,y), 3, cv::Scalar(0,255,0), 1);
    cv::imshow("current", img_show );
    cv::waitKey(1);
    */
    
    x = x-x_small;
    y = y-y_small;

    // check the board 
    if ( x_small < 0 || y_small < 0 || x_big+2 >= frame_->image_pyramid_[keypoint_.octave].cols || y_big+2 >= frame_->image_pyramid_[keypoint_.octave].rows )
    {
        cerr<<"this feature is outside the image."<<endl;
        return -1;
    }

    weights.topLeft_ = ( 1-x ) * ( 1-y );
    weights.topRight_ = x* ( 1-y );
    weights.bottomLeft_ = ( 1-x ) *y;
    weights.bottomRight_ = x*y;

    //cout<<img_data.channels()<<","<<img_data.type()<<endl;
    uchar* data_ptr = &img_data.data[ y_small*img_data.step + x_small ];
    
    //cout<<"data="<<data_ptr[0]<<endl;
    int step = img_data.step;
    uchar f00 = data_ptr[0];
    uchar f10 = data_ptr[1];
    uchar f01 = data_ptr[step];
    uchar f11 = data_ptr[step+1];

    float current = weights.topLeft_ * f00 + weights.topRight_ * f10
                    + weights.bottomLeft_ * f01
                    + weights.bottomRight_* f11;
                    
    jacob ( 0,0 ) = 0.5 *
                    (
                        weights.topLeft_ * data_ptr[2]
                        + weights.topRight_ * data_ptr[3]
                        + weights.bottomLeft_ * data_ptr[2+step]
                        + weights.bottomRight_* data_ptr[3+step]
                        - current
                    );

    jacob ( 0,1 ) = 0.5 * (
                        weights.topLeft_ * data_ptr[step*2]
                        + weights.topRight_ * data_ptr[step*2+1]
                        + weights.bottomLeft_ * data_ptr[step*3]
                        + weights.bottomRight_* data_ptr[step*3+1]
                        - current
                    );

    return current;
}

Eigen::Matrix< double, 2, 6 > Feature::getJacobPixel2se3()
{
    Eigen::Matrix< double, 2,6 > jacob;
    Eigen::Vector3d point = frame_->world2camera ( position_ );
    float fx = frame_->camera_->fx();
    float fy = frame_->camera_->fy();

    jacob ( 0,0 ) = fx / point[2];
    jacob ( 0,1 ) = 0;
    jacob ( 0,2 ) = - fx * point[0] / ( point[2]*point[2] );
    jacob ( 0,3 ) = - fx *point[0]*point[1]/ ( point[2]*point[2] );
    jacob ( 0,4 ) = fx+fx*point[0]*point[0]/ ( point[2]*point[2] ) ;
    jacob ( 0,5 ) = -fx*point[1]/point[2];

    jacob ( 1,0 ) = 0;
    jacob ( 1,1 ) = fy/point[2];
    jacob ( 1,2 ) = -fy*point[1]/ ( point[2]*point[2] );
    jacob ( 1,3 ) = -fy-fy*point[1]*point[1]/ ( point[2]*point[2] );
    jacob ( 1,4 ) = fy*point[0]*point[1]/ ( point[2]*point[2] );
    jacob ( 1,5 ) = fy*point[0]/point[2];

    return jacob;
}



}// namespace zed_slam

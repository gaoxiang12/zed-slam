#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <boost/timer.hpp>

#include "core/Config.h"
#include "core/Feature.h"
#include "core/Featuredetection.h"
#include "io/Zed_svo_wrapper.h"
#include "utils/SLAMTools.h"

using namespace std; 

int main( int argc, char** argv )
{
    if (argc != 3)
    {
        cout<<"usage: test_feature_detection config_file svo_file"<<endl;
        return -1;
    }
    
    string path_to_config ( argv[1] );
    string svo_file ( argv[2] );
    
    zed_slam::Config::init( path_to_config );
    zed_slam::Zed_svo_wrapper* svo_wrapper = new zed_slam::Zed_svo_wrapper( svo_file );
    
    while (1)
    {
        zed_slam::Frame::Ptr frame = svo_wrapper->next();
        if ( frame == nullptr )
            return -1;
        cout<<"start to detect features"<<endl;
        zed_slam::FeatureDetection featureDetection;
        boost::timer timer;
        featureDetection.detectFAST( frame );
        cout<<"Detecting fast cost time "<<timer.elapsed()<<endl;
        cout<<"total features: "<<frame->features_.size()<<endl;
        
        zed_slam::Feature* feature = *frame->features_.begin();
        cout<<"Feature 0 3d position: "<<feature->position_<<endl;
        cout<<"Feature 0 2d pixel: "<<feature->keypoint_.pt<<endl;
        cout<<"Project from 3d to 2d: "<< frame->world2pixel( feature->position_ )<<endl;
        zed_slam::SLAMTools::drawFeatures( frame );
        cv::waitKey(0);
    }
    
    return 0;
}
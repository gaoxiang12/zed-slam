
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <boost/timer.hpp>
#include <algorithm>

#include "core/Config.h"
#include "core/Feature.h"
#include "core/Featuredetection.h"
#include "core/Tracker.h"
#include "io/Zed_svo_wrapper.h"
#include "utils/SLAMTools.h"

using namespace std;

int main ( int argc, char** argv )
{
    if ( argc != 3 )
    {
        cout<<"usage: test_semi_direct_vo config_file svo_file"<<endl;
        return -1;
    }

    string path_to_config ( argv[1] );
    string svo_file ( argv[2] );

    zed_slam::Config::init ( path_to_config );
    zed_slam::Zed_svo_wrapper* svo_wrapper = new zed_slam::Zed_svo_wrapper ( svo_file );
    
    // remove the first frames where the camera does not move
    for (int i=0; i<40; i++)
    {
        svo_wrapper->next();
    }

    zed_slam::Frame::Ptr frame1 = svo_wrapper->next();
    if ( frame1 == nullptr )
        return -1;

    zed_slam::FeatureDetection featureDetection;
    featureDetection.detectFAST ( frame1 );

    //for (int i=0; i<10; i++)
    //    svo_wrapper->next();
    
    zed_slam::Tracker tracker;
    zed_slam::FrameDrawer drawer;
    while(1)
    {
        zed_slam::Frame::Ptr frame2 = svo_wrapper->next();
        if (frame2==nullptr)
            break;
        
        featureDetection.detectFAST( frame1, true );
        // copy the features in frame1 into frame 2
        for ( auto f:frame1->features_ )
        {
            shared_ptr<zed_slam::Feature> f2 (new zed_slam::Feature( *f.second ));
            f2->frame_ = frame2.get();
            frame2->features_.insert({f.second->id_, f2});
        }
        frame2->T_f_w_ = frame1->T_f_w_;
        
        // do semi direct pose estimation 
        //cout<<"doing semi-direct pose estimation in frame1 and frame2. "<<endl;
        boost::timer timer;
        tracker.poseEstimationTwoFramesSemiDirect( frame1, frame2 );
        cout<<"semi direct pose estimation cost time = "<<timer.elapsed()<<endl;
        
        //zed_slam::SLAMTools::drawFeaturesReprojection(frame2);
        cv::imshow( "image", frame2->color_ );
        drawer.addFrame( frame2 );
        
        cv::waitKey(1);
        
        frame1 = frame2;
    }
    
    drawer.stop();

    return 0;
}

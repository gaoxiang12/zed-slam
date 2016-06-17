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
        cout<<"usage: test_feature_tracking config_file svo_file"<<endl;
        return -1;
    }

    string path_to_config ( argv[1] );
    string svo_file ( argv[2] );

    zed_slam::Config::init ( path_to_config );
    zed_slam::Zed_svo_wrapper* svo_wrapper = new zed_slam::Zed_svo_wrapper ( svo_file );

    zed_slam::Frame::Ptr frame1 = svo_wrapper->next();
    zed_slam::FrameDrawer viewer; 
    if ( frame1 == nullptr )
        return -1;

    cout<<"start to detect features"<<endl;
    zed_slam::FeatureDetection featureDetection;
    boost::timer timer;
    featureDetection.detectFAST ( frame1 );
    cout<<"Detecting fast cost time "<<timer.elapsed() <<endl;
    cout<<"total features: "<<frame1->features_.size() <<endl;

    // track these features in frame 2
    int loop=1;
    while ( 1 )
    {
        cout<<"********* loop "<<loop<<" **********"<<endl;
        zed_slam::Frame::Ptr frame2 = svo_wrapper->next();
        if ( frame2 == nullptr )
            break;
        zed_slam::Tracker tracker;

        boost::timer timer;
        tracker.trackKLT ( frame1, frame2 );
        cout<<"trackKLT uses time "<<timer.elapsed() <<endl;

        // estimate the pose of frame 2
        boost::timer timer_pose;
        tracker.poseEstimationTwoFrames ( frame1, frame2 );
        cout<<"frame 2 pose = "<<endl<<frame2->T_f_w_.matrix()<<endl;
        cout<<"pose estimation cost time="<<timer_pose.elapsed() <<endl;

        if ( frame2->features_.size() <= zed_slam::Config::get<int> ( "min_features" ) )
        {
            // add new features into frame 2
            cout<<"add new features"<<endl;
            featureDetection.detectFAST ( frame2 );
        }
        cout<<"draw features"<<endl;
        zed_slam::SLAMTools::drawFeatures ( frame2 );
        cv::waitKey ( 1 );
        frame1 = frame2;
        
        loop++;
        
        viewer.addFrame( frame2 );
    }

    viewer.stop();
    return 0;
}

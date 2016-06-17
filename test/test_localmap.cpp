#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <boost/timer.hpp>
#include <algorithm>

#include "core/Config.h"
#include "core/Feature.h"
#include "core/Featuredetection.h"
#include "core/Tracker.h"
#include <core/Map.h>
#include "io/Zed_svo_wrapper.h"
#include "utils/SLAMTools.h"

using namespace std;

int main ( int argc, char** argv )
{
    if ( argc != 3 )
    {
        cout<<"usage: test_localmap config_file svo_file"<<endl;
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

    // track these features in frame 2
    zed_slam::Tracker tracker;
    zed_slam::FeatureDetection featureDetection;
    featureDetection.detectFAST( frame1 );
    
    zed_slam::LocalMap  localMap;
    viewer.setLocalMap(&localMap);
    localMap.insertKeyFrame( frame1 );
    
    int loop=1;
    while ( 1 )
    {
        cout<<"********* loop "<<loop<<" **********"<<endl;
        zed_slam::Frame::Ptr frame2 = svo_wrapper->next();
        if ( frame2 == nullptr )
            break;

        boost::timer timer;
        tracker.trackKLT ( frame1, frame2 );
        cout<<"trackKLT uses time "<<timer.elapsed() <<endl;

        // estimate the pose of frame 2
        boost::timer timer_pose;
        tracker.poseEstimationTwoFrames ( frame1, frame2 );

        if ( frame2->features_.size() <= zed_slam::Config::get<int> ( "min_features" ) )
        {
            // add new features into frame 2
            featureDetection.detectFAST ( frame2 );
        }
        
        loop++;
        viewer.addFrame( frame2 );
        // check key-frames 
        zed_slam::SLAMTools::drawFeatures( frame2 );
        cv::waitKey(1);
        
        if ( localMap.checkKeyFrame(frame2) )
        {
            cout<<"this is a new key-frame"<<endl;
            viewer.addKeyframe( frame2 );
            localMap.insertKeyFrame( frame2 );
            cv::waitKey(0);
            //break;
        }
        frame1 = frame2;
    }

    viewer.stop();
    return 0;
}

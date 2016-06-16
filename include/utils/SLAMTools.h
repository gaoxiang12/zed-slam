/*
 * Copyright (c) 2016 <copyright holder> <email>
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#ifndef ZED_SLAM_SLAMTOOLS_H_
#define ZED_SLAM_SLAMTOOLS_H_
#include <Eigen/Geometry>
#include <g2o/types/slam3d/se3quat.h>
#include <thread>
#include <deque>
#include <mutex>
#include <pangolin/pangolin.h>

#include "core/Frame.h"
#include "core/Map.h"
namespace zed_slam
{
class SLAMTools
{
public:
    static void drawFeatures ( Frame::Ptr frame );
    static void drawFeaturesReprojection( Frame::Ptr frame, string window_name="feature" );

    static g2o::SE3Quat toSE3Quat ( Eigen::Isometry3d T )
    {
        return g2o::SE3Quat ( T.rotation().matrix(), T.translation().matrix() );
    }

};

class FrameDrawer
{
public:
    FrameDrawer();
    ~FrameDrawer();

    // draw thread 
    void run();
public:
    // add a frame, will be pushed into the pose queue 
    void addFrame ( Frame::Ptr frame );
    
    // add a key-frame, the pose of key-frame will be displayed and kept until the program stops
    void addKeyframe( Frame::Ptr keyframe );
    void stop(); 
    
    void setLocalMap( LocalMap* local_map ) 
    {
        local_map_ = local_map;
    }
    
protected:
    void getCurrentOpenGLCameraMatrix( vector<pangolin::OpenGlMatrix>& matrix );
    void drawCurrentCamera( vector<pangolin::OpenGlMatrix>& Twc );
    void drawKeyFrameCamera();
    void drawLocalMapPoints();
    
protected:
    std::mutex mutex_poses_;
    deque<Eigen::Isometry3d> poses_;
    deque<Eigen::Isometry3d> poses_keyframes_;
    std::thread* viewerThread_;
    bool stop_;
    
    LocalMap* local_map_        =nullptr;
};

}

#endif // ZED_SLAM_SLAMTOOLS_H_

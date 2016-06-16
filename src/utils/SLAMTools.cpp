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

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pangolin/pangolin.h>
#include <boost/concept_check.hpp>

#include <thread>

#include "utils/SLAMTools.h"
#include "core/Feature.h"


namespace zed_slam
{

void SLAMTools::drawFeatures ( Frame::Ptr frame )
{
    cv::Mat img_show;
    cv::cvtColor ( frame->color_, img_show, CV_GRAY2RGB );
    std::for_each ( frame->features_.begin(), frame->features_.end(),
                    [&] ( zed_slam::Feature* feature )
    {
        if ( feature->quality_ < 50 )
            cv::circle ( img_show, feature->keypoint_.pt, 4* ( feature->keypoint_.octave+1 ),
                         cv::Scalar ( 0,255,0 ), 1 );
        else
            cv::circle ( img_show, feature->keypoint_.pt, 4* ( feature->keypoint_.octave+1 ),
                         cv::Scalar ( 200,0,0 ), 2 );
    }
                  );
    cv::imshow ( "features", img_show );
}

void SLAMTools::drawFeaturesReprojection ( Frame::Ptr frame, string window_name )
{
    cv::Mat img_show;
    cv::cvtColor ( frame->color_, img_show, CV_GRAY2RGB );
    std::for_each ( frame->features_.begin(), frame->features_.end(),
                    [&] ( zed_slam::Feature* feature )
    {
        Eigen::Vector2d pos = frame->world2pixel ( feature->position_ );
        cv::circle ( img_show, cv::Point2f ( pos[0],pos[1] ), 4* ( feature->keypoint_.octave+1 ),
                     cv::Scalar ( 0,0,255 ), 2 );
    } );
    cv::imshow ( window_name.c_str(), img_show );
}


FrameDrawer::FrameDrawer()
{
    stop_ = false;
    viewerThread_ = new std::thread ( std::bind ( &FrameDrawer::run, this ) );
}

FrameDrawer::~FrameDrawer()
{
}

void FrameDrawer::stop()
{
    stop_ = true;
    if ( viewerThread_ )
        viewerThread_->join();
}

void FrameDrawer::run()
{
    pangolin::CreateWindowAndBind ( "zed slam Frame viewer", 640, 480 );
    glEnable ( GL_DEPTH_TEST );
    glEnable ( GL_BLEND );
    glBlendFunc ( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );

    // camera object
    pangolin::OpenGlRenderState s_cam (
        pangolin::ProjectionMatrix ( 640,480,300,300,320,240,0.1,1000 ),
        pangolin::ModelViewLookAt ( 0,-0.7, -1.8, 0,0,0,0.0,-1.0, 0.0 )
    );

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& d_cam = pangolin::CreateDisplay()
                            .SetBounds ( 0.0, 1.0, pangolin::Attach::Pix ( 175 ), 1.0, -640.0f/480.0f )
                            .SetHandler ( new pangolin::Handler3D ( s_cam ) );
    while ( stop_ == false )
    {
        // main loop
        if ( poses_.empty() )
        {
            usleep ( 1e6 );
            continue;
        }
        //cout<<"viewer::mainloop"<<endl;
        glClear ( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
        vector<pangolin::OpenGlMatrix> T;
        getCurrentOpenGLCameraMatrix ( T );
        //s_cam.Follow ( T.back() );
        d_cam.Activate ( s_cam );
        glClearColor ( 1.0f, 1.0f, 1.0f, 1.0f );
        drawCurrentCamera ( T );
        
        drawKeyFrameCamera();
        
        if (local_map_)
            drawLocalMapPoints();
        
        pangolin::FinishFrame();

        usleep ( 10000 );
        //cout<<"viewer::mainloop ends."<<endl;
    }
}


void FrameDrawer::addFrame ( Frame::Ptr frame )
{
    unique_lock<mutex> lock ( mutex_poses_, try_to_lock );
    if ( lock.owns_lock() == true )
    {
        poses_.push_back ( frame->T_f_w_.inverse() );
        if ( poses_.size() > 30 )
            poses_.pop_front();
    }
}

void FrameDrawer::addKeyframe ( Frame::Ptr keyframe )
{
    unique_lock<mutex> lock ( mutex_poses_ );
    poses_keyframes_.push_back( keyframe->T_f_w_.inverse() );
    
    // TODO add the feature points
}


void FrameDrawer::getCurrentOpenGLCameraMatrix ( vector<pangolin::OpenGlMatrix> & matrix )
{
    unique_lock<mutex> lock ( mutex_poses_ );
    for ( auto pose : poses_ )
    {
        Eigen::Matrix4d Twf = pose.matrix();
        pangolin::OpenGlMatrix m;

        m.m[0] = Twf ( 0,0 );
        m.m[1] = Twf ( 1,0 );
        m.m[2] = Twf ( 2,0 );
        m.m[3] = 0;

        m.m[4] = Twf ( 0,1 );
        m.m[5] = Twf ( 1,1 );
        m.m[6] = Twf ( 2,1 );
        m.m[7] = 0;

        m.m[8] = Twf ( 0,2 );
        m.m[9] = Twf ( 1,2 );
        m.m[10] = Twf ( 2,2 );
        m.m[11] = 0;

        m.m[12] = Twf ( 0,3 );
        m.m[13] = Twf ( 1,3 );
        m.m[14] = Twf ( 2,3 );
        m.m[15] = 1;

        matrix.push_back ( m );
    }
}

void FrameDrawer::drawCurrentCamera ( vector<pangolin::OpenGlMatrix>& Twc )
{
    //cout<<"drawing current camera"<<endl;
    const static float w = 0.18;
    const static float h = w*0.75;
    const static float z = w*0.6;

    for ( auto T:Twc )
    {
        // set the camera matrix
        glPushMatrix();
        glMultMatrixd ( T.m );

        glLineWidth ( 0.5f );
        glColor3f ( 0.6f,0.6f,0.6f );
        glBegin ( GL_LINES );
        glVertex3f ( 0,0,0 );
        glVertex3f ( w,h,z );
        glVertex3f ( 0,0,0 );
        glVertex3f ( w,-h,z );
        glVertex3f ( 0,0,0 );
        glVertex3f ( -w,-h,z );
        glVertex3f ( 0,0,0 );
        glVertex3f ( -w,h,z );

        glVertex3f ( w,h,z );
        glVertex3f ( w,-h,z );

        glVertex3f ( -w,h,z );
        glVertex3f ( -w,-h,z );

        glVertex3f ( -w,h,z );
        glVertex3f ( w,h,z );

        glVertex3f ( -w,-h,z );
        glVertex3f ( w,-h,z );
        glEnd();

        glPopMatrix();

    }

    auto T = Twc.back();
    // set the camera matrix
    glPushMatrix();
    glMultMatrixd ( T.m );

    glLineWidth ( 3 );
    glColor3f ( 0.0f,0.0f,1.0f );
    glBegin ( GL_LINES );
    glVertex3f ( 0,0,0 );
    glVertex3f ( w,h,z );
    glVertex3f ( 0,0,0 );
    glVertex3f ( w,-h,z );
    glVertex3f ( 0,0,0 );
    glVertex3f ( -w,-h,z );
    glVertex3f ( 0,0,0 );
    glVertex3f ( -w,h,z );

    glVertex3f ( w,h,z );
    glVertex3f ( w,-h,z );

    glVertex3f ( -w,h,z );
    glVertex3f ( -w,-h,z );

    glVertex3f ( -w,h,z );
    glVertex3f ( w,h,z );

    glVertex3f ( -w,-h,z );
    glVertex3f ( w,-h,z );
    glEnd();

    glPopMatrix();
}

void FrameDrawer::drawKeyFrameCamera()
{
    const static float w = 0.18;
    const static float h = w*0.75;
    const static float z = w*0.6;
    unique_lock<mutex> lock (mutex_poses_);
    for ( Eigen::Isometry3d pose: poses_keyframes_ )
    {
        Eigen::Matrix4d Twf = pose.matrix();
        pangolin::OpenGlMatrix m;

        m.m[0] = Twf ( 0,0 );
        m.m[1] = Twf ( 1,0 );
        m.m[2] = Twf ( 2,0 );
        m.m[3] = 0;

        m.m[4] = Twf ( 0,1 );
        m.m[5] = Twf ( 1,1 );
        m.m[6] = Twf ( 2,1 );
        m.m[7] = 0;

        m.m[8] = Twf ( 0,2 );
        m.m[9] = Twf ( 1,2 );
        m.m[10] = Twf ( 2,2 );
        m.m[11] = 0;

        m.m[12] = Twf ( 0,3 );
        m.m[13] = Twf ( 1,3 );
        m.m[14] = Twf ( 2,3 );
        m.m[15] = 1;
        
        glPushMatrix();
        glMultMatrixd ( m.m );

        glLineWidth ( 2 );
        glColor3f ( 0.0f,0.0f,0.0f );
        glBegin ( GL_LINES );
        glVertex3f ( 0,0,0 );
        glVertex3f ( w,h,z );
        glVertex3f ( 0,0,0 );
        glVertex3f ( w,-h,z );
        glVertex3f ( 0,0,0 );
        glVertex3f ( -w,-h,z );
        glVertex3f ( 0,0,0 );
        glVertex3f ( -w,h,z );

        glVertex3f ( w,h,z );
        glVertex3f ( w,-h,z );

        glVertex3f ( -w,h,z );
        glVertex3f ( -w,-h,z );

        glVertex3f ( -w,h,z );
        glVertex3f ( w,h,z );

        glVertex3f ( -w,-h,z );
        glVertex3f ( w,-h,z );
        glEnd();

        glPopMatrix();
    }
}

void FrameDrawer::drawLocalMapPoints()
{
    auto points = local_map_->getMapPoints();
    
    glPointSize(3);
    glBegin(GL_POINTS);
    glColor3d(0.1,0.1,0.1);
    
    for ( shared_ptr<MapPoint> p: points )
    {
        glVertex3d( p->pos_(0), p->pos_(1),p->pos_(2) );
    }
    
    glEnd();
}


} // namespace zed_slam

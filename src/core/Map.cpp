#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <g2o/core/block_solver.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#include "core/Map.h"
#include "core/Feature.h"
#include "core/Config.h"
#include "utils/SLAMTools.h"


namespace zed_slam {
    
LocalMap::LocalMap():
orb_( new cv::ORB() )
{
    distance_key_frame_ = Config::get<double>("distance_keyframe");
}
    
int LocalMap::insertKeyFrame ( Frame::Ptr frame )
{
    // we assert this frame is already checked as a key-frame 
    key_frames_.push_back( frame );
    
    // compute this frame's descriptors 
    computeKeyframeDescriptor( frame );
    
    if (ref_key_frame_ != nullptr )
    {
        // match this key-frame with the map-points and estimate its pose 
        poseOptimization( frame );
    }
    
    
    // add this frame into key-frames and set the map points 
    key_frames_.push_back( frame );
    
    addMapPoints( frame );
    
    // set the ref frame to this frame 
    ref_key_frame_ = frame;
    
    // TODO Map Points culling and key-frame culling 

    return 0;
}

    
void LocalMap::computeKeyframeDescriptor( Frame::Ptr frame )
{
    vector<int> bad_index;
    for( auto feature:frame->features_ )
    {
        cv::Mat desp;
        vector<cv::KeyPoint> kp{feature.second->keypoint_};
        (*orb_)(frame->color_, cv::Mat(), kp, desp, true );
        // there are key-points near the boarder 
        if (desp.rows==0)
        {
            cout<<"strange point"<<endl;
            cout<<"kp="<<kp[0].pt.x<<","<<kp[1].pt.y<<endl;
            bad_index.push_back();
        }
        current_descriptors_.push_back( desp );
    }
    //(*orb_) ( frame->color_, cv::Mat(), kps, current_descriptors_, true );
    cout<<"descriptor rows="<<current_descriptors_.rows<<",cols="<<current_descriptors_.cols<<endl;
}

void LocalMap::addMapPoints ( Frame::Ptr frame )
{
    int i=0;
    cout<<"descriptor rows="<<current_descriptors_.rows<<endl;
    cout<<"features="<<frame->features_.size()<<endl;
    for ( auto feature: frame->features_ )
    {
        shared_ptr<MapPoint> map_point = MapPoint::createMapPoint(); 
        map_point->pos_ = feature.second->position_;
        map_point->descriptor_ = current_descriptors_.row(i);
        map_points_[map_point->id_] = map_point;
        i++;
        //map_points_.push_back( map_point );
    }
}

bool LocalMap::checkKeyFrame ( Frame::Ptr frame )
{
    Eigen::Isometry3d delta = ref_key_frame_->T_f_w_.inverse()*frame->T_f_w_;
    g2o::SE3Quat q = SLAMTools::toSE3Quat( delta ) ;
    auto d = q.log();
    
    cout<<"delta ="<<d<<endl;
    double d_norm = 0;
    for ( int i=0; i<6; i++ )
        d_norm += d(i)*d(i);
    
    cout<<"delta norm="<<d_norm<<endl;
    if ( d_norm > distance_key_frame_ )
        return true;
    return false;
}

void LocalMap::poseOptimization ( Frame::Ptr frame )
{
    const double knn_match_ratio = 1.2;
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming"); 
    
    // build the descriptors 
    cv::Mat queryDescriptors;
    cv::Mat trainDescriptors = current_descriptors_;
    
    for ( auto p:map_points_ )
    {
        queryDescriptors.push_back( p.second->descriptor_ );
    }
    
    vector<vector<cv::DMatch>> matches;
    matcher->knnMatch( queryDescriptors, trainDescriptors, matches, 2 );
    
    vector<cv::DMatch> good_matches; 
    for ( vector<cv::DMatch> m:matches )
    {
        if ( m.size()>1 && m[0].distance*knn_match_ratio < m[1].distance )
        {
            good_matches.push_back(m[0]);
        }
    }
    cout<<"good matches="<<good_matches.size()<<endl;
    
    // solve the BA to estimate the pose of key-frame and the position of world points 
    vector<Eigen::Vector3d>     points_world;
    vector<Eigen::Vector2d>     points_camera;
    for ( cv::DMatch m:good_matches )
    {
        MapPoint map_point = *map_points_[m.queryIdx];
        points_world.push_back( map_point.pos_ );
        points_camera.push_back( 
            frame->world2pixel( frame->features_[m.trainIdx]->position_) );
    }
    
    // setting up g2o 
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType* linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType> ();
    g2o::BlockSolver_6_3*   solver_ptr = new g2o::BlockSolver_6_3 ( linearSolver );
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr );
    optimizer.setAlgorithm ( solver );
    optimizer.setVerbose ( false );
    
    // vertices
    g2o::VertexSE3Expmap* vtx_pose = new g2o::VertexSE3Expmap();
    vtx_pose->setEstimate( SLAMTools::toSE3Quat(frame->T_f_w_) );
    vtx_pose->setId(0);
    optimizer.addVertex(vtx_pose);
    
    int id=1;
    int number_of_matched_points = points_world.size();
    
    for ( Eigen::Vector3d p:points_world )
    {
        g2o::VertexSBAPointXYZ* vtx_point = new g2o::VertexSBAPointXYZ();
        vtx_point->setId(id++);
        vtx_point->setEstimate(p);
        optimizer.addVertex(vtx_point);
    }
    
    // edges 
    vector<g2o::EdgeSE3ProjectXYZ*> edges;
    for ( Eigen::Vector2d p:points_camera )
    {
        g2o::EdgeSE3ProjectXYZ* edge = new g2o::EdgeSE3ProjectXYZ();
        edge->setVertex( 0, optimizer.vertices()[id-number_of_matched_points] );
        edge->setVertex( 1, vtx_pose );
        edge->setId(id++);
        edge->fx = frame->camera_->fx();
        edge->fy = frame->camera_->fy();
        edge->cx = frame->camera_->cx();
        edge->cy = frame->camera_->cy();
        edge->setMeasurement( p );
        edge->setInformation( Eigen::Matrix2d::Identity() );
        // TODO consider whether use robust kernel. It will be useful because we have mismatches. 
        // edge->setRobustKernel();
        
        optimizer.addEdge(edge);
        
        edge->error();
        edge->id();
        
        edges.push_back(edge);
    }
    
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    
    for (auto edge:edges )
    {
        edge->computeError();
        cout<<"edge "<<edge->id()<<" error="<<edge->error()<<endl;
    }
    
}


void LocalMap::optimize()
{

}

    
    
    
}// namespace zed_slam
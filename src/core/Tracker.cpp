#include <opencv2/video/tracking.hpp>
#include <iostream>

using namespace std;

// g2o
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/format.hpp>

#include "core/Tracker.h"
#include "core/Feature.h"
#include <core/Config.h>
#include <utils/SLAMTools.h>
#include "g2o_types/types_direct.h"

namespace zed_slam
{

typedef Eigen::Matrix<double, 6, 1, Eigen::ColMajor> Vector6d;

Tracker::Tracker()
{
    klt_max_iter_ = Config::get<int> ( "klt_max_iter" );
    klt_eps_ = Config::get<int> ( "klt_eps" );
    klt_window_size_ = Config::get<int> ( "klt_window_size" );

    epsilon_direct_ = Config::get<double> ( "epsilon_direct" );
    confidence_threshold_ = Config::get<double> ( "confidence_threshold" );
}


void Tracker::init()
{

}

Eigen::Isometry3d Tracker::trackImage ( Frame::Ptr frame )
{
    return Eigen::Isometry3d::Identity();
}

int Tracker::trackKLT ( Frame::Ptr first, Frame::Ptr second )
{
    vector<uchar> status;
    vector<float> error;
    vector<float> min_eig_vec;
    cv::TermCriteria termcrit ( cv::TermCriteria::COUNT+cv::TermCriteria::EPS , klt_max_iter_, klt_eps_ );
    vector<cv::Point2f> pixel_keypoint_second = first->getKeypointPixelPosition();
    // use opencv to calculate the optical flow and get the keypoints' position in the second frame
    // API C++: void calcOpticalFlowPyrLK(InputArray prevImg, InputArray nextImg, InputArray prevPts, InputOutputArray nextPts, OutputArray status, OutputArray err, Size winSize=Size(21,21), int maxLevel=3, TermCriteria criteria=TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01), int flags=0, double minEigThreshold=1e-4 )

    cv::calcOpticalFlowPyrLK (
        first->color_, second->color_,
        first->getKeypointPixelPosition(), pixel_keypoint_second,
        status, error, cv::Size ( klt_window_size_, klt_window_size_ ),
        4, termcrit, cv::OPTFLOW_USE_INITIAL_FLOW );

    // check the good
    auto iter_first=first->features_.begin();
    int good = 0;
    for ( size_t i=0; iter_first!=first->features_.end(); ++i )
    {
        if ( status[i]==false )
        {
            // throw the failed feature
            // TODO throw away? or keep them?
            iter_first = first->features_.erase ( iter_first );
            continue;
        }
        // klt is good, add this feature into second frame
        cv::KeyPoint kp = ( *iter_first )->keypoint_;
        kp.pt = pixel_keypoint_second[i];
        Feature* f = new Feature ( second.get(), kp, ( *iter_first )->depth_, ( *iter_first )->confidence_ );
        f->position_ = ( *iter_first )->position_;
        f->quality_ = ( *iter_first )->quality_+1;
        f->quality_ = f->quality_ > 100? 100: f->quality_;
        second->features_.push_back ( f );
        iter_first++;
        good++;
    }

    cout<<"there are "<<good<<" good feature captured by klt"<<endl;
    return 0;
}

int Tracker::poseEstimationTwoFrames ( Frame::Ptr first, Frame::Ptr second )
{
    // we minimize the stereo reprojection error
    // init g2o
    cout<<"setting up g2o"<<endl;
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType* linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType> ();
    g2o::BlockSolver_6_3*   solver_ptr = new g2o::BlockSolver_6_3 ( linearSolver );
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr );
    optimizer.setAlgorithm ( solver );
    optimizer.setVerbose ( false );

    // pose of the second frame
    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
    pose->setEstimate ( SLAMTools::toSE3Quat ( first->T_f_w_ ) );
    pose->setId ( 0 );
    optimizer.addVertex ( pose );

    // points and edges
    vector<g2o::EdgeSE3ProjectXYZOnlyPose*> edges;
    for ( auto iter=second->features_.begin(); iter!=second->features_.end(); iter++ )
    {
        if ( ( *iter )->quality_ == 0 )
        {
            // from this to end are all new features which do not occur in frame 1
            break;
        }
        // 3d point
        g2o::EdgeSE3ProjectXYZOnlyPose* edge = new g2o::EdgeSE3ProjectXYZOnlyPose();
        edge->setVertex ( 0, pose );
        edge->Xw = ( *iter )->position_;
        edge->fx = second->camera_->fx();
        edge->fy = second->camera_->fy();
        edge->cx = second->camera_->cx();
        edge->cy = second->camera_->cy();
        edge->setMeasurement ( Eigen::Vector2d (
                                   ( *iter )->keypoint_.pt.x, ( *iter )->keypoint_.pt.y ) );
        // cout<<edge->Xw<<endl;
        // TODO how to set the information matrix? how to use confidence?
        //edge->setInformation ( ( *iter )->confidence_ * Eigen::Matrix2d::Identity() );
        //cout<<(*iter)->confidence_<<endl;
        edge->setInformation ( ( *iter )->confidence_ * Eigen::Matrix2d::Identity() );
        edge->setRobustKernel ( new g2o::RobustKernelHuber() );
        optimizer.addEdge ( edge );
        edges.push_back ( edge );
    }

    // in optical flow we usually have missing features but hardly have outlier features, but the depth is not good as rgbd sensors, bad features can also occur.
    cout<<"optimizing..."<<endl;
    optimizer.initializeOptimization();
    optimizer.optimize ( 10 );

    int nbad = 0;
    std::for_each ( edges.begin(), edges.end(), [&] ( g2o::EdgeSE3ProjectXYZOnlyPose* edge )
    {
        edge->computeError();
        //cout<<edge->chi2()<<endl;
        if ( edge->chi2() > 5.991 )
        {
            edge->setLevel ( 1 );
            nbad++;
        }
        else
        {
            edge->setRobustKernel ( nullptr );
        }
    } );

    //cout<<"bad features="<<nbad<<endl;
    optimizer.initializeOptimization();
    optimizer.optimize ( 10 );

    // get the pose of camera
    g2o::VertexSE3Expmap* v = dynamic_cast<g2o::VertexSE3Expmap*> ( optimizer.vertex ( 0 ) );
    Eigen::Isometry3d T = g2o::SE3Quat ( v->estimate() );
    second->T_f_w_ = T;

    // project the features into the camera frame
    // remove the outlier
    //cv::Mat features_img;
    //cv::cvtColor( second->color_, features_img, CV_GRAY2RGB );
    int i=0;
    /*
    for ( auto feature:second->features_ )
    {
        cout<<feature->quality_<<endl;
    }*/
    //cout<<first->features_.size()<<endl;
    //cout<<second->features_.size()<<endl;
    for ( auto iter = second->features_.begin(); iter!=second->features_.end(); )
    {
        Feature* feature = *iter;
        //cv::circle( features_img, feature->keypoint_.pt, 4*(feature->keypoint_.octave+1), cv::Scalar( 0, 0, 255), 1);
        if ( feature->quality_ > 0 )
        {
            edges[i]->computeError();
            // this feature is tracked from the first frame, we adjust the 2d pixel position
            if ( edges[i]->chi2() > 5.991 )
            {
                // tracked but a outlier, reduce its quality
                if ( feature->quality_ < 40 )
                {
                    iter = second->features_.erase ( iter );
                    i++;
                    continue;
                }
                else
                {
                    // this looks like a long tracked feature, keep it
                    Eigen::Vector2d pixel = second->world2pixel ( feature->position_ );
                    feature->keypoint_.pt.x = pixel[0];
                    feature->keypoint_.pt.y = pixel[1];
                    iter++;
                    i++;
                }
            }
            else
            {
                // update the pixel position
                Eigen::Vector2d pixel = second->world2pixel ( feature->position_ );
                feature->keypoint_.pt.x = pixel[0];
                feature->keypoint_.pt.y = pixel[1];
                iter++;
                i++;
            }
        }
        else
        {
            // this is a new feature, we compute the 3d position according to the pose of second frame
            //cout<<feature->keypoint_.pt.x<<','<<feature->keypoint_.pt.y<<endl;
            feature->position_ = second->pixel2world (
                                     Eigen::Vector2d ( feature->keypoint_.pt.x, feature->keypoint_.pt.y )
                                 );
            iter++;
            i++;
        }
        //cv::circle( features_img, feature->keypoint_.pt, 4*(feature->keypoint_.octave+1), cv::Scalar( 0, 255, 0), 1);
    }
    //cv::imshow("feature_optimized", features_img );

    //SLAMTools::drawFeatures( first );
    //cv::waitKey(0);
    return 0;
}

// direct method to estimate the pose
// because we have confidence map, we can choose points that have high confidence (which indicates their texture is good in stereo matching), instead of computing the derivatives of pixels
int Tracker::poseEstimationTwoFramesDirect ( Frame::Ptr first, Frame::Ptr second )
{
    // use the high confiden points to estimate the pose
    // confidence is 8UC4 but an int number
    double residual=1e10, new_residual = 0;

    float step = 1;
    cout<<"calling direct pose estimation"<<endl;
    vector<pair<Eigen::Vector3d, float>> points_first; // the 3d points in world frame with high confidence value, they need only to be computed once

    int high_confidence_points = 0;
    for ( int y=50; y<first->color_.rows-50; y+=2 )
    {
        for ( int x=50; x<first->color_.cols-50; x+=2 )
        {
            int confidence = first->confidence_.data[ y*first->confidence_.step + x*4 ];
            if ( confidence > confidence_threshold_ )
            {
                high_confidence_points ++ ;
                BiLinearWeights weights;
                pair<Eigen::Vector3d, float> p;
                float pixel_first = first->getBiLinearPixelValue ( Eigen::Vector2d ( x,y ),weights );
                p.first = first->pixel2world ( Eigen::Vector2d ( x,y ) );
                p.second = pixel_first;
                points_first.push_back ( p );
            }
        }
    }
    cout<<"high confidence points="<<high_confidence_points<<endl;

    for ( int iterate = 0; iterate<20; iterate ++ )
    {
        // residual and Jacobians
        Eigen::Matrix<double, 6, 6> H;
        H.setZero();
        Eigen::Matrix<double, 6, 1> b;
        b.setZero();

        new_residual = 0;
        int good = 0;
        // we don't use the pixels that are near the boarder
        for ( auto p:points_first )
        {
            // compute the reprojection of this point
            Eigen::Vector2d pixel_position_second = second->world2pixel ( p.first );
            if ( second->inFrame ( pixel_position_second, 3 ) ==false )
            {
                continue;
            }

            // compute the reprojection error
            BiLinearWeights weights;
            float pixel_second = second->getBiLinearPixelValue ( pixel_position_second, weights );
            Eigen::Matrix<double, 2, 6> jacobian_uv_ksai = second->getJacobianUV2SE3 ( p.first );
            Eigen::Matrix<double,1,2> jacobian_pixel_uv = second->getJacobianPixel2UV ( pixel_position_second );

            Eigen::Matrix<double, 1, 6> jacobian_pixel_ksai = -jacobian_pixel_uv * jacobian_uv_ksai;

            float error = p.second - pixel_second;
            new_residual += error*error;
            good++;
            //cout<<"j_uv_ksai="<<jacobian_pixel_ksai<<endl;
            H.noalias() += jacobian_pixel_ksai.transpose() * jacobian_pixel_ksai;
            b.noalias() += -jacobian_pixel_ksai*error;
        }

        if ( good == 0 )
            break;
        new_residual /= good;
        Eigen::Matrix<double,6,1> x = H.ldlt().solve ( b );
        if ( bool ( std::isnan ( double ( x[0] ) ) ) )
        {
            cerr<<"solving gauss-newton failed."<<endl;
            return 0;
        }

        Vector6d dx;
        dx ( 0 ) = x ( 3 );
        dx ( 1 ) = x ( 4 );
        dx ( 2 ) = x ( 5 );
        dx ( 3 ) = x ( 0 );
        dx ( 4 ) = x ( 1 );
        dx ( 5 ) = x ( 2 );
        dx = step * dx;
        Eigen::Isometry3d dT = ( Eigen::Isometry3d ) g2o::SE3Quat::exp ( dx );
        //cout<<"dT="<<dT.matrix()<<endl;
        second->T_f_w_ = dT*second->T_f_w_;

        cout<<"new residual="<<new_residual<<", old="<<residual<<endl;
        if ( iterate>3 && ( new_residual>residual || ( residual-new_residual ) <epsilon_direct_ ) )
        {
            cout<<"iteration="<<iterate<<endl;
            break;
        }
        residual = new_residual;
    }
    return 0;
}

// TODO: (1) use patch instead of single pixel value (2) use image pyramid level information (3) robustify the error norm and use weights in each term
int Tracker::poseEstimationTwoFramesSemiDirect ( Frame::Ptr first, Frame::Ptr second )
{
    // residual of photometric error
    double residual=1e10, new_residual = 0;

    //SLAMTools::drawFeatures( first );

    double step = 1;

    boost::format fmt ( "iterator: %d" );
    for ( int iterate = 0; iterate<20; iterate ++ )
    {
        Eigen::Matrix<double, 6, 6> H;
        H.setZero();
        Eigen::Matrix<double, 6, 1> b;
        b.setZero();

        new_residual = 0;
        int good = 0;
        // residual and Jacobians
        for ( auto iter_second = second->features_.begin(), iter_first = first->features_.begin(); iter_second != second->features_.end(); )
        {
            // jacobian: from pixel value to se(3)
            Eigen::Matrix<double, 1, 6> jacobian_pixel_ksai;
            // jacobian: from pixel value to pixel coordinates;
            Eigen::Matrix<double, 1, 2> jacobian_pixel_uv;
            jacobian_pixel_uv.setZero();

            Feature* feature_second = *iter_second;
            Feature* feature_first = *iter_first;
            //if ( feature_second->quality_ > 0 )
            // this is a feature tracked from first frame
            // compute the bilinear interpolated pixel value
            BiLinearWeights bilinear_weights;
            float pixel_value_first = feature_first->getBilinearPixelValue ( bilinear_weights, jacobian_pixel_uv, false );
            // in second frame we should reproject the 3d position into second frame
            float pixel_value_second_bilinear = feature_second->getBilinearPixelValue ( bilinear_weights, jacobian_pixel_uv, true );

            // we use zero order interpolation here
            //uchar pixel_value_first = feature_first->getPixelValue( false );
            //uchar pixel_value_second = feature_second->getPixelValue( true );

            //cout<<"pixel:"<< ( int ) pixel_value_first<<","<< ( int ) pixel_value_second<<","<<(int)pixel_value_second_bilinear<<endl;
            if ( pixel_value_first < 0 || pixel_value_second_bilinear<0 )
            {
                // remove this feature
                iter_first = first->features_.erase ( iter_first );
                iter_second = second->features_.erase ( iter_second );
                continue;
            }
            else
            {
                iter_first++;
                iter_second++;
            }

            //cout<<"pixel:"<< pixel_value_first<<","<<pixel_value_second_bilinear<<endl;
            double dv = double ( pixel_value_first - pixel_value_second_bilinear );
            //cout<<"j_pixel_uv="<<jacobian_pixel_uv.matrix()<<endl;
            new_residual += dv*dv;
            good++;

            // compute the jacobian_pixel_ksai
            Eigen::Matrix<double,2,6> jacobian_uv_ksai = feature_second->getJacobPixel2se3();
            //jacobian_pixel_uv = feature_second->getZeroOrderJacobian().transpose();
            //cout<<"j_pixel_uv="<<jacobian_pixel_uv.matrix()<<endl;
            //cout<<"Ji="<<jacobian_uv_ksai.matrix() <<endl;
            jacobian_pixel_ksai = -jacobian_pixel_uv * jacobian_uv_ksai;
            H.noalias() += jacobian_pixel_ksai.transpose() *jacobian_pixel_ksai;
            b.noalias() += - jacobian_pixel_ksai * dv;


        } // end of feature loop
        if ( good == 0 )
            break;

        new_residual/=good;
        //cout<<"new residual = "<<new_residual<<", old = "<<residual<<", good points= "<<good<<endl;
        //cout<<H.matrix() <<endl;
        //cout<<b.matrix() <<endl;

        // solve H dx = b and iterate it!
        Eigen::Matrix<double,6,1> x = H.ldlt().solve ( b );
        if ( bool ( std::isnan ( double ( x[0] ) ) ) )
        {
            cerr<<"solving gauss-newton failed."<<endl;
            return 0;
        }

        //cout<<"delta ksai = "<<x<<endl;
        // The se3 used in g2o is [\omega, \upsilon] where the first one is so(3) and second one is translation, but in our formulation the first three numbers are translation and the second are so(3).
        Vector6d dx;
        dx ( 0 ) = x ( 3 );
        dx ( 1 ) = x ( 4 );
        dx ( 2 ) = x ( 5 );
        dx ( 3 ) = x ( 0 );
        dx ( 4 ) = x ( 1 );
        dx ( 5 ) = x ( 2 );
        dx = step * dx;
        Eigen::Isometry3d dT = ( Eigen::Isometry3d ) g2o::SE3Quat::exp ( dx );
        //cout<<"dT="<<endl<<dT.matrix()<<endl;
        second->T_f_w_ = dT*second->T_f_w_;
        //second->T_f_w_ = second->T_f_w_ * dT;

        if ( iterate>5 && ( new_residual>residual || ( residual-new_residual ) <epsilon_direct_ ) )
        {
            //cout<<"iterate="<<iterate<<endl;
            break;
        }
        residual = new_residual;

        /*
        if (iterate%10==0)
        {
            SLAMTools::drawFeaturesReprojection( second, (fmt%iterate).str() );
            cv::waitKey(0);
        }
        cv::waitKey(0);
        */
    }

    //cout<<"first="<<first->T_f_w_.matrix() <<endl;
    //cout<<"second="<<second->T_f_w_.matrix() <<endl;
    // update the features in second frame
    for ( Feature* feature:second->features_ )
    {
        Eigen::Vector2d pixel = second->world2pixel ( feature->position_ );
        //cout<<"pixel="<<pixel.matrix()<<endl;
        //cout<<"keypoint: "<<feature->keypoint_.pt<<endl;
        feature->keypoint_.pt.x = pixel[0];
        feature->keypoint_.pt.y = pixel[1];
        //feature->position_;
    }

    return 1;
}

int Tracker::poseEstimationTwoFramesSemiDirectUsingG2O ( Frame::Ptr first, Frame::Ptr second )
{
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,1>> DirectBlock;
    //typedef g2o::BlockSolverX DirectBlock;
    cout<<"calling pose estimation semi-direct using g2o"<<endl;

    // copy the features into second
    std::for_each ( first->features_.begin(), first->features_.end(), [&] ( Feature* f )
    {
        Feature* f2 = new Feature ( *f );
        f2->frame_ = second.get();
        second->features_.push_back ( f2 );
    } );

    g2o::SparseOptimizer optimizer;
    DirectBlock::LinearSolverType* linearSolver = new g2o::LinearSolverDense< DirectBlock::PoseMatrixType > ();
    //DirectBlock::LinearSolverType* linearSolver = new g2o::LinearSolverEigen<DirectBlock::PoseMatrixType> ();
    DirectBlock* solver_ptr = new DirectBlock ( linearSolver );
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr );
    //g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton ( solver_ptr );
    optimizer.setAlgorithm ( solver );
    optimizer.setVerbose ( true );

    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
    pose->setEstimate ( SLAMTools::toSE3Quat ( second->T_f_w_ ) );
    pose->setId ( 0 );
    optimizer.addVertex ( pose );

    int id = 1;
    vector< g2o::EdgeSE3ProjectDirect*> edges;
    //Feature* feature = *second->features_.begin();
    for ( Feature* feature: second->features_ )
    {
        //if ( id>5 )
        //    break;
        //cout<<"feature position="<<feature->position_<<endl;
        //cout<<"feature pixel in frame 1="<<feature->keypoint_.pt<<endl;

        float grayscale_value = first->getBiLinearPixelValue ( feature->getPixelPosition ( false ) ) ;
        //cout<<"measurement="<<grayscale_value<<endl;
        // check whether inside the image
        //if ( feature->isInsideFrameImage ( 10 ) ==false )
        //    continue;
        g2o::EdgeSE3ProjectDirect* edge = new g2o::EdgeSE3ProjectDirect (
            feature->position_,
            second->camera_->fx(),
            second->camera_->fy(),
            second->camera_->cx(),
            second->camera_->cy(),
            &second->color_,
            (feature->keypoint_.octave+1)*2
        );
        edge->setVertex ( 0, pose );
        //edge->setMeasurement ( first->getBiLinearPixelValue ( feature->getPixelPosition ( false ) )-second->getBiLinearPixelValue ( feature->getPixelPosition ( true ) ) );
        edge->setMeasurement ( first->getBiLinearPixelValue ( feature->getPixelPosition ( false ) ) );
        edge->setInformation ( Eigen::Matrix<double,1,1>::Identity() );
        //edge->setRobustKernel( new g2o::RobustKernelHuber() );
        edge->setId ( id );
        optimizer.addEdge ( edge );
        edges.push_back ( edge );
        id++;

        //edge->computeError();
        //cout<<"before error="<<edge->error() <<endl;
    }
    cout<<"add total "<<id<<" edges."<<endl;

    cout<<"calling optimize()"<<endl;
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    cout<<"pose="<<endl<<pose->estimate()<<endl;
    
    // optimizer.optimize ( 10 );
    /*
    cout<<"doing gauss-newton optimization"<<endl;
    for ( int i=0; i<10; i++ )
    {
        Eigen::Matrix<double,6,6> H;
        Eigen::Matrix<double,6,1> b;
        H.setZero();
        b.setZero();
        double chi2 = 0;
        for ( g2o::EdgeSE3ProjectDirect* edge :edges )
        {
            edge->computeError();
            edge->linearizeOplus();

            double error = edge->error() ( 0,0 );

            Eigen::Matrix<double, 1,6> J = edge->jacobian;
            H.noalias() += J.transpose() *J;
            b.noalias() -= J.transpose() * error;
            chi2 += error*error;
        }
        Eigen::Matrix<double,6,1> x = H.ldlt().solve ( b );
        cout<<"x*="<<x<<endl;
        cout<<"chi2="<<chi2<<endl;

        //Eigen::Isometry3d T = pose->estimate();
        //T = Eigen::Isometry3d(g2o::SE3Quat::exp(x))*T;
        //pose->setEstimate( SLAMTools::toSE3Quat(T) );
        //pose->setEstimate( g2o::SE3Quat.exp(x.data())* );
        pose->oplusImpl( x.data() );
    }

    for ( g2o::EdgeSE3ProjectDirect* e:edges )
    {
        e->computeError();
        // cout<<"measurement="<<e->measurement() <<endl;
        cout<<"error="<<e->error() <<endl;
        //cout<<"information="<<e->information() <<endl;
        //cout<<"chi2="<<e->chi2() <<endl;
    }

    */
    cout<<"optimize stops"<<endl;
    cout<<"pose="<<endl<<Eigen::Isometry3d ( pose->estimate() ).matrix() <<endl;

    second->T_f_w_ = pose->estimate();
    return 0;
}


// direct method to estimate the pose
// because we have confidence map, we can choose points that have high confidence (which indicates their texture is good in stereo matching), instead of computing the derivatives of pixels
int Tracker::poseEstimationTwoFramesDirectUsingG2o ( Frame::Ptr first, Frame::Ptr second )
{
    // use the high confiden points to estimate the pose
    // confidence is 8UC4 but an int number
    float step = 1;
    cout<<"calling direct pose estimation"<<endl;
    vector<pair<Eigen::Vector3d, float>> points_first; // the 3d points in world frame with high confidence value, they need only to be computed once

    int high_confidence_points = 0;
    for ( int y=50; y<first->color_.rows-50; y+=2 )
    {
        for ( int x=50; x<first->color_.cols-50; x+=2 )
        {
            int confidence = first->confidence_.data[ y*first->confidence_.step + x*4 ];
            if ( confidence > confidence_threshold_ )
            {
                high_confidence_points ++ ;
                pair<Eigen::Vector3d, float> p;
                float pixel_first = first->getBiLinearPixelValue ( Eigen::Vector2d ( x,y ) );
                p.first = first->pixel2world ( Eigen::Vector2d ( x,y ) );
                p.second = pixel_first;
                points_first.push_back ( p );
            }
        }
    }
    
    cout<<"high confidence points="<<high_confidence_points<<endl;

    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,1>> DirectBlock;
    g2o::SparseOptimizer optimizer;
    DirectBlock::LinearSolverType* linearSolver = new g2o::LinearSolverDense< DirectBlock::PoseMatrixType > ();
    //DirectBlock::LinearSolverType* linearSolver = new g2o::LinearSolverEigen<DirectBlock::PoseMatrixType> ();
    DirectBlock* solver_ptr = new DirectBlock ( linearSolver );
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr );
    //g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton ( solver_ptr );
    optimizer.setAlgorithm ( solver );
    optimizer.setVerbose ( false );
    
    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
    pose->setEstimate ( SLAMTools::toSE3Quat ( second->T_f_w_ ) );
    pose->setId ( 0 );
    optimizer.addVertex ( pose );
    int id=1;
    for ( auto p:points_first )
    {
        g2o::EdgeSE3ProjectDirect* edge = new g2o::EdgeSE3ProjectDirect(
            p.first, 
            second->camera_->fx(),
            second->camera_->fy(),
            second->camera_->cx(),
            second->camera_->cy(),
            &second->color_,2
        );
        edge->setVertex(0, pose);
        edge->setMeasurement( p.second );
        edge->setId(id);
        edge->setInformation ( Eigen::Matrix<double,1,1>::Identity() );
        edge->setRobustKernel( new g2o::RobustKernelHuber() );
        
        optimizer.addEdge(edge);
        id++;
    }
    cout<<"add total "<<id<<" edges."<<endl;
    
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    
    cout<<"pose="<<pose->estimate()<<endl;
    second->T_f_w_ = pose->estimate();
    return 0;
    
}


} //namespace zed_slam

/*
 * Copyright (c) 2016 xiang <email>
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

#ifndef ZED_SLAM_MAPPOINT_H_
#define ZED_SLAM_MAPPOINT_H_

#include <memory>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>

using namespace std; 

namespace zed_slam
{
class Feature;    
class Frame;
    
/**********************************
 * A map point is an estimated and optimized point in the 3d map
 * 
 * ********************************/
class MapPoint
{
public:
    enum MapPointType {
        GOOD, 
        DELETED,
        CANDIDATE,
        UNKNOWN
    };
    
    int                         id_;            // id
    Eigen::Vector3d             pos_;           // position in world frame
    Eigen::Vector3d             normal_;        // surface normal
    Eigen::Matrix3d             normal_information_;    // inverse convariance of normal
    int                         count_observed_;// number of being observed
    MapPointType                type_;          // the status of this point
    
    cv::Mat                     descriptor_;    // the descriptor, used when matching 
    
public:
    static shared_ptr<MapPoint> createMapPoint(); 
    
protected:
    MapPoint( );
    
    // optimize this point through minimizing the reprojection error
    void optimize( const size_t n_iter );
    
    static int current_id_;
};
} //namespace zed_slam

#endif// ZED_SLAM_MAPPOINT_H_

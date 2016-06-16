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

#ifndef ZED_SVO_WRAPPER_H_
#define ZED_SVO_WRAPPER_H_

#include <string>
#include "vikit/pinhole_camera.h"
#include "core/Frame.h"
#include <zed/Camera.hpp>

using namespace std; 
namespace zed_slam
{

class Frame;

/***************************************
 * the zed svo wrapper reads the images from a svo file recorded by zed sdk
 * but, as zed sdk does not record the depth image, we need to compute the depth during the play back. 
 ***************************************/
class Zed_svo_wrapper
{
public:
    Zed_svo_wrapper( const string& svo_filename );
    ~Zed_svo_wrapper();
    
public:
    // get the next frame, return nullptr if there does not exist
    Frame::Ptr next();
    
protected:
    sl::zed::Camera* zed_;
    vk::PinholeCamera* camera_;
    int         index_;
    int         number_of_frames_;
};

}//namespace zed_slam
#endif // ZED_SVO_WRAPPER_H_

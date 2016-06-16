#include <string>
using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/timer.hpp>

#include "io/Zed_svo_wrapper.h"
#include "core/Config.h"
#include "core/Frame.h"

/*************************************
 * 测试svo wrapper程序
 * 请输入配置文件和svo文件的位置，将会回放整个svo并计算depth和confidence.
 * 由于svo文件中只含左右图的数据，depth和confidence需要在线计算，依赖cuda，速度受分辨率和显卡性能的影响。
 * ***********************************/
int main(int argc, char** argv )
{
    if (argc != 3)
    {
        cout<<"Usage: test_svo_wrapper config_file svo_file"<<endl;
        return -1;
    }
    
    string config_file ( argv[1] );
    string svo_file ( argv[2] );
    
    zed_slam::Config::init( config_file );
    zed_slam::Zed_svo_wrapper* svo_wrapper = new zed_slam::Zed_svo_wrapper( svo_file );
    
    cv::namedWindow("color");
    cv::namedWindow("depth");
    cv::namedWindow("confidence");
    while (1)
    {
        boost::timer timer;
        zed_slam::Frame::Ptr pFrame = svo_wrapper->next();
        if ( pFrame == nullptr )
            break;
        cv::imshow( "color", pFrame->color_ );
        cv::imshow( "depth", pFrame->depth_ );
        cv::imshow( "confidence", pFrame->confidence_ );
        if (cv::waitKey(1) == 'q')
            break;
        
        cout<<"fps="<<1.0/timer.elapsed()<<endl;
    }
    
    delete svo_wrapper;
    cv::destroyAllWindows();
    return 0;
}
#include "core/Config.h"
namespace zed_slam {
    
Config::Config( const string& filename )
{
    file = cv::FileStorage ( filename.c_str(), cv::FileStorage::READ );
    if ( file.isOpened() == false )
    {
        cerr<<"file "<<filename<<" does not exist!"<<endl;
        return ;
    }
}

void Config::init( const string& filename )
{
    // 若对象已经存在，则删除并获取一个新的对象
    if ( config_ != nullptr )
        delete config_; 
    config_ = new Config( filename );
}

Config* Config::config_ = nullptr;

} // namespace zed_slam
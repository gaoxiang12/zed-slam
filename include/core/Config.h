#ifndef ZED_SLAM_CONFIG_H_
#define ZED_SLAM_CONFIG_H_
/**
 * \brief the config class read the config file and then provide parameter values for other parts
 */

#include <map>
#include <string>
#include <boost/lexical_cast.hpp>
#include <opencv2/core/core.hpp>
#include <iostream>
using namespace std;

namespace zed_slam
{

/*********************************
 * Config: 参数文件配置类
 * 单件模式，使用前调用init指明参数文件，
 * 随后在任意地方调用getInstance获取对象实例
 ********************************/ 
class Config // 单件模式
{
public:
    static Config* getInstance ()   
    {
        return config_; 
    }
    
private:
    static Config* config_; 
    cv::FileStorage file;
    
private:
    Config ( const string& filename ); // path to the config yaml file
    
public:
    static void init( const string& filename ); 
    
    // get the data, provide the type
    template< typename T >
    T getValue ( const string& key )
    {
        return T ( file[key] );
    }
    
    // fast access 
    template< typename T >
    static T get( const string& key )
    {
        return Config::getInstance()->getValue<T>(key);
    }
    
private:
    // deconstruct the static config object
    class ConfigGarbo
    {
    public:
        ~ConfigGarbo()
        {
            if ( Config::config_ != nullptr )
                delete Config::config_;
        }
    };
    static ConfigGarbo configGarbo_;
    
};

}; //namespace zed_slam

#endif //ZED_SLAM_CONFIG_H_

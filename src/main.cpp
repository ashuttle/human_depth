#include <iostream>
#include "human_depth/imageProcess.h"
#include "human_depth/pose.h"

int main(int argc, char** argv)
{
    // 初始化ros节点
    ros::init(argc, argv, "human_depth");

    if(!ros::ok()){
        return 0;
    }

    std::string ns = K2_DEFAULT_NS;
    std::string topicColor = K2_TOPIC_QHD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
    std::string topicDepth = K2_TOPIC_QHD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
    bool useExact = true;
    bool useCompressed = false;
    ImageProcess::Mode mode = ImageProcess::CLOUD;

    for(size_t i = 1; i < (size_t)argc; ++i)
    {
        std::string param(argv[i]);

    // if(param == "-h" || param == "--help" || param == "-?" || param == "--?")
    // {
    //   help(argv[0]);
    //   ros::shutdown();
    //   return 0;
    // }
     // 图像的清晰度 越高实时性越差
        if(param == "qhd")
        {
            topicColor = K2_TOPIC_QHD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
            topicDepth = K2_TOPIC_QHD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
        }
        else if(param == "hd")
        {
            topicColor = K2_TOPIC_HD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
            topicDepth = K2_TOPIC_HD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
        }
        else if(param == "ir")
        {
            topicColor = K2_TOPIC_SD K2_TOPIC_IMAGE_IR K2_TOPIC_IMAGE_RECT;
            topicDepth = K2_TOPIC_SD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
        }
        else if(param == "sd")
        {
            topicColor = K2_TOPIC_SD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
            topicDepth = K2_TOPIC_SD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
        }
        // 图像校准
        else if(param == "approx")
        {
            useExact = false;
        }

        else if(param == "compressed")
        {
            useCompressed = true;
        }
        // 模式 图像（RGB）和（cloud)深度信息，both是点云和图像图（同时显示）
        else if(param == "image")
        {
            mode = ImageProcess::IMAGE;
        }
        else if(param == "cloud")
        {
            mode = ImageProcess::CLOUD;
        }
        else if(param == "both")
        {   
            mode = ImageProcess::BOTH;
        }
        else
        {
            ns = param;
        }
    }

    //topicColor = "/" + ns + topicColor;
    //topicDepth = "/" + ns + topicDepth;

    // 构建一个对象
    ImageProcess cam_a(1, topicColor, topicDepth, useExact, useCompressed);

    OUT_INFO("starting receiver...");
    // 运行类ImageProcess的接口run
    cam_a.run(mode);
    ros::spin();
    ros::shutdown();
    return 0; 
}
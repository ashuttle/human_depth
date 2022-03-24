#include "human_depth/imageProcess.h"
#include <fstream>
// 参数:
//   相机的ID
//   图像的话题（ros图像的路径）
//   深度图的话题  路径
ImageProcess::ImageProcess(int camera_index, std::string &topicColor, std::string &topicDepth, const bool useExact, const bool useCompressed)
:camera_index(camera_index),
 useExact(useExact), useCompressed(useCompressed),
 running(false), updateColor(false), updateDepth(false), updatePose(false),
 nh("~"), spinner(4), queueSize(10), it(nh), mode(IMAGE)
{
    cameraMatrixColor = cv::Mat::zeros(3, 3, CV_64F);
    cameraMatrixDepth = cv::Mat::zeros(3, 3, CV_64F);
    distCoeffs = cv::Mat::zeros(1, 5, CV_64F);
    camera_rot = Eigen::Matrix3d::Zero();
    camera_trans = Eigen::Matrix<double, 3, 1>::Zero();
    
    this->topicColor = "/" + ns + "_" + to_string(this->camera_index) + topicColor;
    this->topicDepth = "/" + ns + "_" + to_string(this->camera_index) + topicDepth;
    OUT_INFO("topic color: " FG_CYAN << this->topicColor << NO_COLOR);
    OUT_INFO("topic depth: " FG_CYAN << this->topicDepth << NO_COLOR);
}//构造函数
// spinner 线程对象 （4）4指线程数
ImageProcess::~ImageProcess(){}

void ImageProcess::run(const Mode mode)
{
    start(mode);
    stop();
}

void ImageProcess::start(const Mode mode)
{
    this->mode = mode;
    running = true;

    std::string topicCameraInfoColor = topicColor.substr(0, topicColor.rfind('/')) + "/camera_info"; //camera_info//完整图片的话题路径
    std::string topicCameraInfoDepth = topicDepth.substr(0, topicDepth.rfind('/')) + "/camera_info"; //完整深度图片的话题路径
    OUT_INFO("topic colorinfo: " FG_CYAN << topicCameraInfoColor << NO_COLOR);
    OUT_INFO("topic depthinfo: " FG_CYAN << topicCameraInfoDepth << NO_COLOR);
    std::string humanPoseInfoTopic = "/openpose_ros/human_list_" + to_string(this->camera_index); //新建的一个，openpose关键点话题
    OUT_INFO("human pose topic: " FG_CYAN << humanPoseInfoTopic << NO_COLOR);
    image_transport::TransportHints hints(useCompressed ? "compressed" : "raw");
    //订阅器 就是通过话题路径得到图像或深度图像的信息
    subImageColor = new image_transport::SubscriberFilter(it, topicColor, queueSize, hints);
    subImageDepth = new image_transport::SubscriberFilter(it, topicDepth, queueSize, hints);
    
    // 两个相机的订阅器 分别得到深度相机和彩色相机的内参和外参
    subCameraInfoColor = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, topicCameraInfoColor, queueSize);
    subCameraInfoDepth = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, topicCameraInfoDepth, queueSize);
    
    // 订阅器。 queueSize队列的大小（监听的次数）保存回调函数最近几次的结果， 回调函数 （通过human_keypoints_callback不断来（监听）接受openpose发布的数据（关节点信息））
    // this 自带的指针
    human_keypoints_sub = nh.subscribe(humanPoseInfoTopic, queueSize, &ImageProcess::human_keypoints_callback, this);
   
    // 同步RGB相机和深度相机的数据
    if(useExact)
    {
      syncExact = new message_filters::Synchronizer<ExactSyncPolicy>(ExactSyncPolicy(queueSize), *subImageColor, *subImageDepth, *subCameraInfoColor, *subCameraInfoDepth);
      syncExact->registerCallback(boost::bind(&ImageProcess::callback, this, _1, _2, _3, _4));
    }
    else
    {
      syncApproximate = new message_filters::Synchronizer<ApproximateSyncPolicy>(ApproximateSyncPolicy(queueSize), *subImageColor, *subImageDepth, *subCameraInfoColor, *subCameraInfoDepth);
      syncApproximate->registerCallback(boost::bind(&ImageProcess::callback, this, _1, _2, _3, _4));
    }
    // 开始多线程
    spinner.start();

    std::chrono::milliseconds duration(1);
    // 等到ros准备好就继续 当RGB和深度图相机不敢坐就结束
    // OUT_INFO("r1");
    
    while(!updateColor || !updateDepth)
    {
        if(!ros::ok())
        {
            OUT_INFO("ros !ok...");; 
            return;
        }
    }

    imageViewer();
 
}

void ImageProcess::stop()
{
    spinner.stop();
    ros::waitForShutdown();
    running = false;

    if(useExact){
        delete syncExact;
    }else{
        delete syncApproximate;
    }

    delete subImageColor;
    delete subImageDepth;
    delete subCameraInfoColor;
    delete subCameraInfoDepth;
    delete subHumanPoseInfo;

}
// 相机图像和内参的callback
void ImageProcess::callback(const sensor_msgs::Image::ConstPtr imageColor, const sensor_msgs::Image::ConstPtr imageDepth,
                            const sensor_msgs::CameraInfo::ConstPtr cameraInfoColor, const sensor_msgs::CameraInfo::ConstPtr cameraInfoDepth
                            )
{
    cv::Mat color, depth;
    
    // cout << "1"  << endl;
    readCameraInfo(cameraInfoColor, cameraMatrixColor);
    readCameraInfo(cameraInfoDepth, cameraMatrixDepth);
    // cout << "2"  << endl;
    readImage(imageColor, color);
    readImage(imageDepth, depth);
    // cout << "3"  << endl;

    if(color.type() == CV_16U)
    {
      cv::Mat tmp;
      color.convertTo(tmp, CV_8U, 0.02);
      cv::cvtColor(tmp, color, CV_GRAY2BGR);
    }
  
    lock.lock();
    this->color = color;
    this->depth = depth;

    updateColor = true;
    updateDepth = true;
    lock.unlock();
}

// 可视化RGB图像
void ImageProcess::imageViewer()
{
    cv::Mat color, depth, depthDisp, combined;
    // vector<Pose> pose;

    std::chrono::time_point<std::chrono::high_resolution_clock> start, now; //计时
    double fps = 0;//帧数
    size_t frameCount = 0;
    std::ostringstream oss; //字符流
    const cv::Point pos(5, 18);
    const cv::Scalar colorText = CV_RGB(255, 255, 255);
    const double sizeText = 0.5;
    const int lineText = 1;
    const int font = cv::FONT_HERSHEY_SIMPLEX;
    std::ofstream fout;
    ros::Rate rate(30); //延时  代码的频率
    
    
    // cout << "1"  << endl;
    cv::namedWindow("Image Viewer");
    cv::namedWindow("Depth Viewer");
    oss << "starting...";

    string data_path = "xxx";

    start = std::chrono::high_resolution_clock::now();
    for(; running && ros::ok();)
    {

        if(updateColor)
        {
            // 临时变量
            lock.lock();
            color = this->color;
            depth = this->depth;
            // pose = this->pose; //3.1
            updateColor = false;
            lock.unlock();
            // 统计帧数
            ++frameCount;
            now = std::chrono::high_resolution_clock::now();
            // 统计时间
            double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() / 1000.0;
            // 统计并输出 一秒处理的帧数
            if(elapsed >= 1.0)
            {
                fps = frameCount / elapsed;
                oss.str("");
                oss << "fps: " << fps << " ( " << elapsed / frameCount * 1000.0 << " ms)";
                start = now;
                frameCount = 0;
            }
            //cv::resize(color, color, cv::Size(960, 540));
            // 显示深度图
            dispDepthImage(depth, depthDisp, 12000.0f);
            // combine(color, depthDisp, combined);
           // 显示关节点
            showOpenPosePoints(color, pose);
            
            pose_2d_to_3d(color, depth, pose); //12.29
            //保存关节点
            save(fout,pose);
            
            // 在图像上绘制关节点
            cv::putText(color, oss.str(), pos, font, sizeText, colorText, lineText);
            cv::imshow("Image Viewer", color);
            cv::imshow("Depth Viewer", depthDisp);
            cv::waitKey(3);
        }
        
        ros::spinOnce(); //标准程序
        rate.sleep(); 
    }
    cv::destroyAllWindows();
    cv::waitKey(100);
}
void ImageProcess::save(std::ofstream& fout,vector<Pose>& pose){
    fout.open("/home/xuchengjun/catkin_ws/src/human_depth/data.txt",std::ios::app);
    
    if(!fout.is_open()){
       std::cout<<"打开文件失败！！\n";
       return;
   }
    if(!pose.empty()){
        for(int i=0; i<pose.size(); ++i){
            for(int j=0; j<pose[i].pose_joints.size(); ++j){
                fout << pose[i].pose_joints[j].x << " "<< pose[i].pose_joints[j].y << " "<< pose[i].pose_joints[j].z <<" ";
            }
        fout << endl;
        }
    }
    // if(updatePose){
    //     fout << endl;
    //     updatePose=false;
    // }
    fout.close();
}
// 转换成pose类的 关节点的格式
void ImageProcess::human_keypoints_callback(const human_pose_msgs::HumanList keypoints)
{
    updatePose = true;
    pose.clear();
    person_num=0;
    person_num = keypoints.human_list.size();
    // cout << person_num << endl;

    if(person_num>0){
        for(int per=0; per< person_num; ++per){
            auto body_keypoints = keypoints.human_list[per].body_key_points_prob;
            //去掉假的人
            int count=0;
			double prob_sum = 0.0;
			for(int i=0;i < body_keypoints.size();++i)
			{
				prob_sum += body_keypoints[i].p;
                count++;
			}
            // cout << i << endl;
			double prob_eval = prob_sum/count;

			if(prob_eval < 0.4) //原来是0.4
			{
				continue;
			}

            Pose pose_new;
            // cout << pose_new << endl;
            pose_new.setCameraId(camera_index);
            pose_new.setPose(keypoints, per);
            pose.push_back(pose_new); 
        }
    }
}

// 根据像素点访问深度值
void ImageProcess::getDepthValue(cv::Mat& image, KeyPoint_prob& joint)
{
    //if() return -1;
    //const float badPoint = std::numeric_limits<float>::quiet_NaN();
    float eta = 5.0;
    float count = 1.0;
    float sum = 0;
    // std::ofstream fout;
    // fout.open("/home/xuchengjun/catkin_ws/src/human_depth/data.txt",std::ios::app);
    
    // if(!fout.is_open()){
    //    std::cout<<"打开文件失败！！\n";
    //    return;
    // }
    // joint.y=618;
    // joint.x =618;
    
    for(auto i = joint.y - eta; i < joint.y + eta; ++i){
        for(auto j= joint.x - eta; j < joint.x + eta; ++j){
            float deValue=0;
            if(i < 0 || i >= image.rows || j < 0  || j >= image.cols){
               deValue=0;
               //cout<<image.cols<<image.rows; //输出：1920 1080
            }
            else{
                register const float depthValue = image.ptr<uint16_t>((int)i)[(int)j];
                deValue = depthValue;
            }
            
            // if(depthValue == 0){
            //     //joint.x = joint.y = joint.z = badPoint;
            //     continue;
            // }
            sum += deValue;
            count++;
        }
    }

    joint.z = sum / count / 1000.0;
    // fout << image.ptr<uint16_t>((int)618)[(int)618]/1000.0 <<" ";
    
    // fout.close();
    //cout << "sum  " << sum << "  " << "count  " << count << endl;
    //return joint;
}
// 像素坐标计算到相机坐标系下
void ImageProcess::pixel_to_camera(vector<Pose>& pose)
{
    const float fx = cameraMatrixColor.at<double>(0, 0);
    const float fy = cameraMatrixColor.at<double>(1, 1);
    const float cx = cameraMatrixColor.at<double>(0, 2);
    const float cy = cameraMatrixColor.at<double>(1, 2);
    //cout << fx << "  "  << fy << "  " << cx << "  " << cy << endl;

    if(!pose.empty()){
        for(int i=0; i<pose.size(); ++i){
            for(int j=0; j<pose[i].pose_joints.size(); ++j){

                if(pose[i].pose_joints[j].x != 0 && pose[i].pose_joints[j].y != 0 && pose[i].pose_joints[j].z != 0){
                    pose[i].pose_joints[j].x = (pose[i].pose_joints[j].x - cx * pose[i].pose_joints[j].z) / fx;
                    pose[i].pose_joints[j].y = (pose[i].pose_joints[j].y - cy * pose[i].pose_joints[j].z) / fy;
                }
                cout << "trans later，转换到相机坐标系下" << endl;
                cout << "x:= " << pose[i].pose_joints[j].x << "  "
                 << "y:= " << pose[i].pose_joints[j].y << "  "
                 << "z:= " << pose[i].pose_joints[j].z << "\n";
            }
        cout << endl;
        }
    }
}
// 结合2D关节点信息和深度信息得到3D坐标
void ImageProcess::pose_2d_to_3d(cv::Mat& color, cv::Mat &depth, vector<Pose>& pose_2d)
{
    if(pose_2d.empty()) return;


    for(int i=0; i<pose_2d.size(); ++i){
        for(int j=0; j<pose_2d[i].pose_joints.size(); ++j){

            if(pose_2d[i].pose_joints[j].x == 0 && pose_2d[i].pose_joints[j].y == 0){
                continue;
            }
            
            getDepthValue(depth, pose_2d[i].pose_joints[j]); //此时pose_2d[i].pose_joints[j].z是深度值，x,y则是像素值
            //将（x，y）转化到相机坐标系下
            
            // cout << "x:= " << pose_2d[i].pose_joints[j].x << "  "
            //      << "y:= " << pose_2d[i].pose_joints[j].y << "  "
            //      << "z:= " << pose_2d[i].pose_joints[j].z << "\n";
            
        }

        // cout << endl;
    }
    pixel_to_camera(pose_2d);

}

// 显示

void ImageProcess::showOpenPosePoints(cv::Mat& image, vector<Pose>& pose2d)
{
    if(pose2d.empty()) return;

    for(int i=0; i<pose2d.size(); ++i){
        for(int j=0; j<pose2d[i].pose_joints.size(); ++j){
            if(pose[i].pose_joints[j].x == 0 || pose[i].pose_joints[j].y == 0 || pose[i].pose_joints[j].x > image.cols || pose[i].pose_joints[j].y > image.rows){
                continue;
            }

            cv::circle(image, cv::Point2d(pose2d[i].pose_joints[j].x, pose2d[i].pose_joints[j].y), 4, cv::Scalar(0,255,0), -1);
            std::ostringstream oss;
            oss << pose[i].pose_joints[j].p;
            // putText(image, oss.str(), cv::Point(pose2d[i].pose_joints[j].x+20, pose2d[i].pose_joints[j].y), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0,255,0), 1);
        }
    }
}

// 在callback里读取
void ImageProcess::readImage(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image) const
{
    cv_bridge::CvImageConstPtr pCvImage;
    pCvImage = cv_bridge::toCvShare(msgImage, msgImage->encoding);
    pCvImage->image.copyTo(image);
}
// 在callback里读取相机参数
void ImageProcess::readCameraInfo(const sensor_msgs::CameraInfo::ConstPtr cameraInfo, cv::Mat &cameraMatrix) const
{
    double *itC = cameraMatrix.ptr<double>(0, 0);
    for(size_t i = 0; i < 9; ++i, ++itC)
    {
      *itC = cameraInfo->K[i];
    }
}
// 显示深度图
void ImageProcess::dispDepthImage(cv::Mat& input, cv::Mat& output, const float maxValue)
{
    cv::Mat tmp = cv::Mat(input.rows, input.cols, CV_8U);
    const uint32_t maxInt = 255;

    for(int r = 0; r < input.rows; ++r)
    {
        const uint16_t *itI = input.ptr<uint16_t>(r);
        uint8_t *itO = tmp.ptr<uint8_t>(r);

        for(int c = 0; c < input.cols; ++c, ++itI, ++itO)
        {
            *itO = (uint8_t)std::min((*itI * maxInt / maxValue), 255.0f);
        }
    }

    cv::applyColorMap(tmp, output, cv::COLORMAP_JET);
}

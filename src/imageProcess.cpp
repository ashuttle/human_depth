#include "human_depth/imageProcess.h"
// 参数:
//   相机的ID
//   图像的话题（ros图像的路径）
//   深度图的话题  路径
ImageProcess::ImageProcess(int camera_index, std::string &topicColor, std::string &topicDepth, const bool useExact, const bool useCompressed)
:camera_index(camera_index),
 useExact(useExact), useCompressed(useCompressed),
 calib_start(false), running(false), updateColor(false), updateDepth(false), updatePose(false),
 nh("~"), spinner(4), queueSize(1), it(nh), mode(CLOUD)
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

    std::string topicCameraInfoColor = topicColor.substr(0, topicColor.rfind('/')) + "/camera_info"; //完整图片的话题路径
    std::string topicCameraInfoDepth = topicDepth.substr(0, topicDepth.rfind('/')) + "/camera_info"; //完整深度图片的话题路径


    std::string humanPoseInfoTopic = "/openpose_ros/human_list_" + to_string(this->camera_index); //新建的一个，openpose关键点话题

    image_transport::TransportHints hints(useCompressed ? "compressed" : "raw");
    //订阅器 就是通过话题路径得到图像或深度图像的信息
    subImageColor = new image_transport::SubscriberFilter(it, topicColor, queueSize, hints);
    subImageDepth = new image_transport::SubscriberFilter(it, topicDepth, queueSize, hints);
    // subHumanPoseInfo = new message_filters::subscriber<openpose_ros_msgs::OpenPoseHumanList>(nh, humanPoseInfoTopic, queueSize); 
    
    // 两个相机的订阅器 分别得到深度相机和彩色相机的内参和外参
    subCameraInfoColor = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, topicCameraInfoColor, queueSize);
    subCameraInfoDepth = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, topicCameraInfoDepth, queueSize);
    
    // message_filters::Subscriber<openpose_ros_msgs::OpenPoseHumanList> sub_pose(nh, humanPoseInfoTopic, queueSize, ros::TransportHints().tcpNoDelay());
    
    // 订阅器。 queueSize队列的大小（监听的次数）保存回调函数最近几次的结果， 回调函数 （通过human_keypoints_callback不断来（监听）接受openpose发布的数据（关节点信息））
    // this 自带的指针
    human_keypoints_sub = nh.subscribe(humanPoseInfoTopic, queueSize, &ImageProcess::human_keypoints_callback, this);
   
    //cloud_pub = nh.advertise<pcl::PCLPointCloud2>("human_cloud", 1);
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
    OUT_INFO("starting1...");;
// 开始多线程
    spinner.start();
    // 
    // waitforKinectPose();
    OUT_INFO("starting2...");;
    std::chrono::milliseconds duration(1);
    // 等到ros准备好就继续 当RGB和深度图相机不敢坐就结束
    OUT_INFO("starting3...");;
    while(!updateColor || !updateDepth)
    {
        if(!ros::ok())
        {
            OUT_INFO("ros !ok...");; 
            return;
        }
        std::this_thread::sleep_for(duration);
    }
    OUT_INFO("starting4...");; 
// 新建 一个对象点云 cloud ptr是动态指针 新new一个对象
    cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());

    cloud->height = color.rows;
    cloud->width = color.cols;
    cloud->is_dense = false; //稀疏点云和稠密点云
    cloud->points.resize(cloud->height * cloud->width);
    // 设置点云范围 他的大小是RGB图像的大小
    createLookup(this->color.cols, this->color.rows);

    // 可视化
    switch (mode)
    {
    case IMAGE:
        imageViewer();
        break;
    case CLOUD:
        cloudViewer();
        break;
    case BOTH:
        imageViewerThread = std::thread(&ImageProcess::imageViewer, this); //开启图像显示线程
        cloudViewer();
        break;
    default:
        ROS_INFO("Shit!..");
        break;
    }
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

    if(mode == BOTH){
        imageViewerThread.join();
    }
}
// 相机图像和内外参的callback
void ImageProcess::callback(const sensor_msgs::Image::ConstPtr imageColor, const sensor_msgs::Image::ConstPtr imageDepth,
                            const sensor_msgs::CameraInfo::ConstPtr cameraInfoColor, const sensor_msgs::CameraInfo::ConstPtr cameraInfoDepth
                            )
{
    cv::Mat color, depth;
    //vector<Pose> pose;

    readCameraInfo(cameraInfoColor, cameraMatrixColor);
    readCameraInfo(cameraInfoDepth, cameraMatrixDepth);
    readImage(imageColor, color);
    readImage(imageDepth, depth);
    // IR image input
    if(color.type() == CV_16U)
    {
      cv::Mat tmp;
      color.convertTo(tmp, CV_8U, 0.02);
      cv::cvtColor(tmp, color, CV_GRAY2BGR);
    }

    lock.lock();
    this->color = color;
    this->depth = depth;
    // this->pose = pose;
    updateColor = true;
    updateDepth = true;
    lock.unlock();
}

// 可视化RGB图像
void ImageProcess::imageViewer()
{
    cv::Mat color, depth, depthDisp, combined;
    //vector<Pose> pose;

    std::chrono::time_point<std::chrono::high_resolution_clock> start, now; //计时
    double fps = 0;//帧数
    size_t frameCount = 0;
    std::ostringstream oss; //字符流
    const cv::Point pos(5, 15);
    const cv::Scalar colorText = CV_RGB(255, 255, 255);
    const double sizeText = 0.5;
    const int lineText = 1;
    const int font = cv::FONT_HERSHEY_SIMPLEX;

    ros::Rate rate(30); //延时  代码的频率

    cv::namedWindow("Image Viewer");
    cv::namedWindow("Depth Viewer");
    oss << "starting...";

    start = std::chrono::high_resolution_clock::now();
    for(; running && ros::ok();)
    {
        if(updateColor)
        {
            // 临时变量
            lock.lock();
            color = this->color;
            depth = this->depth;
            pose = this->pose;
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
            //combine(color, depthDisp, combined);
           

            pose_2d_to_3d(color, depth, pose); //12.29
            // sendPoseToRviz(pose); //12.29
             // 显示关节点
            showOpenPosePoints(color, pose);

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
// 显示点云信息 点云空间
void ImageProcess::cloudViewer()
{
    cv::Mat color, depth;
    vector<Pose> pose;
    pcl::visualization::PCLVisualizer::Ptr visualizer(new pcl::visualization::PCLVisualizer("Cloud Viewer"));
    const std::string cloudName = "rendered";

    lock.lock();
    color = this->color;
    depth = this->depth;
    pose = this->pose;
    updateDepth = false;
    lock.unlock();

    //createHumanCloud(depth, color, pose, cloud);
    createCloud(depth, color, cloud);
    //createXYZCloud(depth, color, xyz_cloud);

    visualizer->addPointCloud(cloud, cloudName);
    visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloudName);
    visualizer->initCameraParameters();
    visualizer->setBackgroundColor(0, 0, 0);
    visualizer->setPosition(mode == BOTH ? color.cols : 0, 0);
    visualizer->setSize(color.cols, color.rows);
    visualizer->setShowFPS(true);
    visualizer->setCameraPosition(0, 0, 0, 0, -1, 0);
    //visualizer->registerKeyboardCallback(&Receiver::keyboardEvent, *this);

    for(; running && ros::ok();)
    {
        if(updateDepth)
        {
            lock.lock();
            color = this->color;
            depth = this->depth;
            updateDepth = false;
            lock.unlock();

            //createHumanCloud(depth, color, pose, cloud);
            createCloud(depth, color, cloud);
            //createXYZCloud(depth, color, xyz_cloud);
            //pubCloud(xyz_cloud);
            visualizer->updatePointCloud(cloud, cloudName);
        }
        visualizer->spinOnce(10);
    }
    visualizer->close();
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
// 没有用到
void ImageProcess::createLookup(size_t width, size_t height)
{
    const float fx = 1.0f / cameraMatrixColor.at<double>(0, 0);
    const float fy = 1.0f / cameraMatrixColor.at<double>(1, 1);
    const float cx = cameraMatrixColor.at<double>(0, 2);
    const float cy = cameraMatrixColor.at<double>(1, 2);
    float *it;

    lookupY = cv::Mat(1, height, CV_32F);
    it = lookupY.ptr<float>();
    for(size_t r = 0; r < height; ++r, ++it)
    {
      *it = (r - cy) * fy;
    }

    lookupX = cv::Mat(1, width, CV_32F);
    it = lookupX.ptr<float>();
    for(size_t c = 0; c < width; ++c, ++it)
    {
      *it = (c - cx) * fx;
    }
}
//发布点云
void ImageProcess::pubCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &input) const
{
    // pcl::PCLPointCloud2 output;
    // //pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPoint(new pcl::PointCloud<pcl::PointXYZ>);

    // // pcl::fromPCLPointCloud2(*input, output)
    // // //pcl::PointCloud<pcl::PointXYZRGBA> cloud;
    // // //cloud = *input;
    // output = *input;

    // cloud_pub.publish(output);
}

//cloudViewer中引用 创建一个点云图（利用深度信息和RGB）
void ImageProcess::createCloud(const cv::Mat &depth, const cv::Mat &color, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud) const
{
    const float badPoint = std::numeric_limits<float>::quiet_NaN();

    for(int r = 0; r < depth.rows; ++r)
    {
        pcl::PointXYZRGBA *itP = &cloud->points[r * depth.cols];
        const uint16_t *itD = depth.ptr<uint16_t>(r);
        const cv::Vec3b *itC = color.ptr<cv::Vec3b>(r);
        const float y = lookupY.at<float>(0, r);
        const float *itX = lookupX.ptr<float>();

        for(size_t c = 0; c < (size_t)depth.cols; ++c, ++itP, ++itD, ++itC, ++itX)
        {
            register const float depthValue = *itD / 1000.0f;
            // Check for invalid measurements
            if(*itD == 0)
            {
                // not valid
                itP->x = itP->y = itP->z = badPoint;
                itP->rgba = 0;
                continue;
            }
            itP->z = depthValue;
            itP->x = *itX * depthValue;
            itP->y = y * depthValue;
            if(itP->z < 2.2){
                itP->b = 0;
                itP->g = 255;
                itP->r = 0;
            }else{
                itP->b = itC->val[0];
                itP->g = itC->val[1];
                itP->r = itC->val[2];
            }
            itP->a = 255;
        }
    }
}
// 没用
void ImageProcess::createHumanCloud(cv::Mat &depth, cv::Mat &color, vector<Pose>& pose, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud)
{
    if(pose.empty()) return;

    for(int i=0; i<pose.size(); ++i){
        for(int j=0;j<pose[i].pose_joints.size(); ++j){
            createJointCloud(depth, color, pose[i].pose_joints[j], cloud);
        }
    }
    
}

void ImageProcess::createJointCloud(cv::Mat &depth, cv::Mat &color, KeyPoint_prob& joint, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud)
{
    const float badPoint = std::numeric_limits<float>::quiet_NaN();
    const int D = 5;

    for(int r = joint.y - D; r < joint.y + D; ++r)
    {
        pcl::PointXYZRGBA *itP = &cloud->points[2*D];
        const uint16_t *itD = depth.ptr<uint16_t>(r);
        const cv::Vec3b *itC = color.ptr<cv::Vec3b>(r);
        const float y = lookupY.at<float>(joint.y - D,r);
        const float *itX = lookupX.ptr<float>();

        for(size_t c = joint.x - D; c < joint.x + D; ++c, ++itP, ++itD, ++itC, ++itX)
        {
            register const float depthValue = *itD / 1000.0f;
            // Check for invalid measurements
            if(*itD == 0)
            {
                // not valid
                itP->x = itP->y = itP->z = badPoint;
                continue;
            }
            itP->z = depthValue;
            itP->x = *itX * depthValue;
            itP->y = y * depthValue;
            itP->b = itC->val[0];
            itP->g = itC->val[1];
            itP->r = itC->val[2];
            itP->a = 255;
        }
    }
}


// 没用
void ImageProcess::keyboardEvent(const pcl::visualization::KeyboardEvent &event, void *)
{
    if(event.keyUp()){
        switch (event.getKeyCode())
        {
        case ' ':
            calib_start = true;
            break;
        
        default:
            ROS_INFO("Please enter space to calibration!");
            break;
        }
    }
}
// 监听相机的外参 
void ImageProcess::waitforKinectPose()
{
    geometry_msgs::TransformStamped transform;
    tf2_ros::Buffer tf_Buffer;
    tf2_ros::TransformListener tf_listener(tf_Buffer);
    std::string camera_id = "camera_base_" + to_string(camera_index);

    while(ros::ok()){
    try
    {
        transform = tf_Buffer.lookupTransform("marker_0", camera_id, ros::Time(0));
    }
    catch(tf2::TransformException& ex)
    {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
        continue;
    }
    Eigen::Quaterniond q(transform.transform.rotation.w,
                         transform.transform.rotation.x,
                         transform.transform.rotation.y,
                         transform.transform.rotation.z);
    Eigen::Vector3d trans(transform.transform.translation.x,
                          transform.transform.translation.y,
                          transform.transform.translation.z);
        
        this->camera_rot = q.toRotationMatrix();
        this->camera_trans = trans;
        ROS_INFO("camera_%d pose load successfully!", this->camera_index);
        break;
    } 
    //delete tf_Buffer;
}
// 转换成pose类的 关节点的格式
void ImageProcess::human_keypoints_callback(const human_pose_msgs::HumanList keypoints)
{
    //updatePose = true;
    pose.clear();
    int person_num = keypoints.human_list.size();

    if(person_num>0){
        for(int person=0; person < person_num; ++person){
            auto body_keypoints = keypoints.human_list[person].body_key_points_prob;
            //去掉假的人
			int count = 0;
			double prob_sum = 0.0;
			for(int i=0;i < body_keypoints.size();i++)
			{
				if(body_keypoints[i].p > 0.0)
				{
					prob_sum += body_keypoints[i].p;
					count++;
				}
			}
			double prob_eval = prob_sum/count;

			if(prob_eval < 0.1) //原来是0.4
			{
				continue;
			}

            Pose pose_new;
            
            pose_new.setCameraId(camera_index);
            pose_new.setPose(keypoints, keypoints.human_list[person].human_id);
            pose.emplace_back(pose_new);
            
        }
    }
}

// void ImageProcess::human_boundbox_callback(const human_pose_msgs::BoundingBox& bboxs)
// {
//     // int person_num = bboxs.num_humans;

//     // auto body_bbox = bboxs.x;
// }

// 相机坐标系到世界坐标系
void ImageProcess::camera_to_world(vector<Pose>& pose)
{
    Eigen::Matrix<double, 3, 1> camera_point;
    Eigen::Matrix<double, 3, 1> world_point;

    for(int i=0; i<pose.size(); ++i){
        for(int j=0; j<pose[i].pose_joints.size(); ++j){
            camera_point(0,0) = pose[i].pose_joints[j].x;
            camera_point(1,0) = pose[i].pose_joints[j].y;
            camera_point(2,0) = pose[i].pose_joints[j].z;

            world_point = camera_rot.cast<double>() * camera_point + camera_trans.cast<double>();

            pose[i].pose_joints[j].x = world_point(0,0);
            pose[i].pose_joints[j].y = world_point(1,0);
            pose[i].pose_joints[j].z = world_point(2,0);
            // cout << "world: " << pose[i].pose_joints[j].x << "  " << pose[i].pose_joints[j].y
            // << "  " << pose[i].pose_joints[j].z << "\n";
        }
        //cout << endl;
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

    for(auto i = joint.y - eta; i < joint.y + eta; ++i){
        for(auto j= joint.x - eta; j < joint.x + eta; ++j){
            register const float depthValue = image.ptr<uint16_t>((int)i)[(int)j];
            if(depthValue == 0){
                //joint.x = joint.y = joint.z = badPoint;
                continue;
            }
            sum += depthValue;
            count++;
        }
    }

    joint.z = sum / count / 1000.0;
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

    float *it;
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

        cout << endl;
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

            cv::circle(image, cv::Point2d(pose2d[i].pose_joints[j].x, pose2d[i].pose_joints[j].y), 5, cv::Scalar(255,0,0), 3);
            std::ostringstream oss;
            oss << pose[i].pose_joints[j].p;
            putText(image, oss.str(), cv::Point(pose2d[i].pose_joints[j].x+20, pose2d[i].pose_joints[j].y), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0,255,0), 1);
        }
    }
}
// 发送3D关节点到ros
void ImageProcess::sendPoseToRviz(vector<Pose>& pose_3d)
{
    pixel_to_camera(pose_3d);
    //camera_to_world(pose_3d);

    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;


    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "marker_0";
    transformStamped.child_frame_id = "head";
    tf2::Quaternion q;
    q.setRPY(0,0,0);

    if(pose_3d[0].pose_joints[1].x != 0 && pose_3d[0].pose_joints[1].y != 0 && pose_3d[0].pose_joints[1].z != 0){
        transformStamped.transform.translation.x = pose_3d[0].pose_joints[1].x;
        transformStamped.transform.translation.y = pose_3d[0].pose_joints[1].y;
        transformStamped.transform.translation.z = pose_3d[0].pose_joints[1].z;
    }
    
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    //br.sendTransform(transformStamped);

    // for(int i=0; i<pose_3d.size(); ++i){
    //     pose_3d[i].setJointName();
    //     for(int j=0; j<pose_3d[i].pose_joints.size(); ++j){
    //         transformStamped.header.stamp = ros::Time::now();
    //         transformStamped.header.frame_id = "marker_0";
    //         transformStamped.child_frame_id = "head";

    //         //if(pose_3d[i].pose_joints[j].x != 0 && pose_3d[i].pose_joints[j].x != 0 && pose_3d[i].pose_joints[j].x != 0){
    //             transformStamped.transform.translation.x = pose_3d[i].pose_joints[j].x;
    //             transformStamped.transform.translation.y = pose_3d[i].pose_joints[j].y;
    //             transformStamped.transform.translation.z = pose_3d[i].pose_joints[j].z;
    //         //}
            
    //         transformStamped.transform.rotation.x = q.x();
    //         transformStamped.transform.rotation.y = q.y();
    //         transformStamped.transform.rotation.z = q.z();
    //         transformStamped.transform.rotation.w = q.w();

    //         br.sendTransform(transformStamped);
    //     }
    // }

    // for(auto it=pose_3d[0].joint_name.begin(); it!=pose_3d[0].joint_name.end(); it++){
    //     cout << *it << endl;
    // }
}
#pragma once
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <string>
#include <vector>
#include <cstdio>
#include <mutex>
#include <thread>
#include <cmath>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Core>

#include <ros/ros.h>
#include <ros/spinner.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <kinect2_bridge/kinect2_definitions.h>

#include "human_depth/pose.h"

using namespace std;
using namespace cv;

class ImageProcess
{
public:
    //派生数据类型
    enum Mode
    {
        IMAGE = 0,
        CLOUD,
        BOTH
    };

    ImageProcess(int camera_index, std::string &topicColor, std::string &topicDepth, const bool useExact, const bool useCompressed);
    ~ImageProcess();

    void run(const Mode mode);
    void start(const Mode mode);
    void stop();

    void callback(const sensor_msgs::Image::ConstPtr imageColor, const sensor_msgs::Image::ConstPtr imageDepth,
                const sensor_msgs::CameraInfo::ConstPtr cameraInfoColor, const sensor_msgs::CameraInfo::ConstPtr cameraInfoDepth
                );
    void imageViewer();
    void cloudViewer();
    void createLookup(size_t width, size_t height);
    void createCloud(const cv::Mat &depth, const cv::Mat &color, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud) const;
    void createJointCloud(cv::Mat &depth, cv::Mat &color, KeyPoint_prob& joint, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud);
    void createHumanCloud(cv::Mat& depth, cv::Mat &color, vector<Pose>& pose, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud);
    void pubCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &input)const;
    void keyboardEvent(const pcl::visualization::KeyboardEvent &event, void *);
    void readImage(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image) const;
    void readCameraInfo(const sensor_msgs::CameraInfo::ConstPtr cameraInfo, cv::Mat &cameraMatrix) const;
    void dispDepthImage(cv::Mat& input, cv::Mat& output, const float maxValue);
    void getDepthValue(cv::Mat& image, KeyPoint_prob& joint);



    void pose_2d_to_3d(cv::Mat& color, cv::Mat &depth, vector<Pose>& pose_2d);
    void pixel_to_camera(vector<Pose>& pose);
    void camera_to_world(vector<Pose>& pose);
    void waitforKinectPose();
    void human_keypoints_callback(const human_pose_msgs::HumanList keypoints);
    // void human_boundbox_callback(const openpose_ros_msgs::BoundingBox& bboxs);
    void showOpenPosePoints(cv::Mat& image, vector<Pose>& pose2d);
    void sendPoseToRviz(vector<Pose>& pose_3d);

private:
    vector<Pose> pose;

    std::mutex lock;

    const bool useExact, useCompressed;
    bool calib_start, running, updateColor, updateDepth, updatePose;
    
    int camera_index;
    std::string ns = K2_DEFAULT_NS;
    int queueSize;
    std::string topicColor, topicDepth;
    cv::Mat color, depth;
    cv::Mat cameraMatrixColor, cameraMatrixDepth;
    cv::Mat distCoeffs;
    cv::Mat lookupX, lookupY;
    Eigen::Matrix3d camera_rot;
    Eigen::Matrix<double, 3, 1> camera_trans;

    typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ExactSyncPolicy;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ApproximateSyncPolicy;

    ros::NodeHandle nh;
    ros::Publisher cloud_pub;
    ros::Subscriber human_keypoints_sub;
    ros::AsyncSpinner spinner;  //话题接收线程
    image_transport::ImageTransport it;
    image_transport::SubscriberFilter *subImageColor, *subImageDepth;
    message_filters::Subscriber<sensor_msgs::CameraInfo> *subCameraInfoColor, *subCameraInfoDepth;
    message_filters::Subscriber<human_pose_msgs::HumanList> *subHumanPoseInfo;

    message_filters::Synchronizer<ExactSyncPolicy> *syncExact;
    message_filters::Synchronizer<ApproximateSyncPolicy> *syncApproximate;

    std::thread imageViewerThread;
    Mode mode;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud;
    std::ostringstream oss;
};
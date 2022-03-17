#pragma once 
#include <iostream>
#include <vector>
//#include <memory>

#include<human_pose_msgs/Human.h>
#include<human_pose_msgs/HumanList.h>
#include<human_pose_msgs/PointWithProb.h>
// #include <openpose_ros_msgs/BoundingBox.h>
// #include <openpose_ros_msgs/OpenPoseHuman.h>
// #include <openpose_ros_msgs/OpenPoseHumanList.h>
// #include <openpose_ros_msgs/PointWithProb.h>

using namespace std;

struct KeyPoint_prob{
    float x;
    float y;
    float z;
    float p;
};

struct Pose_3d{
    bool available = false;
    float x;
    float y;
    float z;
};

class Pose
{
public:
    vector<KeyPoint_prob> pose_joints;
    //vector<Pose_3d> pose_joints_3d;
    vector<string> joint_name;

public:
    //typedef std::shared_ptr<Pose> Ptr;
    //typedef std::shared_ptr<Pose const> ConstPtr;

    Pose();
    ~Pose();
    
    void setPose(const human_pose_msgs::HumanList& keypoints, int id);
    void setJointName();
    void setCameraId(int camera_index);
    int getCameraId();

private:  
    int camera_index;
    
};
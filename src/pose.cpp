#include "human_depth/pose.h"

Pose::Pose(){
    pose_joints.resize(18);
    //pose_joints_3d.resize(18);
    joint_name.resize(18);
    //setJointName();
}

Pose::~Pose(){}

void Pose::setCameraId(int camera_index){
    this->camera_index = camera_index;
}

int Pose::getCameraId(){
    return this->camera_index;
}

void Pose::setPose(const human_pose_msgs::HumanList& keypoints, int id)
{
    for(int i = 0; i <18; ++i) {
        pose_joints[i].x = keypoints.human_list[id].body_key_points_prob[i].x;
        pose_joints[i].y = keypoints.human_list[id].body_key_points_prob[i].y;
        pose_joints[i].z = 0;
        pose_joints[i].p = keypoints.human_list[id].body_key_points_prob[i].p;
    }
}

void Pose::setJointName()
{
    joint_name.clear();
    joint_name.emplace_back("head");
    joint_name.emplace_back("neck");
    joint_name.emplace_back("r_shoulder");
    joint_name.emplace_back("r_elbow");
    joint_name.emplace_back("r_hand");
    joint_name.emplace_back("l_shoulder");
    joint_name.emplace_back("l_elbow");
    joint_name.emplace_back("l_hand");
    joint_name.emplace_back("crotch");
    joint_name.emplace_back("r_hip");
    joint_name.emplace_back("r_knee");
    joint_name.emplace_back("r_ankle");
    joint_name.emplace_back("l_hip");
    joint_name.emplace_back("l_knee");
    joint_name.emplace_back("l_ankle");

    //     joint_name.push_back(string("head"));
    // joint_name.push_back(string("neck"));
    // joint_name.push_back("r_shoulder");
    // joint_name.push_back("r_elbow");
    // joint_name.push_back("r_hand");
    // joint_name.push_back("l_shoulder");
    // joint_name.push_back("l_elbow");
    // joint_name.push_back("l_hand");
    // joint_name.push_back("crotch");
    // joint_name.push_back("r_hip");
    // joint_name.push_back("r_knee");
    // joint_name.push_back("r_ankle");
    // joint_name.push_back("l_hip");
    // joint_name.push_back("l_knee");
    // joint_name.push_back("l_ankle");
}
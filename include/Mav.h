#ifndef MAV_H
#define MAV_H
#pragma once

#include <cmath>
#include <cstdio>
#include <sstream>
#include <string>
#include <iostream>

#include <ros/ros.h>
#include "ros/param.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <cstdio>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <cmath>
#include <tf/tf.h>
#include <geometry_msgs/Point.h>
#include <queue>
#include <Eigen/Dense>
#include <std_msgs/Int32.h>
using namespace std;

struct MAV_eigen
{
    Eigen::Vector3d r;        // position
    Eigen::Vector3d r_c;      // camera position
    Eigen::Vector3d v;        // velocity
    Eigen::Vector3d a_imu;    // acceleration from IMU
    Eigen::Vector3d omega_c;  // angular velocity command
    Eigen::Matrix3d R_w2b;    // rotation matrix from world to body frame
    Eigen::Quaterniond q;     // quaternion orientation
};

class MAV
{
private:
    geometry_msgs::PoseStamped MAV_pose;
    ros::Subscriber pose_sub;
    queue<geometry_msgs::PoseStamped> pose_queue;
    int id;
    double roll;
    double pitch;
    double yaw;
public:
    MAV(ros::NodeHandle nh, string subTopic, int ID);
    ~MAV();
    void setCurr_Pose_Vel(std::vector<MAV_eigen> mavs_eigen);
    void setID(int id);
    void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    geometry_msgs::PoseStamped getPose();
    double getYaw();
    int UAV_ID;
    int delay_step;    
    bool gotPose;
};


MAV_eigen mavMsg2Eigen(MAV Mav);

#endif
#include "Mav.h"

MAV::MAV(ros::NodeHandle nh, string subTopic, int ID)
{
    pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(subTopic, 10, &MAV::pose_cb, this);
    id = ID;
    gotPose = false;
    delay_step = 0;
}
MAV::~MAV(){};

void MAV::pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if(!gotPose)
        gotPose = true;

    if(id != UAV_ID)
    {
        pose_queue.push(*msg);
        if(pose_queue.size() >= delay_step)
        {
            MAV_pose = pose_queue.front();
            pose_queue = queue<geometry_msgs::PoseStamped>();
        }
    }
    else
        MAV_pose = *msg;

    tf::Quaternion Q(
        MAV_pose.pose.orientation.x,
        MAV_pose.pose.orientation.y,
        MAV_pose.pose.orientation.z,
        MAV_pose.pose.orientation.w);
    tf::Matrix3x3(Q).getRPY(roll, pitch, yaw);
}

geometry_msgs::PoseStamped MAV::getPose() { return MAV_pose; }
double MAV::getYaw() { return yaw; }
// void MAV::setCurr_Pose_Vel(std::vector<MAV_eigen> mavs_eigen)
// {
//     Mavs_eigen = mavs_eigen;
// }
void MAV::setID(int id)
{
    UAV_ID = id;
}


/*  MAV_eigen   */

MAV_eigen mavMsg2Eigen(MAV Mav)
{
    MAV_eigen Mav_eigen;
    Mav_eigen.r << Mav.getPose().pose.position.x,
                   Mav.getPose().pose.position.y,
                   Mav.getPose().pose.position.z;
    Mav_eigen.v << 0, 0, 0; // Placeholder for velocity
    Mav_eigen.a_imu << 0, 0, 0; // Placeholder for IMU acceleration
    Mav_eigen.omega_c << 0, 0, 0; // Placeholder for angular velocity command
    Mav_eigen.R_w2b = Eigen::Quaterniond(
        Mav.getPose().pose.orientation.w,
        Mav.getPose().pose.orientation.x,
        Mav.getPose().pose.orientation.y,
        Mav.getPose().pose.orientation.z
    ).toRotationMatrix().inverse();
    Mav_eigen.q.w() = Mav.getPose().pose.orientation.w;
    Mav_eigen.q.x() = Mav.getPose().pose.orientation.x;
    Mav_eigen.q.y() = Mav.getPose().pose.orientation.y;
    Mav_eigen.q.z() = Mav.getPose().pose.orientation.z;

    return Mav_eigen;
}
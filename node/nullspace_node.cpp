#include <cmath>
#include <iostream>
#include <queue>
#include <ros/ros.h>
#include "ros/param.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <mavros_msgs/Mavlink.h>
#include <nullspace_control/desire.h>


#include "Nullspace.h"



bool init = false;
bool start = false;
//set control P-gain
double KPx=1, KPy=1, KPz=1.2;
//float KPx=5, KPy=5, KPz=1.2;
double KPyaw = 1;
double roll = 0, pitch = 0, yaw = 0;
geometry_msgs::TwistStamped leader_vel;

int start_all_drone = 0;



geometry_msgs::PoseStamped MAV::getPose(){return MAV_pose;}
double MAV::getYaw(){return yaw;}




void leader_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    leader_vel = *msg;
}

void start_cb(const std_msgs::Int32 msg)
{
    start_all_drone = msg.data;
}

void laplacian_remap(XmlRpc::XmlRpcValue laplacian_param, bool laplacian_map[][5])
{
    int k = 0;
    for(int i = 0; i < 5; i++)
    {
        for(int j = 0; j < 5; j++)
        {
	    ROS_ASSERT(laplacian_param[k].getType() == XmlRpc::XmlRpcValue::TypeInt);
            int a = laplacian_param[k];
	    laplacian_map[i][j] = a!=0;
	    k++;
        }
    }
}

geometry_msgs::TwistStamped vel_limit(geometry_msgs::TwistStamped desired_vel, float limit)
{
    float vel_norm = sqrt(pow(desired_vel.twist.linear.x, 2) + pow(desired_vel.twist.linear.y, 2) + pow(desired_vel.twist.linear.z, 2));
    if(vel_norm > limit)
    {
        desired_vel.twist.linear.x *= limit/vel_norm;
        desired_vel.twist.linear.y *= limit/vel_norm;
        desired_vel.twist.linear.z *= limit/vel_norm;
    }

    return desired_vel;
}

void bound_yaw(double* yaw)
{
    if(*yaw>M_PI)
        *yaw = *yaw - 2*M_PI;
    else if(*yaw<-M_PI)
        *yaw = *yaw + 2*M_PI;
}

void follow_yaw(geometry_msgs::TwistStamped& desired_vel, double current_yaw, double desired_yaw)
{
    double err_yaw, u_yaw;
    err_yaw = desired_yaw - current_yaw;
    bound_yaw( &err_yaw );
    u_yaw = 0.5*err_yaw;
    desired_vel.twist.angular.z = u_yaw;
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "nullspace_ctrl");
    ros::NodeHandle nh;
    
    int ID;
    std::string vehicle;
    ros::param::get("vehicle", vehicle);
    ros::param::get("mav_id", ID);
    
    geometry_msgs::TwistStamped vel_msg;
    nullspace_control::desire q_msg;
    // ros::Publisher vel_cmd_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);
    ros::Publisher q_pub = nh.advertise<nullspace_control::desire>("desire",10);
    ros::Publisher desired_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("desired_velocity_raw", 100);
 
    // MAV mavs[]={MAV(nh, "target", 0 ,0),
    //             MAV(nh, vehicle, 1 ,0) ,
    //             MAV(nh, vehicle, 2 ,0),
    //             MAV(nh, vehicle, 3 ,0)};
    // MAV mavs[5] = {MAV(nh, "/leader_pose", 0),
    //                 MAV(nh, "/MAV1/mavros/local_position/pose_initialized", 1),
    //                 MAV(nh, "/MAV2/mavros/local_position/pose_initialized", 2),
    //             MAV(nh, "/MAV3/mavros/local_position/pose_initialized", 3),
    //             MAV(nh, "/MAV4/mavros/local_position/pose_initialized", 4)};

    MAV mavs[4] = {MAV(nh, "/leader_pose", 0),
                MAV(nh, "/typhoon_h4801/mavros/local_position/pose_initialized", 1),
                MAV(nh, "/typhoon_h4802/mavros/local_position/pose_initialized", 2),
                MAV(nh, "/typhoon_h4803/mavros/local_position/pose_initialized", 3)};
                // MAV(nh, "/MAV4/mavros/local_position/pose_initialized", 4)};
    int mavNum = 3;
    std::vector<MAV_eigen> Mavs_eigen(mavNum+1);
    Eigen::VectorXd q_d;
    q_d.setZero(9);
    q_d(0) = 0;  // Initial position of the leader
    q_d(1) = 0; // Initial position of the leader
    q_d(2) = 10;  // Initial position of the leader
    q_d(3) = M_PI/ 3;  // empty
    q_d(4) = 8;  // Initial distance between MAV1 and MAV3
    q_d(5) = 8;  // Initial distance between MAV1 and MAV2
    q_d(6) = M_PI/ 3;  // Initial angle between MAV1 and MAV2
    q_d(7) = 0;  // Initial angle between MAV1 and MAV3 z
    q_d(8) = 0 ; // Initial angle between MAV2 and MAV3 z
    
    Nullspace nullspace(mavNum);
    nullspace.setID(ID);

    // CMD cmd(nh, ID);

    ros::Rate rate(30);
    
    // while(ros::ok())
    // {
    //     if (!mavs[ID].getState().connected)
    //         break;
    //     printf("wait for UAV_%d FCU connect ...\n" ,ID);
    // }
    for(int i=0; i < 30; i++)
    {
        rate.sleep();
        ros::spinOnce();
    }
    nullspace.set_q_d(q_d);
    
    double yaw_vel;
    Eigen::VectorXd mavs_vel;
    Eigen::Vector3d self_vel;
    Eigen::VectorXd q_test;
    while(ros::ok())
    {
        for(int i=0; i<mavNum +1; i++)
            Mavs_eigen[i] = mavMsg2Eigen(mavs[i]);
        std::cout << "MAVs_eigen leader: " << Mavs_eigen[0].r.transpose() << "\n";
        std::cout << "MAVs_eigen MAV1: " << Mavs_eigen[1].r.transpose() << "\n";
        std::cout << "MAVs_eigen MAV2: " << Mavs_eigen[2].r.transpose() << "\n";
        std::cout << "MAVs_eigen MAV3: " << Mavs_eigen[3].r.transpose() << "\n";
        q_d.segment(0,3) = Mavs_eigen[0].r.segment(0,3);
        nullspace.set_q_d(q_d);
        nullspace.setCurr_Pose_Vel(Mavs_eigen);
        nullspace.compute_q_d_dot();
        yaw_vel = nullspace.computeDesiredYawVelocity();
        // self_vel = nullspace.center_nullspace();
        self_vel = nullspace.computeVel();
        q_test = nullspace.get_q_d_err();
        std::cout << "q_d_err: " << q_test.transpose() << "\n";
        vel_msg.header.stamp = ros::Time::now();
        // vel_msg.twist.linear.x = mavs_vel(mavNum* (ID-1));
        // vel_msg.twist.linear.y = mavs_vel(mavNum* (ID-1)+ 1);
        // vel_msg.twist.linear.z = mavs_vel(mavNum* (ID-1)+ 2);
        vel_msg.twist.linear.x = self_vel(0);
        vel_msg.twist.linear.y = self_vel(1);
        vel_msg.twist.linear.z = self_vel(2);
        vel_msg.twist.angular.z = yaw_vel;
        // q_msg.header.stamp = ros::Time::now();
        // std::cout <<q_test<<"\n";
        std::vector<double> q_vec(q_test.data(),q_test.data() + q_test.size());
        q_msg.q = q_vec;
        q_msg.GT_twist.linear.x = self_vel(0);
        q_msg.GT_twist.linear.y = self_vel(1);
        q_msg.GT_twist.linear.z = self_vel(2);
        q_pub.publish(q_msg);
        desired_vel_pub.publish(vel_msg);
        
        std::cout << vel_msg.twist.linear <<"\n\n";
        rate.sleep();
        ros::spinOnce();
    }

}


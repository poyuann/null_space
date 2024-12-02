#include <cmath>

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
#include <Nullspace_control/desire.h>

#include "Nullspace.h"
#include "state_estimation/Mav.h"

class CMD
{
    private:
        bool arm_cmd;
        int mode_cmd;
        int ID;
        int curr_mode;
        string offb_curr_mode;

        ros::NodeHandle nh;
        ros::ServiceClient arming_client;
        ros::ServiceClient set_mode_client;
        ros::ServiceClient takeoff_client;
    public:
        bool vision_tracking = false;

        CMD(ros::NodeHandle &nh_, int id)
        {
            nh = nh_;
            ID = id;
            offb_curr_mode = "";

            arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
            set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
            takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/takeoff");
        }
        ~CMD(){}
        ros::Subscriber arm_cmd_sub = nh.subscribe<std_msgs::Bool>("/formation/all_uav_arm", 5, &CMD::arm_cmd_cb, this);
        ros::Subscriber mode_cmd_sub = nh.subscribe<std_msgs::Int32>("/formation/all_uav_mode", 5, &CMD::mode_cmd_cb, this);
        void arm_cmd_cb(const std_msgs::Bool::ConstPtr& msg)
        {
            arm_cmd = msg->data;
            setArm(arm_cmd);
        }
        void mode_cmd_cb(const std_msgs::Int32::ConstPtr& msg)
        {
            mode_cmd = msg->data;
            setMode(mode_cmd);
        }
        void setArm(bool arm_CMD)
        {
            mavros_msgs::CommandBool arm_cmd;
            arm_cmd.request.value = arm_CMD;
            if( arming_client.call(arm_cmd) && arm_cmd.response.success) 
                ROS_INFO("UAV_%i armed switch successfully", ID);
            else
                ROS_INFO("UAV_%i failed to arm", ID);
        }
        void setMode(int mode_CMD)
        {
            mavros_msgs::SetMode offb_set_mode;
            switch (mode_CMD)
            {
            case 0: 
                offb_set_mode.request.custom_mode = "STABILIZED";
                break;
            case 1: 
                offb_set_mode.request.custom_mode = "AUTO.TAKEOFF";
                break;
            case 2: 
                offb_set_mode.request.custom_mode = "AUTO.LAND";
                break;
            case 3:
                offb_set_mode.request.custom_mode = "OFFBOARD";
                break;
            case 5:
                if(!vision_tracking)
                {
                    ROS_INFO("Tracking by vision");
                    vision_tracking = true;
                }
                else
                {
                    ROS_INFO("Stop vision tracking");
                    vision_tracking = false;
                }
                break;
            
            default:
                break;
            }
            if(offb_set_mode.request.custom_mode != offb_curr_mode)
            {
                if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                {
                    ROS_INFO("UAV_%i mode switched to %s", ID, offb_set_mode.request.custom_mode.c_str());
                    offb_curr_mode = offb_set_mode.request.custom_mode;
                    if(offb_set_mode.request.custom_mode == "AUTO.TAKEOFF")
                        sleep(5);
                }
                else
                    ROS_INFO("UAV_%i failed to switch offb_mode", ID);
            }
        }
        int mode_CMD(){return mode_cmd;}
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nullspace_ctrl");
    ros::NodeHandle nh;
    
    int ID;
    std::string vehicle;
    ros::param::get("vehicle", vehicle);
    ros::param::get("mav_id", ID);
    
    geometry_msgs::TwistStamped vel_msg;
    Nullspace_control::desire q_msg;
    ros::Publisher vel_cmd_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);
    ros::Publisher q_pub = nh.advertise<Nullspace_control::desire>("desire",10);

    MAV mavs[]={MAV(nh, "target", 0 ,0),
                MAV(nh, vehicle, 1 ,0) ,
                MAV(nh, vehicle, 2 ,0),
                MAV(nh, vehicle, 3 ,0)};
    int mavNum = 3;
    std::vector<MAV_eigen> Mavs_eigen(mavNum+1);
    Eigen::VectorXd q_d;
    q_d.setZero(9);
    q_d(0) = 0;
    q_d(1) = 0;
    q_d(2) = 10;
    q_d(3) = M_PI/ 3;
    q_d(4) = 6;
    q_d(5) = 6;
    q_d(6) = M_PI/ 6;
    q_d(7) = 0;
    q_d(8) = M_PI/4;  
    
    Nullspace nullspace(mavNum);
    nullspace.setID(ID);

    CMD cmd(nh, ID);

    ros::Rate rate(30);
    
    while(ros::ok())
    {
        if (!mavs[ID].getState().connected)
            break;
        printf("wait for UAV_%d FCU connect ...\n" ,ID);
    }
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
        q_d.segment(0,3) = Mavs_eigen[0].r.segment(0,3);
        nullspace.set_q_d(q_d);
        nullspace.setCurr_Pose_Vel(Mavs_eigen);
        nullspace.compute_q_d_dot();
        yaw_vel = nullspace.computeDesiredYawVelocity();
        // self_vel = nullspace.center_nullspace();
        self_vel = nullspace.computeVel();
        q_test = nullspace.get_q_d_err();

        vel_msg.header.stamp = ros::Time::now();
        // vel_msg.twist.linear.x = mavs_vel(mavNum* (ID-1));
        // vel_msg.twist.linear.y = mavs_vel(mavNum* (ID-1)+ 1);
        // vel_msg.twist.linear.z = mavs_vel(mavNum* (ID-1)+ 2);
        vel_msg.twist.linear.x = self_vel(0);
        vel_msg.twist.linear.y = self_vel(1);
        vel_msg.twist.linear.z = self_vel(2);
        vel_msg.twist.angular.z = yaw_vel;
        q_msg.header.stamp = ros::Time::now();
        // std::cout <<q_test<<"\n";
        std::vector<double> q_vec(q_test.data(),q_test.data() + q_test.size());
        q_msg.q = q_vec;
        q_msg.GT_twist.linear.x = self_vel(0);
        q_msg.GT_twist.linear.y = self_vel(1);
        q_msg.GT_twist.linear.z = self_vel(2);
        q_pub.publish(q_msg);
        vel_cmd_pub.publish(vel_msg);
        std::cout << vel_msg.twist.linear <<"\n\n";
        rate.sleep();
        ros::spinOnce();
    }

}


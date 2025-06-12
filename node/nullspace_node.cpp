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


bool init = false;
bool start = false;
//set control P-gain
double KPx=1, KPy=1, KPz=1.2;
//float KPx=5, KPy=5, KPz=1.2;
double KPyaw = 1;
double roll = 0, pitch = 0, yaw = 0;
geometry_msgs::TwistStamped leader_vel;

int start_all_drone = 0;

struct MAV_eigen
{
	Eigen::Vector3d r;
    Eigen::Vector3d r_c;
	Eigen::Vector3d v;
	Eigen::Vector3d a_imu;
	Eigen::Vector3d omega_c;
	Eigen::Matrix3d R_w2b;
    Eigen::Quaterniond q;
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
    void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    geometry_msgs::PoseStamped getPose();
    double getYaw();
    static int UAV_ID;
    static int delay_step;    
    bool gotPose;
};

int MAV::UAV_ID = 0;
int MAV::delay_step = 0;

MAV::MAV(ros::NodeHandle nh, string subTopic, int ID)
{
    pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(subTopic, 10, &MAV::pose_cb, this);
    id = ID;
    gotPose = false;
}

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
    tf::Matrix3x3(Q).getRPY(roll,pitch,yaw);
}
MAV_eigen mavMsg2Eigen(MAV Mav)
{
	MAV_eigen Mav_eigen;
    // std::cout << Mav.getPose().pose << std::endl;
	Mav_eigen.r(0) = Mav.getPose().pose.position.x;
	Mav_eigen.r(1) = Mav.getPose().pose.position.y;
	Mav_eigen.r(2) = Mav.getPose().pose.position.z;
	Mav_eigen.v(0) = Mav.getVel().twist.linear.x;
	Mav_eigen.v(1) = Mav.getVel().twist.linear.y;
	Mav_eigen.v(2) = Mav.getVel().twist.linear.z;
	Mav_eigen.a_imu(0) = Mav.getAcc().x;
	Mav_eigen.a_imu(1) = Mav.getAcc().y;
	Mav_eigen.a_imu(2) = Mav.getAcc().z;
	
	Mav_eigen.omega_c(0) = Mav.getVel().twist.angular.x;
	Mav_eigen.omega_c(1) = Mav.getVel().twist.angular.y;
	Mav_eigen.omega_c(2) = Mav.getVel().twist.angular.z;
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
    
    Mav_eigen.r_c = Mav_eigen.r + Mav_eigen.R_w2b*Mav.getCamera().t_B2C();
	return Mav_eigen;
}

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
    Nullspace_control::desire q_msg;
    // ros::Publisher vel_cmd_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);
    ros::Publisher q_pub = nh.advertise<Nullspace_control::desire>("desire",10);
    ros::Publisher desired_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("desired_velocity_raw", 100);
 
    // MAV mavs[]={MAV(nh, "target", 0 ,0),
    //             MAV(nh, vehicle, 1 ,0) ,
    //             MAV(nh, vehicle, 2 ,0),
    //             MAV(nh, vehicle, 3 ,0)};
    MAV mav[5] = {MAV(nh, "/leader_pose", 0),
                    MAV(nh, "/MAV1/mavros/local_position/pose_initialized", 1),
                    MAV(nh, "/MAV2/mavros/local_position/pose_initialized", 2),
                MAV(nh, "/MAV3/mavros/local_position/pose_initialized", 3),
                MAV(nh, "/MAV4/mavros/local_position/pose_initialized", 4)};
    int mavNum = 3;
    std::vector<MAV_eigen> Mavs_eigen(mavNum+1);
    Eigen::VectorXd q_d;
    q_d.setZero(9);
    q_d(0) = 0;  // Initial position of the leader
    q_d(1) = 0; // Initial position of the leader
    q_d(2) = 10;  // Initial position of the leader
    q_d(3) = M_PI/ 3;  // empty
    q_d(4) = 6;  // Initial distance between MAV1 and MAV3
    q_d(5) = 6;  // Initial distance between MAV1 and MAV2
    q_d(6) = M_PI/ 3;  // Initial angle between MAV1 and MAV2
    q_d(7) = 0;  // Initial angle between MAV1 and MAV3 z
    q_d(8) = 0 ; // Initial angle between MAV2 and MAV3 z
    
    Nullspace nullspace(mavNum);
    nullspace.setID(ID);

    // CMD cmd(nh, ID);

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
        desired_vel_pub.publish(vel_msg);
        std::cout << vel_msg.twist.linear <<"\n\n";
        rate.sleep();
        ros::spinOnce();
    }

}


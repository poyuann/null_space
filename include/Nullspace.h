#ifndef NULLSPACE_H
#define NULLSPACE_H
#include <vector>
#include <Eigen/Dense>
#include "state_estimation/Mav.h"
#include <tf/tf.h>
#include <Eigen/QR>    

class Nullspace
{
    private:
        int mav_num;
        Eigen::MatrixXd J_s;
        Eigen::MatrixXd J_p;
        Eigen::VectorXd q_d;
        Eigen::VectorXd q_d_dot;
        std::vector<MAV_eigen> Mavs_eigen;
        double yaw;
        int ID;
    public:
        Nullspace(int mav_num);
        ~Nullspace();
        void setID(int);
        void setCurr_Pose_Vel(std::vector<MAV_eigen> mavs_eigen);
        void set_q_d(Eigen::VectorXd);
        void compute_q_d_dot();
        double bound_angle(double);
        double computeDesiredYawVelocity();
        Eigen::VectorXd computeVel();
        Eigen::VectorXd center_nullspace();
        Eigen::VectorXd get_q_d_err();
};
#endif
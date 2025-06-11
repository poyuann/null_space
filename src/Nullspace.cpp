#include "Nullspace.h"

Nullspace::Nullspace(int mavNum)
{
    mav_num = mavNum;
    J_s.setZero(5, 3* mav_num);
    J_p.setZero(4, 3* mav_num);
    q_d.setZero(9);
    q_d_dot.setZero(9);
}
Nullspace::~Nullspace(){}

double Nullspace::bound_angle(double angle)
{
    if(angle > M_PI)
        angle = angle - 2*M_PI;
    else if(angle < -M_PI)
        angle = angle + 2*M_PI;
    // std::cout << angle <<"\n";
    return angle;
}
void Nullspace::setCurr_Pose_Vel(std::vector<MAV_eigen> mavs_eigen)
{
    Mavs_eigen = mavs_eigen;
    Mavs_eigen[ID].q;
    tf::Quaternion Q(
    Mavs_eigen[ID].q.x(),
    Mavs_eigen[ID].q.y(),
    Mavs_eigen[ID].q.z(),
    Mavs_eigen[ID].q.w());
    yaw = tf::getYaw(Q);
}
void Nullspace::setID(int id)
{
    ID = id;
}
void Nullspace::set_q_d(Eigen::VectorXd init_q_d)
{
    q_d = init_q_d;
}
void Nullspace::compute_q_d_dot()
{   
    double d_1, d_2, d_3;
    Eigen::Vector3d r13, r12, r32;
    r13 = Mavs_eigen[1].r.segment(0, 3) - Mavs_eigen[3].r.segment(0, 3);
    r12 = Mavs_eigen[1].r.segment(0, 3) - Mavs_eigen[2].r.segment(0, 3);
    r32 = Mavs_eigen[3].r.segment(0, 3) - Mavs_eigen[2].r.segment(0, 3);
    d_1 = r13.norm();
    d_2 = r12.norm();
    d_3 = r32.norm();
    
    q_d_dot(0) = tanh(q_d(0) - ((Mavs_eigen[1].r(0)+ Mavs_eigen[2].r(0)+ Mavs_eigen[3].r(0))/3));
    q_d_dot(1) = tanh(q_d(1) - ((Mavs_eigen[1].r(1)+ Mavs_eigen[2].r(1)+ Mavs_eigen[3].r(1))/3));
    q_d_dot(2) = tanh(q_d(2) - ((Mavs_eigen[1].r(2)+ Mavs_eigen[2].r(2)+ Mavs_eigen[3].r(2))/3));
    // q_d_dot(3) = tanh(bound_angle((q_d(3) - atan2(r32(1),r32(0)))));
    // q_d_dot(3) = bound_angle((q_d(3) - atan2(Mavs_eigen[3].r(1),Mavs_eigen[3].r(0))));

    q_d_dot(4) = 3*tanh(q_d(4) - r13.norm());
    q_d_dot(5) = 3*tanh(q_d(5) - r12.norm());
    // q_d_dot(5) = 3*tanh(q_d(5) - r32.norm());
    q_d_dot(6) = tanh(q_d(6) - acos((d_1*d_1 + d_2*d_2 - d_3*d_3)/ (2*d_1*d_2))); 
    q_d_dot(7) = sin(bound_angle(q_d(7) - atan2(r32(2), r32(0))));
    q_d_dot(8) = tanh(bound_angle(q_d(8) - atan2(r12(2), r12(0))));
    // std::cout << q_d_dot.transpose() << "\n";
}
Eigen::VectorXd Nullspace::get_q_d_err()
{
    double d_1, d_2, d_3;
    Eigen::Vector3d r13, r12, r32;
    r13 = Mavs_eigen[1].r.segment(0, 3) - Mavs_eigen[3].r.segment(0, 3);
    r12 = Mavs_eigen[1].r.segment(0, 3) - Mavs_eigen[2].r.segment(0, 3);
    r32 = Mavs_eigen[3].r.segment(0, 3) - Mavs_eigen[2].r.segment(0, 3);
    d_1 = r13.norm();
    d_2 = r12.norm();
    d_3 = r32.norm();

    Eigen::VectorXd q_test;
    q_test.setZero(9);
    q_test(0) = q_d(0) - ((Mavs_eigen[1].r(0)+ Mavs_eigen[2].r(0)+ Mavs_eigen[3].r(0))/3);
    q_test(1) = q_d(1) - ((Mavs_eigen[1].r(1)+ Mavs_eigen[2].r(1)+ Mavs_eigen[3].r(1))/3);
    q_test(2) = q_d(2) - ((Mavs_eigen[1].r(2)+ Mavs_eigen[2].r(2)+ Mavs_eigen[3].r(2))/3);
    q_test(3) = bound_angle((q_d(3) - atan2(r32(1),r32(0))));
    q_test(4) = q_d(4) - r13.norm();
    q_test(5) = q_d(5) - r12.norm();
    q_test(6) = q_d(6) - acos((d_1*d_1 + d_2*d_2 - d_3*d_3)/ (2*d_1*d_2));
    q_test(7) = sin(q_d(7) - atan2(r32(2), r32(0)));
    q_test(8) = q_d(8) - atan2(r12(2), r12(0));
    return q_test;
}
Eigen::VectorXd Nullspace::computeVel()
{
    double d_1, d_2, d_3, beta;
    Eigen::Vector3d r13, r12, r32;
    r13 = Mavs_eigen[1].r.segment(0, 3) - Mavs_eigen[3].r.segment(0, 3);
    r12 = Mavs_eigen[1].r.segment(0, 3) - Mavs_eigen[2].r.segment(0, 3);
    r32 = Mavs_eigen[3].r.segment(0, 3) - Mavs_eigen[2].r.segment(0, 3);
    d_1 = r13.norm();
    d_2 = r12.norm();
    d_3 = r32.norm();

    J_s(0,0) = r13(0)/ d_1;
    J_s(0,1) = r13(1)/ d_1;
    J_s(0,2) = r13(2)/ d_1;
    J_s(0,6) = -r13(0)/ d_1;
    J_s(0,7) = -r13(1)/ d_1;
    J_s(0,8) = -r13(2)/ d_1;
    J_s(1,0) = r12(0)/ d_2;
    J_s(1,1) = r12(1)/ d_2;
    J_s(1,2) = r12(2)/ d_2;
    J_s(1,3) = -r12(0)/ d_2;
    J_s(1,4) = -r12(1)/ d_2;
    J_s(1,5) = -r12(2)/ d_2;
    J_s(2,0) = (-1/(sqrt(1-((d_1* d_1+ d_2* d_2- d_3* d_3)*(d_1* d_1+ d_2* d_2- d_3* d_3)/(4* d_1* d_2*d_1*d_2)))))
                * (d_1* d_2* (2* r13(0)+ 2* r12(0))-(r13(0)* d_2/ d_1+ r12(0)* d_1/ d_2)*(d_1* d_1 + d_2* d_2- d_3* d_3))/ (d_1* d_1* d_2*d_2);
    J_s(2,1) = (-1/(sqrt(1-((d_1* d_1+ d_2* d_2- d_3* d_3)/(2* d_1* d_2*d_1*d_2)))))
                * (d_1* d_2* (2* r13(1)+ 2* r12(1))-(r13(1)* d_2/ d_1+ r12(1)* d_1/ d_2)*(d_1* d_1 + d_2* d_2- d_3* d_3))/ (d_1* d_1* d_2*d_2);
    J_s(2,2) = (-1/(sqrt(1-((d_1* d_1+ d_2* d_2- d_3* d_3)/(2* d_1* d_2*d_1*d_2)))))
                * (d_1* d_2* (2* r13(2)+ 2* r12(2))-(r13(2)* d_2/ d_1+ r12(2)* d_1/ d_2)*(d_1* d_1 + d_2* d_2- d_3* d_3))/ (d_1* d_1* d_2*d_2);
    J_s(2,3) = (-1/(sqrt(1-((d_1* d_1+ d_2* d_2- d_3* d_3)/(2* d_1* d_2*d_1*d_2)))))
                * (d_1* d_2* (2* r32(0)- 2* r12(0))- (-r12(0)* d_1/ d_2)* (d_1* d_1 + d_2* d_2- d_3* d_3))/ (d_1* d_1* d_2*d_2);
    J_s(2,4) = (-1/(sqrt(1-((d_1* d_1+ d_2* d_2- d_3* d_3)/(2* d_1* d_2*d_1*d_2)))))
                * (d_1* d_2* (2* r32(1)- 2* r12(1))- (-r12(1)* d_1/ d_2)* (d_1* d_1 + d_2* d_2- d_3* d_3))/ (d_1* d_1* d_2*d_2);
    J_s(2,5) = (-1/(sqrt(1-((d_1* d_1+ d_2* d_2- d_3* d_3)/(2* d_1* d_2*d_1*d_2)))))
                * (d_1* d_2* (2* r32(2)- 2* r12(2))- (-r12(2)* d_1/ d_2)* (d_1* d_1 + d_2* d_2- d_3* d_3))/ (d_1* d_1* d_2*d_2);
    J_s(2,6) = (-1/(sqrt(1-((d_1* d_1+ d_2* d_2- d_3* d_3)/(2* d_1* d_2*d_1*d_2)))))
                * (d_1* d_2* (-2* r32(0)- 2* r13(0))- (-r13(0)* d_2/ d_1)* (d_1* d_1 + d_2* d_2- d_3* d_3))/ (d_1* d_1* d_2*d_2);
    J_s(2,7) = (-1/(sqrt(1-((d_1* d_1+ d_2* d_2- d_3* d_3)/(2* d_1* d_2*d_1*d_2)))))
                * (d_1* d_2* (-2* r32(1)- 2* r13(1))- (-r13(1)* d_2/ d_1)* (d_1* d_1 + d_2* d_2- d_3* d_3))/ (d_1* d_1* d_2*d_2);
    J_s(2,8) = (-1/(sqrt(1-((d_1* d_1+ d_2* d_2- d_3* d_3)/(2* d_1* d_2*d_1*d_2)))))
                * (d_1* d_2* (-2* r32(2)- 2* r12(2))- (-r13(2)* d_2/ d_1)* (d_1* d_1 + d_2* d_2- d_3* d_3))/ (d_1* d_1* d_2*d_2);

    // J_s(3,0) = -r13(2)/ (r13(0)* r13(0)+ r13(2)* r13(2));
    // J_s(3,2) = r13(0)/ (r13(0)* r13(0)+ r13(2)* r13(2));
    // J_s(3,6) = r13(2)/ (r13(0)* r13(0)+ r13(2)* r13(2));
    // J_s(3,8) = -r13(0)/ (r13(0)* r13(0)+ r13(2)* r13(2));
    J_s(4,0) = -r12(2)/ (r12(0)* r12(0)+ r12(2)* r12(2));
    J_s(4,2) = r12(0)/ (r12(0)* r12(0)+ r12(2)* r12(2));
    J_s(4,6) = r12(2)/ (r12(0)* r12(0)+ r12(2)* r12(2));
    J_s(4,8) = -r12(0)/ (r12(0)* r12(0)+ r12(2)* r12(2));
    J_s(3,3) = r32(2)/ (r32(0)* r32(0)+ r32(2)* r32(2));
    J_s(3,5) = -r32(0)/ (r32(0)* r32(0)+ r32(2)* r32(2));
    J_s(3,6) = -r32(2)/ (r32(0)* r32(0)+ r32(2)* r32(2));
    J_s(3,8) = r32(0)/ (r32(0)* r32(0)+ r32(2)* r32(2));

    J_p(0,0) = 1.0/3;
    J_p(0,3) = 1.0/3;
    J_p(0,6) = 1.0/3;
    J_p(1,1) = 1.0/3;
    J_p(1,4) = 1.0/3;
    J_p(1,7) = 1.0/3;
    J_p(2,2) = 1.0/3;
    J_p(2,5) = 1.0/3;
    J_p(2,8) = 1.0/3;
    // J_p(3,3) = r32(1)/ (r32(0)* r32(0)+ r32(1)* r32(1));
    // J_p(3,4) = -r32(0)/ (r32(0)* r32(0)+ r32(1)* r32(1));
    // J_p(3,6) = -r32(1)/ (r32(0)* r32(0)+ r32(1)* r32(1));
    // J_p(3,7) = r32(0)/ (r32(0)* r32(0)+ r32(1)* r32(1));
    J_p(3,6) = -Mavs_eigen[3].r(1)/(Mavs_eigen[3].r(0) * Mavs_eigen[3].r(0) + Mavs_eigen[3].r(1) * Mavs_eigen[3].r(1));
    J_p(3,7) = Mavs_eigen[3].r(0)/(Mavs_eigen[3].r(0) * Mavs_eigen[3].r(0) + Mavs_eigen[3].r(1) * Mavs_eigen[3].r(1));

    Eigen::MatrixXd pinvJ_s = J_s.completeOrthogonalDecomposition().pseudoInverse();
    Eigen::MatrixXd pinvJ_p = J_p.completeOrthogonalDecomposition().pseudoInverse();
    Eigen::MatrixXd I(9,9);
    I.setIdentity();
    Eigen::VectorXd v_r, q_s_dot, q_p_dot;
    q_p_dot = q_d_dot.segment(0,4);
    q_s_dot = q_d_dot.segment(4,5);
    v_r = pinvJ_s* q_s_dot + (I - pinvJ_s* J_s)* pinvJ_p* q_p_dot; 
    // v_r = pinvJ_p *q_p_dot + (I - pinvJ_p* J_p)* pinvJ_s* q_s_dot; 
    // std::cout << pinvJ_s << "\n\n";
    return v_r.segment(mav_num* (ID-1),3);
}   
Eigen::VectorXd Nullspace::center_nullspace()
{
    double d_1, d_2, d_3;
    Eigen::Vector3d r13, r12, r32;
    r13 = Mavs_eigen[1].r.segment(0, 3) - Mavs_eigen[3].r.segment(0, 3);
    r12 = Mavs_eigen[1].r.segment(0, 3) - Mavs_eigen[2].r.segment(0, 3);
    r32 = Mavs_eigen[3].r.segment(0, 3) - Mavs_eigen[2].r.segment(0, 3);
    d_1 = r13.norm();
    d_2 = r12.norm();
    d_3 = r32.norm();
    Eigen::MatrixXd J1,J2;
    J1.setZero(3,9);
    J2.setZero(3,9);

    J1(0,0) = 1.0/3;
    J1(0,3) = 1.0/3;
    J1(0,6) = 1.0/3;
    J1(1,1) = 1.0/3;
    J1(1,4) = 1.0/3;
    J1(1,7) = 1.0/3;
    J1(2,2) = 1.0/3;
    J1(2,5) = 1.0/3;
    J1(2,8) = 1.0/3;

    J2(0,0) = r13(0)/ d_1;
    J2(0,1) = r13(1)/ d_1;
    J2(0,2) = r13(2)/ d_1;
    J2(0,6) = -r13(0)/ d_1;
    J2(0,7) = -r13(1)/ d_1;
    J2(0,8) = -r13(2)/ d_1;
    J2(1,0) = r12(0)/ d_2;
    J2(1,1) = r12(1)/ d_2;
    J2(1,2) = r12(2)/ d_2;
    J2(1,3) = -r12(0)/ d_2;
    J2(1,4) = -r12(1)/ d_2;
    J2(1,5) = -r12(2)/ d_2;
    J2(2,3) = -r32(0)/ d_3;
    J2(2,4) = -r32(1)/ d_3;
    J2(2,5) = -r32(2)/ d_3;
    J2(2,6) = r32(0)/ d_3;
    J2(2,7) = r32(1)/ d_3;
    J2(2,8) = r32(2)/ d_3;

    Eigen::MatrixXd pinvJ_p = J1.completeOrthogonalDecomposition().pseudoInverse();
    Eigen::MatrixXd pinvJ_s = J2.completeOrthogonalDecomposition().pseudoInverse();

    Eigen::MatrixXd I(9,9);
    I.setIdentity();
    Eigen::VectorXd v_r, q_s_dot, q_p_dot;
    q_p_dot = q_d_dot.segment(0,3);
    q_s_dot = q_d_dot.segment(3,3);
    // v_r = pinvJ_p *q_p_dot + (I - pinvJ_p* J1)* pinvJ_s* q_s_dot; 
    v_r = pinvJ_s* q_s_dot + (I - pinvJ_s* J2)* pinvJ_p* q_p_dot; 
    std::cout << "J_p\n"<< J1<<"\npinvJ_p \n" << pinvJ_p  <<"\n\n";
    return  v_r.segment(mav_num* (ID-1),3);
}

double Nullspace::computeDesiredYawVelocity()
{   

    double desired_yaw = 0 ;//atan2(Mavs_eigen[0].r(1) - Mavs_eigen[ID].r(1), Mavs_eigen[0].r(0) - Mavs_eigen[ID].r(0));
    double error_yaw = desired_yaw - yaw;
    if(error_yaw>M_PI)
        error_yaw = error_yaw - 2*M_PI;
    else if(error_yaw<-M_PI)
        error_yaw = error_yaw + 2*M_PI;

    return error_yaw;
}
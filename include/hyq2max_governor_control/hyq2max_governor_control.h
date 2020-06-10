#ifndef HYQ2MAX_GOVERNOR_CONTROL_H
#define HYQ2MAX_GOVERNOR_CONTROL_H

#include <hyq2max_governor_control/hyq2max_jacobian_functions.h>
#include <Eigen/Dense>


double joint_Kp;
double joint_Kd;
double hyq2max_mass;
double gravity_value;
double Ts = 0.1;
Eigen::Matrix<double,3,12> I_sigma;
Eigen::Matrix<double,12,12> Jc;
Eigen::Matrix<double,3,12> J;
Eigen::Matrix<double,12,3> J_plus;
Eigen::Matrix<double,3,1> gravity_vector;


void setup_values(double, double, double, double, double);

void update_ss_matrices(Eigen::Matrix<double,16,16> *, Eigen::Matrix<double,16,12> *, Eigen::Matrix<double,16,16> *, Eigen::Matrix<double,16,12> *, Eigen::Matrix<double,28,28> *, Eigen::Matrix<double,28,12> *, Eigen::Matrix<double,12,28> *, Eigen::Matrix<double,3,12> *, Eigen::Matrix<double,3,12> *);
//void update_ss_matrices(Eigen::Matrix<double,16,16> *, Eigen::Matrix<double,16,12> *, Eigen::Matrix<double,16,16> *, Eigen::Matrix<double,16,12> *, Eigen::Matrix<double,3,12> *, Eigen::Matrix<double,3,12> *);

void update_opt_matrices(Eigen::Matrix<double,28,28> *, Eigen::Matrix<double,28,12> *, Eigen::Matrix<double,12,28> *, Eigen::MatrixXd *, Eigen::MatrixXd *, double, double);

#endif
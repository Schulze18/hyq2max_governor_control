#ifndef HYQ2MAX_GOVERNOR_CONTROL_H
#define HYQ2MAX_GOVERNOR_CONTROL_H

#include <hyq2max_governor_control/hyq2max_jacobian_functions.h>
#include <Eigen/Dense>


double joint_Kp;
double joint_Kd;
double hyq2max_mass;
Eigen::Matrix<double,3,12> I_sigma;
Eigen::Matrix<double,12,12> Jc;
Eigen::Matrix<double,3,12> J;
Eigen::Matrix<double,12,3> J_plus;

void setup_values(double, double, double);
void update_ss_matrices(Eigen::Matrix<double,3,12> *, Eigen::Matrix<double,3,12> *);

#endif
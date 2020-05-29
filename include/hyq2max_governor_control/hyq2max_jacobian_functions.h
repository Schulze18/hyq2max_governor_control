#ifndef HYQ2MAX_JACOBIAN_FUNCTIONS_H
#define HYQ2MAX_JACOBIAN_FUNCTIONS_H

#include <Eigen/Dense>

const double c1 = 0.605e-3;
const double c2 = 0.1346e-2;
const double c3 = 0.144296e0;
const double c4 = 0.2071e-2;
const double c5 = 0.7214800000e-1;
const double c6 = 0.1035500000e-2;
const double c7 = 0.1090150000e0;
const double c8 = 0.5450750000e-1;
const double c9 = 0.4214700000e-1;
const double c10 = 0.360e0;
const double c11 = 0.2107350000e-1;
const double c12 = 0.111906e0;
const double c13 = 0.117219e0;
const double c14 = 0.1800000000e0;
const double c15 = 0.122094e0;
const double c16 = 0.116781e0;
const double c17 = 0.134599999999999997e-2;
const double c18 =  0.207099999999999992e-2;
const double c19 = 0.103549999999999996e-2;
const double c20 = 0.122094000000000008e0;
const double c21 =  0.421469999999999970e-1;
const double c22 = 0.210734999999999985e-1;
const double c23 = 0.117219000000000004e0;
const double c24 = 0.116781000000000010e0;
const double c25 = 0.111906000000000005e0;

const double c101 = 0.3800000000e0;
const double c102 = 0.360e0;
const double c103 = 0.1900000000e0;
const double c104 = 0.1800000000e0;
const double c105 = 0.117e0;

const double mi_1 = 3.49;
const double mi_2 = 4.95;
const double mi_3 = 1.71;
const double mi_total = 80.51;

Eigen::Matrix<double,3,3> J_CoM_HAA_LF, J_CoM_HFE_LF, J_CoM_KFE_LF;
Eigen::Matrix<double,3,3> J_CoM_HAA_RF, J_CoM_HFE_RF, J_CoM_KFE_RF;
Eigen::Matrix<double,3,3> J_CoM_HAA_LH, J_CoM_HFE_LH, J_CoM_KFE_LH;
Eigen::Matrix<double,3,3> J_CoM_HAA_RH, J_CoM_HFE_RH, J_CoM_KFE_RH;

void updateCoMJacobian(Eigen::Matrix<double,12,1> *, Eigen::Matrix<double,3,3> *, Eigen::Matrix<double,3,3> *, Eigen::Matrix<double,3,3> *, Eigen::Matrix<double,3,3> *);

void updateJacobian(Eigen::Matrix<double,12,1> *, Eigen::Matrix<double,3,3> *, Eigen::Matrix<double,3,3> *, Eigen::Matrix<double,3,3> *, Eigen::Matrix<double,3,3> *);

#endif
#include <hyq2max_governor_control/hyq2max_governor_control.h>
#include <Eigen/Dense>
#include <Eigen/QR> 
#include <iostream>


void setup_values(double kp, double kd, double mass){
    joint_Kp = kp;
    joint_Kd = kd;
    hyq2max_mass = mass;
    I_sigma << Eigen::MatrixXd::Identity(3,3), Eigen::MatrixXd::Identity(3,3), Eigen::MatrixXd::Identity(3,3), Eigen::MatrixXd::Identity(3,3);

}


void update_ss_matrices(Eigen::Matrix<double,3,12> *J_foot, Eigen::Matrix<double,3,12> *J_CoM){
    //Update Jc
    Jc.block(0,0,3,3) = (*J_foot).block(0,0,3,3).inverse().transpose();
    Jc.block(3,3,3,3) = (*J_foot).block(0,3,3,3).inverse().transpose();
    Jc.block(6,6,3,3) = (*J_foot).block(0,6,3,3).inverse().transpose();
    Jc.block(9,9,3,3) = (*J_foot).block(0,9,3,3).inverse().transpose();

    //std::cout << Jc << std::endl << std::endl;

    //Update J and J_plus
    J.block(0,0,3,3) = ((*J_CoM).block(0,0,3,3) - 0.25*(*J_foot).block(0,0,3,3)).transpose();
    J.block(0,3,3,3) = ((*J_CoM).block(0,3,3,3) - 0.25*(*J_foot).block(0,3,3,3)).transpose();
    J.block(0,6,3,3) = ((*J_CoM).block(0,6,3,3) - 0.25*(*J_foot).block(0,6,3,3)).transpose();
    J.block(0,9,3,3) = ((*J_CoM).block(0,9,3,3) - 0.25*(*J_foot).block(0,9,3,3)).transpose();

    //std::cout << J << std::endl;

    //J_plus = J.completeOrthogonalDecomposition().pseudoInverse();
    Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cqr(J);
    J_plus = cqr.pseudoInverse();
    //std::cout << J_plus << std::endl << std::endl;

    std::cout << I_sigma*Jc*J_plus*joint_Kd/hyq2max_mass << std::endl << std::endl;

}
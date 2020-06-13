#include <hyq2max_governor_control/hyq2max_governor_control.h>
#include <Eigen/Dense>
#include <Eigen/QR> 
#include <iostream>


void setup_values(double kp, double kd, double mass, double gravity, double discrete_time){
    joint_Kp = kp;
    joint_Kd = kd;
    hyq2max_mass = mass;
    gravity_value = gravity;
    Ts = discrete_time;
    I_sigma << Eigen::MatrixXd::Identity(3,3), Eigen::MatrixXd::Identity(3,3), Eigen::MatrixXd::Identity(3,3), Eigen::MatrixXd::Identity(3,3);
    gravity_vector(2,0) = 1;

    //std::cout << gravity_vector << std::endl;

    /*Qe = qe*MatrixXd::Identity(12*Ny, 12*Ny);
    Qw = qw*MatrixXd::Identity(12*Ny, 12*Ny);
    Qq = qq*MatrixXd::Identity(12*Ny, 12*Ny);
    Qdw = qdw*MatrixXd::Identity(12*Nu, 12*Nu);*/
    
}

//update_ss_matrices(&Ac, &Bc, &Ad, &Bd, &Ae, &Be, &Ce, &J_foot, &J_CoM);
void update_ss_matrices(Eigen::Matrix<double,16,16> *Ac, Eigen::Matrix<double,16,12> *Bc, Eigen::Matrix<double,16,16> *Ad, Eigen::Matrix<double,16,12> *Bd, Eigen::Matrix<double,28,28> *Ae, Eigen::Matrix<double,28,12> *Be, Eigen::Matrix<double,12,28> *Ce, Eigen::Matrix<double,3,12> *J_foot, Eigen::Matrix<double,3,12> *J_CoM){
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
    //std::cout << J_plus << std::endl;

    //std::cout << I_sigma*Jc*J_plus*joint_Kd/hyq2max_mass << std::endl << std::endl;
    // Udpate Dynamic Matrix
    (*Ac).block(0,0,3,3) = I_sigma*Jc*J_plus*joint_Kd/hyq2max_mass;
    (*Ac).block(0,3,3,1) = gravity_vector;
    (*Ac).block(0,4,3,12) = I_sigma*Jc*joint_Kp/hyq2max_mass;
    (*Ac).block(4,0,12,3) = J_plus;
    //std::cout << (*Ac) << std::endl << std::endl;

    // Update Control Matrix
    (*Bc).block(0,0,3,12) = -I_sigma*Jc*joint_Kp/hyq2max_mass;
    //std::cout << (*Bc) << std::endl << std::endl;

    //Update Discrete state space matrices
    (*Ad) =  Eigen::MatrixXd::Identity(16,16) + Ts*(*Ac);
    (*Bd) = Ts*(*Bc);

    //std::cout << "Print " << std::endl;
    //std::cout << (*Ad) << std::endl << std::endl;
    //std::cout << (*Bd) << std::endl << std::endl;
    
    //Update Augmented Matrices
    (*Ae).block(0,0,16,16) = (*Ad);
    (*Ae).block(0,16,16,12) = (*Bd);
    (*Ae).block(16,16,12,12) = Eigen::MatrixXd::Identity(12,12);
    
    (*Be).block(0,0,16,12) = (*Bd);
    (*Be).block(16,0,12,12) = Eigen::MatrixXd::Identity(12,12);

    (*Ce).block(0,4,12,12) = Eigen::MatrixXd::Identity(12,12);

    //std::cout << "Aug " << std::endl;
    //std::cout << (*Ae) << std::endl << std::endl;
    //std::cout << (*Be) << std::endl << std::endl;
    //std::cout << (*Ce) << std::endl << std::endl;

}


void update_opt_matrices(Eigen::Matrix<double,28,28> *Ae, Eigen::Matrix<double,28,12> *Be, Eigen::Matrix<double,12,28> *Ce, double Ny, double Nu, Eigen::MatrixXd *PHI, Eigen::MatrixXd *Gbar){

    (*PHI).block(0,0,12,28) = (*Ce)*((*Ae));
    for(int i = 1; i < Ny; i++){
        //(*PHI).block(0 + 28*i,0,12,28) = (*Ce)*((*Ae));//.pow(i+1));
        (*PHI).block(0 + 12*i,0,12,28) = (*PHI).block(0 + 12*(i-1),0,12,28)*(*Ae);
    }

    ////std::cout << "Opt PHI " << std::endl;
    //std::cout << (*PHI) << std::endl << std::endl;
    //std::cout << (*PHI).block(0,0,10,12) << std::endl << std::endl;
    ////std::cout << (*PHI).block(5*12,0,10,28)*(*Be) << std::endl << std::endl;

    for (int i = 0; i < Ny; i++){
        for (int j = 0; j < Nu; j++){
            if ( i>=j ){
                //(*Gbar).block(12*i, 12*j, 12, 12) = (*PHI).block(12*i,0,12,28)*(*Be);
                (*Gbar).block(12*i, 12*j, 12, 12) = (*PHI).block(12*(i-j),0,12,28)*(*Be);
            }
        }
    }

    ////std::cout << "Opt Gbar " << std::endl;
    //std::cout << (*Gbar) << std::endl << std::endl;
    ////std::cout << (*Gbar).block(168,0,10,12) << std::endl << "HUE" << std::endl << (*Gbar).block(168,108,10,12)  << std::endl << std::endl;

    //std::cout << "Opt PHI " << std::endl;
    //std::cout << (*PHI).rows() << "  " << (*PHI).cols() << std::endl << std::endl;
    //(*Hqp) = ((*Gbar).transpose())*(Qy)*(*Gbar) + Qw;

}
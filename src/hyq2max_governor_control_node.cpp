// Includes
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/JointState.h"
#include "gazebo_msgs/ContactsState.h"
#include <hyq2max_joints_position_controller/HyQ2max_joints.h>
#include <hyq2max_joints_position_controller/HyQ2max_command.h>
#include <Eigen/Dense>
#include <tf/tf.h>
#include <hyq2max_governor_control/hyq2max_governor_control.h>
#include "hyq2max_governor_control/hyq2max_jacobian_functions.h"
#include <sys/time.h>
#include "osqp.h"
#include "OsqpEigen/OsqpEigen.h"
#include "OsqpEigen/Solver.hpp"

// instantiate the solver
OsqpEigen::Solver solver;

// Global Variables definition
// Jacobians referred to foot
Eigen::Matrix<double,3,3> J_foot_LF, J_foot_RF, J_foot_LH, J_foot_RH;
Eigen::Matrix<double,3,12> J_foot;
// Jacobians referred to CoM
Eigen::Matrix<double,3,3> J_CoM_LF, J_CoM_RF, J_CoM_LH, J_CoM_RH;
Eigen::Matrix<double,3,12> J_CoM;
//
Eigen::Matrix<double,3,1> CoM_position, CoM_vel;
Eigen::Matrix<double,16,16> Ac, Ad;
Eigen::Matrix<double,16,12> Bc, Bd;
//Eigen::Matrix<double,12,16> Cd;
Eigen::Matrix<double,28,28> Ae;
Eigen::Matrix<double,28,12> Be;
Eigen::Matrix<double,12,28> Ce;
Eigen::Matrix<double,28,1> Xe;
Eigen::Matrix<double,12,1> W, DeltaW, Wold;
//Eigen::Matrix<double,180,28> PHI;   // Allocated size for Ny = Nu = 20
//Eigen::Matrix<double,180,120> Gbar; // Allocated size for Ny = Nu = 20
Eigen::MatrixXd PHI(180,28);
Eigen::MatrixXd Gbar(180,120);
Eigen::MatrixXd Gbar_transpose(120,180);
Eigen::MatrixXd Hqp(120,120);
//Eigen::Matrix<double,120,120> Hqp;
Eigen::VectorXd Fqp;
Eigen::VectorXd QPSolution, ctrl;

Eigen::MatrixXd ref_array(180,1);
Eigen::SparseMatrix<double> hessian_sparse;
//Eigen::Matrix<double,180,28> PHI;   // Allocated size for Ny = Nu = 20
//Eigen::Matrix<double,180,120> Gbar; // Allocated size for Ny = Nu = 20
Eigen::MatrixXd Qy, Qe, Qw, Qq, Qdw;
//Eigen::SparseMatrix<double> Qy, Qe, Qw, Qq, Qdw;
double time_old, time_now;
int Ny = 15, Nu = 10;
double Kp = 300, Kd = 10;
double total_mass = 80.51;
double time_step = 0.01;
double timer_period = 0;

hyq2max_joints_position_controller::HyQ2max_command msg_pos_cmd;
ros::Publisher joint_cmd_pub;


double get_cpu_time(){
    		return (double)clock() / CLOCKS_PER_SEC;
}

void groundTruthCallback(const nav_msgs::Odometry::ConstPtr& msg, Eigen::Matrix<double,6,1> *base_pos, Eigen::Matrix<double,6,1> *base_vel)
{

    //ROS_INFO("I heard: [%s]", msg->child_frame_id.c_str());
    //ROS_INFO("%f %f %f", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    // Get linear position
    (*base_pos)(0,0) = msg->pose.pose.position.x;
    (*base_pos)(1,0) = msg->pose.pose.position.y;
    (*base_pos)(2,0) = msg->pose.pose.position.z;

    // Get quaternion and conver to RPY
     tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf::Matrix3x3 temp_matrix(q);
    temp_matrix.getRPY((*base_pos)(3,0), (*base_pos)(4,0), (*base_pos)(5,0));

    // Get linear velocity
    (*base_vel)(0,0) = msg->twist.twist.linear.x;
    (*base_vel)(1,0) = msg->twist.twist.linear.y;
    (*base_vel)(2,0) = msg->twist.twist.linear.z;
    // Get angular velocity
    (*base_vel)(3,0) = msg->twist.twist.angular.x;
    (*base_vel)(4,0) = msg->twist.twist.angular.y;
    (*base_vel)(5,0) = msg->twist.twist.angular.z;
}

void jointStatesCallback(const hyq2max_joints_position_controller::HyQ2max_joints::ConstPtr& msg, Eigen::Matrix<double,12,1> *q,Eigen::Matrix<double,12,1> *qp,Eigen::Matrix<double,12,1> *q_torque)
{

    //ROS_INFO("I heard joint: %s", msg->name[0].c_str());
    //ROS_INFO("%f %f %f", msg->position[0], msg->position[1], msg->position[2]);
    for (int i = 0; i < 12; i++){
        (*q)(i,0) = msg->joints_pos[i];
        (*qp)(i,0) = msg->joints_vel[i];
        (*q_torque)(i,0) = msg->joints_command[i];
    }
    /*std::cout << "pos: " << q << std::endl;
    std::cout << "vel: " << qp << std::endl << std::endl;
    std::cout << "tranpose: " << q.transpose()*qp << std::endl << std::endl;*/

}

void footBumperCallback(const gazebo_msgs::ContactsState::ConstPtr& msg, std::string *name)
{

    //ROS_INFO("I heard bumper: %s", msg->states[0].collision2_name.c_str());
    //ROS_INFO("%f %f %f", msg->states[0].contact_positions[0].x, msg->states[0].contact_positions[0].y, msg->states[0].contact_positions[0].z);

    *name = msg->states[0].collision1_name;
    //ROS_INFO("I heard bumper: %s", (*name).c_str());

}


void timerCallback(const ros::TimerEvent& event, std::string *name, Eigen::Matrix<double,12,1> *q, Eigen::Matrix<double,12,1> *qp, Eigen::Matrix<double,12,1> *q_torque, Eigen::Matrix<double,6,1> *Xw, Eigen::Matrix<double,6,1> *Xwp, Eigen::Matrix<double,12,1> *qref)//, ros::Publisher *joint_w_pub)//, Eigen::Matrix<double,3,3> *J_CoM_LF, Eigen::Matrix<double,3,3> *J_CoM_LH)//, Eigen::Matrix<double,3,3> *J_CoM_RH)//, Eigen::Matrix<double,3,3> *J_foot_LF, Eigen::Matrix<double,3,3> *J_foot_RF, Eigen::Matrix<double,3,3> *J_foot_LH, Eigen::Matrix<double,3,3> *J_foot_RH)
{
    //ROS_INFO("number of bumpers %d", )
   // ROS_INFO("I SAVED bumper string %s", (*name).c_str());
/*
    std::string joint_topic_cmd_pub = "/hyq2max/HyQ2maxJointsPositionController/command";
    //ros::Publisher joint_cmd_pub = node_handle.advertise<hyq2max_joints_position_controller::HyQ2max_command>(joint_topic_cmd_pub, 100);
    hyq2max_joints_position_controller::HyQ2max_command msg_cmd;
    //msg_cmd.pos_comamnd = [-0.1,0.75, -1.5, -0.1,-0.75,1.5,-0.1,-0.75,1.5,-0.1,0.75,-1.5]
    msg_cmd.pos_command[2] = -1.5; msg_cmd.pos_command[5] = -1.5; msg_cmd.pos_command[8] = -1.5; msg_cmd.pos_command[11] = -1.5;

    cmd_publisher.publish(msg_cmd);*/
    /*
    std::cout << "pos: " << (*q) << std::endl;
    std::cout << "vel: " << (*qp) << std::endl << std::endl;
    std::cout << "tranpose: " << (*q).transpose()*(*qp) << std::endl << std::endl;
    std::cout << "torque: " << (*q_torque) << std::endl;
    std::cout << "TRunk        ********  #############3" << std::endl;
    std::cout << "trunk pos: " << (*Xw) << std::endl;
    std::cout << "trunk vel: " << (*Xwp) << std::endl << std::endl;*/

    time_old = get_cpu_time();
    updateCoMJacobian( q, &J_CoM_LF, &J_CoM_RF, &J_CoM_LH, &J_CoM_RH, &J_CoM);

    updateJacobian( q, &J_foot_LF, &J_foot_RF, &J_foot_LH, &J_foot_RH, &J_foot);
    
    J_CoM << J_CoM_LF, J_CoM_RF, J_CoM_LH, J_CoM_RH;
    J_foot << J_foot_LF, J_foot_RF, J_foot_LH, J_foot_RH;
    CoM_vel = (*Xwp).topRows(3) + J_CoM*(*qp);
    
    //std::cout << CoM_vel << std::endl << std::endl;
    //Debug CoM position
    //CoM_position = CoM_position + 0.1*CoM_vel;
    //std::cout << "Position CoM: "<< std::endl << CoM_position << std::endl;
    //time_now = get_cpu_time();
    //std::cout << "Compute time: " << time_now - time_old  << std::endl << std::endl;

    //Update State Space Matrices
    //std::cout << J_foot << std::endl;
    update_ss_matrices(&Ac, &Bc, &Ad, &Bd, &Ae, &Be, &Ce, &J_foot, &J_CoM);

    update_opt_matrices(&Ae, &Be, &Ce, Ny, Nu, &PHI, &Gbar);

    time_now = get_cpu_time();
    //std::cout << "Compute time opt: " << get_cpu_time() - time_old  << std::endl;

   // Gbar_transpose = Gbar.transpose()*Qy;
    //std::cout << "Compute time transp Gbar: " << get_cpu_time() - time_old  << std::endl;

    Hqp = (Gbar.transpose())*Qy*Gbar + Qdw;
    //Hqp = (Gbar.transpose())*Qy*Gbar + Qw;
    //Hqp = (Gbar.transpose())*Qy*Gbar;
   // Hqp = Gbar_transpose*Gbar; 
    //hessian_sparse = Gbar_transpose*Gbar; 
    //std::cout << "Size: " << Hqp.size()  << std::endl;
    //std::cout << "Size: " << Qw.size()  << std::endl << std::endl;

    //std::cout << "Compute time Hqp: " << get_cpu_time() - time_old  << std::endl;

    //Eigen::MatrixXd st(28,1);
    Xe.block(0, 0, 3, 1) = CoM_vel;
    //Xe.block(3, 0, 1, 1) = -9.81;
    Xe(3, 0) = 9.81;
    Xe.block(4, 0, 12, 1) = (*q);
    Xe.block(16, 0, 12, 1) = Wold;

    Fqp = 2*((PHI*Xe-ref_array).transpose())*Qy*Gbar;

    //std::cout << "Compute time Fqp: " << get_cpu_time() - time_old  << std::endl << std::endl << std::endl;

    /*time_now = get_cpu_time();
    std::cout << "Compute time: " << time_now - time_old  << std::endl << std::endl;*/


    //Update Solver Data
    hessian_sparse = Hqp.sparseView();
    //std::cout << "Compute time sparse hqp: " << get_cpu_time() - time_old  << std::endl;
    solver.updateHessianMatrix(hessian_sparse);
    solver.updateGradient(Fqp.transpose());

    //std::cout << "Compute time solver matrix: " << get_cpu_time() - time_old  << std::endl;
    //Solve the QP problem
    if(!solver.solve()){
         std::cout << "ERROs" << std::endl;
    }
    // std::cout << "Compute time solver compute: " << get_cpu_time() - time_old  << std::endl;

    // get the controller input
    QPSolution = solver.getSolution();
    DeltaW = QPSolution.block(0, 0, 12, 1);

    W = Wold + DeltaW;
    Wold = W;
    //std::cout << "W:" << W << std::endl << std::endl;

    //Pub Joint Command
    for(int i = 0; i < 12; i++){
        //msg_pos_cmd.pos_command[i] = ref_array(i,0);
        msg_pos_cmd.pos_command[i] = W(i,0);
    }
    //(*joint_w_pub).publish(msg_pos_cmd);
    joint_cmd_pub.publish(msg_pos_cmd);

    //Teste time from timer
    //std::cout << "timer period: " << timer_period - get_cpu_time()  << std::endl;
    //timer_period = get_cpu_time();
}


// Main Loop
int main(int argc, char **argv)
{
    
    // Variables definition
    Eigen::Matrix<double,6,1> Xw, Xwp; // Base/Trunk position and velocity
    Eigen::Matrix<double,12,1> q, qp, q_torque; // Joint position and velocity
    Eigen::Matrix<double,12,4> contact_point, contact_force; // Joint position and velocity
    Eigen::Matrix<double,12,1> qref;
    // Jacobians referred to foot
    //Eigen::Matrix<double,3,3> J_foot_LF, J_foot_RF, J_foot_LH, J_foot_RH;
    // Jacobians referred to CoM
    /*
    Eigen::Matrix<double,3,3> J_CoM_HAA_LF, J_CoM_HFE_LF, J_CoM_KFE_LF;
    Eigen::Matrix<double,3,3> J_CoM_HAA_RF, J_CoM_HFE_RF, J_CoM_KFE_RF;
    Eigen::Matrix<double,3,3> J_CoM_HAA_LH, J_CoM_HFE_LH, J_CoM_KFE_LH;
    Eigen::Matrix<double,3,3> J_CoM_HAA_RH, J_CoM_HFE_RH, J_CoM_KFE_RH;*/
  //  Eigen::Matrix<double,3,3> J_CoM_LF, J_CoM_RF, J_CoM_LH, J_CoM_RH;

    ros::init(argc, argv, "hyq2max_governor_control_node");

    ros::NodeHandle node_handle;
    std::vector<std::string> bumper_topic_names;//({"/lf_foot_bumper", "/lh_foot_bumper", "/rf_foot_bumper", "/rh_foot_bumper"});
    bumper_topic_names.push_back("/lf_foot_bumper");
    bumper_topic_names.push_back("/lh_foot_bumper");
    bumper_topic_names.push_back("/rf_foot_bumper");
    bumper_topic_names.push_back("/rh_foot_bumper");
    
    std::string colission_names[4];
    std::vector<ros::Subscriber> sub_foot_bumper;//[4];

    std::string joints_cmd_topic_name;// = "/hyq2max/HyQ2maxJointsPositionController/command";
    node_handle.param<std::string>("joints_cmd_topic", joints_cmd_topic_name, "/hyq2max/HyQ2maxJointsPositionController/command");

    std::string ground_truth_topic_name;
    node_handle.param<std::string>("ground_truth_topic", ground_truth_topic_name, "/hyq2max/ground_truth");

    std::string joints_state_topic_name;
    node_handle.param<std::string>("joints_state_topic", joints_state_topic_name, "/hyq2max/HyQ2maxJointsPositionController/state_joint");

    // Set Trunk state subscriber (Ground Truth)
    //ros::Subscriber sub_ground_truth = node_handle.subscribe(ground_truth_topic_name, 100, groundTruthCallback);
    ros::Subscriber sub_ground_truth = node_handle.subscribe<nav_msgs::Odometry> (ground_truth_topic_name, 100, boost::bind(groundTruthCallback, _1, &Xw, &Xwp));

    // Set Joint state subscriber
    //ros::Subscriber sub_joint_states = node_handle.subscribe(joints_state_topic_name, 100, jointStatesCallback, q, qp);

    ros::Subscriber sub_joint_states = node_handle.subscribe<hyq2max_joints_position_controller::HyQ2max_joints> (joints_state_topic_name, 100, boost::bind(jointStatesCallback, _1, &q, &qp, &q_torque));
    // boost::bind(
    //ros::Subscriber sub_foot_bumper = node_handle.subscribe("/lf_foot_bumper", 100, footBumperCallback);
    //ros::Subscriber sub_foot_bumper = node_handle.subscribe<gazebo_msgs::ContactsState> ("/lf_foot_bumper", 100, boost::bind(footBumperCallback, _1, &colission_names[0]));

/*
    for(int i = 0; i < bumper_topic_names.size() ; i++){
        ros::Subscriber single_sub_foot = node_handle.subscribe<gazebo_msgs::ContactsState> (bumper_topic_names[i], 100, boost::bind(footBumperCallback, _1, &colission_names[i]));
        sub_foot_bumper.push_back(single_sub_foot);
    }*/

    setup_values(Kp, Kd, total_mass, 9.81, time_step);

    //Set Weight Matrices
    /*Qy.resize(12*Ny, 12*Ny);
    Qe.resize(12*Ny, 12*Ny);
    Qw.resize(12*Nu, 12*Nu);
    Qq.resize(12*Ny, 12*Ny);
    Qdw.resize(12*Nu, 12*Nu);*/
    Qe = 0.9 * Eigen::MatrixXd::Identity(12*Ny, 12*Ny);
    //Qy = 0.9 * Eigen::MatrixXd::Identity(12*Ny, 12*Ny);
    //Qy = 0.3 * Eigen::MatrixXd::Identity(12*Ny, 12*Ny);
    Qy = 0.1 * Eigen::MatrixXd::Identity(12*Ny, 12*Ny);
    Qq = 1.4 * Eigen::MatrixXd::Identity(12*Ny, 12*Ny);
    Qw = 1.4 * Eigen::MatrixXd::Identity(12*Nu, 12*Nu);
    Qdw = 100 * Eigen::MatrixXd::Identity(12*Nu, 12*Nu);

    //Set Reference for Joints
    Eigen::MatrixXd ref_joint(12,1);
    ref_joint << -0.05, 0.65, -1.18, -0.05, 0.65, -1.18, -0.05, -0.65, 1.18, -0.05, -0.65, 1.18;
    //Wold << 0.0005677374429069459, 0.3055045008659363, -0.9169064164161682, -0.00456430297344923, 0.30609703063964844, -0.9157984256744385, 0.015615483745932579, -0.2985933721065521, 0.8632257580757141, 0.0168602354824543, -0.2996049225330353, 0.8676934838294983;
    Wold << -0.05, 1.3, -2.65,-0.05, 1.3, -2.65, -0.05, -1.3, 2.65, -0.05, -1.3, 2.65;
    
    for(int i = 0; i < Ny; i++){
        ref_array.block(i*12,0,12,1) = ref_joint; 
    }

    //Set dumb Constrait Matrices
    Eigen::SparseMatrix<double> linearMatrix(1,12*Nu);
    Eigen::VectorXd lowerBound(1);
    Eigen::VectorXd upperBound(1);

    //linearMatrix.resize(1,1);
    //upperBound.resize(1,1);
    //lowerBound.resize(1,1);
    //linearMatrix(0,0) = 1;
    linearMatrix.insert(0,0) = 1;
    upperBound(0) = 100;
    lowerBound(0) = -100;


    // Initial Matrices Data
    Hqp = Eigen::MatrixXd::Identity(12*Nu, 12*Nu);
    Fqp = Eigen::MatrixXd::Constant(12*Nu, 1, 1);

    //Solver Parameters
    solver.settings()->setVerbosity(false);

    //Init Data do Solver
    solver.data()->setNumberOfVariables(12 * Nu);
    solver.data()->setNumberOfConstraints(1);
    hessian_sparse = Hqp.sparseView();
    solver.data()->setHessianMatrix(hessian_sparse);
    //Fqp = Eigen::MatrixXd::Zero(120, 1);
    solver.data()->setGradient(Fqp.transpose());
    solver.data()->setLinearConstraintsMatrix(linearMatrix);
    solver.data()->setLowerBound(lowerBound);
    solver.data()->setUpperBound(upperBound);

    // instantiate the solver
    if(!solver.initSolver()){
        std::cout << "Problem " << std::endl;
    }
    else{
         std::cout << "OK " << std::endl;
    }

    //updateCoMJacobian( &q, &J_CoM_LF, &J_CoM_RF, &J_CoM_LH, &J_CoM_RH);

    //updateJacobian( &q, &J_foot_LF, &J_foot_RF, &J_foot_LH, &J_foot_RH);

    //Eigen::Matrix<double,28,28> Ae;
    //Eigen::MatrixPower<Matrix> Apow(Ae);
   // std::cout << Apow(4) << std::endl << std::endl;

    


    //ros::Publisher joint_cmd_pub = node_handle.advertise<hyq2max_joints_position_controller::HyQ2max_command>(joints_cmd_topic_name, 100);
    joint_cmd_pub = node_handle.advertise<hyq2max_joints_position_controller::HyQ2max_command>(joints_cmd_topic_name, 100);

    ros::Timer timer = node_handle.createTimer(ros::Duration(time_step), boost::bind(timerCallback, _1, &colission_names[3], &q, &qp, &q_torque, &Xw, &Xwp, &qref));//, &joint_cmd_pub));//, &J_CoM_LF, &J_CoM_LH));//, &J_CoM_RH));//, &J_foot_LF, &J_foot_RF, &J_foot_LH, &J_foot_RH));

    ros::spin();

/*
    ros::Rate loop_rate(100);
    while (ros::ok()){

        //hyq2max_joints_position_controller::HyQ2max_command msg_cmd;
        //msg_cmd.pos_comamnd = [-0.1,0.75, -1.5, -0.1,-0.75,1.5,-0.1,-0.75,1.5,-0.1,0.75,-1.5]
       // msg_cmd.pos_command[2] = -1.5; msg_cmd.pos_command[5] = -1.5; msg_cmd.pos_command[8] = -1.5; msg_cmd.pos_command[11] = -1.5;

        // std::cout << "teste pub" << std::endl << std::endl;
        for(int i = 0; i < 12; i++){
            //msg_pos_cmd.pos_command[i] = ref_array(i,0);
            msg_pos_cmd.pos_command[i] = W(i,0);
        }
        Wold = W;

        joint_cmd_pub.publish(msg_pos_cmd);

        ros::spinOnce();
        loop_rate.sleep();

    }*/


    
    return 0;
}


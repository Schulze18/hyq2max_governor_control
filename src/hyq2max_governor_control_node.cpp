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


void timerCallback(const ros::TimerEvent& event, std::string *name, Eigen::Matrix<double,12,1> *q, Eigen::Matrix<double,12,1> *qp, Eigen::Matrix<double,12,1> *q_torque, Eigen::Matrix<double,6,1> *Xw,Eigen::Matrix<double,6,1> *Xwp){//}, ros::Publisher cmd_publisher){
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

}


// Main Loop
int main(int argc, char **argv)
{
    // Variables definition
    Eigen::Matrix<double,6,1> Xw, Xwp; // Base/Trunk position and velocity
    Eigen::Matrix<double,12,1> q, qp, q_torque; // Joint position and velocity
    Eigen::Matrix<double,12,4> contact_point, contact_force; // Joint position and velocity
    // Jacobians referred to foot
    Eigen::Matrix<double,3,3> J_foot_LF, J_foot_RF, J_foot_LH, J_foot_RH;
    // Jacobians referred to CoM
    Eigen::Matrix<double,3,3> J_CoM_HAA_LF, J_CoM_HFE_LF, J_CoM_KFE_LF;
    Eigen::Matrix<double,3,3> J_CoM_HAA_RF, J_CoM_HFE_RF, J_CoM_KFE_RF;
    Eigen::Matrix<double,3,3> J_CoM_HAA_LH, J_CoM_HFE_LH, J_CoM_KFE_LH;
    Eigen::Matrix<double,3,3> J_CoM_HAA_RH, J_CoM_HFE_RH, J_CoM_KFE_RH;
    Eigen::Matrix<double,3,3> J_CoM_LF, J_CoM_RF, J_CoM_LH, J_CoM_RH;

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

    for(int i = 0; i < bumper_topic_names.size() ; i++){
        ros::Subscriber single_sub_foot = node_handle.subscribe<gazebo_msgs::ContactsState> (bumper_topic_names[i], 100, boost::bind(footBumperCallback, _1, &colission_names[i]));
        sub_foot_bumper.push_back(single_sub_foot);
    }

    updateCoMJacobian( &q, &J_CoM_LF, &J_CoM_RF, &J_CoM_LH, &J_CoM_RH);

    updateJacobian( &q, &J_foot_LF, &J_foot_RF, &J_foot_LH, &J_foot_RH);

    ros::Publisher joint_cmd_pub = node_handle.advertise<hyq2max_joints_position_controller::HyQ2max_command>(joints_cmd_topic_name, 100);

    ros::Timer timer = node_handle.createTimer(ros::Duration(2), boost::bind(timerCallback, _1, &colission_names[3], &q, &qp, &q_torque, &Xw, &Xwp));

    ros::spin();

/*
    ros::Rate loop_rate(10);
    while (ros::ok()){

        hyq2max_joints_position_controller::HyQ2max_command msg_cmd;
        //msg_cmd.pos_comamnd = [-0.1,0.75, -1.5, -0.1,-0.75,1.5,-0.1,-0.75,1.5,-0.1,0.75,-1.5]
        msg_cmd.pos_command[2] = -1.5; msg_cmd.pos_command[5] = -1.5; msg_cmd.pos_command[8] = -1.5; msg_cmd.pos_command[11] = -1.5;

        joint_cmd_pub.publish(msg_cmd);


        ros::spinOnce();
        loop_rate.sleep();

    }*/


    
    return 0;
}


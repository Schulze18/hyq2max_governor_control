// Includes
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/JointState.h"
#include "gazebo_msgs/ContactsState.h"
#include <hyq2max_joints_position_controller/HyQ2max_joints.h>
#include <hyq2max_joints_position_controller/HyQ2max_command.h>


void groundTruthCallback(const nav_msgs::Odometry::ConstPtr& msg)
{

    ROS_INFO("I heard: [%s]", msg->child_frame_id.c_str());
    //ROS_INFO("%f %f %f", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

}

void jointStatesCallback(const hyq2max_joints_position_controller::HyQ2max_joints::ConstPtr& msg)
{

    //ROS_INFO("I heard joint: %s", msg->name[0].c_str());
    //ROS_INFO("%f %f %f", msg->position[0], msg->position[1], msg->position[2]);

}

void footBumperCallback(const gazebo_msgs::ContactsState::ConstPtr& msg, std::string *name)
{

    //ROS_INFO("I heard bumper: %s", msg->states[0].collision2_name.c_str());
    //ROS_INFO("%f %f %f", msg->states[0].contact_positions[0].x, msg->states[0].contact_positions[0].y, msg->states[0].contact_positions[0].z);

    *name = msg->states[0].collision1_name;
    //ROS_INFO("I heard bumper: %s", (*name).c_str());

}


void timerCallback(const ros::TimerEvent& event, std::string *name){//}, ros::Publisher cmd_publisher){
    //ROS_INFO("number of bumpers %d", )
    ROS_INFO("I SAVED bumper string %s", (*name).c_str());
/*
    std::string joint_topic_cmd_pub = "/hyq2max/HyQ2maxJointsPositionController/command";
    //ros::Publisher joint_cmd_pub = node_handle.advertise<hyq2max_joints_position_controller::HyQ2max_command>(joint_topic_cmd_pub, 100);
    hyq2max_joints_position_controller::HyQ2max_command msg_cmd;
    //msg_cmd.pos_comamnd = [-0.1,0.75, -1.5, -0.1,-0.75,1.5,-0.1,-0.75,1.5,-0.1,0.75,-1.5]
    msg_cmd.pos_command[2] = -1.5; msg_cmd.pos_command[5] = -1.5; msg_cmd.pos_command[8] = -1.5; msg_cmd.pos_command[11] = -1.5;

    cmd_publisher.publish(msg_cmd);*/

}


// Main Loop
int main(int argc, char **argv)
{

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
    ros::Subscriber sub_ground_truth = node_handle.subscribe(ground_truth_topic_name, 1000, groundTruthCallback);

    // Set Joint state subscriber
    ros::Subscriber sub_joint_states = node_handle.subscribe(joints_state_topic_name, 1000, jointStatesCallback);

    // boost::bind(
    //ros::Subscriber sub_foot_bumper = node_handle.subscribe("/lf_foot_bumper", 1000, footBumperCallback);
    //ros::Subscriber sub_foot_bumper = node_handle.subscribe<gazebo_msgs::ContactsState> ("/lf_foot_bumper", 100, boost::bind(footBumperCallback, _1, &colission_names[0]));

    for(int i = 0; i < bumper_topic_names.size() ; i++){
        ros::Subscriber single_sub_foot = node_handle.subscribe<gazebo_msgs::ContactsState> (bumper_topic_names[i], 100, boost::bind(footBumperCallback, _1, &colission_names[i]));
        sub_foot_bumper.push_back(single_sub_foot);
    }

    ros::Publisher joint_cmd_pub = node_handle.advertise<hyq2max_joints_position_controller::HyQ2max_command>(joints_cmd_topic_name, 100);

    ros::Timer timer = node_handle.createTimer(ros::Duration(0.1), boost::bind(timerCallback, _1, &colission_names[3]));

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

